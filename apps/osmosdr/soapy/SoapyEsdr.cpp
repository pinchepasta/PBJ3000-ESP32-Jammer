/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * SoapySDR module for the osmosdr app's protocol (esdr_proto.h), so GNU
 * Radio (gr-soapy), gqrx and SoapySDRUtil can use the board directly. Two
 * links: USB (the osmosdr app, found over libusb: vendor requests and bulk
 * blocks as tools/esdr_usb.c) and serial (the osmosdr_serial app, selected
 * with port=/dev/...: the same requests as frames on a byte stream). Half
 * duplex: activating one direction stops the other.
 *
 * Build (CMakeLists.txt next to this file):
 *   cmake -B build && cmake --build build
 * then point SOAPY_SDR_PLUGIN_PATH at build/ (or install it into SoapySDR's
 * module directory).
 */

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>

#include <libusb.h>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "esdr_proto.h"

#define ESDR_VID 0x2fe3
#define ESDR_PID 0x0005

namespace {

/* Device handle plus the endpoints and serial, found once at enumeration or open. */
struct UsbEsdr {
	libusb_device_handle *h = nullptr;
	uint8_t ep_in = 0, ep_out = 0;
	std::string serial;
};

bool openByCriteria(libusb_context *ctx, const std::string &wantSerial, UsbEsdr &out)
{
	libusb_device **list;
	ssize_t n = libusb_get_device_list(ctx, &list);
	bool found = false;

	for (ssize_t i = 0; i < n && !found; i++) {
		libusb_device_descriptor dd;
		libusb_device_handle *h;
		libusb_config_descriptor *cfg;
		char sn[64] = "";

		if (libusb_get_device_descriptor(list[i], &dd) != 0 || dd.idVendor != ESDR_VID ||
		    dd.idProduct != ESDR_PID) {
			continue;
		}
		if (libusb_open(list[i], &h) != 0) {
			continue;
		}
		if (dd.iSerialNumber != 0) {
			libusb_get_string_descriptor_ascii(h, dd.iSerialNumber, (unsigned char *)sn,
							   sizeof(sn));
		}
		if (!wantSerial.empty() && wantSerial != sn) {
			libusb_close(h);
			continue;
		}
		if (libusb_get_active_config_descriptor(list[i], &cfg) != 0) {
			libusb_close(h);
			continue;
		}
		for (int k = 0; k < cfg->bNumInterfaces && !found; k++) {
			const libusb_interface_descriptor *id = &cfg->interface[k].altsetting[0];

			if (id->bInterfaceClass != LIBUSB_CLASS_VENDOR_SPEC || id->bNumEndpoints != 2) {
				continue;
			}
			if (libusb_claim_interface(h, id->bInterfaceNumber) != 0) {
				continue;
			}
			for (int e = 0; e < 2; e++) {
				uint8_t a = id->endpoint[e].bEndpointAddress;

				if (a & LIBUSB_ENDPOINT_IN) {
					out.ep_in = a;
				} else {
					out.ep_out = a;
				}
			}
			out.h = h;
			out.serial = sn;
			found = true;
		}
		libusb_free_config_descriptor(cfg);
		if (!found) {
			libusb_close(h);
		}
	}
	libusb_free_device_list(list, 1);
	return found;
}

/* Receive-side ring of converted samples, written by the USB callback thread. */
class SampleRing {
public:
	void reset(size_t itemBytes)
	{
		std::lock_guard<std::mutex> g(m_);
		itemBytes_ = itemBytes;
		buf_.assign(CAPACITY * itemBytes, 0);
		head_ = tail_ = 0;
		overflowed_ = false;
		dropped_ = 0;
	}

	/* Items dropped because the reader fell behind, since the last reset. */
	uint64_t dropped(void)
	{
		std::lock_guard<std::mutex> g(m_);
		return dropped_;
	}

	/* Appends n items (dropping the oldest on overflow, flagged for the next read). */
	void push(const uint8_t *data, size_t n)
	{
		std::lock_guard<std::mutex> g(m_);
		for (size_t k = 0; k < n; k++) {
			std::memcpy(&buf_[(head_ % CAPACITY) * itemBytes_], data + k * itemBytes_,
				   itemBytes_);
			head_++;
			if (head_ - tail_ > CAPACITY) {
				tail_ = head_ - CAPACITY;
				overflowed_ = true;
				dropped_++;
			}
		}
		cv_.notify_one();
	}

	/* Copies at most n items into out; returns how many, and whether an overflow happened since. */
	size_t pop(uint8_t *out, size_t n, int timeoutUs, bool &overflow)
	{
		std::unique_lock<std::mutex> lk(m_);
		if (head_ == tail_) {
			cv_.wait_for(lk, std::chrono::microseconds(timeoutUs),
				    [&] { return head_ != tail_; });
		}
		size_t avail = head_ - tail_;
		size_t k = std::min(avail, n);

		for (size_t i = 0; i < k; i++) {
			std::memcpy(out + i * itemBytes_, &buf_[(tail_ % CAPACITY) * itemBytes_],
				   itemBytes_);
			tail_++;
		}
		overflow = overflowed_;
		overflowed_ = false;
		return k;
	}

private:
	/* About 1 s at 250 kS/s: more only adds latency when the reader lags. */
	static constexpr size_t CAPACITY = 1u << 18; /* items */
	std::mutex m_;
	std::condition_variable cv_;
	std::vector<uint8_t> buf_;
	size_t itemBytes_ = 2;
	uint64_t head_ = 0, tail_ = 0, dropped_ = 0;
	bool overflowed_ = false;
};

/* Requests and sample transport: USB (osmosdr) or a serial byte stream (osmosdr_serial). */
class Link {
public:
	using RxFn = std::function<void(const uint8_t *, size_t)>;

	virtual ~Link() = default;
	/* Data length, or negative on error. */
	virtual int ctrlOut(uint8_t req, uint16_t value, const void *data, uint16_t len) = 0;
	virtual int ctrlIn(uint8_t req, void *data, uint16_t len) = 0;
	/* Transmit bytes: how many were taken (0 on timeout), or negative on error. */
	virtual int writeSamples(const uint8_t *p, size_t n, long timeoutUs) = 0;
	/* Receive: the stream's bytes go to fn (in order, any split) until stopRx(). */
	virtual void startRx(RxFn fn) = 0;
	virtual void stopRx() = 0;
	virtual std::string serial() const = 0;
};

class UsbLink : public Link {
public:
	explicit UsbLink(const std::string &serial)
	{
		if (libusb_init(&ctx_) != 0) {
			throw std::runtime_error("esdr: libusb_init failed");
		}
		if (!openByCriteria(ctx_, serial, usb_)) {
			libusb_exit(ctx_);
			throw std::runtime_error("esdr: device not found" +
						 (serial.empty() ? "" : " (serial " + serial + ")"));
		}
	}

	~UsbLink() override
	{
		stopRx();
		libusb_release_interface(usb_.h, 0);
		libusb_close(usb_.h);
		libusb_exit(ctx_);
	}

	int ctrlOut(uint8_t req, uint16_t value, const void *data, uint16_t len) override
	{
		return libusb_control_transfer(usb_.h,
					       LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
						       LIBUSB_RECIPIENT_DEVICE,
					       req, value, 0, (unsigned char *)data, len, 1000);
	}

	int ctrlIn(uint8_t req, void *data, uint16_t len) override
	{
		return libusb_control_transfer(usb_.h,
					       LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR |
						       LIBUSB_RECIPIENT_DEVICE,
					       req, 0, 0, (unsigned char *)data, len, 1000);
	}

	/* Blocking bulk OUT: the device holds the transfer back while its backlog is full. */
	int writeSamples(const uint8_t *p, size_t n, long timeoutUs) override
	{
		int done = 0;
		int r = libusb_bulk_transfer(usb_.h, usb_.ep_out, const_cast<uint8_t *>(p), (int)n,
					     &done, (unsigned int)std::max(1L, timeoutUs / 1000));

		if (r != 0 && r != LIBUSB_ERROR_TIMEOUT) {
			return -1;
		}
		return done;
	}

	void startRx(RxFn fn) override
	{
		fn_ = std::move(fn);
		stop_ = false;
		thread_ = std::thread(&UsbLink::rxLoop, this);
	}

	void stopRx() override
	{
		stop_ = true;
		if (thread_.joinable()) {
			thread_.join();
		}
	}

	std::string serial() const override { return usb_.serial; }

private:
	/*
	 * Async bulk IN into fixed buffers, resubmitted as they complete. No
	 * timeout: queued transfers fill one after another, so a timeout counted
	 * from submission would expire on the later ones at low rates.
	 */
	void rxLoop()
	{
		static constexpr int XFERS = 8;
		static constexpr size_t XFER_BYTES = 16 * ESDR_BLOCK_BYTES;
		std::vector<libusb_transfer *> xfers(XFERS);
		std::vector<std::vector<uint8_t>> bufs(XFERS);

		auto cb = [](libusb_transfer *t) {
			auto *self = reinterpret_cast<UsbLink *>(t->user_data);

			/* A cancelled transfer may still hold data: never drop what arrived. */
			if (t->status == LIBUSB_TRANSFER_COMPLETED ||
			    t->status == LIBUSB_TRANSFER_CANCELLED ||
			    t->status == LIBUSB_TRANSFER_TIMED_OUT) {
				self->fn_(t->buffer, (size_t)t->actual_length);
			}
			if (self->stop_ || libusb_submit_transfer(t) != 0) {
				self->pending_--;
			}
		};
		pending_ = 0;
		for (int k = 0; k < XFERS; k++) {
			bufs[k].resize(XFER_BYTES);
			xfers[k] = libusb_alloc_transfer(0);
			libusb_fill_bulk_transfer(xfers[k], usb_.h, usb_.ep_in, bufs[k].data(),
						  (int)XFER_BYTES, cb, this, 0);
			if (libusb_submit_transfer(xfers[k]) == 0) {
				pending_++;
			}
		}
		while (!stop_) {
			timeval tv{0, 50000};

			libusb_handle_events_timeout(ctx_, &tv);
		}
		for (int k = 0; k < XFERS; k++) {
			libusb_cancel_transfer(xfers[k]);
		}
		/* Until every cancelled transfer has come back (at most a second). */
		for (int tries = 0; tries < 50 && pending_ > 0; tries++) {
			timeval tv{0, 20000};

			libusb_handle_events_timeout(ctx_, &tv);
		}
		for (auto *x : xfers) {
			libusb_free_transfer(x);
		}
	}

	libusb_context *ctx_ = nullptr;
	UsbEsdr usb_;
	RxFn fn_;
	std::thread thread_;
	std::atomic<bool> stop_{true};
	int pending_ = 0;
};

/*
 * The serial transport: request frames and their replies, receive blocks in
 * between (esdr_proto.h). One reader thread sorts what arrives. The port is
 * opened raw without touching DTR/RTS: on USB-Serial-JTAG that resets the chip.
 */
class SerialLink : public Link {
public:
	explicit SerialLink(const std::string &port) : port_(port)
	{
		fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
		if (fd_ < 0) {
			throw std::runtime_error("esdr: cannot open " + port);
		}
		termios a{};

		tcgetattr(fd_, &a);
		cfmakeraw(&a);
		a.c_cflag |= CLOCAL | CREAD;
		a.c_cflag &= ~HUPCL;
		a.c_cc[VMIN] = 0;
		a.c_cc[VTIME] = 1;
		tcsetattr(fd_, TCSANOW, &a);
		tcflush(fd_, TCIOFLUSH);
		/*
		 * A previous host may have stopped inside a transmit frame: zeros
		 * complete any payload, then the device resynchronizes on the magic.
		 */
		std::vector<uint8_t> zeros(sizeof(esdr_ser_hdr) + 4096, 0);

		writeAll(zeros.data(), zeros.size(), 0);
		reader_ = std::thread(&SerialLink::readLoop, this);
	}

	~SerialLink() override
	{
		stop_ = true;
		if (reader_.joinable()) {
			reader_.join();
		}
		::close(fd_);
	}

	int ctrlOut(uint8_t req, uint16_t value, const void *data, uint16_t len) override
	{
		std::vector<uint8_t> rep;

		return request(req, 0, value, data, len, rep);
	}

	int ctrlIn(uint8_t req, void *data, uint16_t len) override
	{
		std::vector<uint8_t> rep;
		int r = request(req, ESDR_SER_IN, 0, nullptr, len, rep);

		if (r < 0) {
			return r;
		}
		size_t n = std::min<size_t>(rep.size(), len);

		std::memcpy(data, rep.data(), n);
		return (int)n;
	}

	/* Frames of transmit data; the device's flow control paces the writes. */
	int writeSamples(const uint8_t *p, size_t n, long timeoutUs) override
	{
		size_t done = 0;

		while (done < n) {
			uint16_t k = (uint16_t)std::min<size_t>(n - done, 4096);
			esdr_ser_hdr h{ESDR_SER_MAGIC, ESDR_SER_TX_DATA, 0, 0, k, 0};
			std::lock_guard<std::mutex> g(wm_);

			if (!writeAll(&h, sizeof(h), timeoutUs) || !writeAll(p + done, k, timeoutUs)) {
				return done > 0 ? (int)done : -1;
			}
			done += k;
		}
		return (int)done;
	}

	void startRx(RxFn fn) override
	{
		std::lock_guard<std::mutex> g(rxm_);
		fn_ = std::move(fn);
	}

	void stopRx() override
	{
		std::lock_guard<std::mutex> g(rxm_);
		fn_ = nullptr;
	}

	std::string serial() const override { return port_; }

private:
	bool writeAll(const void *p, size_t n, long timeoutUs)
	{
		const uint8_t *b = static_cast<const uint8_t *>(p);
		int ms = timeoutUs > 0 ? (int)std::max(1L, timeoutUs / 1000) : 2000;

		while (n > 0) {
			pollfd pf{fd_, POLLOUT, 0};

			if (poll(&pf, 1, ms) <= 0) {
				return false;
			}
			ssize_t w = ::write(fd_, b, n);

			if (w < 0) {
				return false;
			}
			b += w;
			n -= (size_t)w;
		}
		return true;
	}

	int request(uint8_t req, uint8_t flags, uint16_t value, const void *data, uint16_t len,
		    std::vector<uint8_t> &rep)
	{
		std::unique_lock<std::mutex> lk(cm_);
		esdr_ser_hdr h{ESDR_SER_MAGIC, req, flags, value, len, 0};

		gotReply_ = false;
		wantReq_ = req;
		{
			std::lock_guard<std::mutex> g(wm_);

			if (!writeAll(&h, sizeof(h), 0) ||
			    ((flags & ESDR_SER_IN) == 0 && len > 0 && !writeAll(data, len, 0))) {
				return -1;
			}
		}
		if (!cv_.wait_for(lk, std::chrono::seconds(3), [&] { return gotReply_; })) {
			SoapySDR_logf(SOAPY_SDR_WARNING, "esdr: no reply to request %u", req);
			return -1;
		}
		if (replyStatus_ != 0) {
			return replyStatus_;
		}
		rep = reply_;
		return (flags & ESDR_SER_IN) != 0 ? (int)rep.size() : (int)len;
	}

	/* Sort the stream into reply frames and receive blocks; skip anything else byte by byte. */
	void readLoop()
	{
		std::vector<uint8_t> buf;
		uint8_t tmp[65536];

		while (!stop_) {
			pollfd pf{fd_, POLLIN, 0};

			if (poll(&pf, 1, 100) <= 0) {
				continue;
			}
			ssize_t r = ::read(fd_, tmp, sizeof(tmp));

			if (r <= 0) {
				continue;
			}
			buf.insert(buf.end(), tmp, tmp + r);
			size_t at = 0;

			while (buf.size() - at >= 4) {
				uint32_t magic;

				std::memcpy(&magic, &buf[at], 4);
				if (magic == ESDR_BLOCK_MAGIC) {
					if (buf.size() - at < ESDR_BLOCK_BYTES) {
						break;
					}
					{
						std::lock_guard<std::mutex> g(rxm_);

						if (fn_) {
							fn_(&buf[at], ESDR_BLOCK_BYTES);
						}
					}
					at += ESDR_BLOCK_BYTES;
				} else if (magic == ESDR_SER_MAGIC) {
					esdr_ser_hdr h;

					if (buf.size() - at < sizeof(h)) {
						break;
					}
					std::memcpy(&h, &buf[at], sizeof(h));
					if (buf.size() - at < sizeof(h) + h.len) {
						break;
					}
					{
						std::lock_guard<std::mutex> g(cm_);

						if (h.req == wantReq_) {
							reply_.assign(&buf[at + sizeof(h)],
								      &buf[at + sizeof(h) + h.len]);
							replyStatus_ = h.status;
							gotReply_ = true;
							cv_.notify_all();
						}
					}
					at += sizeof(h) + h.len;
				} else {
					at++;
				}
			}
			buf.erase(buf.begin(), buf.begin() + (long)at);
		}
	}

	std::string port_;
	int fd_ = -1;
	std::thread reader_;
	std::atomic<bool> stop_{false};
	/* Writes (frames stay whole), requests (one at a time), receive callback. */
	std::mutex wm_, cm_, rxm_;
	std::condition_variable cv_;
	bool gotReply_ = false;
	uint8_t wantReq_ = 0;
	int16_t replyStatus_ = 0;
	std::vector<uint8_t> reply_;
	RxFn fn_;
};

} /* namespace */

class SoapyEsdr : public SoapySDR::Device {
public:
	explicit SoapyEsdr(const SoapySDR::Kwargs &args)
	{
		if (args.count("port")) {
			link_ = std::make_unique<SerialLink>(args.at("port"));
			/* About 500 kB/s each way: 250 kS/s fits as 8-bit items only. */
			narrow_ = true;
		} else {
			link_ = std::make_unique<UsbLink>(args.count("serial") ? args.at("serial") : "");
		}
		struct esdr_info info{};

		if (link_->ctrlIn(ESDR_REQ_GET_INFO, &info, sizeof(info)) == (int)sizeof(info) &&
		    info.magic == ESDR_INFO_MAGIC) {
			freqMinHz_ = (double)info.freq_min_hz;
			freqMaxHz_ = (double)info.freq_max_hz;
			rateMaxHz_ = (double)info.rx_rate_max_hz;
			rateSteps_ = info.rx_rate_count;
			rxGainMax_ = info.rx_gain_max;
			txGainMax_ = info.tx_gain_max;
		}
		/* Start from what the device runs with. */
		struct esdr_state st{};

		if (link_->ctrlIn(ESDR_REQ_GET_STATE, &st, sizeof(st)) == (int)sizeof(st)) {
			freqHz_ = (double)st.freq_hz;
			rateHz_ = (double)st.sample_rate_hz;
			bwHz_ = (double)st.bandwidth_hz;
			rxAuto_ = st.rx_gain == ESDR_GAIN_AUTO;
			rxGain_ = rxAuto_ ? 0 : st.rx_gain;
			txGain_ = st.tx_gain;
			corrPpm_ = st.freq_corr_ppb / 1000.0;
		}
	}

	~SoapyEsdr() override
	{
		if (rxRunning_) {
			stopRx();
		}
		if (txRunning_) {
			stopTx();
		}
	}

	/* ---- identification ---- */
	std::string getDriverKey(void) const override { return "esdr"; }
	std::string getHardwareKey(void) const override { return "ESP-SDR osmosdr"; }

	SoapySDR::Kwargs getHardwareInfo(void) const override
	{
		SoapySDR::Kwargs info;

		info["serial"] = link_->serial();
		return info;
	}

	/* ---- channels ---- */
	size_t getNumChannels(const int) const override { return 1; }
	bool getFullDuplex(const int, const size_t) const override { return false; }

	/* ---- streaming ---- */
	std::vector<std::string> getStreamFormats(const int, const size_t) const override
	{
		return {SOAPY_SDR_CS8, SOAPY_SDR_CS16, SOAPY_SDR_CF32};
	}

	std::string getNativeStreamFormat(const int, const size_t, double &fullScale) const override
	{
		fullScale = narrow_ ? 127 : 32767;
		return narrow_ ? SOAPY_SDR_CS8 : SOAPY_SDR_CS16;
	}

	SoapySDR::Stream *setupStream(const int direction, const std::string &format,
				      const std::vector<size_t> &channels,
				      const SoapySDR::Kwargs &) override
	{
		if (!channels.empty() && (channels.size() != 1 || channels[0] != 0)) {
			throw std::runtime_error("esdr: one channel only");
		}
		if (format != SOAPY_SDR_CS8 && format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32) {
			throw std::runtime_error("esdr: format must be CS8, CS16 or CF32");
		}
		/* CF32 travels as 16-bit items, or 8-bit ones on the serial link. */
		int floatBits = narrow_ ? 8 : 16;

		if (direction == SOAPY_SDR_TX) {
			txFloat_ = format == SOAPY_SDR_CF32;
			txBits_ = format == SOAPY_SDR_CS8 ? 8 : format == SOAPY_SDR_CS16 ? 16 : floatBits;
			return reinterpret_cast<SoapySDR::Stream *>(&txTag_);
		}
		float_ = format == SOAPY_SDR_CF32;
		bits_ = format == SOAPY_SDR_CS8 ? 8 : format == SOAPY_SDR_CS16 ? 16 : floatBits;
		return reinterpret_cast<SoapySDR::Stream *>(&rxTag_);
	}

	void closeStream(SoapySDR::Stream *stream) override
	{
		deactivateStream(stream, 0, 0);
	}

	size_t getStreamMTU(SoapySDR::Stream *) const override { return 4096; }

	/* Half duplex: starting one direction stops the other. */
	int activateStream(SoapySDR::Stream *stream, const int, const long long, const size_t) override
	{
		if (isTx(stream)) {
			if (rxRunning_) {
				stopRx();
			}
			startTx();
		} else {
			if (txRunning_) {
				stopTx();
			}
			startRx();
		}
		return 0;
	}

	int deactivateStream(SoapySDR::Stream *stream, const int, const long long) override
	{
		if (isTx(stream)) {
			if (txRunning_) {
				stopTx();
			}
		} else if (rxRunning_) {
			stopRx();
		}
		return 0;
	}

	/*
	 * Blocking write: the device holds the data back while its backlog is
	 * full, which paces the caller to the sample rate.
	 */
	int writeStream(SoapySDR::Stream *, const void *const *buffs, const size_t numElems,
			int &flags, const long long, const long timeoutUs) override
	{
		size_t item = txBits_ == 16 ? 4 : 2;
		size_t n = std::min(numElems, txBuf_.size() / item);
		uint8_t *o = txBuf_.data();
		int done = 0;

		flags = 0;
		if (!txRunning_) {
			return SOAPY_SDR_STREAM_ERROR;
		}
		if (txFloat_) {
			const float *in = reinterpret_cast<const float *>(buffs[0]);

			for (size_t k = 0; k < 2 * n; k++) {
				float v = std::max(-1.0f, std::min(1.0f, in[k]));

				if (txBits_ == 16) {
					int16_t q = (int16_t)std::lrint(v * 32767.0f);

					o[2 * k] = (uint8_t)q;
					o[2 * k + 1] = (uint8_t)(q >> 8);
				} else {
					o[k] = (uint8_t)(int8_t)std::lrint(v * 127.0f);
				}
			}
		} else {
			std::memcpy(o, buffs[0], n * item);
		}
		done = link_->writeSamples(o, n * item, timeoutUs);
		if (done < 0) {
			return SOAPY_SDR_STREAM_ERROR;
		}
		if (done == 0) {
			return SOAPY_SDR_TIMEOUT;
		}
		return done / (int)item;
	}

	int readStream(SoapySDR::Stream *, void *const *buffs, const size_t numElems,
		       int &flags, long long &timeNs, const long timeoutUs) override
	{
		bool overflow = false;
		size_t n;

		if (float_) {
			size_t want = std::min(numElems, conv_.size() / 2);
			float *out = reinterpret_cast<float *>(buffs[0]);

			n = ring_.pop(reinterpret_cast<uint8_t *>(conv_.data()), want, (int)timeoutUs,
				      overflow);
			if (bits_ == 8) {
				const int8_t *c = reinterpret_cast<const int8_t *>(conv_.data());

				for (size_t k = 0; k < 2 * n; k++) {
					out[k] = (float)c[k] * (1.0f / 128.0f);
				}
			} else {
				for (size_t k = 0; k < 2 * n; k++) {
					out[k] = (float)conv_[k] * (1.0f / 32768.0f);
				}
			}
		} else {
			n = ring_.pop(reinterpret_cast<uint8_t *>(buffs[0]), numElems, (int)timeoutUs,
				      overflow);
		}

		flags = 0;
		timeNs = 0;
		if (n == 0) {
			return SOAPY_SDR_TIMEOUT;
		}
		if (overflow) {
			flags |= SOAPY_SDR_END_BURST;
		}
		return (int)n;
	}

	/* ---- gains, frequency, rate, bandwidth: forwarded to the device ---- */
	std::vector<std::string> listGains(const int direction, const size_t) const override
	{
		if (direction == SOAPY_SDR_RX) {
			return {"RF"};
		}
		return {"TX"};
	}

	bool hasGainMode(const int direction, const size_t) const override
	{
		return direction == SOAPY_SDR_RX;
	}

	void setGainMode(const int direction, const size_t, const bool automatic) override
	{
		if (direction != SOAPY_SDR_RX) {
			return;
		}
		rxAuto_ = automatic;
		link_->ctrlOut(ESDR_REQ_SET_RX_GAIN, automatic ? ESDR_GAIN_AUTO : (uint16_t)rxGain_,
			nullptr, 0);
	}

	bool getGainMode(const int direction, const size_t) const override
	{
		return direction == SOAPY_SDR_RX && rxAuto_;
	}

	void setGain(const int direction, const size_t channel, const double value) override
	{
		setGain(direction, channel, direction == SOAPY_SDR_RX ? "RF" : "TX", value);
	}

	void setGain(const int direction, const size_t, const std::string &, const double value) override
	{
		if (direction == SOAPY_SDR_RX) {
			rxAuto_ = false;
			rxGain_ = (int)std::max(0.0, std::min((double)rxGainMax_, value));
			link_->ctrlOut(ESDR_REQ_SET_RX_GAIN, (uint16_t)rxGain_, nullptr, 0);
		} else {
			txGain_ = (int)std::max(0.0, std::min((double)txGainMax_, value));
			link_->ctrlOut(ESDR_REQ_SET_TX_GAIN, (uint16_t)txGain_, nullptr, 0);
		}
	}

	double getGain(const int direction, const size_t) const override
	{
		return direction == SOAPY_SDR_RX ? (double)rxGain_ : (double)txGain_;
	}

	double getGain(const int direction, const size_t channel, const std::string &) const override
	{
		return getGain(direction, channel);
	}

	/* PHY gain table indices and TX power steps, not dB. */
	SoapySDR::Range getGainRange(const int direction, const size_t) const override
	{
		return SoapySDR::Range(0, direction == SOAPY_SDR_RX ? rxGainMax_ : txGainMax_, 1);
	}

	SoapySDR::Range getGainRange(const int direction, const size_t channel,
				     const std::string &) const override
	{
		return getGainRange(direction, channel);
	}

	/* One LO: a direction's frequency goes to the device while that direction runs. */
	void setFrequency(const int direction, const size_t, const double frequency,
			  const SoapySDR::Kwargs &) override
	{
		if (direction == SOAPY_SDR_TX) {
			txFreqHz_ = frequency;
			if (txRunning_) {
				setFreq((uint64_t)frequency);
			}
			return;
		}
		freqHz_ = frequency;
		if (!txRunning_) {
			setFreq((uint64_t)frequency);
		}
	}

	double getFrequency(const int direction, const size_t) const override
	{
		return direction == SOAPY_SDR_TX ? txFreqHz_ : freqHz_;
	}

	SoapySDR::RangeList getFrequencyRange(const int, const size_t) const override
	{
		return {SoapySDR::Range(freqMinHz_, freqMaxHz_)};
	}

	/*
	 * The device takes the nearest of its rates: report that one, or the
	 * application reads at a different rate than the samples arrive.
	 */
	void setSampleRate(const int direction, const size_t, const double rate) override
	{
		double actual = deviceRate(rate);

		if (actual != rate) {
			SoapySDR_logf(SOAPY_SDR_INFO, "esdr: %.0f S/s requested, %.0f S/s used", rate,
				      actual);
		}
		if (direction == SOAPY_SDR_TX) {
			txRateHz_ = actual;
			if (txRunning_) {
				setU32(ESDR_REQ_SET_SAMPLE_RATE, (uint32_t)actual);
			}
			return;
		}
		rateHz_ = actual;
		if (!txRunning_) {
			setU32(ESDR_REQ_SET_SAMPLE_RATE, (uint32_t)actual);
		}
	}

	/* The device's rate nearest to @p rate on a log scale (its own rounding). */
	double deviceRate(double rate) const
	{
		for (unsigned k = 0; k + 1 < rateSteps_; k++) {
			double r = rateMaxHz_ / (double)(1u << k);

			if (2.0 * rate * rate >= r * r) {
				return r;
			}
		}
		return rateMaxHz_ / (double)(1u << (rateSteps_ - 1));
	}

	double getSampleRate(const int direction, const size_t) const override
	{
		return direction == SOAPY_SDR_TX ? txRateHz_ : rateHz_;
	}

	std::vector<double> listSampleRates(const int, const size_t) const override
	{
		std::vector<double> rates;

		for (unsigned k = 0; k < rateSteps_; k++) {
			rates.push_back(rateMaxHz_ / (double)(1u << k));
		}
		return rates;
	}

	SoapySDR::RangeList getSampleRateRange(const int, const size_t) const override
	{
		return {SoapySDR::Range(rateMaxHz_ / (double)(1u << (rateSteps_ - 1)), rateMaxHz_)};
	}

	void setBandwidth(const int, const size_t, const double bw) override
	{
		bwHz_ = bw;
		setU32(ESDR_REQ_SET_BANDWIDTH, (uint32_t)bw);
	}

	double getBandwidth(const int, const size_t) const override { return bwHz_; }

	bool hasFrequencyCorrection(const int direction, const size_t) const override
	{
		return direction == SOAPY_SDR_RX;
	}

	void setFrequencyCorrection(const int direction, const size_t, const double value) override
	{
		if (direction != SOAPY_SDR_RX) {
			return;
		}
		corrPpm_ = value;
		setU32(ESDR_REQ_SET_FREQ_CORR, (uint32_t)(int32_t)(value * 1000.0));
	}

	double getFrequencyCorrection(const int, const size_t) const override { return corrPpm_; }

private:
	int setU32(uint8_t req, uint32_t v)
	{
		uint8_t b[4] = {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};

		return link_->ctrlOut(req, 0, b, 4);
	}

	int setFreq(uint64_t hz)
	{
		uint8_t b[8];

		for (int i = 0; i < 8; i++) {
			b[i] = (uint8_t)(hz >> (8 * i));
		}
		return link_->ctrlOut(ESDR_REQ_SET_FREQ, 0, b, 8);
	}

	bool isTx(SoapySDR::Stream *stream) const
	{
		return reinterpret_cast<const char *>(stream) == &txTag_;
	}

	void startTx()
	{
		link_->ctrlOut(ESDR_REQ_SET_MODE, ESDR_MODE_OFF, nullptr, 0);
		setFreq((uint64_t)txFreqHz_);
		setU32(ESDR_REQ_SET_SAMPLE_RATE, (uint32_t)txRateHz_);
		link_->ctrlOut(ESDR_REQ_SET_TX_GAIN, (uint16_t)txGain_, nullptr, 0);
		link_->ctrlOut(ESDR_REQ_SET_FORMAT, (uint16_t)txBits_, nullptr, 0);
		link_->ctrlOut(ESDR_REQ_SET_MODE, ESDR_MODE_TX, nullptr, 0);
		txRunning_ = true;
	}

	void stopTx()
	{
		link_->ctrlOut(ESDR_REQ_SET_MODE, ESDR_MODE_OFF, nullptr, 0);
		txRunning_ = false;
	}

	void startRx()
	{
		size_t itemBytes = bits_ == 16 ? 4 : 2;

		ring_.reset(itemBytes);
		expect_ = 0;
		haveExpect_ = false;
		synced_ = false;
		carryLen_ = 0;
		rxSamples_ = gaps_ = gapZeros_ = resyncs_ = 0;
		link_->ctrlOut(ESDR_REQ_SET_MODE, ESDR_MODE_OFF, nullptr, 0);
		setFreq((uint64_t)freqHz_);
		setU32(ESDR_REQ_SET_SAMPLE_RATE, (uint32_t)rateHz_);
		if (bwHz_ > 0) {
			setU32(ESDR_REQ_SET_BANDWIDTH, (uint32_t)bwHz_);
		}
		link_->ctrlOut(ESDR_REQ_SET_RX_GAIN, rxAuto_ ? ESDR_GAIN_AUTO : (uint16_t)rxGain_,
			nullptr, 0);
		link_->ctrlOut(ESDR_REQ_SET_FORMAT, (uint16_t)bits_, nullptr, 0);
		link_->ctrlOut(ESDR_REQ_SET_MODE, ESDR_MODE_RX, nullptr, 0);

		rxRunning_ = true;
		link_->startRx([this](const uint8_t *p, size_t n) { onBytes(p, n); });
	}

	void stopRx()
	{
		/* The device first: with no transfers queued it would only fill and overflow. */
		link_->ctrlOut(ESDR_REQ_SET_MODE, ESDR_MODE_OFF, nullptr, 0);
		link_->stopRx();
		rxRunning_ = false;
		SoapySDR_logf(SOAPY_SDR_INFO,
			      "esdr: rx stopped: %llu samples, %llu gaps (%llu zero filled), "
			      "%llu dropped by a slow reader, %llu resync steps",
			      (unsigned long long)rxSamples_, (unsigned long long)gaps_,
			      (unsigned long long)gapZeros_, (unsigned long long)ring_.dropped(),
			      (unsigned long long)resyncs_);
	}

	/*
	 * The receive stream is split into whole EBLK blocks (the device never
	 * splits one across transfers) and converted into the ring. A gap in the
	 * device's sample index is filled with zeros, bounded, so GRC/gqrx see a
	 * continuous time axis instead of a jump.
	 */
	/* A plausible block header (the magic alone could occur in sample data). */
	static bool headerOk(const uint8_t *p)
	{
		struct esdr_block_hdr h{};

		std::memcpy(&h, p, sizeof(h));
		return h.magic == ESDR_BLOCK_MAGIC && h.samples <= ESDR_BLOCK_PAYLOAD / 2U &&
		       (h.flags & ~0x000fU) == 0U;
	}

	void onBlock(const uint8_t *p)
	{
		struct esdr_block_hdr h{};

		std::memcpy(&h, p, sizeof(h));
		if (haveExpect_ && h.index > expect_ && h.index - expect_ < 1000000U) {
			/* Missing samples become zeros: the time axis stays continuous. */
			static const uint8_t zeros[4 * 1024] = {0};
			size_t item = bits_ == 16 ? 4 : 2;

			for (uint64_t z = h.index - expect_; z > 0;) {
				uint64_t k = std::min<uint64_t>(z, sizeof(zeros) / item);

				ring_.push(zeros, (size_t)k);
				z -= k;
			}
			gaps_++;
			gapZeros_ += h.index - expect_;
		}
		ring_.push(p + sizeof(h), h.samples);
		rxSamples_ += h.samples;
		expect_ = h.index + h.samples;
		haveExpect_ = true;
	}

	/*
	 * Bulk data into whole blocks. Alignment is lost when an earlier session
	 * stopped mid-block; it is found again on the device's 64-byte packets.
	 */
	void onBytes(const uint8_t *p, size_t n)
	{
		while (n > 0) {
			if (!synced_) {
				if (n < 64) {
					return;
				}
				if (headerOk(p)) {
					synced_ = true;
					carryLen_ = 0;
					continue;
				}
				p += 64;
				n -= 64;
				resyncs_++;
				continue;
			}
			if (carryLen_ > 0 || n < ESDR_BLOCK_BYTES) {
				size_t k = std::min(ESDR_BLOCK_BYTES - carryLen_, n);

				std::memcpy(carry_ + carryLen_, p, k);
				carryLen_ += k;
				p += k;
				n -= k;
				if (carryLen_ < ESDR_BLOCK_BYTES) {
					return;
				}
				carryLen_ = 0;
				if (!headerOk(carry_)) {
					synced_ = false;
					continue;
				}
				onBlock(carry_);
				continue;
			}
			if (!headerOk(p)) {
				synced_ = false;
				continue;
			}
			onBlock(p);
			p += ESDR_BLOCK_BYTES;
			n -= ESDR_BLOCK_BYTES;
		}
	}

	std::unique_ptr<Link> link_;

	double freqMinHz_ = 100e6, freqMaxHz_ = 6000e6, rateMaxHz_ = 250e3;
	unsigned rateSteps_ = 5;
	unsigned rxGainMax_ = 82, txGainMax_ = 17;

	double freqHz_ = 100e6, rateHz_ = 250e3, bwHz_ = 0, corrPpm_ = 0;
	int rxGain_ = 0, txGain_ = 0;
	bool rxAuto_ = true;
	/* Serial link: CF32 and the native format travel as 8-bit items. */
	bool narrow_ = false;
	int bits_ = 16;
	bool float_ = false;
	/* Stream handles: addresses of these tags. */
	char rxTag_ = 0, txTag_ = 0;
	/* Transmit: own frequency and rate (the device has one of each, set at activation). */
	double txFreqHz_ = 2450e6, txRateHz_ = 250e3;
	int txBits_ = 16;
	bool txFloat_ = false, txRunning_ = false;
	std::vector<uint8_t> txBuf_ = std::vector<uint8_t>(16 * ESDR_BLOCK_BYTES);
	/* CF32 staging: CS16 items popped from the ring. */
	std::vector<int16_t> conv_ = std::vector<int16_t>(2 * 65536);

	bool rxRunning_ = false;
	/* Receive parser state and counters (the link's receive thread). */
	uint64_t rxSamples_ = 0, gaps_ = 0, gapZeros_ = 0, resyncs_ = 0;
	uint64_t expect_ = 0;
	bool haveExpect_ = false, synced_ = false;
	uint8_t carry_[ESDR_BLOCK_BYTES];
	size_t carryLen_ = 0;
	SampleRing ring_;
};

static SoapySDR::KwargsList findEsdr(const SoapySDR::Kwargs &args)
{
	SoapySDR::KwargsList result;
	libusb_context *ctx;

	if (libusb_init(&ctx) != 0) {
		return result;
	}
	libusb_device **list;
	ssize_t n = libusb_get_device_list(ctx, &list);

	for (ssize_t i = 0; i < n; i++) {
		libusb_device_descriptor dd;
		libusb_device_handle *h;
		char sn[64] = "";

		if (libusb_get_device_descriptor(list[i], &dd) != 0 || dd.idVendor != ESDR_VID ||
		    dd.idProduct != ESDR_PID) {
			continue;
		}
		if (libusb_open(list[i], &h) == 0) {
			if (dd.iSerialNumber != 0) {
				libusb_get_string_descriptor_ascii(h, dd.iSerialNumber,
								   (unsigned char *)sn, sizeof(sn));
			}
			libusb_close(h);
		}
		if (args.count("serial") && args.at("serial") != sn) {
			continue;
		}
		SoapySDR::Kwargs dev;

		dev["driver"] = "esdr";
		dev["serial"] = sn;
		dev["label"] = std::string("ESP-SDR osmosdr (") + sn + ")";
		result.push_back(dev);
	}
	libusb_free_device_list(list, 1);
	libusb_exit(ctx);
	/* The serial link (osmosdr_serial) is not probed: name its port. */
	if (args.count("port")) {
		SoapySDR::Kwargs dev;

		dev["driver"] = "esdr";
		dev["port"] = args.at("port");
		dev["label"] = "ESP-SDR osmosdr_serial (" + args.at("port") + ")";
		result.push_back(dev);
	}
	return result;
}

static SoapySDR::Device *makeEsdr(const SoapySDR::Kwargs &args)
{
	return new SoapyEsdr(args);
}

static SoapySDR::Registry registerEsdr("esdr", &findEsdr, &makeEsdr, SOAPY_SDR_ABI_VERSION);
