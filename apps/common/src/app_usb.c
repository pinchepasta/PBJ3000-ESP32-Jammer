/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * USB device on the OTG port with DFU into the MCUboot secondary slot.
 * OTG takes the PHY from USB-Serial-JTAG, so it starts only after a delay
 * that leaves the RTS/DTR flashing path usable right after reset.
 */

#include <sample_usbd.h>
#include <string.h>

#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/usb/usbd.h>

#include "app_crash.h"
#include "app_usb.h"

#include <esp_attr.h>
#if defined(CONFIG_SHELL)
#include <zephyr/shell/shell.h>
#include <soc/rtc_cntl_reg.h>
#endif

LOG_MODULE_REGISTER(app_usb, LOG_LEVEL_INF);

#define REBOOT_DELAY K_MSEC(1000)

static struct k_work_delayable start_work;
/* The device once started, for "usj". */
static struct usbd_context *usb_ctx;
static struct k_work_delayable reboot_work;

#if defined(CONFIG_APP_TRIAL_WATCHDOG) || defined(CONFIG_APP_LIVENESS_WATCHDOG)
static const struct device *const app_wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
static struct k_work_delayable wdt_work;
static bool trial;

#define FEED_PERIOD K_MSEC(500)
#if defined(CONFIG_APP_TRIAL_TIMEOUT_S)
#define TRIAL_S CONFIG_APP_TRIAL_TIMEOUT_S
#else
#define TRIAL_S 0
#endif

/*
 * Liveness: fed from the system work queue, so a wedged system resets. Trial:
 * an unconfirmed image is no longer fed after CONFIG_APP_TRIAL_TIMEOUT_S, or
 * (without liveness) the watchdog is only disarmed once confirmed.
 */
static void wdt_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (trial && boot_is_img_confirmed()) {
		trial = false;
		LOG_INF("image confirmed");
		if (!IS_ENABLED(CONFIG_APP_LIVENESS_WATCHDOG)) {
			(void)wdt_disable(app_wdt);
			return;
		}
	}
	if (IS_ENABLED(CONFIG_APP_LIVENESS_WATCHDOG)) {
		/* The driver's tick scale halves the trial timeout; keep that. */
		if (!trial || k_uptime_get() < (int64_t)TRIAL_S * 1000 / 2) {
			(void)wdt_feed(app_wdt, 0);
		}
		k_work_schedule(&wdt_work, FEED_PERIOD);
	} else {
		k_work_schedule(&wdt_work, K_SECONDS(1));
	}
}

/*
 * Stage 0 of the ESP32 watchdog: the driver feeds it after this callback, so a
 * live system only resets if the callback does it. A hung one, which cannot
 * run the interrupt, gets the stage 1 hardware reset.
 */
static void wdt_expired(const struct device *dev, int channel_id)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);
	if (!trial) {
		app_crash_note_watchdog();
	}
	sys_reboot(SYS_REBOOT_COLD);
}

/*
 * A DFU image boots unconfirmed. Unless the host confirms it in time, the
 * chip resets and MCUboot reverts to the previous image, so an update that
 * breaks USB never needs a replug. With the liveness watchdog a hang resets
 * any image.
 */
static void app_watchdog_arm(void)
{
	struct wdt_timeout_cfg cfg = {
		.callback = wdt_expired,
		.flags = WDT_FLAG_RESET_SOC,
	};

	trial = IS_ENABLED(CONFIG_APP_TRIAL_WATCHDOG) && IS_ENABLED(CONFIG_BOOTLOADER_MCUBOOT) &&
		!boot_is_img_confirmed();
	if (!trial && !IS_ENABLED(CONFIG_APP_LIVENESS_WATCHDOG)) {
		return;
	}
#if defined(CONFIG_APP_LIVENESS_WATCHDOG)
	cfg.window.max = CONFIG_APP_LIVENESS_TIMEOUT_MS;
#else
	cfg.window.max = TRIAL_S * MSEC_PER_SEC;
#endif
	if (!device_is_ready(app_wdt) || wdt_install_timeout(app_wdt, &cfg) < 0 ||
	    wdt_setup(app_wdt, 0) < 0) {
		LOG_ERR("watchdog unavailable");
		return;
	}
	if (trial) {
		LOG_WRN("trial boot: reverting in %d s unless confirmed (mcuboot confirm)",
			TRIAL_S);
	}
	k_work_init_delayable(&wdt_work, wdt_handler);
	k_work_schedule(&wdt_work, K_NO_WAIT);
}
#endif

#if defined(CONFIG_APP_USB_DFU)
/* DFU mode is a separate device with only the DFU class, as in samples/subsys/usb/dfu. */
USBD_DEVICE_DEFINE(dfu_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), CONFIG_SAMPLE_USBD_VID,
		   CONFIG_APP_USB_DFU_PID);
USBD_DESC_LANG_DEFINE(dfu_lang);
USBD_DESC_PRODUCT_DEFINE(dfu_product, CONFIG_SAMPLE_USBD_PRODUCT " DFU");
USBD_DESC_CONFIG_DEFINE(dfu_cfg_desc, "DFU FS Configuration");
USBD_CONFIGURATION_DEFINE(dfu_fs_config, 0, CONFIG_SAMPLE_USBD_MAX_POWER, &dfu_cfg_desc);

static void msg_cb(struct usbd_context *const ctx, const struct usbd_msg *const msg);

__weak void app_usb_dfu_prepare(void)
{
}

static void switch_to_dfu_mode(struct usbd_context *const ctx)
{
	int err;

	LOG_INF("switching to DFU mode");
	usbd_disable(ctx);
	usbd_shutdown(ctx);

	err = usbd_add_descriptor(&dfu_usbd, &dfu_lang);
	if (err == 0) {
		err = usbd_add_descriptor(&dfu_usbd, &dfu_product);
	}
	if (err == 0) {
		err = usbd_add_configuration(&dfu_usbd, USBD_SPEED_FS, &dfu_fs_config);
	}
	if (err == 0) {
		err = usbd_register_class(&dfu_usbd, "dfu_dfu", USBD_SPEED_FS, 1);
	}
	if (err == 0) {
		usbd_device_set_code_triple(&dfu_usbd, USBD_SPEED_FS, 0, 0, 0);
		err = usbd_init(&dfu_usbd);
	}
	if (err == 0) {
		err = usbd_msg_register_cb(&dfu_usbd, msg_cb);
	}
	if (err == 0) {
		err = usbd_enable(&dfu_usbd);
	}
	if (err != 0) {
		LOG_ERR("DFU mode failed (%d)", err);
	}
}
#endif /* CONFIG_APP_USB_DFU */

static void msg_cb(struct usbd_context *const ctx, const struct usbd_msg *const msg)
{
	LOG_DBG("USBD message: %s", usbd_msg_type_string(msg->type));

	if (usbd_can_detect_vbus(ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY && usbd_enable(ctx) != 0) {
			LOG_ERR("enable failed");
		}
		if (msg->type == USBD_MSG_VBUS_REMOVED && usbd_disable(ctx) != 0) {
			LOG_ERR("disable failed");
		}
	}

#if defined(CONFIG_APP_USB_DFU)
	if (msg->type == USBD_MSG_DFU_APP_DETACH) {
		app_usb_dfu_prepare();
		switch_to_dfu_mode(ctx);
	}
	if (msg->type == USBD_MSG_DFU_DOWNLOAD_COMPLETED) {
		int err = 0;

		/* MCUboot test-boots a new image; the host confirms it over the shell. */
		if (app_usb_dfu_was_slot()) {
			err = boot_request_upgrade(BOOT_UPGRADE_TEST);
		}

		LOG_INF("update stored (%d), rebooting", err);
		k_work_schedule(&reboot_work, REBOOT_DELAY);
	}
#endif
}

#if defined(CONFIG_APP_USB_DFU)
/* The USB thread's stack use in the last DFU download, kept over the reboot. */
#define DFU_STACK_MAGIC 0x44465553U
static RTC_NOINIT_ATTR struct {
	uint32_t magic;
	uint32_t used, size;
} dfu_stack;

#if defined(CONFIG_INIT_STACKS) && defined(CONFIG_THREAD_NAME) && defined(CONFIG_THREAD_MONITOR)
static void note_usbd_stack(const struct k_thread *thread, void *user_data)
{
	size_t unused;

	ARG_UNUSED(user_data);
	if (strcmp(k_thread_name_get((k_tid_t)thread), "usbd") == 0 &&
	    k_thread_stack_space_get(thread, &unused) == 0) {
		dfu_stack.used = thread->stack_info.size - unused;
		dfu_stack.size = thread->stack_info.size;
		dfu_stack.magic = DFU_STACK_MAGIC;
	}
}
#endif

static void reboot_handler(struct k_work *work)
{
	ARG_UNUSED(work);
#if defined(CONFIG_INIT_STACKS) && defined(CONFIG_THREAD_NAME) && defined(CONFIG_THREAD_MONITOR)
	k_thread_foreach(note_usbd_stack, NULL);
#endif
	sys_reboot(SYS_REBOOT_COLD);
}

bool app_usb_dfu_stack(size_t *used, size_t *size)
{
	if (dfu_stack.magic != DFU_STACK_MAGIC) {
		return false;
	}
	*used = dfu_stack.used;
	*size = dfu_stack.size;
	return true;
}
#else
bool app_usb_dfu_stack(size_t *used, size_t *size)
{
	ARG_UNUSED(used);
	ARG_UNUSED(size);
	return false;
}
#endif

__weak bool app_usb_dfu_was_slot(void)
{
	return true;
}

__weak void app_usb_classes(struct usbd_context *ctx)
{
	ARG_UNUSED(ctx);
}

static void start_handler(struct k_work *work)
{
	struct usbd_context *ctx;
	int err;

	ARG_UNUSED(work);

	/* sample_usbd_init_device() with app_usb_classes() between its two steps. */
	ctx = sample_usbd_setup_device(msg_cb);
	if (ctx != NULL) {
		app_usb_classes(ctx);
		if (usbd_init(ctx) != 0) {
			ctx = NULL;
		}
	}
	if (ctx == NULL) {
		LOG_ERR("USB device init failed");
		return;
	}
	if (!usbd_can_detect_vbus(ctx)) {
		err = usbd_enable(ctx);
		if (err != 0) {
			LOG_ERR("USB enable failed (%d)", err);
			return;
		}
	}
	usb_ctx = ctx;
	LOG_INF("USB device enabled");
}

int app_usb_init(void)
{
#if defined(CONFIG_APP_TRIAL_WATCHDOG) || defined(CONFIG_APP_LIVENESS_WATCHDOG)
	app_watchdog_arm();
#endif
	k_work_init_delayable(&start_work, start_handler);
#if defined(CONFIG_APP_USB_DFU)
	k_work_init_delayable(&reboot_work, reboot_handler);
#endif
	LOG_INF("USB OTG starts in %d ms", CONFIG_APP_USB_START_DELAY_MS);
	k_work_schedule(&start_work, K_MSEC(CONFIG_APP_USB_START_DELAY_MS));
	return 0;
}

#if defined(CONFIG_SHELL)
/*
 * "usj": the USB PHY to the USB-Serial-JTAG (esptool, OpenOCD) until the OTG
 * driver takes it back at its next init. The device leaves the bus first, so
 * the PC sees it go.
 */
static void usj_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (usb_ctx != NULL) {
		(void)usbd_disable(usb_ctx);
	}
	k_msleep(200);
	REG_SET_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_SW_HW_USB_PHY_SEL);
	REG_CLR_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_SW_USB_PHY_SEL);
}
static K_WORK_DELAYABLE_DEFINE(usj_work, usj_handler);

static int cmd_usj(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "USB goes to the USB-Serial-JTAG until the next reset");
	/* After the reply has left over CDC-ACM. */
	k_work_schedule(&usj_work, K_MSEC(100));
	return 0;
}
SHELL_CMD_REGISTER(usj, NULL, "Hand the USB port to the USB-Serial-JTAG (esptool, OpenOCD)",
		   cmd_usj);
#endif
