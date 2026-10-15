# SPDX-License-Identifier: GPL-3.0-or-later
"""west dfu: update a running esp-sdr board over USB DFU and confirm the image.

The apps in this repository expose DFU (runtime 2fe3:0005, DFU mode
2fe3:ffff) and an MCUboot shell on their CDC-ACM port. This command detaches
the board into DFU mode, downloads the signed image into the secondary slot,
waits until the board has left USB and come back running the new image (on
trial), checks that its shell still answers after a settle time, and only
then confirms it. An image that hangs or crashes is never confirmed, so the
trial watchdog resets it and MCUboot reverts to the previous one.
"""

import glob
import os
import re
import subprocess
import time
from pathlib import Path

from west.commands import WestCommand

VID = "2fe3"
PID_APP = "0005"
PID_DFU = "ffff"
USB = Path("/sys/bus/usb/devices")
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


def usb_boards():
    """{serial: (port path, product id)} of the attached esp-sdr boards.

    A board left in DFU mode has no serial number: it is keyed "dfu@<port>".
    """
    boards = {}
    for d in USB.glob("*-*"):
        if ":" in d.name or not (d / "idVendor").exists():
            continue
        try:
            if (d / "idVendor").read_text().strip() != VID:
                continue
            pid = (d / "idProduct").read_text().strip()
            serial = (d / "serial").read_text().strip() if (d / "serial").exists() else ""
        except OSError:
            continue
        if pid in (PID_APP, PID_DFU):
            boards[serial or f"dfu@{d.name}"] = (d.name, pid)
    return boards


def port_pid(port):
    try:
        return (USB / port / "idProduct").read_text().strip()
    except OSError:
        return None


def shell_tty(serial):
    links = glob.glob(f"/dev/serial/by-id/usb-Zephyr_Project_ESP-SDR*_{serial}-if00")
    return links[0] if links else None


class Shell:
    def __init__(self, path):
        import serial

        self.s = serial.Serial(path, 115200, timeout=0.2)
        time.sleep(0.3)
        self.s.read(65536)

    def cmd(self, line, wait=1.0):
        self.s.write((line + "\r\n").encode())
        end, out = time.time() + wait, b""
        while time.time() < end:
            out += self.s.read(4096)
        return ANSI.sub("", out.decode(errors="replace"))

    def close(self):
        self.s.close()


class Dfu(WestCommand):
    def __init__(self):
        super().__init__("dfu", "update a running esp-sdr board over USB DFU",
                         __doc__, accepts_unknown_args=False)

    def do_add_parser(self, parser_adder):
        p = parser_adder.add_parser(self.name, help=self.help, description=self.description)
        p.add_argument("-d", "--build-dir", default="build",
                       help="build directory (sysbuild or plain), default: build")
        p.add_argument("-i", "--image", help="signed image, instead of the build directory's")
        p.add_argument("-s", "--serial", help="USB serial of the board; needed with several")
        p.add_argument("--tries", type=int, default=3, help="detach/download attempts")
        p.add_argument("--settle", type=float, default=5.0,
                       help="seconds the new image must keep its shell alive before confirm")
        p.add_argument("--no-confirm", action="store_true",
                       help="leave the image on trial (reverted at the next reset)")
        p.add_argument("--storage", nargs="?", const="", metavar="IMAGE",
                       help="download the multiapp FAT volume (the build's "
                            "multiapp_fat.img, or IMAGE) instead of the firmware")
        p.add_argument("--esptool", action="store_true",
                       help="write MCUboot and the image (and with --storage the FAT "
                            "volume) with esptool over the USB-Serial-JTAG, which the "
                            "running app hands over (its usj command): no button, no DFU")
        p.add_argument("--dfu-board", action="store_true",
                       help="if --serial is not on USB, update the one board found in DFU "
                            "mode (one an earlier try left there; it has no serial to check)")
        return p

    def find_image(self, args):
        if args.image:
            if not Path(args.image).is_file():
                self.die(f"{args.image}: no such image")
            return Path(args.image)
        b = Path(args.build_dir)
        if (b / "zephyr" / "zephyr.signed.bin").exists():
            return b / "zephyr" / "zephyr.signed.bin"
        dom = b / "domains.yaml"
        if dom.exists():
            m = re.search(r"^default:\s*(\S+)", dom.read_text(), re.M)
            if m and (b / m.group(1) / "zephyr" / "zephyr.signed.bin").exists():
                return b / m.group(1) / "zephyr" / "zephyr.signed.bin"
        self.die(f"no zephyr.signed.bin in {b} (sysbuild with MCUboot?)")

    def find_storage(self, args):
        if args.storage:
            if not Path(args.storage).is_file():
                self.die(f"{args.storage}: no such image")
            return Path(args.storage)
        b = Path(args.build_dir)
        for cand in (b / "multiapp_fat.img", b / "multiapp" / "multiapp_fat.img"):
            if cand.exists():
                return cand
        self.die(f"no multiapp_fat.img in {b} (a multiapp build?)")

    def slot1_last_sector(self, args):
        """Offset of the update slot's last sector, from the build's devicetree."""
        dts = None
        b = Path(args.build_dir)
        for cand in b.glob("*/zephyr/zephyr.dts"):
            if "mcuboot" not in str(cand):
                dts = cand
        if dts is None:
            dts = b / "zephyr" / "zephyr.dts"
        m = re.search(r"slot1_partition: partition@[0-9a-f]+ \{.*?reg = < (0x[0-9a-f]+) (0x[0-9a-f]+) >",
                      dts.read_text(), re.S)
        if not m:
            self.die(f"{dts}: no slot1_partition")
        return int(m.group(1), 16) + int(m.group(2), 16) - 4096

    def esptool_flash(self, args, sn):
        """Everything in one esptool run: its reset at the end starts the app, which takes the
        USB port back."""
        b = Path(args.build_dir)
        mcuboot = b / "mcuboot" / "zephyr" / "zephyr.bin"
        if not mcuboot.exists():
            self.die(f"no {mcuboot}: --esptool needs a sysbuild with MCUboot")
        parts = [("0x0", mcuboot), ("0x20000", self.find_image(args))]
        if args.storage is not None:
            parts.append(("0x400000", self.find_storage(args)))
        # A stale update request in the update slot would make MCUboot swap the new image out.
        blank = b / "slot1_trailer_blank.bin"
        blank.write_bytes(b"\xff" * 4096)
        parts.append((hex(self.slot1_last_sector(args)), blank))

        mac = ":".join(sn[i:i + 2] for i in range(0, len(sn), 2))
        usj = f"/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_{mac}-if00"
        if not os.path.exists(usj):
            try:
                sh = Shell(shell_tty(sn))
                sh.cmd("usj", 0.5)
                sh.close()
            except Exception:  # the port goes away while usj answers, as it should
                pass
            if not self.wait_for(lambda: os.path.exists(usj), 15):
                self.die(f"{sn}: no USB-Serial-JTAG ({usj}) after usj")
        argv = ["esptool", "--port", usj, "--before", "default-reset", "--after", "hard-reset",
                "write-flash", "-u", "--flash-mode", "dio", "--flash-freq", "80m",
                "--flash-size", "8MB"]
        for off, path in parts:
            argv += [off, str(path)]
            self.inf(f"{sn}: {off} {path}")
        r = subprocess.run(argv, capture_output=True, text=True)
        if r.returncode != 0:
            self.die(f"{sn}: esptool failed:\n{(r.stdout + r.stderr).strip()[-600:]}")
        if not self.wait_for(lambda: shell_tty(sn) is not None, 60):
            self.die(f"{sn}: did not come back after esptool")
        time.sleep(1.0)
        sh = Shell(shell_tty(sn))
        out = sh.cmd("app list") if args.storage is not None else sh.cmd("kernel version")
        sh.close()
        self.inf(f"{sn}: written with esptool\n{out.strip()}")

    def find_board(self, args):
        boards = usb_boards()
        in_dfu = [k for k in boards if k.startswith("dfu@")]
        if args.serial:
            if args.serial in boards:
                return args.serial, boards[args.serial][0]
            if len(in_dfu) == 1 and args.dfu_board:
                # A board stuck in DFU mode (an earlier try failed) has no serial to match.
                self.wrn(f"{args.serial}: not on USB, using the board in DFU mode")
                return args.serial, boards[in_dfu[0]][0]
            hint = " (one board is in DFU mode: --dfu-board updates it)" if len(in_dfu) == 1 else ""
            self.die(f"{args.serial}: not on USB (found: {', '.join(boards) or 'none'}){hint}")
        if len(boards) != 1:
            self.die(f"{len(boards)} esp-sdr boards on USB ({', '.join(boards) or 'none'}): "
                     "pick one with --serial")
        sn = next(iter(boards))
        if sn.startswith("dfu@"):
            self.die(f"{sn}: the board is in DFU mode and has no serial number: "
                     "pass --serial to find its shell after the update")
        return sn, boards[sn][0]

    def wait_for(self, cond, seconds, step=0.2):
        end = time.time() + seconds
        while time.time() < end:
            if cond():
                return True
            time.sleep(step)
        return False

    def dfu_util(self, argv, timeout):
        try:
            return subprocess.run(["dfu-util"] + argv, capture_output=True, text=True,
                                  timeout=timeout)
        except FileNotFoundError:
            self.die("dfu-util not found")
        except subprocess.TimeoutExpired:
            self.wrn(f"dfu-util {' '.join(argv)}: no result after {timeout} s")
            return None

    def download(self, sn, port, image, alt="slot1_image"):
        if port_pid(port) != PID_DFU:
            self.dfu_util(["-S", sn, "-d", f"{VID}:{port_pid(port) or PID_APP}", "-e"], 15)
        if not self.wait_for(lambda: port_pid(port) == PID_DFU, 15):
            self.wrn(f"{sn}: no DFU mode")
            return False
        r = self.dfu_util(["-d", f"{VID}:{PID_DFU}", "-p", port, "-a", alt, "-D",
                           str(image)], 120)
        if r is None:
            return False
        if "Download done" not in r.stdout:
            self.wrn(f"{sn}: download failed: {(r.stdout + r.stderr).strip()[-300:]}")
            return False
        return True

    def do_run(self, args, unknown):
        if args.esptool:
            sn, _ = self.find_board(args)
            self.esptool_flash(args, sn)
            return
        storage = args.storage is not None
        image = self.find_storage(args) if storage else self.find_image(args)
        sn, port = self.find_board(args)
        self.inf(f"{sn} ({port}): {image} ({image.stat().st_size} bytes)")

        for attempt in range(1, args.tries + 1):
            if not self.download(sn, port, image, "fat" if storage else "slot1_image"):
                continue
            self.inf(f"{sn}: downloaded (try {attempt}), waiting for the swap")
            # Leave USB first: the old image's shell link can outlive the download.
            if not self.wait_for(lambda: port_pid(port) != PID_DFU, 30):
                self.wrn(f"{sn}: did not reset after the download")
                continue
            if not self.wait_for(lambda: port_pid(port) == PID_APP and shell_tty(sn), 90):
                self.wrn(f"{sn}: did not come back")
                continue
            break
        else:
            self.die(f"{sn}: update failed")

        time.sleep(1.0)
        if storage:
            try:
                sh = Shell(shell_tty(sn))
                out = sh.cmd("app list")
                sh.close()
            except Exception as e:  # serial.SerialException, or the link went away
                self.die(f"{sn}: no shell after the volume download ({e})")
            self.inf(f"{sn}: volume written\n{out.strip()}")
            return
        try:
            sh = Shell(shell_tty(sn))
            state = sh.cmd("mcuboot")
            first = re.search(r"Uptime: *(\d+)", sh.cmd("kernel uptime"))
        except Exception as e:  # serial.SerialException, or the link went away
            self.die(f"{sn}: the new image dropped off USB right after booting ({e}); "
                     "not confirmed, MCUboot reverts it after the trial watchdog")
        if re.search(r"^confirmed: *1", state, re.M):
            sh.close()
            self.die(f"{sn}: running a confirmed image: the new one did not boot "
                     "(MCUboot kept or restored the old one)")
        if args.no_confirm:
            sh.close()
            self.inf(f"{sn}: new image on trial, not confirmed")
            return
        self.inf(f"{sn}: new image on trial, checking it for {args.settle:.0f} s")
        sh.close()
        time.sleep(args.settle)
        tty = shell_tty(sn)
        alive = ""
        if tty and port_pid(port) == PID_APP:
            try:
                sh = Shell(tty)
                alive = sh.cmd("kernel uptime")
            except Exception as e:  # the device went away while opening
                alive = str(e)
        up = re.search(r"Uptime: *(\d+)", alive)
        if not up:
            self.die(f"{sn}: the new image stopped answering ({alive.strip()[-120:]!r}); "
                     "not confirmed, MCUboot reverts it after the trial watchdog")
        # The uptime must have grown by the settle time since the first look.
        since = int(first.group(1)) + args.settle * 1000 if first else args.settle * 1000
        if int(up.group(1)) < since:
            self.die(f"{sn}: the board restarted during the check (uptime {up.group(1)} ms): "
                     "not confirmed")
        try:
            # A revert inside the settle time leaves an old, already confirmed image answering.
            if re.search(r"^confirmed: *1", sh.cmd("mcuboot"), re.M):
                sh.close()
                self.die(f"{sn}: the board is running a confirmed image again: the new one "
                         "was reverted, not confirmed")
            out = sh.cmd("mcuboot confirm", 0.5) + sh.cmd("mcuboot")
            sh.close()
        except Exception as e:  # serial.SerialException, or the link went away
            self.die(f"{sn}: the shell went away while confirming ({e}): not confirmed")
        if not re.search(r"^confirmed: *1", out, re.M):
            self.die(f"{sn}: confirm failed:\n{out}")
        self.inf(f"{sn}: confirmed")
