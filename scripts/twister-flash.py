#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Flash command for twister's --flash-command: one script for every board on the desk.

Twister calls it as `twister-flash.py --build-dir <dir> [--board-id <id>]` for each
board it flashes, with the build directory and the id of that board's hardware map entry.

- A sysbuild image with MCUboot (the S3 apps) goes on with `west dfu`, the id being the
  board's USB serial number (esptool cannot reset such a board into the ROM bootloader:
  the app owns the USB OTG port); multiapp's FAT volume follows with `west dfu --storage`.
- Anything else is flashed with `west flash`. The id is the serial number of the board's
  USB-Serial-JTAG (as in /dev/serial/by-id) or a /dev path.

After a successful flash, LINK_DIR/<id> points at the board's serial port. The hardware
map names the S3's port by that link: its /dev/serial/by-id name carries the USB product
string, which differs between the images the scenarios put on it, and its by-path name
changes with the USB socket.
"""

import argparse
import glob
import os
import re
import subprocess
import sys
from pathlib import Path

LINK_DIR = Path("/tmp/esp-sdr-tty")


def signed_image(build_dir):
    """The MCUboot-signed image of a sysbuild build directory, or None."""
    domains = build_dir / "domains.yaml"
    if not domains.exists():
        return None
    m = re.search(r"^default:\s*(\S+)", domains.read_text(), re.M)
    image = build_dir / m.group(1) / "zephyr" / "zephyr.signed.bin" if m else None
    return image if image and image.exists() else None


def fat_volume(build_dir):
    """multiapp's FAT volume image in a sysbuild build directory, or None."""
    images = list(build_dir.glob("*/multiapp_fat.img"))
    return images[0] if images else None


def esp_port(board_id):
    if board_id.startswith("/"):
        return board_id
    ports = glob.glob(f"/dev/serial/by-id/*_{board_id}-if00")
    if len(ports) != 1:
        sys.exit(f"{board_id}: {len(ports)} matching ports in /dev/serial/by-id")
    return ports[0]


def link_port(board_id):
    """Points LINK_DIR/<board_id> at the board's port, whatever its product string."""
    port = esp_port(board_id)
    LINK_DIR.mkdir(exist_ok=True)
    tmp = LINK_DIR / f".{board_id}.tmp"
    tmp.unlink(missing_ok=True)
    tmp.symlink_to(port)
    os.replace(tmp, LINK_DIR / board_id)
    print(f"{LINK_DIR / board_id} -> {port}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--build-dir", required=True, type=Path)
    ap.add_argument("--board-id", help="USB serial number or /dev path of the board")
    args = ap.parse_args()

    if signed_image(args.build_dir):
        if not args.board_id:
            sys.exit("an MCUboot image needs --board-id, the USB serial number of the board")
        cmd = ["west", "dfu", "-d", str(args.build_dir), "-s", args.board_id]
    else:
        cmd = ["west", "flash", "-d", str(args.build_dir), "--no-rebuild"]
        if args.board_id:
            cmd += ["--esp-device", esp_port(args.board_id)]

    print("+", " ".join(cmd), flush=True)
    ret = subprocess.run(cmd).returncode
    if ret == 0 and signed_image(args.build_dir) and fat_volume(args.build_dir):
        cmd = ["west", "dfu", "--storage", "-d", str(args.build_dir), "-s", args.board_id]
        print("+", " ".join(cmd), flush=True)
        ret = subprocess.run(cmd).returncode
    if ret == 0 and args.board_id and not args.board_id.startswith("/"):
        link_port(args.board_id)
    sys.exit(ret)


if __name__ == "__main__":
    main()
