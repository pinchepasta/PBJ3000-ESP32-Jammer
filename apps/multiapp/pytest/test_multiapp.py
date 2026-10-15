# SPDX-License-Identifier: GPL-3.0-or-later
"""Desk check of the multiapp host (doc/multiapp.md): the image and its FAT volume are
on the XIAO ESP32-S3 (scripts/twister-flash.py), and the boot runs hello from its file.

Run with the boards attached:

    west twister -T apps/multiapp -s sample.esp_sdr.multiapp.hil --device-testing \\
        --hardware-map tests/regression/hardware-map.yaml \\
        --flash-command "$PWD/scripts/twister-flash.py"
"""
import re

import pytest
from twister_harness import DeviceAdapter, Shell
from twister_harness.fixtures import determine_scope, get_ready_shell


@pytest.fixture(scope=determine_scope)
def shell(dut: DeviceAdapter) -> Shell:
    # Ctrl-C first: west dfu's CR LF leaves the shell swallowing the harness's bare LF
    # (tests/regression/conftest.py).
    dut.write(b"\x03")
    return get_ready_shell(dut)


def test_files(shell: Shell):
    out = shell.exec_command("app list")
    apps = [line for line in out if re.match(r"\w+\s+(\d+ bytes|-)", line)]
    assert len(apps) == 7, out
    for line in apps:
        # Every app has its file, the same as the built-in copy.
        assert re.search(r"file \d+ bytes", line) and "differs" not in line, line


def test_hello_from_file(shell: Shell):
    out = shell.exec_command("app info")
    assert any(line.startswith("boot: hello, start 0") for line in out), out
    assert any(line.startswith("hello (file):") for line in out), out


def test_hello_runs(shell: Shell):
    out = shell.exec_command("hello 5")
    assert any(line.startswith("hello: ops 10 15, direct 15") for line in out), out
    out = shell.exec_command("hello atomic", timeout=20)
    m = [re.search(r"psram \S+ (\d+), internal \S+ (\d+) \(expect (\d+)\)", line) for line in out]
    m = [x for x in m if x]
    assert m and m[0][1] == m[0][2] == m[0][3], out
