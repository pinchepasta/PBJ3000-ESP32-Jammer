# SPDX-License-Identifier: GPL-3.0-or-later
"""
Two-board regression test: a real xiao_esp32c6 (native radio) and a real
xiao_esp32s3 (esp_sdr software radio) exchanging apps/wpan's test frames,
both ways, on every channel the software radio is expected to handle
cleanly.

This is the twister-driven version of apps/wpan/tools/interop.py,
run here instead of by hand so it keeps gating regressions in two bugs found
and fixed while developing the software radio, both of which showed up only
as lost or corrupted frames between two real boards (never on native_sim,
and not reliably from a single board's own counters):

- the Wi-Fi MAC interrupt running inside the receive ring's interrupt
  windows, starving it (doc/hardware-quirks.md, "The Wi-Fi MAC interrupt
  must not run inside the ring");
- the sample ring generation race (the PHY thread pairing a stale
  generation with another run's write index).

Needs two connected boards and a hardware map with required_devices (see
doc/testing.md); twister flashes and resets both.
"""
import logging
import re
import time

import pytest
from twister_harness import Shell

logger = logging.getLogger(__name__)

RX_RE = re.compile(r"rx frames (\d+) test (\d+) bad (\d+) dups (\d+) lost (\d+)")
TX_RE = re.compile(r"tx sent (\d+) ok (\d+) noack (\d+) busy (\d+) err (\d+)")
CHANNELS = (11, 15, 20, 25)  # not 26: the C6's waveform there does not decode (parked, see
# doc/ieee802154.md)
FRAMES = 100
PAYLOAD = 100
INTERVAL_MS = 5
SETTLE_S = 1.0  # the software radio retunes and restarts its receive ring on a channel change
# Frames a transmitter may fail to send per burst. Both radios do now and then: the S3 when
# the DAC generator fills a block late and cuts the frame (7 in 3133 frames by the driver
# counters), the C6 with a tx error of its own. The sender reports it, and the frame never
# goes out.
MAX_TX_ERRORS = 3
ACK_FRAMES = 20
MAX_ACK_FAILURES = 2  # of ACK_FRAMES, sent or not answered


def set_channel(shells: list[Shell], channel: int) -> None:
    for s in shells:
        s.exec_command(f"wpan set chan {channel}")
    time.sleep(SETTLE_S)


def counts(shell: Shell, pattern: re.Pattern) -> tuple[int, ...]:
    lines = shell.exec_command("wpan status")
    for line in lines:
        if m := pattern.search(line):
            return tuple(int(x) for x in m.groups())
    pytest.fail(f"no line matching {pattern.pattern!r} in: {lines}")


def check_direction(direction: str, channel: int, tx: Shell, rx: Shell) -> None:
    """Sends FRAMES test frames tx -> rx. Every frame the transmitter completed must arrive,
    none corrupt or twice; a frame the transmitter itself reports as failed is its own matter
    (MAX_TX_ERRORS), not a receive loss."""
    rx.exec_command("wpan clear")
    tx.exec_command(f"wpan tx {FRAMES} {INTERVAL_MS} {PAYLOAD}")
    # "wpan tx" starts a background job and its shell prompt returns at once; the job
    # itself (and the frames arriving at rx) take about FRAMES * INTERVAL_MS plus the
    # time each frame is on air. Generous margin, as tools/interop.py uses.
    time.sleep(FRAMES * (INTERVAL_MS / 1000.0 + 0.01) + 5)
    sent, ok, noack, busy, err = counts(tx, TX_RE)
    _, test, bad, dups, lost = counts(rx, RX_RE)

    problems = []
    if sent != FRAMES or ok < FRAMES - MAX_TX_ERRORS or busy:
        problems.append(f"the transmitter sent {sent} of {FRAMES} frames, {ok} ok, {err} errors, "
                        f"{busy} busy")
    if test < ok:
        problems.append(f"the receiver counted {test} test frames, fewer than the {ok} the "
                        "transmitter completed")
    if test > sent:
        problems.append(f"the receiver counted {test} test frames, more than the {sent} sent")
    if bad or dups:
        problems.append(f"the receiver counted {bad} corrupt and {dups} duplicate test frames")
    if problems:
        status = {name: "\n".join(s.exec_command("wpan status")) for name, s in
                  (("tx", tx), ("rx", rx))}
        pytest.fail(
            f"channel {channel}, {direction}: " + "; ".join(problems) + f" (receiver lost {lost})"
            f"\n--- tx wpan status ---\n{status['tx']}\n--- rx wpan status ---\n{status['rx']}"
        )


@pytest.fixture(scope="module", autouse=True)
def radios_on(shells: list[Shell]):
    assert len(shells) > 1, (
        "this test needs two connected boards (a hardware map with required_devices; "
        "see doc/testing.md)"
    )
    # shells[0] must be the native radio (the scenario's own board), shells[1] the software one.
    for shell, radio in zip(shells, ("radio ieee802154 ", "radio esp_sdr_154 ")):
        lines = shell.exec_command("wpan status")
        assert any(radio in line for line in lines), f"expected '{radio.strip()}' in: {lines}"
    for s in shells:
        s.exec_command("wpan set print 0")
        s.exec_command("wpan on")
    # The native radio ACKs frames addressed to its own address.
    shells[0].exec_command("wpan set pan 0xabcd addr 0x0002")
    shells[1].exec_command("wpan set pan 0xabcd addr 0x0001")


@pytest.mark.parametrize("channel", CHANNELS)
def test_frames_both_ways_no_loss(shells: list[Shell], channel):
    native, sdr = shells[0], shells[1]

    set_channel(shells, channel)
    check_direction("native -> sdr", channel, native, sdr)
    check_direction("sdr -> native", channel, sdr, native)


def test_ack_requests_answered(shells: list[Shell]):
    """channel 11 only, as a lighter regression check that ACK reception itself (the
    generator session ending exactly at the frame's last sample, then one capture
    timed to the 192 us turnaround) still works."""
    native, sdr = shells[0], shells[1]

    set_channel(shells, 11)
    native.exec_command("wpan clear")
    lines = sdr.exec_command(
        f"wpan tx {ACK_FRAMES} {INTERVAL_MS} {PAYLOAD} 0x0002 1", get_full_output=True,
        full_output_timeout=ACK_FRAMES * (INTERVAL_MS / 1000.0 + 0.05) + 5,
    )
    joined = "\n".join(lines)
    m = re.search(r"tx done: sent (\d+) ok (\d+) noack (\d+) busy (\d+) err (\d+)", joined)
    assert m, f"no 'tx done' line in: {lines}"
    sent, ok, noack, busy, err = (int(x) for x in m.groups())
    # A frame can fail to be sent (err) or to be answered (noack) now and then, see
    # MAX_TX_ERRORS; ACK reception broken altogether leaves nearly all of them unanswered.
    if sent != ACK_FRAMES or busy or noack + err > MAX_ACK_FAILURES:
        status = {name: "\n".join(s.exec_command("wpan status")) for name, s in
                  (("sdr", sdr), ("native", native))}
        pytest.fail(
            f"ack requests: sent {sent} ok {ok} noack {noack} busy {busy} err {err}"
            f"\n--- sdr wpan status ---\n{status['sdr']}"
            f"\n--- native wpan status ---\n{status['native']}"
        )
