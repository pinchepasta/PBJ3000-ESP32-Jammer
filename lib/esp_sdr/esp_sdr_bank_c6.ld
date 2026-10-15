/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Capture bank of the C6 dump engine: all of SRAM block 2 (128 KiB at
 * 0x40840000, usage bit 10, as upstream); a capture uses up to 64 KiB of it. Lowering _heap_sentry keeps the libc malloc arena below it.
 */

__esp_sdr_bank_org = 0x40840000;
__esp_sdr_bank_len = 0x20000;

.esp_sdr_bank __esp_sdr_bank_org (NOLOAD) :
{
	__esp_sdr_bank_start = .;
	. += __esp_sdr_bank_len;
	__esp_sdr_bank_end = .;
}

/*
 * Block 3 up to the end of user DRAM: the engine takes the whole of block 2
 * during a capture, so nothing else may live there. Not zeroed at boot.
 */
.esp_sdr_high __esp_sdr_bank_end (NOLOAD) :
{
	__esp_sdr_high_start = .;
	*(.esp_sdr_high .esp_sdr_high.*)
	__esp_sdr_high_end = .;
}

_heap_sentry = __esp_sdr_bank_org;
_libc_heap_size = _heap_sentry - _end;

ASSERT(_end <= __esp_sdr_bank_org, "esp-sdr: DRAM image overlaps the capture bank")
ASSERT(__esp_sdr_high_end <= DRAM_USER_END, "esp-sdr: high buffers beyond user DRAM")
