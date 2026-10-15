/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Capture bank of the S3 dump engine: SRAM bank 2 (MAC_DUMP_USAGE bit 2).
 * Lowering _heap_sentry keeps the libc malloc arena below it.
 */

__esp_sdr_bank_org = 0x3fcd0000;
__esp_sdr_bank_len = 0x10000;

.esp_sdr_bank __esp_sdr_bank_org (NOLOAD) :
{
	__esp_sdr_bank_start = .;
	. += __esp_sdr_bank_len;
	__esp_sdr_bank_end = .;
}

/*
 * Internal SRAM between the capture bank and the bootloader's loader segment
 * (bank 3 below its ROM data; the engine never owns it). Not zeroed at boot.
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
ASSERT(__esp_sdr_bank_end <= DRAM_USER_END, "esp-sdr: capture bank beyond user DRAM")
ASSERT(__esp_sdr_high_end <= DRAM_USER_END, "esp-sdr: high buffers beyond user DRAM")
