/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Three dump banks for the continuous ring (CONFIG_ESP_SDR_RING): SRAM banks
 * 0 to 2 (MAC_DUMP_USAGE bits 0 to 2), used instead of esp_sdr_bank_tx.ld.
 * Banks 1 and 2 keep their roles for captures and loop playback. While the
 * dump engine owns a bank the CPUs cannot reach any of it, so nothing else
 * may live in these; the libc malloc arena stays below them.
 */

__esp_sdr_bank0_org = 0x3fcb0000;
__esp_sdr_bank1_org = 0x3fcc0000;
__esp_sdr_bank_org = 0x3fcd0000;
__esp_sdr_bank_len = 0x10000;

/*
 * Bank 0 is only reserved, not a section: an image whose ring may stay off
 * (apps/multiapp) places buffers there that are used only while it is off.
 */
__esp_sdr_bank0_start = __esp_sdr_bank0_org;
__esp_sdr_bank0_end = __esp_sdr_bank0_org + __esp_sdr_bank_len;

.esp_sdr_bank1 __esp_sdr_bank1_org (NOLOAD) :
{
	__esp_sdr_bank1_start = .;
	. += __esp_sdr_bank_len;
	__esp_sdr_bank1_end = .;
}

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

_heap_sentry = __esp_sdr_bank0_org;
_libc_heap_size = _heap_sentry - _end;

ASSERT(_end <= __esp_sdr_bank0_org, "esp-sdr: DRAM image overlaps the ring banks")
ASSERT(__esp_sdr_bank0_end == __esp_sdr_bank1_org && __esp_sdr_bank1_end == __esp_sdr_bank_org,
       "esp-sdr: ring banks not contiguous")
ASSERT(__esp_sdr_bank_end <= DRAM_USER_END, "esp-sdr: capture bank beyond user DRAM")
ASSERT(__esp_sdr_high_end <= DRAM_USER_END, "esp-sdr: high buffers beyond user DRAM")
