/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Capture bank of the C5 dump engine: all of SRAM block 1 (128 KiB at
 * 0x40820000, usage bit 9). Owning the block takes all of it from the CPU;
 * the samples land 64 KiB into it (MAC_DUMP_ALLOC). Included among the RAM
 * sections: code and initialized data stay below it in block 0, the
 * sections linked after it (the rest of .noinit, .bss, the libc heap) move
 * to block 2, above it.
 */

__esp_sdr_bank_org = 0x40820000;
__esp_sdr_bank_len = 0x20000;

/* Network and Wi-Fi driver stacks and buffers fill block 0 up, to balance the halves. */
.esp_sdr_low (NOLOAD) :
{
	. = ALIGN(8);
	*libsubsys__net__ip.a:(.noinit .noinit.*)
	*libdrivers__wifi.a:(.noinit .noinit.*)
	__esp_sdr_low_end = .;
} GROUP_NOLOAD_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)

.esp_sdr_bank __esp_sdr_bank_org (NOLOAD) :
{
	. += __esp_sdr_bank_len / 2;
	__esp_sdr_bank_start = .;
	. += __esp_sdr_bank_len / 2;
	__esp_sdr_bank_end = .;
} GROUP_NOLOAD_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)

/* Buffers the CPU must reach during a capture. Not zeroed at boot. */
.esp_sdr_high (NOLOAD) :
{
	__esp_sdr_high_start = .;
	*(.esp_sdr_high .esp_sdr_high.*)
	__esp_sdr_high_end = .;
} GROUP_NOLOAD_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)

ASSERT(__esp_sdr_low_end <= __esp_sdr_bank_org, "esp-sdr: code and data overlap the capture bank")
ASSERT(__esp_sdr_high_start >= __esp_sdr_bank_end, "esp-sdr: high buffers below the capture bank")
