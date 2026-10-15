/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Ring bank 0 (esp_sdr_bank_ring.ld only reserves it) for buffers no ring
 * user needs: stream's CDC-NCM DMA buffers, and the FAT volume's FATFS
 * object, flash disk cache and USB drive buffer, used only before an app or
 * without one (src/drive.c). Included before .dram0.noinit and .dram0.bss, so
 * these sections land here; nothing zeroes them.
 */

.multiapp_bank0 0x3fcb0000 (NOLOAD) :
{
	__multiapp_bank0_start = .;
	*usbd_cdc_ncm.c.obj(.noinit .noinit.* .bss.net_buf_data_*)
	*flashdisk.c.obj(.bss.flashdisk*_cache)
	*drive.c.obj(.bss.multiapp_fat_fs)
	*usbd_msc.c.obj(.bss.scsi_buf*)
	__multiapp_bank0_end = .;
}

ASSERT(__multiapp_bank0_end <= 0x3fcc0000, "multiapp: bank 0 buffers beyond bank 0")
