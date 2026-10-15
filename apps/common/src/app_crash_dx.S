/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Double exception record: the vector calls xtensa_triple_fault_breadcrumb
 * (call0, weak and empty in the arch) and then spins until a watchdog resets
 * the chip. This one leaves the state in RTC memory for the next boot
 * (app_crash_dx_last()). Clobbers a2 and a3; the vector never returns.
 */

#include <xtensa/corebits.h>

	.section .iram1.app_crash_dx, "ax"
	.literal_position
	.global xtensa_triple_fault_breadcrumb
	.type xtensa_triple_fault_breadcrumb, @function
	.align 4
xtensa_triple_fault_breadcrumb:
	movi	a2, app_crash_dx_record
	rsr.depc	a3
	s32i	a3, a2, 4
	rsr.exccause	a3
	s32i	a3, a2, 8
	rsr.excvaddr	a3
	s32i	a3, a2, 12
	rsr.epc1	a3
	s32i	a3, a2, 16
	rsr.ps	a3
	s32i	a3, a2, 20
	s32i	a1, a2, 24
	rsr.prid	a3
	s32i	a3, a2, 28
	movi	a3, 0x44584331 /* "DXC1" */
	s32i	a3, a2, 0
	memw
	ret.n
