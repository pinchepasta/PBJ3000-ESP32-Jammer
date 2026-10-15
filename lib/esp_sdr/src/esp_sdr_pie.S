/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32-S3 vector (PIE) ramp for the transmit interpolator: four DAC words per
 * step. Lanes hold I in Q10 and Q in Q20, so one arithmetic shift right by 10
 * puts I's integer part in bits 9:0 and Q's in bits 19:10; two masks and an or
 * give the DAC word. Uses q0-q7 and SAR; the caller enables the coprocessor.
 *
 * void esp_sdr_pie_ramp(uint32_t *dst, uint32_t groups, const int32_t *vec)
 *   dst    16-byte aligned, groups * 4 words written
 *   vec    16-byte aligned: I lanes, Q lanes, I step, Q step, I mask, Q mask
 *          (4 words each)
 */

	/* IRAM: the transmit hot path must not wait for the flash cache. */
	.section .iram1.esp_sdr_pie, "ax"
	.align	4
	.global	esp_sdr_pie_ramp
	.type	esp_sdr_pie_ramp, @function
esp_sdr_pie_ramp:
	entry	a1, 32
	ee.vld.128.ip	q0, a4, 16	/* I lanes */
	ee.vld.128.ip	q1, a4, 16	/* Q lanes */
	ee.vld.128.ip	q2, a4, 16	/* I step, four samples */
	ee.vld.128.ip	q3, a4, 16	/* Q step */
	ee.vld.128.ip	q4, a4, 16	/* 0x3ff */
	ee.vld.128.ip	q5, a4, 16	/* 0xffc00 */
	movi.n	a8, 10
	wsr.sar	a8
	loopnez	a3, .Lramp_end
	ee.vsr.32	q6, q0
	ee.vsr.32	q7, q1
	ee.andq	q6, q6, q4
	ee.andq	q7, q7, q5
	ee.orq	q6, q6, q7
	ee.vst.128.ip	q6, a2, 16
	ee.vadds.s32	q0, q0, q2
	ee.vadds.s32	q1, q1, q3
.Lramp_end:
	retw.n
	.size	esp_sdr_pie_ramp, . - esp_sdr_pie_ramp
