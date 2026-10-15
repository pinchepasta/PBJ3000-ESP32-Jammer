/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Stage 2 of the ring's decimating FIR (esp_sdr_ring.c): the job layout of
 * upstream's s3_fir_split (esp_sdr_ring_fir_split in esp_sdr_ring_fir.S), with each multiply-accumulate
 * also loading the next data vector: two instructions per 8 taps instead of
 * three. Needs an even number of tap groups; reads one vector past each
 * window (the window buffers have room for it).
 *
 * void esp_sdr_ring_dot(const struct fir_job *job)
 *   +0 xi, +4 xq, +8 h (16-byte aligned), +12 groups, +16 nout, +20 step
 *   (bytes, multiple of 16), +24 shift, +28 oi, +32 oq.
 */

	.section .iram1.esp_sdr_ring_dot, "ax"
	.align	4
	.global	esp_sdr_ring_dot
	.type	esp_sdr_ring_dot, @function
esp_sdr_ring_dot:
	entry	a1, 32
	l32i	a3, a2, 16		/* nout */
	beqz	a3, .Ldot_done
	l32i	a4, a2, 0		/* xi */
	l32i	a5, a2, 4		/* xq */
	l32i	a6, a2, 28		/* oi */
	l32i	a7, a2, 32		/* oq */
	l32i	a10, a2, 12		/* groups */
	srli	a10, a10, 1		/* pairs of groups */
	l32i	a11, a2, 24		/* shift */
	l32i	a13, a2, 20		/* step */
.Ldot_out:
	l32i	a8, a2, 8		/* h */
	mov	a9, a4
	ee.zero.accx
	ee.vld.128.ip	q1, a9, 16
	loopnez	a10, .Ldot_i
	ee.vld.128.ip	q0, a8, 16
	ee.vmulas.s16.accx.ld.ip	q2, a9, 16, q0, q1
	ee.vld.128.ip	q0, a8, 16
	ee.vmulas.s16.accx.ld.ip	q1, a9, 16, q0, q2
.Ldot_i:
	ee.srs.accx	a12, a11, 0
	s16i	a12, a6, 0
	addi	a6, a6, 2
	l32i	a8, a2, 8
	mov	a9, a5
	ee.zero.accx
	ee.vld.128.ip	q1, a9, 16
	loopnez	a10, .Ldot_q
	ee.vld.128.ip	q0, a8, 16
	ee.vmulas.s16.accx.ld.ip	q2, a9, 16, q0, q1
	ee.vld.128.ip	q0, a8, 16
	ee.vmulas.s16.accx.ld.ip	q1, a9, 16, q0, q2
.Ldot_q:
	ee.srs.accx	a12, a11, 0
	s16i	a12, a7, 0
	addi	a7, a7, 2
	add	a4, a4, a13
	add	a5, a5, a13
	addi	a3, a3, -1
	bnez	a3, .Ldot_out
.Ldot_done:
	retw.n
	.size	esp_sdr_ring_dot, . - esp_sdr_ring_dot
