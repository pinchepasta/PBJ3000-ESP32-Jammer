/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Extended Hamming (128,120): 15 data bytes and one parity byte per block;
 * corrects one bit error and detects two. Portable C.
 */

#ifndef LINK_HAMMING_H_
#define LINK_HAMMING_H_

#include <stdint.h>

#define HAM_N 16
#define HAM_K 15

/* Build the syndrome tables; call once before the others. */
void ham_init(void);

/* Set blk[HAM_K] from blk[0..HAM_K). */
void ham_encode(uint8_t *blk);

/* Correct blk in place: 0 clean, 1 one bit corrected, -1 two or more errors. */
int ham_decode(uint8_t *blk);

#endif /* LINK_HAMMING_H_ */
