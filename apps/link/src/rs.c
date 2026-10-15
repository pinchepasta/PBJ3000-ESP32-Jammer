/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * RS(255,223) codec. The encoder runs the parity register a word at a time
 * from a 256 x 32 byte table; the decoder first checks the codeword with the
 * same register (a clean one costs no more than encoding) and only computes
 * syndromes, Berlekamp-Massey, Chien search and Forney for a dirty one,
 * after Phil Karn's decode_rs().
 */

#include <string.h>

#include "rs.h"

#if defined(__ZEPHYR__)
#include <esp_attr.h>
#define RS_HOT IRAM_ATTR __attribute__((noinline))
#else
#define RS_HOT
#endif

#define A0     RS_N /* log of zero */
#define FCR    1
#define WORDS  (RS_NROOTS / 4)
/* Largest even i <= min(a, b): even powers of the formal derivative of lambda. */
#define MIN_EVEN(a, b) (((a) < (b) ? (a) : (b)) & ~1)

static uint8_t alpha_to[RS_N + 1], index_of[RS_N + 1];
/* Generator polynomial in index form, genpoly[RS_NROOTS] = 1 (monic). */
static uint8_t genpoly[RS_NROOTS + 1];
/* Feedback byte times the generator taps, packed for the word-wide register. */
static uint32_t enc_table[256][WORDS];

static inline unsigned int modnn(unsigned int x)
{
	while (x >= RS_N) {
		x -= RS_N;
		x = (x >> 8) + (x & RS_N);
	}
	return x;
}

/* x mod RS_N for x < 2 RS_N: the sum of two indices, without modnn()'s loop. */
static inline unsigned int mod2(unsigned int x)
{
	return x >= RS_N ? x - RS_N : x;
}

static uint8_t gf_mul(uint8_t a, uint8_t b)
{
	if (a == 0 || b == 0) {
		return 0;
	}
	return alpha_to[modnn(index_of[a] + index_of[b])];
}

void rs_init(void)
{
	unsigned int sr = 1;
	uint8_t g[RS_NROOTS + 1];

	index_of[0] = A0;
	alpha_to[A0] = 0;
	for (unsigned int i = 0; i < RS_N; i++) {
		index_of[sr] = (uint8_t)i;
		alpha_to[i] = (uint8_t)sr;
		sr <<= 1;
		if (sr & 0x100) {
			sr ^= 0x11d;
		}
	}

	/* g(x) = prod (x - alpha^(FCR + i)), poly form, g[i] the x^i coefficient. */
	memset(g, 0, sizeof(g));
	g[0] = 1;
	for (unsigned int i = 0; i < RS_NROOTS; i++) {
		uint8_t root = alpha_to[FCR + i];

		g[i + 1] = 1;
		for (unsigned int j = i; j > 0; j--) {
			g[j] = g[j - 1] ^ gf_mul(g[j], root);
		}
		g[0] = gf_mul(g[0], root);
	}
	for (unsigned int i = 0; i <= RS_NROOTS; i++) {
		genpoly[i] = index_of[g[i]];
	}

	/*
	 * Register byte j (j = 0 the highest degree) after one data byte:
	 * reg[j + 1] ^ fb * g[RS_NROOTS - 1 - j]. Byte j sits in word j / 4,
	 * bits 8 * (j % 4), so the shift is one byte down across the words.
	 */
	for (unsigned int fb = 0; fb < 256; fb++) {
		for (unsigned int w = 0; w < WORDS; w++) {
			uint32_t v = 0;

			for (unsigned int b = 0; b < 4; b++) {
				unsigned int j = 4 * w + b;

				v |= (uint32_t)gf_mul((uint8_t)fb, g[RS_NROOTS - 1 - j]) << (8 * b);
			}
			enc_table[fb][w] = v;
		}
	}
}

/* Parity register over data[0..RS_K). */
RS_HOT static void parity(const uint8_t *data, uint32_t reg[WORDS])
{
	uint32_t r0 = 0, r1 = 0, r2 = 0, r3 = 0, r4 = 0, r5 = 0, r6 = 0, r7 = 0;

	for (unsigned int i = 0; i < RS_K; i++) {
		const uint32_t *t = enc_table[data[i] ^ (r0 & 0xff)];

		r0 = ((r0 >> 8) | (r1 << 24)) ^ t[0];
		r1 = ((r1 >> 8) | (r2 << 24)) ^ t[1];
		r2 = ((r2 >> 8) | (r3 << 24)) ^ t[2];
		r3 = ((r3 >> 8) | (r4 << 24)) ^ t[3];
		r4 = ((r4 >> 8) | (r5 << 24)) ^ t[4];
		r5 = ((r5 >> 8) | (r6 << 24)) ^ t[5];
		r6 = ((r6 >> 8) | (r7 << 24)) ^ t[6];
		r7 = (r7 >> 8) ^ t[7];
	}
	reg[0] = r0;
	reg[1] = r1;
	reg[2] = r2;
	reg[3] = r3;
	reg[4] = r4;
	reg[5] = r5;
	reg[6] = r6;
	reg[7] = r7;
}

void rs_encode(uint8_t *cw)
{
	uint32_t reg[WORDS];

	parity(cw, reg);
	for (unsigned int j = 0; j < RS_NROOTS; j++) {
		cw[RS_K + j] = (uint8_t)(reg[j / 4] >> (8 * (j % 4)));
	}
}

RS_HOT int rs_decode(uint8_t *cw)
{
	uint8_t rem[RS_NROOTS], s[RS_NROOTS];
	uint8_t lambda[RS_NROOTS + 1], b[RS_NROOTS + 1], t[RS_NROOTS + 1];
	uint8_t omega[RS_NROOTS + 1], root[RS_NROOTS], loc[RS_NROOTS], reg[RS_NROOTS + 1];
	uint32_t pr[WORDS];
	unsigned int dirty = 0;
	int deg_lambda, el, deg_omega, count;

	/* Remainder of the received word modulo g(x); zero for a codeword. */
	parity(cw, pr);
	for (unsigned int j = 0; j < RS_NROOTS; j++) {
		rem[j] = (uint8_t)(pr[j / 4] >> (8 * (j % 4))) ^ cw[RS_K + j];
		dirty |= rem[j];
	}
	if (dirty == 0) {
		return 0;
	}

	/* Syndromes: g(x) vanishes at the roots, so r(alpha^i) = rem(alpha^i). */
	for (unsigned int i = 0; i < RS_NROOTS; i++) {
		unsigned int acc = rem[0];

		for (unsigned int j = 1; j < RS_NROOTS; j++) {
			acc = rem[j] ^ (acc == 0 ? 0 : alpha_to[mod2(index_of[acc] + mod2(FCR + i))]);
		}
		s[i] = index_of[acc];
	}

	/* Berlekamp-Massey. */
	memset(lambda + 1, 0, RS_NROOTS);
	lambda[0] = 1;
	for (unsigned int i = 0; i <= RS_NROOTS; i++) {
		b[i] = index_of[lambda[i]];
	}
	el = 0;
	for (int r = 1; r <= RS_NROOTS; r++) {
		unsigned int discr = 0;

		for (int i = 0; i < r; i++) {
			if (lambda[i] != 0 && s[r - i - 1] != A0) {
				discr ^= alpha_to[mod2(index_of[lambda[i]] + s[r - i - 1])];
			}
		}
		discr = index_of[discr];
		if (discr == A0) {
			memmove(&b[1], b, RS_NROOTS);
			b[0] = A0;
			continue;
		}
		/* lambda and b have degree below r: the coefficients above stay as they are. */
		int top = r < RS_NROOTS ? r : RS_NROOTS;

		memcpy(t, lambda, RS_NROOTS + 1);
		for (int i = 0; i < top; i++) {
			if (b[i] != A0) {
				t[i + 1] = lambda[i + 1] ^ alpha_to[mod2(discr + b[i])];
			}
		}
		if (2 * el <= r - 1) {
			el = r - el;
			for (int i = 0; i <= RS_NROOTS; i++) {
				b[i] = lambda[i] == 0 ? A0
						      : (uint8_t)mod2(index_of[lambda[i]] + RS_N - discr);
			}
		} else {
			memmove(&b[1], b, RS_NROOTS);
			b[0] = A0;
		}
		memcpy(lambda, t, RS_NROOTS + 1);
	}

	deg_lambda = 0;
	for (int i = 0; i <= RS_NROOTS; i++) {
		lambda[i] = index_of[lambda[i]];
		if (lambda[i] != A0) {
			deg_lambda = i;
		}
	}
	if (deg_lambda > RS_NROOTS / 2) {
		return -1;
	}

	/* Chien search for the roots of lambda. */
	memcpy(&reg[1], &lambda[1], RS_NROOTS);
	count = 0;
	for (int i = 1, k = 0; i <= RS_N; i++, k++) {
		unsigned int q = 1;

		for (int j = deg_lambda; j > 0; j--) {
			if (reg[j] != A0) {
				reg[j] = (uint8_t)mod2(reg[j] + j);
				q ^= alpha_to[reg[j]];
			}
		}
		if (q != 0) {
			continue;
		}
		root[count] = (uint8_t)i;
		loc[count] = (uint8_t)k;
		if (++count == deg_lambda) {
			break;
		}
	}
	if (count != deg_lambda) {
		return -1;
	}

	/* omega(x) = s(x) lambda(x) mod x^RS_NROOTS, index form. */
	deg_omega = deg_lambda - 1;
	for (int i = 0; i <= deg_omega; i++) {
		unsigned int tmp = 0;

		for (int j = i; j >= 0; j--) {
			if (s[i - j] != A0 && lambda[j] != A0) {
				tmp ^= alpha_to[mod2(s[i - j] + lambda[j])];
			}
		}
		omega[i] = index_of[tmp];
	}

	/* Forney: error values, cw[0] is the highest degree coefficient. */
	for (int j = count - 1; j >= 0; j--) {
		unsigned int num1 = 0, num2, den = 0;

		for (int i = deg_omega; i >= 0; i--) {
			if (omega[i] != A0) {
				num1 ^= alpha_to[modnn(omega[i] + i * root[j])];
			}
		}
		num2 = alpha_to[modnn(root[j] * (FCR - 1) + RS_N)];
		for (int i = MIN_EVEN(deg_lambda, RS_NROOTS - 1); i >= 0; i -= 2) {
			if (lambda[i + 1] != A0) {
				den ^= alpha_to[modnn(lambda[i + 1] + i * root[j])];
			}
		}
		if (den == 0) {
			return -1;
		}
		if (num1 != 0) {
			cw[loc[j]] ^= alpha_to[modnn(index_of[num1] + index_of[num2] + RS_N -
						     index_of[den])];
		}
	}
	return count;
}
