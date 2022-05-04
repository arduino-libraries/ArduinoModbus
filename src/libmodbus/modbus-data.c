/*
 * Copyright © 2010-2014 Stéphane Raimbault <stephane.raimbault@gmail.com>
 * Copyright © 2018 Arduino SA. All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.1+
 */

#include <stdlib.h>

#ifndef _MSC_VER
#  include <stdint.h>
#else
#  include "stdint.h"
#endif

#include <string.h>
#include <assert.h>

#if defined(_WIN32)
#  include <winsock2.h>
#elif defined(ARDUINO)
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define htonl(x) bswap_32(x)
#define htons(x) bswap_16(x)
#define ntohl(x) bswap_32(x)
#define ntohs(x) bswap_16(x)
#else
#define htonl(x) (x)
#define htons(x) (x)
#define ntohl(x) (x)
#define ntohs(x) (x)
#endif
#else
#  include <arpa/inet.h>
#endif

#ifndef ARDUINO
#include <config.h>
#endif

#include "modbus.h"

#if defined(HAVE_BYTESWAP_H)
#  include <byteswap.h>
#endif

#if defined(__APPLE__)
#  include <libkern/OSByteOrder.h>
#  define bswap_16 OSSwapInt16
#  define bswap_32 OSSwapInt32
#  define bswap_64 OSSwapInt64
#endif

#if defined(__GNUC__)
#  define GCC_VERSION (__GNUC__ * 100 + __GNUC_MINOR__ * 10)
#  if GCC_VERSION >= 430
// Since GCC >= 4.30, GCC provides __builtin_bswapXX() alternatives so we switch to them
#    undef bswap_32
#    define bswap_32 __builtin_bswap32
#  endif
#endif

#if defined(_MSC_VER) && (_MSC_VER >= 1400)
#  define bswap_32 _byteswap_ulong
#  define bswap_16 _byteswap_ushort
#endif

#if !defined(__CYGWIN__) && !defined(bswap_16)
#ifndef ARDUINO
#  warning "Fallback on C functions for bswap_16"
#endif
static inline uint16_t bswap_16(uint16_t x)
{
    return (x >> 8) | (x << 8);
}
#endif

#if !defined(bswap_32)
#ifndef ARDUINO
#  warning "Fallback on C functions for bswap_32"
#endif
static inline uint32_t bswap_32(uint32_t x)
{
    return (bswap_16(x & 0xffff) << 16) | (bswap_16(x >> 16));
}
#endif

/* Sets many bits from a single byte value (all 8 bits of the byte value are
   set) */
void modbus_set_bits_from_byte(uint8_t *dest, int idx, const uint8_t value)
{
    int i;

    for (i=0; i < 8; i++) {
        dest[idx+i] = (value & (1 << i)) ? 1 : 0;
    }
}

/* Sets many bits from a table of bytes (only the bits between idx and
   idx + nb_bits are set) */
void modbus_set_bits_from_bytes(uint8_t *dest, int idx, unsigned int nb_bits,
                                const uint8_t *tab_byte)
{
    unsigned int i;
    int shift = 0;

    for (i = idx; i < idx + nb_bits; i++) {
        dest[i] = tab_byte[(i - idx) / 8] & (1 << shift) ? 1 : 0;
        /* gcc doesn't like: shift = (++shift) % 8; */
        shift++;
        shift %= 8;
    }
}

/* Gets the byte value from many bits.
   To obtain a full byte, set nb_bits to 8. */
uint8_t modbus_get_byte_from_bits(const uint8_t *src, int idx,
                                  unsigned int nb_bits)
{
    unsigned int i;
    uint8_t value = 0;

    if (nb_bits > 8) {
        /* Assert is ignored if NDEBUG is set */
        assert(nb_bits < 8);
        nb_bits = 8;
    }

    for (i=0; i < nb_bits; i++) {
        value |= (src[idx+i] << i);
    }

    return value;
}

/* Get a float from 4 bytes (Modbus) without any conversion (ABCD) */
float modbus_get_float_abcd(const uint16_t *src)
{
    float f;
    uint32_t i;

    i = ntohl(((uint32_t)src[0] << 16) + src[1]);
    memcpy(&f, &i, sizeof(float));

    return f;
}

/* Get a float from 4 bytes (Modbus) in inversed format (DCBA) */
float modbus_get_float_dcba(const uint16_t *src)
{
    float f;
    uint32_t i;

    i = ntohl(bswap_32((((uint32_t)src[0]) << 16) + src[1]));
    memcpy(&f, &i, sizeof(float));

    return f;
}

/* Get a float from 4 bytes (Modbus) with swapped bytes (BADC) */
float modbus_get_float_badc(const uint16_t *src)
{
    float f;
    uint32_t i;

#if defined(ARDUINO) && defined(__AVR__)
    i = ntohl((uint32_t)((uint32_t)bswap_16(src[0]) << 16) + bswap_16(src[1]));
#else
    i = ntohl((uint32_t)(bswap_16(src[0]) << 16) + bswap_16(src[1]));
#endif
    memcpy(&f, &i, sizeof(float));

    return f;
}

/* Get a float from 4 bytes (Modbus) with swapped words (CDAB) */
float modbus_get_float_cdab(const uint16_t *src)
{
    float f;
    uint32_t i;

    i = ntohl((((uint32_t)src[1]) << 16) + src[0]);
    memcpy(&f, &i, sizeof(float));

    return f;
}

/* DEPRECATED - Get a float from 4 bytes in sort of Modbus format */
float modbus_get_float(const uint16_t *src)
{
    float f;
    uint32_t i;

    i = (((uint32_t)src[1]) << 16) + src[0];
    memcpy(&f, &i, sizeof(float));

    return f;
}

/* Set a float to 4 bytes for Modbus w/o any conversion (ABCD) */
void modbus_set_float_abcd(float f, uint16_t *dest)
{
    uint32_t i;

    memcpy(&i, &f, sizeof(uint32_t));
    i = htonl(i);
    dest[0] = (uint16_t)(i >> 16);
    dest[1] = (uint16_t)i;
}

/* Set a float to 4 bytes for Modbus with byte and word swap conversion (DCBA) */
void modbus_set_float_dcba(float f, uint16_t *dest)
{
    uint32_t i;

    memcpy(&i, &f, sizeof(uint32_t));
    i = bswap_32(htonl(i));
    dest[0] = (uint16_t)(i >> 16);
    dest[1] = (uint16_t)i;
}

/* Set a float to 4 bytes for Modbus with byte swap conversion (BADC) */
void modbus_set_float_badc(float f, uint16_t *dest)
{
    uint32_t i;

    memcpy(&i, &f, sizeof(uint32_t));
    i = htonl(i);
    dest[0] = (uint16_t)bswap_16(i >> 16);
    dest[1] = (uint16_t)bswap_16(i & 0xFFFF);
}

/* Set a float to 4 bytes for Modbus with word swap conversion (CDAB) */
void modbus_set_float_cdab(float f, uint16_t *dest)
{
    uint32_t i;

    memcpy(&i, &f, sizeof(uint32_t));
    i = htonl(i);
    dest[0] = (uint16_t)i;
    dest[1] = (uint16_t)(i >> 16);
}

/* DEPRECATED - Set a float to 4 bytes in a sort of Modbus format! */
void modbus_set_float(float f, uint16_t *dest)
{
    uint32_t i;

    memcpy(&i, &f, sizeof(uint32_t));
    dest[0] = (uint16_t)i;
    dest[1] = (uint16_t)(i >> 16);
}
