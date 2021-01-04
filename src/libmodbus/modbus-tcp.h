/*
 * Copyright © 2001-2010 Stéphane Raimbault <stephane.raimbault@gmail.com>
 * Copyright © 2018 Arduino SA. All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.1+
 */

#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H

#ifdef ARDUINO
// check if __has_include ArduinoAPI
#if defined __has_include
#  if __has_include("api/ArduinoAPI.h")
#define __NEED_NAMESPACE__
namespace arduino {
#  endif
#endif
class Client;
class IPAddress;
#endif
#ifdef __NEED_NAMESPACE__
}
#endif

#include "modbus.h"

MODBUS_BEGIN_DECLS

#if defined(_WIN32) && !defined(__CYGWIN__)
/* Win32 with MinGW, supplement to <errno.h> */
#include <winsock2.h>
#if !defined(ECONNRESET)
#define ECONNRESET   WSAECONNRESET
#endif
#if !defined(ECONNREFUSED)
#define ECONNREFUSED WSAECONNREFUSED
#endif
#if !defined(ETIMEDOUT)
#define ETIMEDOUT    WSAETIMEDOUT
#endif
#if !defined(ENOPROTOOPT)
#define ENOPROTOOPT  WSAENOPROTOOPT
#endif
#if !defined(EINPROGRESS)
#define EINPROGRESS  WSAEINPROGRESS
#endif
#endif

#define MODBUS_TCP_DEFAULT_PORT   502
#define MODBUS_TCP_SLAVE         0xFF

/* Modbus_Application_Protocol_V1_1b.pdf Chapter 4 Section 1 Page 5
 * TCP MODBUS ADU = 253 bytes + MBAP (7 bytes) = 260 bytes
 */
#define MODBUS_TCP_MAX_ADU_LENGTH  260

#ifdef ARDUINO
#ifdef __NEED_NAMESPACE__
MODBUS_API modbus_t* modbus_new_tcp(arduino::Client* client, arduino::IPAddress ip_address, int port);
MODBUS_API int modbus_tcp_accept(modbus_t *ctx, arduino::Client* client);
#else
MODBUS_API modbus_t* modbus_new_tcp(Client* client, IPAddress ip_address, int port);
MODBUS_API int modbus_tcp_accept(modbus_t *ctx, Client* client);
#endif
MODBUS_API int modbus_tcp_listen(modbus_t *ctx);
#else
MODBUS_API modbus_t* modbus_new_tcp(const char *ip_address, int port);
MODBUS_API int modbus_tcp_listen(modbus_t *ctx, int nb_connection);
MODBUS_API int modbus_tcp_accept(modbus_t *ctx, int *s);

MODBUS_API modbus_t* modbus_new_tcp_pi(const char *node, const char *service);
MODBUS_API int modbus_tcp_pi_listen(modbus_t *ctx, int nb_connection);
MODBUS_API int modbus_tcp_pi_accept(modbus_t *ctx, int *s);
#endif

MODBUS_END_DECLS

#endif /* MODBUS_TCP_H */
