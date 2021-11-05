/* Copyright(c) 2021 Enrico Pozzobon
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef SIO_UART_H
#define SIO_UART_H

#include <mgba-util/common.h>

CXX_GUARD_START

#include <mgba/core/timing.h>
#include <mgba/internal/gba/sio.h>
#ifdef _WIN32
#include <fileapi.h>
#endif

#define GBASIO_FIFO_SIZE 8
struct GBASIOUART {
	struct GBASIODriver d;
	struct mTimingEvent event;

#ifdef _WIN32
	HANDLE handlePort;
#else
	int fd;
#endif

	unsigned rxFIFOCount;
	unsigned rxFIFOHead;
	uint8_t rxFIFOBuffer[GBASIO_FIFO_SIZE];
	unsigned txFIFOCount;
	unsigned txFIFOHead;
	uint8_t txFIFOBuffer[GBASIO_FIFO_SIZE];
	uint8_t txByte;
};

void GBASIOUARTCreate(struct GBASIOUART*);
void GBASIOUARTDestroy(struct GBASIOUART*);

bool GBASIOUARTConnect(struct GBASIOUART*, const char *device);
bool GBASIOUARTIsConnected(struct GBASIOUART*);

CXX_GUARD_END

#endif
