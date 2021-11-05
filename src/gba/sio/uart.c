/* Copyright(c) 2021 Enrico Pozzobon
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <mgba/internal/gba/sio/uart.h>

#include <mgba/internal/gba/gba.h>
#include <mgba/internal/gba/io.h>

#ifdef _WIN32
#include <fileapi.h>
#include <errhandlingapi.h>
#else
#include <termios.h>
#include <errno.h>
#endif

// Bitfields in SIOCNT when in UART mode
#define SIOCNT_BAUD_RATE 3
#define SIOCNT_CTS_FLAG (1 << 2)
#define SIOCNT_PARITY_CONTROL (1 << 3)
#define SIOCNT_TX_FULL_FLAG (1 << 4)
#define SIOCNT_RX_EMPTY_FLAG (1 << 5)
#define SIOCNT_ERROR_FLAG (1 << 6)
#define SIOCNT_DATA_LENGTH (1 << 7)
#define SIOCNT_FIFO_ENABLE_FLAG (1 << 8)
#define SIOCNT_PARITY_ENABLE_FLAG (1 << 9)
#define SIOCNT_SEND_ENABLE_FLAG (1 << 10)
#define SIOCNT_RECEIVE_ENABLE_FLAG (1 << 11)
#define SIOCNT_IRQ_ENABLE (1 << 14)
unsigned const baud_rates[4] = {9600, 38400, 57600, 115200};

static bool GBASIOUARTInit(struct GBASIODriver* driver);
static bool GBASIOUARTLoad(struct GBASIODriver* driver);
static bool GBASIOUARTUnload(struct GBASIODriver* driver);
static void GBASIOUARTProcessEvents(struct mTiming* timing, void* context, uint32_t cyclesLate);
static uint16_t GBASIOUARTWriteRegister(struct GBASIODriver* driver, uint32_t address, uint16_t value);
static uint16_t GBASIOUARTReadRegister(struct GBASIODriver* driver, uint32_t address);
static void GBASIOUARTRaiseIRQ(struct GBASIO *sio);
static void GBASIOUARTSetupTTY(struct GBASIOUART* dri);
static bool GBASIOUARTReceiveByte(struct GBASIOUART* dri);
static bool GBASIOUARTSendByte(struct GBASIOUART* dri);
static bool GBASIOUARTWrite(struct GBASIOUART* driver, uint8_t byte);
static bool GBASIOUARTRead(struct GBASIOUART* driver, uint8_t *byte);

void GBASIOUARTCreate(struct GBASIOUART* dri) {
	GBASIOJOYCreate(&dri->d);
	dri->d.init = GBASIOUARTInit;
	dri->d.load = GBASIOUARTLoad;
	dri->d.unload = GBASIOUARTUnload;
	dri->d.writeRegister = GBASIOUARTWriteRegister;
	dri->d.readRegister = GBASIOUARTReadRegister;
	dri->event.context = dri;
	dri->event.name = "GBA SIO UART TTY";
	dri->event.callback = GBASIOUARTProcessEvents;
	dri->event.priority = 0x80;
}

static bool GBASIOUARTInit(struct GBASIODriver* driver) {
	struct GBASIOUART* dri = (struct GBASIOUART*) driver;
	dri->txFIFOHead = 0;
	dri->txFIFOCount = 0;
	dri->rxFIFOHead = 0;
	dri->rxFIFOCount = 0;
#ifdef _WIN32
	dri->handlePort = INVALID_HANDLE_VALUE;
#else
	dri->fd = -1;
#endif
	return true;
}

static bool GBASIOUARTLoad(struct GBASIODriver* driver) {
	struct GBASIOUART* dri = (struct GBASIOUART*) driver;
	mTimingDeschedule(&dri->d.p->p->timing, &dri->event);
	mTimingSchedule(&dri->d.p->p->timing, &dri->event, 0);
	return true;
}

static bool GBASIOUARTUnload(struct GBASIODriver* driver) {
	struct GBASIOUART* dri = (struct GBASIOUART*) driver;
	mTimingDeschedule(&dri->d.p->p->timing, &dri->event);
	return true;
}

static void GBASIOUARTProcessEvents(struct mTiming* timing, void* context, uint32_t cyclesLate) {
	(void) cyclesLate;
	struct GBASIOUART* dri = context;
	struct GBASIO *sio = dri->d.p;

	if (sio->mode == SIO_UART && GBASIOUARTIsConnected(dri)) {
		GBASIOUARTSendByte(dri);
		GBASIOUARTReceiveByte(dri);
	}

	unsigned baud_rate = baud_rates[sio->siocnt & 3];
	unsigned data_length = (sio->siocnt & SIOCNT_DATA_LENGTH) ? 8 : 7;
	unsigned parity = (sio->siocnt & SIOCNT_PARITY_ENABLE_FLAG) ? 1 : 0;
	int32_t nextEvent = GBA_ARM7TDMI_FREQUENCY * (data_length + 2 + parity) / baud_rate;

	mTimingSchedule(timing, &dri->event, nextEvent);
}

static bool GBASIOUARTSendByte(struct GBASIOUART* dri) {
	uint8_t txbyte;
	struct GBASIO *sio = dri->d.p;
	if (!(sio->siocnt & SIOCNT_SEND_ENABLE_FLAG)) {
		return false;
	}

	if (sio->siocnt & SIOCNT_FIFO_ENABLE_FLAG) {
		if (dri->txFIFOCount > 0) {
			unsigned pos = dri->txFIFOHead % GBASIO_FIFO_SIZE;
			txbyte = dri->txFIFOBuffer[pos];
			dri->txFIFOHead = (pos + 1) % GBASIO_FIFO_SIZE;
			dri->txFIFOCount--;
			if (sio->siocnt & SIOCNT_TX_FULL_FLAG) {
				sio->siocnt &= ~SIOCNT_TX_FULL_FLAG;
				GBASIOUARTRaiseIRQ(sio);
			}
		} else {
			return false;  // nothing to transmit
		}
	} else {
		if (sio->siocnt & SIOCNT_TX_FULL_FLAG) {
			txbyte = dri->txByte;
			if (sio->siocnt & SIOCNT_TX_FULL_FLAG) {
				sio->siocnt &= ~SIOCNT_TX_FULL_FLAG;
				GBASIOUARTRaiseIRQ(sio);
			}
		} else {
			return false;  // nothing to transmit
		}
	}

	txbyte &= (sio->siocnt & SIOCNT_DATA_LENGTH) ? 0xff : 0x7f;
	return GBASIOUARTWrite(dri, txbyte);
}

static bool GBASIOUARTReceiveByte(struct GBASIOUART* dri) {
	uint8_t rxbyte;
	struct GBASIO *sio = dri->d.p;
	if (!(sio->siocnt & SIOCNT_RECEIVE_ENABLE_FLAG)) {
		return false;
	}
	if (!GBASIOUARTRead(dri, &rxbyte)) {
		return false;
	}

	rxbyte &= (sio->siocnt & SIOCNT_DATA_LENGTH) ? 0xff : 0x7f;
	if (sio->siocnt & SIOCNT_FIFO_ENABLE_FLAG) {
		// FIFO mode
		if (dri->rxFIFOCount < GBASIO_FIFO_SIZE) {
			dri->rxFIFOBuffer[(dri->rxFIFOHead + dri->rxFIFOCount) % GBASIO_FIFO_SIZE] = rxbyte;
			dri->rxFIFOCount++;
		} else if (dri->rxFIFOCount == 0) {
			sio->p->memory.io[REG_SIODATA8 >> 1] = rxbyte;
		} if (!(sio->siocnt & SIOCNT_ERROR_FLAG)) {
			sio->siocnt |= SIOCNT_ERROR_FLAG;
			GBASIOUARTRaiseIRQ(sio);
		}
	} else {
		if (sio->siocnt & SIOCNT_RX_EMPTY_FLAG) {
			sio->p->memory.io[REG_SIODATA8 >> 1] = rxbyte;
		} else if (!(sio->siocnt & SIOCNT_ERROR_FLAG)) {
			sio->siocnt |= SIOCNT_ERROR_FLAG;
			GBASIOUARTRaiseIRQ(sio);
		}
	}
	if (sio->siocnt & SIOCNT_RX_EMPTY_FLAG) {
		sio->siocnt &= ~SIOCNT_RX_EMPTY_FLAG;
		GBASIOUARTRaiseIRQ(sio);
	}
	return true;
}

static uint16_t GBASIOUARTWriteRegister(struct GBASIODriver* driver, uint32_t address, uint16_t value) {
	struct GBASIOUART *dri = (struct GBASIOUART*) driver;
	struct GBASIO *sio = dri->d.p;
	if (sio->mode != SIO_UART) {
		return value;
	}
	if (!GBASIOUARTIsConnected(dri)) {
		return value;
	}
	
	if (address == REG_SIODATA8) {
		uint8_t byte = (uint8_t) value;
		if (sio->siocnt & SIOCNT_FIFO_ENABLE_FLAG) {
			if (dri->txFIFOCount < GBASIO_FIFO_SIZE) {
				dri->txFIFOBuffer[(dri->txFIFOHead + dri->txFIFOCount) % GBASIO_FIFO_SIZE] = byte;
				dri->txFIFOCount++;
			}
			if (dri->txFIFOCount == GBASIO_FIFO_SIZE) {
				sio->siocnt |= SIOCNT_TX_FULL_FLAG;
			}
		} else {
			dri->txByte = byte;
			sio->siocnt |= SIOCNT_TX_FULL_FLAG;
		}
		return sio->p->memory.io[address >> 1];
	} else if (address == REG_SIOCNT) {
		uint16_t diff = sio->siocnt ^ value;
		sio->siocnt = (value & 0x7f8f) | (0x8070 & sio->siocnt);
		if (diff & (SIOCNT_BAUD_RATE | SIOCNT_CTS_FLAG | SIOCNT_PARITY_CONTROL | SIOCNT_DATA_LENGTH | SIOCNT_PARITY_ENABLE_FLAG)) {
			GBASIOUARTSetupTTY(dri);
		}
		if (diff & SIOCNT_CTS_FLAG) {
			// Just enabled/disabled CTS flag
			mLOG(GBA_SIO, WARN, "unimplemended CTS switch");
		}
		if (diff & SIOCNT_FIFO_ENABLE_FLAG) {
			if (sio->siocnt & SIOCNT_TX_FULL_FLAG) {
				sio->siocnt &= ~SIOCNT_TX_FULL_FLAG;
				GBASIOUARTRaiseIRQ(sio);
			}
			sio->siocnt |= SIOCNT_RX_EMPTY_FLAG;
			// Just enabled/disabled FIFO mode
			if (0 == (sio->siocnt & SIOCNT_FIFO_ENABLE_FLAG)) {
				// The content of the FIFO is reset when FIFO is disabled in UART mode
				dri->rxFIFOHead = 0;
				dri->rxFIFOCount = 0;
				dri->txFIFOHead = 0;
				dri->txFIFOCount = 0;
			}
		}
		if (diff & SIOCNT_SEND_ENABLE_FLAG) {
			// Just enabled/disabled send
			sio->siocnt &= ~SIOCNT_TX_FULL_FLAG;
		}
		if (diff & SIOCNT_RECEIVE_ENABLE_FLAG) {
			// Just enabled/disabled receive
			sio->siocnt |= SIOCNT_RX_EMPTY_FLAG;
		}
		return sio->siocnt;
	}

	return value;
}

static uint16_t GBASIOUARTReadRegister(struct GBASIODriver* driver, uint32_t address) {
	struct GBASIOUART *dri = (struct GBASIOUART*) driver;
	struct GBASIO *sio = dri->d.p;
	uint16_t value = sio->p->memory.io[address >> 1];
	if (sio->mode != SIO_UART) {
		return value;
	}
	if (!GBASIOUARTIsConnected(dri)) {
		return value;
	}
	
	if (address == REG_SIODATA8) {
		if (sio->siocnt & SIOCNT_FIFO_ENABLE_FLAG) {
			// FIFO mode
			if (dri->rxFIFOCount > 0) {
				sio->p->memory.io[REG_SIODATA8 >> 1] = dri->rxFIFOBuffer[dri->rxFIFOHead % GBASIO_FIFO_SIZE];
				dri->rxFIFOCount--;
				dri->rxFIFOHead = (dri->rxFIFOHead + 1) % GBASIO_FIFO_SIZE;
			}
			if (dri->rxFIFOCount == 0) {
				sio->siocnt |= SIOCNT_RX_EMPTY_FLAG;
			}
		} else {
			sio->siocnt |= SIOCNT_RX_EMPTY_FLAG;
		}
	}

	return value;
}

static void GBASIOUARTRaiseIRQ(struct GBASIO *sio) {
	if (sio->siocnt & SIOCNT_IRQ_ENABLE) {
		GBARaiseIRQ(sio->p, IRQ_SIO, 0);
	}
}

#ifdef _WIN32

#include <fileapi.h>
#include <errhandlingapi.h>

static bool GBASIOUARTWrite(struct GBASIOUART* dri, uint8_t byte) {
	DWORD length;
	if (!GBASIOUARTIsConnected(dri)) {
		return false;
	}
	bool r = WriteFile(dri->handlePort, &byte, 1, &length, NULL);
	if (r == 0) {
		mLOG(GBA_SIO, ERROR, "error %d writing into tty", GetLastError());
		CloseHandle(dri->handlePort);
		dri->handlePort = INVALID_HANDLE_VALUE;
	}
	if (length == 0) {
		mLOG(GBA_SIO, ERROR, "error, tty write would block");
	}
	return length == 1;
}

static bool GBASIOUARTRead(struct GBASIOUART* dri, uint8_t *byte) {
	DWORD length;
	if (!GBASIOUARTIsConnected(dri)) {
		return false;
	}
	bool r = ReadFile(dri->handlePort, byte, 1, &length, NULL);
	if (r == 0) {
		mLOG(GBA_SIO, ERROR, "error %d reading from tty", GetLastError());
		CloseHandle(dri->handlePort);
		dri->handlePort = INVALID_HANDLE_VALUE;
		return false;
	}
	return length == 1;
}

static void GBASIOUARTSetupTTY(struct GBASIOUART* dri) {
	struct GBASIO *sio = dri->d.p;
	DCB config;
	// Get current configuration of serial communication port.
	if (GetCommState(dri->handlePort, &config) == 0)
	{
		mLOG(GBA_SIO, ERROR, "Get configuration port has problem %d.", GetLastError());
		CloseHandle(dri->handlePort);
		dri->handlePort = INVALID_HANDLE_VALUE;
		return;
	}

	unsigned const rates[4] = {9600, 38400, 57600, 115200};
	config.ByteSize = sio->siocnt & SIOCNT_DATA_LENGTH ? 8 : 7;
	config.fOutxCtsFlow |= sio->siocnt & SIOCNT_CTS_FLAG ? 1 : 0;
	config.Parity = sio->siocnt & SIOCNT_PARITY_ENABLE_FLAG ? (sio->siocnt & SIOCNT_PARITY_CONTROL ? 1 : 2) : 0;
	config.BaudRate = rates[sio->siocnt & SIOCNT_BAUD_RATE];
	config.StopBits = 0;

	if (SetCommState(dri->handlePort, &config) == 0)
	{
		mLOG(GBA_SIO, ERROR, "Set configuration port has problem %d.", GetLastError());
		CloseHandle(dri->handlePort);
		dri->handlePort = INVALID_HANDLE_VALUE;
		return;
	}
	mLOG(GBA_SIO, DEBUG, "UART settings: %d %d %d %d %d.",
		config.ByteSize, config.StopBits, config.fOutxCtsFlow, config.Parity, config.BaudRate);

	COMMTIMEOUTS comTimeOut;
	comTimeOut.ReadIntervalTimeout = MAXDWORD;
	comTimeOut.ReadTotalTimeoutMultiplier = 0;
	comTimeOut.ReadTotalTimeoutConstant = 0;
	comTimeOut.WriteTotalTimeoutMultiplier = 0;
	comTimeOut.WriteTotalTimeoutConstant = 0;
	if (SetCommTimeouts(dri->handlePort, &comTimeOut) == 0) {
		mLOG(GBA_SIO, ERROR, "Set port timeouts has problem %d.", GetLastError());
		CloseHandle(dri->handlePort);
		dri->handlePort = INVALID_HANDLE_VALUE;
		return;
	}
	return;
}

void GBASIOUARTDestroy(struct GBASIOUART* dri) {
	if (dri->handlePort != INVALID_HANDLE_VALUE) {
		CloseHandle(dri->handlePort);
	}
}

bool GBASIOUARTConnect(struct GBASIOUART* dri, const char *portName) {
	mLOG(GBA_SIO, INFO, "Connecting to tty %s", portName);
	dri->handlePort = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0,
		NULL, OPEN_EXISTING, 0, NULL);
	if (dri->handlePort == INVALID_HANDLE_VALUE) {
		mLOG(GBA_SIO, ERROR, "error %d writing into tty", GetLastError());
		return false;
	}

	GBASIOUARTSetupTTY(dri);

	return GBASIOUARTIsConnected(dri);
}
	

bool GBASIOUARTIsConnected(struct GBASIOUART* dri) {
	return dri->handlePort != INVALID_HANDLE_VALUE;
}

#else

static bool GBASIOUARTWrite(struct GBASIOUART* dri, uint8_t byte) {
	int r = write(dri->fd, &byte, 1);
	if (r < 0) {
		mLOG(GBA_SIO, ERROR, "error %d writing into tty", errno);
		close(dri->fd);
		dri->fd = -1;
	} else if (r == 0) {
		mLOG(GBA_SIO, ERROR, "error, tty write would block");
	}
	return r == 1;
}

static bool GBASIOUARTRead(struct GBASIOUART* dri, uint8_t *byte) {
	int r = read(dri->fd, byte, 1);
	if (r == 1) {
		return true;
	}
	if (r < 0) {
		mLOG(GBA_SIO, ERROR, "error %d reading from tty", errno);
		close(dri->fd);
		dri->fd = -1;
		return false;
	}
	return false;
}

static void GBASIOUARTSetupTTY(struct GBASIOUART* dri) {
	int fd = dri->fd;
	struct GBASIO *sio = dri->d.p;
	struct termios tty;
	if (tcgetattr(fd, &tty) != 0) {
		mLOG(GBA_SIO, ERROR, "error %d from tcgetattr", errno);
		close(fd);
		dri->fd = -1;
		return;
	}
	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 0;
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_iflag &= ~(IGNBRK | IXON | IXOFF | IXANY);
	tty.c_cflag &= ~(CSTOPB | CRTSCTS | CSIZE | PARENB | PARODD);
	tty.c_cflag |= (CLOCAL | CREAD);

	tty.c_cflag |= sio->siocnt & SIOCNT_DATA_LENGTH ? CS8 : CS7;
	tty.c_cflag |= sio->siocnt & SIOCNT_CTS_FLAG ? CRTSCTS : 0;
	tty.c_cflag |= sio->siocnt & SIOCNT_PARITY_ENABLE_FLAG ? PARENB : 0;
	tty.c_cflag |= sio->siocnt & SIOCNT_PARITY_CONTROL ? PARODD : 0;
	speed_t const rates[4] = {B9600, B38400, B57600, B115200};
	cfsetospeed(&tty, rates[sio->siocnt & SIOCNT_BAUD_RATE]);
	cfsetispeed(&tty, rates[sio->siocnt & SIOCNT_BAUD_RATE]);

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		mLOG(GBA_SIO, ERROR, "error %d setting term attributes", errno);
		close(fd);
		dri->fd = -1;
		return;
	}
}

void GBASIOUARTDestroy(struct GBASIOUART* dri) {
	close(dri->fd);
	dri->fd = -1;
}

bool GBASIOUARTConnect(struct GBASIOUART* dri, const char *device) {
	mLOG(GBA_SIO, INFO, "Connecting to tty %s", device);
	int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
	dri->fd = fd;
	if (fd < 0) {
		mLOG(GBA_SIO, ERROR, "error %d opening %s: %s", errno, device, strerror(errno));
		return false;
	}

	GBASIOUARTSetupTTY(dri);

	return GBASIOUARTIsConnected(dri);
}

bool GBASIOUARTIsConnected(struct GBASIOUART* dri) {
	return dri->fd >= 0;
}
#endif
