/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2003 Eric Biederman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <console/cbmem_console.h>
#include <console/ne2k.h>
#include <console/qemu_debugcon.h>
#include <console/spkmodem.h>
#include <console/streams.h>
#include <console/uart.h>
#include <console/usb.h>
#include <console/spi.h>
#include <console/flash.h>
#include <rules.h>
#include <console/console.h>

void console_hw_init(void)
{
	post_code(0x80);
	__cbmemc_init();
	post_code(0x81);
	__spkmodem_init();
	post_code(0x82);
	__qemu_debugcon_init();
	post_code(0x83);
	__uart_init();
	post_code(0x84);
	__ne2k_init();
	post_code(0x85);
	__usbdebug_init();
	post_code(0x86);
	__spiconsole_init();
	post_code(0x87);
	__flashconsole_init();
	post_code(0x88);
}

void console_tx_byte(unsigned char byte)
{
	__cbmemc_tx_byte(byte);
	__spkmodem_tx_byte(byte);
	__qemu_debugcon_tx_byte(byte);

	/* Some consoles want newline conversion
	 * to keep terminals happy.
	 */
	if (byte == '\n') {
		__uart_tx_byte('\r');
		__usb_tx_byte('\r');
	}

	__uart_tx_byte(byte);
	__ne2k_tx_byte(byte);
	__usb_tx_byte(byte);
	__spiconsole_tx_byte(byte);
	__flashconsole_tx_byte(byte);
}

void console_tx_flush(void)
{
	__uart_tx_flush();
	__ne2k_tx_flush();
	__usb_tx_flush();
	__flashconsole_tx_flush();
}

void console_write_line(uint8_t *buffer, size_t number_of_bytes)
{
	/* Finish displaying all of the console data if requested */
	if (number_of_bytes == 0) {
		console_tx_flush();
		return;
	}

	/* Output the console data */
	while (number_of_bytes--)
		console_tx_byte(*buffer++);
}


#if IS_ENABLED(CONFIG_GDB_STUB) && (ENV_ROMSTAGE || ENV_RAMSTAGE)
void gdb_hw_init(void)
{
	__gdb_hw_init();
}

void gdb_tx_byte(unsigned char byte)
{
	__gdb_tx_byte(byte);
}

void gdb_tx_flush(void)
{
	__gdb_tx_flush();
}

unsigned char gdb_rx_byte(void)
{
	return __gdb_rx_byte();
}
#endif
