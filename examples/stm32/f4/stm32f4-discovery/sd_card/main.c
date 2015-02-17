/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2013-2015 Stefan Agner <stefan@agner.ch>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <errno.h>

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include "sd_card.h"

struct sd_card card1;
uint8_t block[512];

int _write(int file, char *ptr, int len);

int _write(int file, char *ptr, int len)
{
	int i;

	if (file == 1) {
		for (i = 0; i < len; i++)
			usart_send_blocking(USART3, ptr[i]);
		return i;
	}

	errno = EIO;
	return -1;
}

static void clock_setup(void)
{
	rcc_periph_clock_enable(RCC_PWR);

	/* Enable GPIO A/B clock for LED & USARTs. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable GPIO C/D clock for SDIO */
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);
}

static void usart_setup(void)
{
	/* Enable clocks for USART3. */
	rcc_periph_clock_enable(RCC_USART3);

	/* Enable the USART3 interrupt. */
	//nvic_enable_irq(NVIC_USART3_IRQ);

	/* Setup GPIO pins for USART3 transmit. */
        gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO10);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);

	/* Setup GPIO pins for USART3 receive. */
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO11);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);

	/* Setup USART3 TX and RX pin as alternate function. */
	gpio_set_af(GPIOB, GPIO_AF7, GPIO10 | GPIO11);

	/* Setup USART3 parameters. */
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	/* Enable USART3 Receive interrupt. */
//	usart_enable_rx_interrupt(USART3);

	/* Finally enable the USART. */
	usart_enable(USART3);
}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO12 on GPIO port D for LED. */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
}

static void print_buffer(uint8_t *buffer, int size)
{
	int i, j, k;
	for (i = 0; i < size / 32; i++)
	{
		for (j = 0; j < 8; j++) {
			for (k = 0; k < 4; k++)
				printf("%02x", buffer[i * 32 + j * 4 + k]);
			printf(" ");
		}
		printf("\r\n");
	}
}

static int sd_read_write_test(struct sd_card *card, int blocknbr)
{
	int i;
	uint8_t data = 0;

	printf("Read single block...\r\n");
	if (sd_card_read_single_block(card, block, blocknbr)) {
		printf("Read single block timed out!\r\n");
			return -1;
	}

	printf("The data:\r\n");
	print_buffer(block, 512);

	printf("Write single block...\r\n");
	for (i = 0; i < 512; i++)
		block[i] = data++;

	//sd_write_single_block(card, block, blocknbr);

	return 0;
}

int main(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

	clock_setup();
	gpio_setup();
	usart_setup();

	if (!sd_card_init(&card1)) {
		sd_card_print_info(&card1);
		sd_read_write_test(&card1, 0);
	}

	while (1) {
		__asm__("NOP");
	}
}

void usart2_isr(void)
{
	static uint8_t data = 'A';

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
			((USART_SR(USART3) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		gpio_toggle(GPIOD, GPIO12);

		/* Retrieve the data from the peripheral. */
		data = usart_recv(USART3);

		/* Enable transmit interrupt so it sends back the data. */
		usart_enable_tx_interrupt(USART3);
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART3) & USART_CR1_TXEIE) != 0) &&
			((USART_SR(USART3) & USART_SR_TXE) != 0)) {

		/* Put data into the transmit register. */
		usart_send(USART3, data);

		/* Disable the TXE interrupt as we don't need it anymore. */
		usart_disable_tx_interrupt(USART3);
	}

        return;
}
