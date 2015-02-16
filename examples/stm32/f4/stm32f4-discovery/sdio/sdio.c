/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2013 Stefan Agner <stefan@agner.ch>
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

#define DEBUG
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/sdio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f4/flash.h>
#include "sd.h"

#ifdef DEBUG
#define DEBUG_PRINT(fmt, args...)    printf(fmt, ## args)
#else
#define DEBUG_PRINT(fmt, args...)    /* Don't do anything in release builds */
#endif

enum sdtype {
	SD, /* SD Standard capacity (legancy) */
	SDV2, /* SD Normal capacity (V 2.00) */
	SDV2HC, /* SD High capacity (V 2.00) */
};

enum sd_state
{
	SD_STATE_UNKNOWN = -1,
	SD_STATE_IDLE    = 0,
	SD_STATE_READY   = 1,
	SD_STATE_IDENT   = 2,
	SD_STATE_STBY    = 3,
	SD_STATE_TRAN    = 4,
	SD_STATE_DATA    = 5,
	SD_STATE_RCV     = 6,
	SD_STATE_PRG     = 7,
	SD_STATE_DIS     = 8
};

struct card {
	enum sdtype type;
	uint32_t rca;
};

struct card card1;
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

static void printf_bin(uint32_t test)
{
	int i = 1;
	int k;
	for (k = 31; k>=0; k--) {
		if((i << k) & test)
			usart_send_blocking(USART3, '1');
		else
			usart_send_blocking(USART3, '0');
	}

	printf("\r\n");
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

static void sdio_setup(void)
{
	sdio_reset();

	/* Enable D0-D3 */
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO8);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO9);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO10);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO11);
	gpio_set_af(GPIOC, GPIO_AF12, GPIO8);
	gpio_set_af(GPIOC, GPIO_AF12, GPIO9);
	gpio_set_af(GPIOC, GPIO_AF12, GPIO10);
	gpio_set_af(GPIOC, GPIO_AF12, GPIO11);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO8 | GPIO9 | GPIO10 | GPIO11);

	/* Enable CLK */
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
	gpio_set_af(GPIOC, GPIO_AF12, GPIO12);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO12);

	/* Enable CMD (on GPIOD!) */
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO2);
	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO2);
	gpio_set_af(GPIOD, GPIO_AF12, GPIO2);

	/* Enable SDIO clock */
	rcc_periph_clock_enable(RCC_SDIO);

	/* Enable DMA2 clock */
	rcc_periph_clock_enable(RCC_DMA2);

	/* Initialize Clock with <400kHz */
	sdio_set_clockdiv(0xee);
	sdio_set_buswidth(SDIO_CLKCR_WIDBUS_1);
	sdio_enable_clock();

	/* Power on... */
	sdio_power_on();
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

void dma2_stream3_isr(void)
{
	if (dma_get_interrupt_flag(DMA2, DMA_STREAM3, DMA_TCIF))
	{
		DEBUG_PRINT("Stream 3 completed\r\n");
		dma_clear_interrupt_flags(DMA2, DMA_STREAM3, DMA_TCIF);
	}
	else if (dma_get_interrupt_flag(DMA2, DMA_STREAM3, DMA_HTIF))
	{
		DEBUG_PRINT("Stream 3 half\r\n");
		dma_clear_interrupt_flags(DMA2, DMA_STREAM3, DMA_HTIF);
	}
	else if (dma_get_interrupt_flag(DMA2, DMA_STREAM3, DMA_TEIF))
	{
		DEBUG_PRINT("Stream 3 error\r\n");
		dma_clear_interrupt_flags(DMA2, DMA_STREAM3, DMA_TEIF);
	}
	else if (dma_get_interrupt_flag(DMA2, DMA_STREAM3, DMA_FEIF))
	{
		DEBUG_PRINT("Stream 3 fifo error\r\n");
		dma_clear_interrupt_flags(DMA2, DMA_STREAM3, DMA_FEIF);
	}
	else if (dma_get_interrupt_flag(DMA2, DMA_STREAM3, DMA_DMEIF))
	{
		DEBUG_PRINT("Stream 3 direct memory error\r\n");
		dma_clear_interrupt_flags(DMA2, DMA_STREAM3, DMA_DMEIF);
	}
}

static void sd_start_transfer(uint8_t *buf, uint32_t dir)
{
	dma_disable_stream(DMA2, DMA_STREAM3);

	/* Enable the DMA interrupt. */
	nvic_enable_irq(NVIC_DMA2_STREAM3_IRQ);

	/* Configure DMA2 Stream 3 for use with SDIO */
	dma_stream_reset(DMA2, DMA_STREAM3);
	dma_channel_select(DMA2, DMA_STREAM3, DMA_SxCR_CHSEL_4);

	dma_set_peripheral_address(DMA2, DMA_STREAM3, (uint32_t)&SDIO_FIFO);
	dma_set_memory_address(DMA2, DMA_STREAM3, (uint32_t)buf);

	/* Control Register */
	dma_set_memory_burst(DMA2, DMA_STREAM3, DMA_SxCR_MBURST_SINGLE);
	dma_set_peripheral_burst(DMA2, DMA_STREAM3, DMA_SxCR_PBURST_INCR4);
	dma_disable_double_buffer_mode(DMA2, DMA_STREAM3);
	dma_set_priority(DMA2, DMA_STREAM3, DMA_SxCR_PL_VERY_HIGH);
	dma_set_memory_size(DMA2, DMA_STREAM3, DMA_SxCR_MSIZE_32BIT);
	dma_set_peripheral_size(DMA2, DMA_STREAM3, DMA_SxCR_PSIZE_32BIT);
	dma_enable_memory_increment_mode(DMA2, DMA_STREAM3);
	dma_disable_peripheral_increment_mode(DMA2, DMA_STREAM3);
	dma_set_peripheral_flow_control(DMA2, DMA_STREAM3);

	dma_enable_transfer_error_interrupt(DMA2, DMA_STREAM3);
	dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM3);
	dma_enable_fifo_error_interrupt(DMA2, DMA_STREAM3);

	/* FIFO Control Register */
	dma_enable_fifo_mode(DMA2, DMA_STREAM3);
	dma_set_fifo_threshold(DMA2, DMA_STREAM3, DMA_SxFCR_FTH_1_4_FULL);

	/* Direction according to parameter... */
	dma_set_transfer_mode(DMA2, DMA_STREAM3, dir);

#ifdef DEBUG
	printf("Combined DMA flags:\r\n");
	printf_bin(DMA_SCR(DMA2, DMA_STREAM3));
#endif

	dma_enable_stream(DMA2, DMA_STREAM3);
}

static int sd_check_status()
{
	enum sd_state state;
	do {
		sd_command(SEND_STATUS, SDIO_CMD_WAITRESP_SHORT, card1.rca);
		state = (SDIO_RESP1 >> 9) & 0xf;
	} while (state != SD_STATE_TRAN);

	return 0;
}

static int sd_write_single_block(uint8_t *buf, uint32_t blk)
{
	uint32_t addr, sta;

	if(card1.type == SDV2HC)
		addr = blk;
	else
		addr = blk * 512;

	sd_check_status();

	sdio_data_timeout(20000); // 20000x1MHz = 20ms

	/* Initialize DMA */
	DEBUG_PRINT("sd_start_transfer\r\n");
	sd_start_transfer(buf, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);

	/* CMD 24 */
	DEBUG_PRINT("sd_command\r\n");
	sta = sd_command(WRITE_BLOCK, SDIO_CMD_WAITRESP_SHORT, addr);
	printf_bin(sta);

	/* Start DMA transfer on SDIO pheripherial */
	DEBUG_PRINT("sdio_start_block_transfer\r\n");
	sdio_start_block_transfer(512, SDIO_DCTRL_DBLOCKSIZE_9, SDIO_DCTRL_DTDIR_CTRL_TO_CARD, true);

	while(!(SDIO_STA & SDIO_STA_DBCKEND))
	{
		if (SDIO_STA & SDIO_STA_DTIMEOUT)
                        return -1;
	}

	SDIO_ICR |= SDIO_ICR_DBCKENDC;

	return 0;
}

static int sd_read_single_block(uint8_t *buf, uint32_t blk)
{
	uint32_t addr;

	if(card1.type == SDV2HC)
		addr = blk;
	else
		addr = blk * 512;

	sd_check_status();

	sdio_data_timeout(20000); // 20000x1MHz = 20ms

	/* Initialize DMA */
	sd_start_transfer(buf, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);

	/* Start DMA transfer on SDIO pheripherial */
	sdio_start_block_transfer(512, SDIO_DCTRL_DBLOCKSIZE_9, SDIO_DCTRL_DTDIR_CARD_TO_CTRL, true);

	/* CMD 17 */
	sd_command(READ_SINGLE_BLOCK, SDIO_CMD_WAITRESP_SHORT, addr);

	printf_bin(SDIO_STA);
	while(!(SDIO_STA & SDIO_STA_DBCKEND))
	{
		if (SDIO_STA & SDIO_STA_DTIMEOUT)
                        return -1;
	}
	printf_bin(SDIO_STA);
	SDIO_ICR |= SDIO_ICR_DBCKENDC;

	return 0;
}

static void sdio_init(void)
{
	uint32_t hcs, sta;

	printf("Go Idle state (CMD0)...\r\n");
	sta = sd_command(GO_IDLE_STATE, SDIO_CMD_WAITRESP_NO_0, 0);
	printf_bin(sta);

	//printf_bin(SDIO_STA);
	printf("Send if condition (CMD8)...\r\n");
	sta = sd_command(SEND_IF_COND, SDIO_CMD_WAITRESP_SHORT, 0x000001AA);

	/* Legancy SD if this command does not answer... */
	if (sta & SDIO_STA_CTIMEOUT) {
		printf("Card type: SD\n");
		card1.type = SD;
	} else if(sta & SDIO_STA_CMDREND) {
		printf("Card type: SDV2 or higher\n");
		/* We set Rsv too, seems to be needed on some cards */
		hcs = 0x40000000; // | 0x80000000;
		card1.type = SDV2;
	} else {
		return;
	}

	/* Enable Application commands */
	printf("Enable application commands... (CMD55)\r\n");
	sta = sd_command(APP_CMD, SDIO_CMD_WAITRESP_SHORT, 0);
	printf_bin(sta);


	/* The card should now be in app cmd mode... */
	if(!(SDIO_RESP1 & SDIO_CRDST_APP_CMD) && SDIO_STA && SDIO_STA_CTIMEOUT)
		return;

	/* Operation Condition, ignore CRC since ACMD41 is R3 type => no CRC */
	sta = sd_command(SD_APP_OP_COND, SDIO_CMD_WAITRESP_SHORT, 0);

	/* The card has to support 3.3V, if not, exit... */
	if(!(SDIO_RESP1 & SDIO_OCR_32_33)) {
		printf("No 3.3V support announced by card!\r\n");
		return;
	}

	printf("Set operation condition\r\n");
	/* Set Operation Condition, wait until ready... */
	do {
		/* Application command */
		sta = sd_command(APP_CMD, SDIO_CMD_WAITRESP_SHORT, 0);

		/* Sent HCS Flag if SDV2! */
		sta = sd_command(SD_APP_OP_COND, SDIO_CMD_WAITRESP_SHORT, hcs | (uint32_t)SDIO_OCR_32_33);
	} while (sta & SDIO_STA_CTIMEOUT || !(SDIO_RESP1 & SDIO_RESP1_READY));


	if (card1.type == SDV2) {
		if(SDIO_RESP1 & 0x40000000)
			card1.type = SDV2HC;
	}

	switch (card1.type) {
	case SD:
		printf("Card type is SD\n\r");
		break;
	case SDV2:
		printf("Card type is SDV2\n\r");
		break;
	case SDV2HC:
		printf("Card type is SDHC\n\r");
		break;
	}

	/* Get card id */
	printf("Get card id (CMD2)...\r\n");
	sd_command(ALL_SEND_CID, SDIO_CMD_WAITRESP_LONG, 0);


	printf("Relative Addr (CMD3)...\r\n");
	sd_command(SEND_RELATIVE_ADDR, SDIO_CMD_WAITRESP_SHORT, 0);
	card1.rca = SDIO_RESP1 & 0xFFFF0000;

	printf("Read specific information (CMD9)...\r\n");
	sd_command(SEND_CSD, SDIO_CMD_WAITRESP_LONG, card1.rca);

	/* Select the card... */
	printf("Put the card in transfer mode (CMD7)...\r\n");
	sd_command(SELECT_CARD, SDIO_CMD_WAITRESP_SHORT, card1.rca);

	sd_command(APP_CMD, SDIO_CMD_WAITRESP_SHORT, card1.rca);

	/* Set bus width (ACMD6)... */
	sd_command(SET_BUS_WIDTH, SDIO_CMD_WAITRESP_SHORT, BUS_WIDTH_4);

	/* Raise Clock with to 1MHz */
	sdio_set_clockdiv(0x2E); //Clock=48000/(46+2)=1MHz
	sdio_set_buswidth(SDIO_CLKCR_WIDBUS_4);
	sdio_enable_clock();

	/* Set block len (CMD16) */
	sd_command(SET_BLOCKLEN, SDIO_CMD_WAITRESP_SHORT, 512);
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

static int sdio_read_write_test(int blocknbr)
{
	int i;
	uint8_t data = 0;

	printf("Read single block...\r\n");
	if (sd_read_single_block(block, blocknbr)) {
		printf("Read single block timed out!\r\n");
			return -1;
	}

	printf("Read finished...\r\n");

	printf("The data:\r\n");
	print_buffer(block, 512);

	printf("Write single block...\r\n");
	for (i = 0; i < 512; i++)
		block[i] = data++;

	sd_write_single_block(block, blocknbr);

	return 0;
}

int main(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

	clock_setup();
	gpio_setup();
	sdio_setup();
	usart_setup();

	sdio_init();

	sdio_read_write_test(0);

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
