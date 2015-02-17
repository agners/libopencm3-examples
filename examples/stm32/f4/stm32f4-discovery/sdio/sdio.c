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

enum sd_type {
	SD, /* SD Standard capacity (legancy) */
	SDV2, /* SD Normal capacity (V 2.00) */
	SDV2HC, /* SD High capacity (V 2.00) */
};

const char const *sd_types[] = {
	[SD] = "SD legancy",
	[SDV2] = "SD",
	[SDV2HC] = "SDHC",
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

struct sd_card {
	enum sd_type type;
	uint32_t rca;
	uint32_t csd[4];
	uint32_t cid[4];
};

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

static int sd_check_status(void)
{
	struct sd_command cmd = {
		.opcode = SEND_STATUS,
		.flags = MMC_RSP_R1,
		.arg = card1.rca,
	};
	enum sd_state state;

	do {
		sd_command(&cmd);
		state = (cmd.resp[0] >> 9) & 0xf;
	} while (state != SD_STATE_TRAN);

	return 0;
}

static int sd_write_single_block(uint8_t *buf, uint32_t blk)
{
	struct sd_command cmd = {
		.opcode = WRITE_BLOCK,
		.flags = MMC_RSP_R1,
	};

	if(card1.type == SDV2HC)
		cmd.arg = blk;
	else
		cmd.arg = blk * 512;

	sd_check_status();

	sdio_data_timeout(20000); // 20000x1MHz = 20ms

	/* Initialize DMA */
	DEBUG_PRINT("sd_start_transfer\r\n");
	sd_start_transfer(buf, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);

	/* CMD 24 */
	DEBUG_PRINT("sd_command\r\n");
	sd_command(&cmd);

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
	struct sd_command cmd = {
		.opcode = READ_SINGLE_BLOCK,
		.flags = MMC_RSP_R1,
	};

	if(card1.type == SDV2HC)
		cmd.arg = blk;
	else
		cmd.arg = blk * 512;

	sd_check_status();

	sdio_data_timeout(20000); // 20000x1MHz = 20ms

	/* Initialize DMA */
	sd_start_transfer(buf, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);

	/* Start DMA transfer on SDIO pheripherial */
	sdio_start_block_transfer(512, SDIO_DCTRL_DBLOCKSIZE_9, SDIO_DCTRL_DTDIR_CARD_TO_CTRL, true);

	/* CMD 17 */
	sd_command(&cmd);

	while(!(SDIO_STA & SDIO_STA_DBCKEND))
	{
		if (SDIO_STA & SDIO_STA_DTIMEOUT)
                        return -1;
	}
	printf_bin(SDIO_STA);
	SDIO_ICR |= SDIO_ICR_DBCKENDC;

	return 0;
}

static int sd_app_cmd(struct sd_card *card, struct sd_command *cmd)
{
	int ret;
	static struct sd_command cmdapp = {
		.opcode = APP_CMD,
		.flags = MMC_RSP_R1,
		.arg = 0,
	};

	/* Enable Application commands */
	if (card)
		cmdapp.arg = card->rca;
	ret = sd_command(&cmdapp);
	if (ret)
		return ret;

	/* The card should now be in app cmd mode... */
	if(!(cmdapp.resp[0] & R1_APP_CMD)) {
		printf("Timed out changing to application mode\n");
		return ENOAPPMODE;
	}

	/* Get operation condition */
	sd_command(cmd);

	return 0;
}

static int sd_card_init(void)
{
	int ret, i;
	struct sd_command cmd = {0};

	printf("Go Idle state (CMD0)...\r\n");
	cmd.opcode = GO_IDLE_STATE;
	cmd.flags = MMC_RSP_NONE;
	sd_command(&cmd);

	printf("Send if condition (CMD8)...\r\n");
	cmd.opcode = SEND_IF_COND;
	cmd.flags = MMC_RSP_R7;
	cmd.arg = 0x000001AA;
	ret = sd_command(&cmd);

	/* No or Legancy SD if this command does not answer... */
	if (ret == ETIMEOUT) {
		card1.type = SD;
	} else if (!ret) {
		card1.type = SDV2;
	} else {
		printf("Error on if condition\n");
		return ret;
	}

	/* Get operation condition */
	cmd.opcode = SD_APP_OP_COND;
	cmd.flags = MMC_RSP_R3;
	cmd.arg = 0;
	sd_app_cmd(NULL, &cmd);

	/* The card has to support 3.3V, if not, exit... */
	if(!(cmd.resp[0] & SDIO_OCR_32_33)) {
		printf("No 3.3V support announced by card!\r\n");
		return EOPNOTSUPPORTED;
	}

	/* Set Operation Condition, wait until ready... */
	printf("Set operation condition\r\n");
	cmd.opcode = SD_APP_OP_COND;
	cmd.flags = MMC_RSP_R3;
	cmd.arg = SDIO_OCR_32_33;

	/* If SD V2, indicate that we can handle SDHC card... */
	if (card1.type == SDV2)
		cmd.arg |= R3_CARD_CAPACITY_STATUS;

	do {
		ret = sd_app_cmd(NULL, &cmd);
	} while (ret == ETIMEOUT || !(cmd.resp[0] & R3_CARD_BUSY));


	/* If the card indicates CCS too, it's a SDHC card... */
	if (card1.type == SDV2) {
		if(cmd.resp[0] & R3_CARD_CAPACITY_STATUS)
			card1.type = SDV2HC;
	}

	/* Get card id */
	printf("Get card id (CMD2)...\r\n");
	cmd.opcode = ALL_SEND_CID;
	cmd.flags = MMC_RSP_R2;
	cmd.arg = 0;
	sd_command(&cmd);

	for (i = 0; i < 4; i++)
		card1.cid[i] = cmd.resp[i];


	printf("Relative Addr (CMD3)...\r\n");
	cmd.opcode = SEND_RELATIVE_ADDR;
	cmd.flags = MMC_RSP_R6;
	cmd.arg = 0;
	sd_command(&cmd);
	card1.rca = cmd.resp[0] & 0xFFFF0000;

	printf("Read specific information (CMD9)...\r\n");
	cmd.opcode = SEND_CSD;
	cmd.flags = MMC_RSP_R2;
	cmd.arg = card1.rca;
	sd_command(&cmd);

	for (i = 0; i < 4; i++)
		card1.csd[i] = cmd.resp[i];

	/* Select the card... */
	printf("Put the card in transfer mode (CMD7)...\r\n");
	cmd.opcode = SELECT_CARD;
	cmd.flags = MMC_RSP_R1B;
	cmd.arg = card1.rca;
	sd_command(&cmd);

	/* Set bus width (ACMD6)... */
	cmd.opcode = SET_BUS_WIDTH;
	cmd.flags = MMC_RSP_R1;
	cmd.arg = BUS_WIDTH_4;
	sd_app_cmd(&card1, &cmd);

	/* Raise Clock with to 1MHz */
	sdio_set_clockdiv(0x2E); //Clock=48000/(46+2)=1MHz
	sdio_set_buswidth(SDIO_CLKCR_WIDBUS_4);
	sdio_enable_clock();

	/* Set block len (CMD16) */
	cmd.opcode = SET_BLOCKLEN;
	cmd.flags = MMC_RSP_R1;
	cmd.arg = 512;
	sd_command(&cmd);

	return 0;
}

static void print_card_info(struct sd_card *card)
{

	printf("Card type... : %s\r\n", sd_types[card->type]);
	printf("CID......... : %08lx %08lx %08lx %08lx\r\n",
		card->cid[0], card->cid[1], card->cid[2], card->cid[3]);
	printf("CSD......... : %08lx %08lx %08lx %08lx\r\n",
		card->csd[0], card->csd[1], card->csd[2], card->csd[3]);

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

	printf("The data:\r\n");
	print_buffer(block, 512);

	printf("Write single block...\r\n");
	for (i = 0; i < 512; i++)
		block[i] = data++;

	//sd_write_single_block(block, blocknbr);

	return 0;
}

int main(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

	clock_setup();
	gpio_setup();
	sdio_setup();
	usart_setup();

	if (!sd_card_init()) {
		print_card_info(&card1);
		sdio_read_write_test(0);
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
