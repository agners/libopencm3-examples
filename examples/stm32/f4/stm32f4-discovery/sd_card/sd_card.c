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

#define DEBUG
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/sdio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f4/flash.h>

#include "sd.h"
#include "sd_card.h"

#ifdef DEBUG
#define DEBUG_PRINT(fmt, args...)    printf(fmt, ## args)
#else
#define DEBUG_PRINT(fmt, args...)    /* Don't do anything in release builds */
#endif


const char const *sd_types[] = {
	[SD] = "SD legancy",
	[SDV2] = "SD",
	[SDV2HC] = "SDHC",
};


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

	dma_enable_stream(DMA2, DMA_STREAM3);
}

static int sd_check_status(struct sd_card *card)
{
	struct sd_command cmd = {
		.opcode = SEND_STATUS,
		.flags = MMC_RSP_R1,
		.arg = card->rca,
	};
	enum sd_state state;

	do {
		sd_command(&cmd);
		state = (cmd.resp[0] >> 9) & 0xf;
	} while (state != SD_STATE_TRAN);

	return 0;
}

int sd_card_write_single_block(struct sd_card *card, uint8_t *buf, uint32_t blk)
{
	struct sd_command cmd = {
		.opcode = WRITE_BLOCK,
		.flags = MMC_RSP_R1,
	};

	if(card->type == SDV2HC)
		cmd.arg = blk;
	else
		cmd.arg = blk * 512;

	sd_check_status(card);

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

int sd_card_read_single_block(struct sd_card *card, uint8_t *buf, uint32_t blk)
{
	struct sd_command cmd = {
		.opcode = READ_SINGLE_BLOCK,
		.flags = MMC_RSP_R1,
	};

	if(card->type == SDV2HC)
		cmd.arg = blk;
	else
		cmd.arg = blk * 512;

	sd_check_status(card);

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

int sd_card_init(struct sd_card *card)
{
	int ret, i;
	struct sd_command cmd = {0};

	sd_setup();

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
		card->type = SD;
	} else if (!ret) {
		card->type = SDV2;
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
	if (card->type == SDV2)
		cmd.arg |= R3_CARD_CAPACITY_STATUS;

	do {
		ret = sd_app_cmd(NULL, &cmd);
	} while (ret == ETIMEOUT || !(cmd.resp[0] & R3_CARD_BUSY));


	/* If the card indicates CCS too, it's a SDHC card... */
	if (card->type == SDV2) {
		if(cmd.resp[0] & R3_CARD_CAPACITY_STATUS)
			card->type = SDV2HC;
	}

	/* Get card id */
	printf("Get card id (CMD2)...\r\n");
	cmd.opcode = ALL_SEND_CID;
	cmd.flags = MMC_RSP_R2;
	cmd.arg = 0;
	sd_command(&cmd);

	for (i = 0; i < 4; i++)
		card->cid[i] = cmd.resp[i];


	printf("Relative Addr (CMD3)...\r\n");
	cmd.opcode = SEND_RELATIVE_ADDR;
	cmd.flags = MMC_RSP_R6;
	cmd.arg = 0;
	sd_command(&cmd);
	card->rca = cmd.resp[0] & 0xFFFF0000;

	printf("Read specific information (CMD9)...\r\n");
	cmd.opcode = SEND_CSD;
	cmd.flags = MMC_RSP_R2;
	cmd.arg = card->rca;
	sd_command(&cmd);

	for (i = 0; i < 4; i++)
		card->csd[i] = cmd.resp[i];

	/* Select the card... */
	printf("Put the card in transfer mode (CMD7)...\r\n");
	cmd.opcode = SELECT_CARD;
	cmd.flags = MMC_RSP_R1B;
	cmd.arg = card->rca;
	sd_command(&cmd);

	/* Set bus width (ACMD6)... */
	cmd.opcode = SET_BUS_WIDTH;
	cmd.flags = MMC_RSP_R1;
	cmd.arg = BUS_WIDTH_4;
	sd_app_cmd(card, &cmd);

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

void sd_card_print_info(struct sd_card *card)
{

	printf("Card type... : %s\r\n", sd_types[card->type]);
	printf("CID......... : %08lx %08lx %08lx %08lx\r\n",
		card->cid[0], card->cid[1], card->cid[2], card->cid[3]);
	printf("CSD......... : %08lx %08lx %08lx %08lx\r\n",
		card->csd[0], card->csd[1], card->csd[2], card->csd[3]);

}

