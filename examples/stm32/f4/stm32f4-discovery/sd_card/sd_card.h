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

#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdint.h>

enum sd_type {
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

struct sd_card {
	enum sd_type type;
	uint32_t rca;
	uint32_t csd[4];
	uint32_t cid[4];
};

int sd_card_init(struct sd_card *card);
void sd_card_print_info(struct sd_card *card);
int sd_card_read_single_block(struct sd_card *card, uint8_t *buf, uint32_t blk);
int sd_card_write_single_block(struct sd_card *card, uint8_t *buf, uint32_t blk);

#endif /* SD_CARD_H */
