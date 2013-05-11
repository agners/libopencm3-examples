#ifndef SD_H
#define SD_H

#include <stdint.h>

struct resp1 {
  int start_bit : 1;
  int transmission_bit : 1;
  int command_index : 6;
  int card_status : 32;
  int crc7 : 7;
  int end_bit : 1;
};

struct resp4 {
  uint8_t operate : 1;
  uint8_t number_of_io_functions : 1;
  uint8_t memory_present : 1;
  uint8_t stuff_bits : 3;
  uint32_t io_ocr : 24; /* Operation condition register */
};

uint32_t sd_command(uint32_t cmd, uint32_t resp, uint32_t arg);
uint32_t sd_response(uint32_t *response, uint32_t type);

#define RESP_R1       (0x01)
#define RESP_R1b      (0x02)
#define RESP_R2       (0x03)
#define RESP_R3       (0x04)
#define RESP_R6       (0x05)
#define RESP_R7       (0x06)

#define SDIO_RESP1_READY (0x1 << 31)

#define BUS_WIDTH_0 0x00
#define BUS_WIDTH_4 0x02

#define SDIO_OCR_20_21 (0x1 << 8)
#define SDIO_OCR_21_22 (0x1 << 9)
#define SDIO_OCR_22_23 (0x1 << 10)
#define SDIO_OCR_23_24 (0x1 << 11)
#define SDIO_OCR_24_25 (0x1 << 12)
#define SDIO_OCR_25_26 (0x1 << 13)
#define SDIO_OCR_26_27 (0x1 << 14)
#define SDIO_OCR_27_28 (0x1 << 15)
#define SDIO_OCR_28_29 (0x1 << 16)
#define SDIO_OCR_29_30 (0x1 << 17)
#define SDIO_OCR_30_31 (0x1 << 18)
#define SDIO_OCR_31_32 (0x1 << 19)
#define SDIO_OCR_32_33 (0x1 << 20)
#define SDIO_OCR_33_34 (0x1 << 21)
#define SDIO_OCR_34_35 (0x1 << 22)
#define SDIO_OCR_35_36 (0x1 << 23)

#define SDIO_CRDST_OUT_OF_RANGE (0x1 << 31)
#define SDIO_CRDST_ADDRESS_ERROR (0x1 << 30)
#define SDIO_CRDST_BLOCK_LEN_ERROR (0x1 << 29)
#define SDIO_CRDST_ERASE_SEQ_ERROR (0x1 << 28)
#define SDIO_CRDST_ERASE_PARAM (0x1 << 27)
#define SDIO_CRDST_WP_VIOLATION (0x1 << 26)
#define SDIO_CRDST_CARD_IS_LOCKED (0x1 << 25)
#define SDIO_CRDST_LOCK_UNLOCK_FAILED (0x1 << 24)
#define SDIO_CRDST_COM_CRC_ERROR (0x1 << 23)
#define SDIO_CRDST_ILLEGAL_COMMAND (0x1 << 22)
#define SDIO_CRDST_CARD_ECC_FAILED (0x1 << 21)
#define SDIO_CRDST_CC_ERROR (0x1 << 20)
#define SDIO_CRDST_ERROR (0x1 << 19)
#define SDIO_CRDST_UNDERRUN (0x1 << 18)
#define SDIO_CRDST_OVERRUN (0x1 << 17)
#define SDIO_CRDST_CID_CSD_OVERWRITE (0x1 << 16)
#define SDIO_CRDST_WP_ERASE_SKIP (0x1 << 15)
#define SDIO_CRDST_CARD_ECC_DISABLED (0x1 << 14)
#define SDIO_CRDST_ERASE_RESET (0x1 << 13)
#define SDIO_CRDST_CURRENT_STATE_MSK (0xf << 9)
#define SDIO_CRDST_READY_FOR_DATA (0x1 << 8)
#define SDIO_CRDST_APP_CMD (0x1 << 5)
#define SDIO_CRDST_AKE_SEQ_ERROR (0x1 << 3)






/* MMC/SD Command Index */
/* Basic command (class 0) */
#define GO_IDLE_STATE   0
#define SEND_OP_COND    1 /* reserved for SD */
#define ALL_SEND_CID    2
#define SET_RELATIVE_ADDR 3
#define SEND_RELATIVE_ADDR  3
#define SET_DSR     4
#define IO_SEND_OP_COND   5
#define SWITCH      6
#define SELECT_CARD   7
#define DESELECT_CARD   7
/* CMD8 is "SEND_EXT_CSD" for MMC4.x Spec
 * while is "SEND_IF_COND" for SD 2.0 */
#define SEND_EXT_CSD    8
#define SEND_IF_COND    8
 /* end  */
#define SEND_CSD    9
#define SEND_CID    10
#define VOLTAGE_SWITCH    11
#define READ_DAT_UTIL_STOP  11 /* reserved for SD */
#define STOP_TRANSMISSION 12
#define SEND_STATUS   13
#define GO_INACTIVE_STATE 15

 /* Block oriented read commands (class 2) */
#define SET_BLOCKLEN    16
#define READ_SINGLE_BLOCK 17
#define READ_MULTIPLE_BLOCK 18
#define SEND_TUNING_PATTERN 19

 /* Bus Width Test */
#define BUSTEST_R   14
#define BUSTEST_W   19
 /* end */

 /* Block oriented write commands (class 4) */
#define WRITE_BLOCK   24
#define WRITE_MULTIPLE_BLOCK  25
#define PROGRAM_CSD   27

 /* Erase commands */
#define ERASE_WR_BLK_START  32
#define ERASE_WR_BLK_END  33
#define ERASE_CMD   38

 /* Block Oriented Write Protection Commands */
#define LOCK_UNLOCK   42

#define IO_RW_DIRECT    52

/* Application specific commands (class 8) */
#define APP_CMD     55
#define GEN_CMD     56

/* SD Application command Index */
#define SET_BUS_WIDTH     6
#define SD_STATUS     13
#define SEND_NUM_WR_BLOCKS    22
#define SET_WR_BLK_ERASE_COUNT    23
#define SD_APP_OP_COND      41
#define SET_CLR_CARD_DETECT   42
#define SEND_SCR      51

#endif
