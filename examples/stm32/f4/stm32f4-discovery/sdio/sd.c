#include "sd.h"
#include <libopencm3/stm32/sdio.h>
#include <stdint.h>

int sd_command(uint32_t cmd, uint32_t resp, uint32_t arg)
{
	uint32_t sta;

	SDIO_ICR = (SDIO_STA_CCRCFAIL | SDIO_STA_CTIMEOUT | SDIO_STA_CMDREND | SDIO_STA_CMDSENT);

	SDIO_ARG = arg;

	cmd &= SDIO_CMD_CMDINDEX_MSK;
	if (resp & MMC_RSP_PRESENT) {
		if (resp & MMC_RSP_LONG)
			cmd |= SDIO_CMD_WAITRESP_LONG;
		else
			cmd |= SDIO_CMD_WAITRESP_SHORT;
	}

	SDIO_CMD = cmd | SDIO_CMD_CPSMEN;

	while (true) {
		sta = SDIO_STA;

		/* Wait until command is done */
		if (sta & SDIO_STA_CMDACT)
			continue;

		if (sta & SDIO_STA_CTIMEOUT)
			return ETIMEOUT;

		if (resp & MMC_RSP_PRESENT) {
			/* R3 commands succeed even when CRC is not valid */
			if (sta & SDIO_STA_CMDREND)
				break;
			else if (sta & SDIO_STA_CCRCFAIL) {
				if (resp & MMC_RSP_CRC)
					return ECRCERR;
				else
					break;
			}
		}

		if (sta & SDIO_STA_CMDSENT)
			return 0;
	}

	/* Fill Response */

	return 0;
}
/*
uint32_t sd_response(uint32_t *response, uint32_t type)
{
	if (type == RESP_R1 || type == RESP_R1b)
	{
		*response=SDIO_RESP1;

	}

	return 0;
}
*/
