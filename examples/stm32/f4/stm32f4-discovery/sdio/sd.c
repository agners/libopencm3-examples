#include "sd.h"
#include <libopencm3/stm32/sdio.h>
#include <stdint.h>

uint32_t sd_command(uint32_t cmd, uint32_t resp, uint32_t arg)
{
	uint32_t sta;

	SDIO_ICR = (SDIO_STA_CCRCFAIL | SDIO_STA_CTIMEOUT | SDIO_STA_CMDREND | SDIO_STA_CMDSENT);

	SDIO_ARG = arg;

	SDIO_CMD = (cmd & SDIO_CMD_CMDINDEX_MSK) | (resp & SDIO_CMD_WAITRESP_LONG) | SDIO_CMD_CPSMEN;

	while (true) {
		sta = SDIO_STA;

		/* Wait until command is done */
		if (sta & SDIO_STA_CMDACT)
			continue;

		if (resp && sta & (SDIO_STA_CTIMEOUT | SDIO_STA_CMDREND | SDIO_STA_CCRCFAIL))
			break;
		else if (!resp && sta & (SDIO_STA_CTIMEOUT | SDIO_STA_CMDSENT))
			break;
	}

	return sta;
}

uint32_t sd_response(uint32_t *response, uint32_t type)
{
	if (type == RESP_R1 || type == RESP_R1b)
	{
		*response=SDIO_RESP1;

		/*
		if (SDIO->RESP1 & (uint32_t)0xFDFFE008) {	 //All error bits must be zero
			printf("test\n");
		}*/
	}

	return 0;
}
