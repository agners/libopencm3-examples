#include "sd.h"
#include <libopencm3/stm32/sdio.h>
#include <stdint.h>

uint32_t sd_command(uint32_t cmd, uint32_t resp, uint32_t arg)
{
	SDIO_ICR = (SDIO_STA_CCRCFAIL | SDIO_STA_CTIMEOUT | SDIO_STA_CMDREND | SDIO_STA_CMDSENT);

	SDIO_ARG = arg;

	SDIO_CMD = (cmd & SDIO_CMD_CMDINDEX_MSK) | (resp & SDIO_CMD_WAITRESP_LONG) | SDIO_CMD_CPSMEN;

	if (resp==0x00) {
		while(!(SDIO_STA & (SDIO_STA_CTIMEOUT | SDIO_STA_CMDSENT))) {};
	}
	else
	{
		while (!(SDIO_STA & (SDIO_STA_CTIMEOUT | SDIO_STA_CMDREND | SDIO_STA_CCRCFAIL))) {};
	}

	return SDIO_STA;
}

uint32_t sd_response(uint32_t *response, uint32_t type)
{
	if(type == RESP_R1 || type == RESP_R1b)
	{
		*response=SDIO_RESP1;

		/*
		if (SDIO->RESP1 & (uint32_t)0xFDFFE008) {	 //All error bits must be zero
			printf("test\n");
		}*/
	}

}


