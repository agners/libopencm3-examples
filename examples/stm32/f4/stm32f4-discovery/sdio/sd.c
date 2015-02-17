#include "sd.h"
#include <libopencm3/stm32/sdio.h>
#include <stdint.h>

int sd_command(struct sd_command *cmd)
{
	uint32_t sta, sdio_cmd;

	SDIO_ICR = (SDIO_STA_CCRCFAIL | SDIO_STA_CTIMEOUT | SDIO_STA_CMDREND | SDIO_STA_CMDSENT);

	SDIO_ARG = cmd->arg;

	sdio_cmd = cmd->opcode & SDIO_CMD_CMDINDEX_MSK;
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_LONG)
			sdio_cmd |= SDIO_CMD_WAITRESP_LONG;
		else
			sdio_cmd |= SDIO_CMD_WAITRESP_SHORT;
	}

	SDIO_CMD = sdio_cmd | SDIO_CMD_CPSMEN;

	while (true) {
		sta = SDIO_STA;

		/* Wait until command is done */
		if (sta & SDIO_STA_CMDACT)
			continue;

		if (sta & SDIO_STA_CTIMEOUT)
			return ETIMEOUT;

		if (cmd->flags & MMC_RSP_PRESENT) {
			/* R3 commands succeed even when CRC is not valid */
			if (sta & SDIO_STA_CMDREND)
				break;
			else if (sta & SDIO_STA_CCRCFAIL) {
				if (cmd->flags & MMC_RSP_CRC)
					return ECRCERR;
				else
					break;
			}
		}

		if (sta & SDIO_STA_CMDSENT)
			return 0;
	}

	/* Fill Response */
	cmd->resp[0] = SDIO_RESP1;
	if (cmd->flags & MMC_RSP_LONG) {
		cmd->resp[1] = SDIO_RESP2;
		cmd->resp[2] = SDIO_RESP3;
		cmd->resp[3] = SDIO_RESP4;
	}

	return 0;
}
