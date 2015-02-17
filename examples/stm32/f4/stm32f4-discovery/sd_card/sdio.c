#include "sd.h"
#include <libopencm3/stm32/sdio.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <stdint.h>

void sd_setup(void)
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
