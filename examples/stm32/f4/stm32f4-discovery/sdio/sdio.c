/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2013 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/sdio.h>
#include <libopencm3/cm3/nvic.h>
#include "sd.h"

static void clock_setup(void)
{
	/* Enable GPIOD clock for LED & USARTs. */
  /* Enable GPIO C/D clock for SDIO */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);

	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);

}

static void sdio_setup(void)
{
  uint32_t tmp;

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
  //gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
  //gpio_set(GPIOC, GPIO12);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO12);

  /* Enable CMD (on GPIOD!) */
  gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO2);
  //gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
  //gpio_set(GPIOD, GPIO4);
  gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO2);
  gpio_set_af(GPIOD, GPIO_AF12, GPIO2);

  /* Enable SDIO clock */
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SDIOEN);

  /* Enable DMA2 clock */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_AHB1ENR_DMA2EN);

  /* Initialize DMA with <400kHz */
  tmp = (SDIO_CLKCR_CLKDIV_MSK & 0x76); //Clock=48000/(118+2)=400Khz
//  tmp = 0xEE;
  tmp |= SDIO_CLKCR_CLKEN;
//  tmp |= SDIO_CLKCR_WIDBUS_4;
  SDIO_CLKCR = tmp;

  /* Power on... */
  SDIO_POWER = SDIO_POWER_PWRCTRL_PWRON;
}

static void usart_setup(void)
{
	/* Enable the USART2 interrupt. */
//	nvic_enable_irq(NVIC_USART2_IRQ);

	/* Setup GPIO pins for USART2 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);

	/* Setup GPIO pins for USART2 receive. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO3);

	/* Setup USART2 TX and RX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO3);

	/* Enable clocks for USART2. */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);

	/* Setup USART2 parameters. */
	usart_set_baudrate(USART2, 38400);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Enable USART2 Receive interrupt. */
//	usart_enable_rx_interrupt(USART2);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO12 on GPIO port D for LED. */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
}

void printf(char* str)
{
  while(*str)
  {
    usart_send_blocking(USART2, *str);
    str++;
  }
}

void printf_bin(uint32_t test)
{
  int i = 1;
  int k;
  for(k = 0; k<32;k++)
  {
    if((i << k) & test)
      usart_send_blocking(USART2, '1');
    else
      usart_send_blocking(USART2, '0');
  }
  printf("\r\n");
}

int main(void)
{
  rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
  uint32_t flag;
  uint32_t f8 = 0;
  uint16_t rca = 0;
  uint32_t tmp;
	clock_setup();
	gpio_setup();
  sdio_setup();
	usart_setup();


  printf("Go Idle state (CMD0)...\r\n");
  sd_command(GO_IDLE_STATE, SDIO_CMD_WAITRESP_NO_0, 0);

  //printf_bin(SDIO_STA);
  printf("Send if condition (CMD8)...\r\n");
  sd_command(SEND_IF_COND, SDIO_CMD_WAITRESP_SHORT, 0x000001AA);
  if(SDIO_STA & SDIO_STA_CMDREND)
    f8 = 1;
  else if(SDIO_STA & SDIO_STA_CTIMEOUT)
    f8 = 0;
  printf_bin(f8);
  /*
  printf_bin(SDIO_STA);
  printf_bin(SDIO_RESP1);
  */

  /* Enable Application commands */
  printf("Enable application commands... (CMD55)\r\n");
  sd_command(APP_CMD, SDIO_CMD_WAITRESP_SHORT, 0);



  /* The card should now be in app cmd mode... */
  if(!(SDIO_RESP1 & SDIO_CRDST_APP_CMD) && SDIO_STA && SDIO_STA_CTIMEOUT)
    return;
  
  /* Check Operation Condition */
  printf("Check operation condition (ACMD41)...\r\n");
  sd_command(SD_APP_OP_COND, SDIO_CMD_WAITRESP_SHORT, 0);
  printf_bin(SDIO_STA);

  /* The card has to support 3.3V, if not, exit... */
  if(!(SDIO_RESP1 & SDIO_OCR_32_33))
    return;

  /* Set Operation Condition, wait until ready... */
  do {
    sd_command(APP_CMD, SDIO_CMD_WAITRESP_SHORT, 0);
   // printf_bin(SDIO_STA);
//    sd_command(SD_APP_OP_COND, SDIO_CMD_WAITRESP_SHORT, 0x80000000 | SDIO_OCR_32_33 | f8 ? 0x40000000 : 0);
    sd_command(SD_APP_OP_COND, SDIO_CMD_WAITRESP_SHORT, (uint32_t) 0x80100000 | (uint32_t) 0x40000000);
//    printf_bin(SDIO_STA);
    printf_bin(SDIO_RESP1);
    printf("-----\r\n\r\n");
  } while(SDIO_STA & SDIO_STA_CTIMEOUT || !(SDIO_RESP1 & SDIO_RESP1_READY));

  /* Get card id */
  printf("Get card id (CMD2)...\r\n");
  sd_command(ALL_SEND_CID, SDIO_CMD_WAITRESP_LONG, 0);
  printf_bin(SDIO_STA);
  printf_bin(SDIO_RESP1);
  printf_bin(SDIO_RESP2);
  printf_bin(SDIO_RESP3);
  printf_bin(SDIO_RESP4);

  sd_command(SEND_RELATIVE_ADDR, SDIO_CMD_WAITRESP_SHORT, 0);
  rca = SDIO_RESP1 >> 16;
  printf_bin(rca);

  /* Select the card... */
  sd_command(SELECT_CARD, SDIO_CMD_WAITRESP_SHORT, rca << 16);
  printf_bin(SDIO_RESP1);

  sd_command(APP_CMD, SDIO_CMD_WAITRESP_SHORT, rca << 16);

  sd_command(SET_BUS_WIDTH, SDIO_CMD_WAITRESP_SHORT, BUS_WIDTH_4);

  /* Initialize DMA with <400kHz */
  tmp = (SDIO_CLKCR_CLKDIV_MSK & 0x76); //Clock=48000/(118+2)=400Khz
  tmp |= SDIO_CLKCR_CLKEN;
  tmp |= SDIO_CLKCR_WIDBUS_4;
  SDIO_CLKCR = tmp;


	while (1) {
		__asm__("NOP");
	}

	return 0;
}

void usart2_isr(void)
{
	static u8 data = 'A';

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		gpio_toggle(GPIOD, GPIO12);

		/* Retrieve the data from the peripheral. */
		data = usart_recv(USART2);

		/* Enable transmit interrupt so it sends back the data. */
		usart_enable_tx_interrupt(USART2);
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_TXE) != 0)) {

		/* Put data into the transmit register. */
		usart_send(USART2, data);

		/* Disable the TXE interrupt as we don't need it anymore. */
		usart_disable_tx_interrupt(USART2);
	}
}
