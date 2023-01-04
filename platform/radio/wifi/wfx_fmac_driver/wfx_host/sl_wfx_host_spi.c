/***************************************************************************//**
 * @file sl_wfx_host_spi.c
 * @brief WFX SPI interface driver implementation
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc.  Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement.  This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "sl_wfx_configuration_defaults.h"

#ifdef  SL_WFX_USE_SPI

#include "sl_wfx.h"
#include "sl_wfx_host_api.h"
#include "sl_wfx_host_pinout.h"

#include "em_usart.h"
#include "em_cmu.h"
#include "dmadrv.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef SLEEP_ENABLED
#include "sl_power_manager.h"
#endif

#ifdef SL_CATALOG_KERNEL_PRESENT
#include "cmsis_os2.h"
#include "sl_cmsis_os2_common.h"

static osSemaphoreId_t spi_sem;
static uint8_t spi_sem_cb[osSemaphoreCbSize];
#endif

#define USART           SL_WFX_HOST_PINOUT_SPI_PERIPHERAL

static unsigned int        tx_dma_channel;
static unsigned int        rx_dma_channel;
static uint32_t            dummy_rx_data;
static uint32_t            dummy_tx_data;
static uint32_t            usart_clock;
static uint32_t            usart_rx_signal;
static uint32_t            usart_tx_signal;
static bool                spi_enabled = false;

uint8_t wirq_irq_nb = SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN;

static int sl_wfx_host_spi_set_config(void *usart)
{
  int ret = -1;

  if ( 0 ) {
#if defined(USART0)
  } else if (usart == USART0) {
    usart_clock     = cmuClock_USART0;
    usart_tx_signal = dmadrvPeripheralSignal_USART0_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USART0_RXDATAV;
    ret = 0;
#endif
#if defined(USART1)
  } else if (usart == USART1 ) {
    usart_clock     = cmuClock_USART1;
    usart_tx_signal = dmadrvPeripheralSignal_USART1_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USART1_RXDATAV;
    ret = 0;
#endif
#if defined(USART2)
  } else if (usart == USART2 ) {
    usart_clock     = cmuClock_USART2;
    usart_tx_signal = dmadrvPeripheralSignal_USART2_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USART2_RXDATAV;
    ret = 0;
#endif
#if defined(USART3)
  } else if (usart == USART3 ) {
    usart_clock     = cmuClock_USART3;
    usart_tx_signal = dmadrvPeripheralSignal_USART3_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USART3_RXDATAV;
    ret = 0;
#endif
#if defined(USART4)
  } else if (usart == USART4 ) {
    usart_clock     = cmuClock_USART4;
    usart_tx_signal = dmadrvPeripheralSignal_USART4_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USART4_RXDATAV;
    ret = 0;
#endif
#if defined(USART5)
  } else if (usart == USART5 ) {
    usart_clock     = cmuClock_USART5;
    usart_tx_signal = dmadrvPeripheralSignal_USART5_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USART5_RXDATAV;
    ret = 0;
#endif
#if defined(USARTRF0)
  } else if (usart == USARTRF0 ) {
    usart_clock     = cmuClock_USARTRF0;
    usart_tx_signal = dmadrvPeripheralSignal_USARTRF0_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USARTRF0_RXDATAV;
    ret = 0;
#endif
#if defined(USARTRF1)
  } else if (usart == USARTRF1 ) {
    usart_clock     = cmuClock_USARTRF1;
    usart_tx_signal = dmadrvPeripheralSignal_USARTRF1_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USARTRF1_RXDATAV;
    ret = 0;
#endif
  }

  return ret;
}

/**************************************************************************//**
 * Initialize SPI peripheral
 *****************************************************************************/
sl_status_t sl_wfx_host_init_bus(void)
{
  sl_status_t status = SL_STATUS_OK;
  int res;

  // Initialize and enable the USART
  USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;

  res = sl_wfx_host_spi_set_config(USART);

  if (res == 0) {
    spi_enabled = true;
    dummy_tx_data = 0;
    usartInit.baudrate = 36000000u;
    usartInit.msbf = true;

    #if defined(EFR32MG24B020F1536IM48) || defined(EFR32MG24A020F1536GM48) || defined(EFR32MG24B220F1536IM48)
      CMU_ClockEnable(cmuClock_USART0, true);
    #elif defined(EFR32MG21A020F1024IM32) || defined(EFR32MG21A010F1024IM32)
      CMU_ClockEnable(cmuClock_USART2, true);
    #else
      CMU_ClockEnable(cmuClock_HFPER, true);
    #endif

    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(usart_clock, true);
    USART_InitSync(USART, &usartInit);
    USART->CTRL |= (1u << _USART_CTRL_SMSDELAY_SHIFT);

#if defined(EFR32MG24B020F1536IM48) || defined(EFR32MG24A020F1536GM48) || defined(EFR32MG24B220F1536IM48) \
    || defined(EFR32MG21A020F1024IM32) || defined(EFR32MG21A010F1024IM32)

    GPIO->USARTROUTE[SL_WFX_HOST_PINOUT_SPI_PERIPHERAL_NO].TXROUTE = 
                                  (SL_WFX_HOST_PINOUT_SPI_TX_PORT 
                                  << _GPIO_USART_TXROUTE_PORT_SHIFT)
                                  | (SL_WFX_HOST_PINOUT_SPI_TX_PIN 
                                  << _GPIO_USART_TXROUTE_PIN_SHIFT);

    GPIO->USARTROUTE[SL_WFX_HOST_PINOUT_SPI_PERIPHERAL_NO].RXROUTE = 
                                  (SL_WFX_HOST_PINOUT_SPI_RX_PORT 
                                  << _GPIO_USART_RXROUTE_PORT_SHIFT)
                                  | (SL_WFX_HOST_PINOUT_SPI_RX_PIN 
                                  << _GPIO_USART_RXROUTE_PIN_SHIFT);

    GPIO->USARTROUTE[SL_WFX_HOST_PINOUT_SPI_PERIPHERAL_NO].CLKROUTE = 
                                  (SL_WFX_HOST_PINOUT_SPI_CLK_PORT 
                                  << _GPIO_USART_CLKROUTE_PORT_SHIFT)
                                  | (SL_WFX_HOST_PINOUT_SPI_CLK_PIN 
                                  << _GPIO_USART_CLKROUTE_PIN_SHIFT);

    GPIO->USARTROUTE[SL_WFX_HOST_PINOUT_SPI_PERIPHERAL_NO].ROUTEEN = 
                                  GPIO_USART_ROUTEEN_RXPEN  |
                                  GPIO_USART_ROUTEEN_TXPEN  |
                                  GPIO_USART_ROUTEEN_CLKPEN;

#else
    USART->ROUTELOC0 = (USART->ROUTELOC0
                          & ~(_USART_ROUTELOC0_TXLOC_MASK
                          | _USART_ROUTELOC0_RXLOC_MASK
                          | _USART_ROUTELOC0_CLKLOC_MASK))
                    | (SL_WFX_HOST_PINOUT_SPI_TX_LOC  << _USART_ROUTELOC0_TXLOC_SHIFT)
                    | (SL_WFX_HOST_PINOUT_SPI_RX_LOC  << _USART_ROUTELOC0_RXLOC_SHIFT)
                    | (SL_WFX_HOST_PINOUT_SPI_CLK_LOC << _USART_ROUTELOC0_CLKLOC_SHIFT);

    USART->ROUTEPEN = USART_ROUTEPEN_TXPEN
                    | USART_ROUTEPEN_RXPEN
                    | USART_ROUTEPEN_CLKPEN;
    GPIO_DriveStrengthSet(SL_WFX_HOST_PINOUT_SPI_CLK_PORT, gpioDriveStrengthStrongAlternateStrong);
#endif

    GPIO_PinModeSet(SL_WFX_HOST_PINOUT_SPI_TX_PORT, SL_WFX_HOST_PINOUT_SPI_TX_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(SL_WFX_HOST_PINOUT_SPI_RX_PORT, SL_WFX_HOST_PINOUT_SPI_RX_PIN, gpioModeInput, 0);
    GPIO_PinModeSet(SL_WFX_HOST_PINOUT_SPI_CLK_PORT, SL_WFX_HOST_PINOUT_SPI_CLK_PIN, gpioModePushPull, 0);

#ifdef SL_CATALOG_KERNEL_PRESENT
    osSemaphoreAttr_t sem_attr;

    sem_attr.name = "WFX SPI semaphore";
    sem_attr.cb_mem = spi_sem_cb;
    sem_attr.cb_size = osSemaphoreCbSize;
    sem_attr.attr_bits = 0;
    spi_sem = osSemaphoreNew(1, 0, &sem_attr);
    if (spi_sem == NULL) {
      status = SL_STATUS_FAIL;
    }
#endif
    DMADRV_Init();
    DMADRV_AllocateChannel(&tx_dma_channel, NULL);
    DMADRV_AllocateChannel(&rx_dma_channel, NULL);
    GPIO_PinModeSet(SL_WFX_HOST_PINOUT_SPI_CS_PORT, SL_WFX_HOST_PINOUT_SPI_CS_PIN, gpioModePushPull, 1);
    USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
  } else {
    status = SL_STATUS_FAIL;
  }
  return status;
}

/**************************************************************************//**
 * De-initialize SPI peripheral and DMAs
 *****************************************************************************/
sl_status_t sl_wfx_host_deinit_bus(void)
{
  osSemaphoreDelete(spi_sem);

  // Stop DMAs.
  DMADRV_StopTransfer(rx_dma_channel);
  DMADRV_StopTransfer(tx_dma_channel);
  DMADRV_FreeChannel(tx_dma_channel);
  DMADRV_FreeChannel(rx_dma_channel);
  DMADRV_DeInit();
  USART_Reset(USART);
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Assert chip select.
 *****************************************************************************/
sl_status_t sl_wfx_host_spi_cs_assert()
{
  GPIO_PinOutClear(SL_WFX_HOST_PINOUT_SPI_CS_PORT, SL_WFX_HOST_PINOUT_SPI_CS_PIN);
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * De-assert chip select.
 *****************************************************************************/
sl_status_t sl_wfx_host_spi_cs_deassert()
{
  GPIO_PinOutSet(SL_WFX_HOST_PINOUT_SPI_CS_PORT, SL_WFX_HOST_PINOUT_SPI_CS_PIN);
  return SL_STATUS_OK;
}

static bool rx_dma_complete(unsigned int channel,
                            unsigned int sequenceNo,
                            void *userParam)
{
  (void)channel;
  (void)sequenceNo;
  (void)userParam;

  osSemaphoreRelease(spi_sem);
  return true;
}

void receiveDMA(uint8_t* buffer, uint16_t buffer_length)
{
// Start receive DMA.
  DMADRV_PeripheralMemory(rx_dma_channel,
                          usart_rx_signal,
                          (void*)buffer,
                          (void *)&(USART->RXDATA),
                          true,
                          buffer_length,
                          dmadrvDataSize1,
                          rx_dma_complete,
                          NULL);

  // Start transmit DMA.
  DMADRV_MemoryPeripheral(tx_dma_channel,
                          usart_tx_signal,
                          (void *)&(USART->TXDATA),
                          (void *)&(dummy_tx_data),
                          false,
                          buffer_length,
                          dmadrvDataSize1,
                          NULL,
                          NULL);
}

void transmitDMA(uint8_t* buffer, uint16_t buffer_length)
{
  // Receive DMA runs only to initiate callback
  // Start receive DMA.
  DMADRV_PeripheralMemory(rx_dma_channel,
                          usart_rx_signal,
                          &dummy_rx_data,
                          (void *)&(USART->RXDATA),
                          false,
                          buffer_length,
                          dmadrvDataSize1,
                          rx_dma_complete,
                          NULL);
  // Start transmit DMA.
  DMADRV_MemoryPeripheral(tx_dma_channel,
                          usart_tx_signal,
                          (void *)&(USART->TXDATA),
                          (void*)buffer,
                          true,
                          buffer_length,
                          dmadrvDataSize1,
                          NULL,
                          NULL);
}

/**************************************************************************//**
 * WFX SPI transfer implementation
 *****************************************************************************/
sl_status_t sl_wfx_host_spi_transfer_no_cs_assert(sl_wfx_host_bus_transfer_type_t type,
                                                  uint8_t *header,
                                                  uint16_t header_length,
                                                  uint8_t *buffer,
                                                  uint16_t buffer_length)
{
  osStatus_t status = osOK;
  const bool is_read = (type == SL_WFX_BUS_READ);

  while (!(USART->STATUS & USART_STATUS_TXBL)) {
  }
  USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;

  if (header_length > 0) {
    for (uint8_t *buffer_ptr = header; header_length > 0; --header_length, ++buffer_ptr) {
      USART->TXDATA = (uint32_t)(*buffer_ptr);

      while (!(USART->STATUS & USART_STATUS_TXC)) {
      }
    }
    while (!(USART->STATUS & USART_STATUS_TXBL)) {
    }
  }
  if (buffer_length > 0) {
    USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
    if (is_read) {
      receiveDMA(buffer, buffer_length);
    } else {
      transmitDMA(buffer, buffer_length);
    }
    status = osSemaphoreAcquire(spi_sem, 1000);
  }

  if (status == osOK) {
    return SL_STATUS_OK;
  } else {
    return SL_STATUS_FAIL;
  }
}

/**************************************************************************//**
 * Enable WFX interrupt
 *****************************************************************************/
sl_status_t sl_wfx_host_enable_platform_interrupt(void)
{
  GPIO_ExtIntConfig(SL_WFX_HOST_PINOUT_SPI_WIRQ_PORT,
                    SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN,
                    wirq_irq_nb,
                    true,
                    false,
                    true);

  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Disable WFX interrupt
 *****************************************************************************/
sl_status_t sl_wfx_host_disable_platform_interrupt(void)
{
  GPIO_IntDisable(1 << wirq_irq_nb);
  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_enable_spi(void)
{
  if (spi_enabled == false) {
#ifdef SLEEP_ENABLED
    // Prevent the host to use lower EM than EM1
    sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
    spi_enabled = true;
  }
  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_disable_spi(void)
{
  if (spi_enabled == true) {
    spi_enabled = false;
#ifdef SLEEP_ENABLED
    // Allow the host to use the lowest allowed EM
    sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  }
  return SL_STATUS_OK;
}

#endif
