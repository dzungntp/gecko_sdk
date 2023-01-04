/***************************************************************************//**
 * @file sl_wfx_host.c
 * @brief WFX FMAC driver host implementation
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "em_gpio.h"
#include "sl_wfx.h"
#include "sl_wfx_host_pinout.h"

// File specific to each platform, it must be created for custom boards
#include "sl_wfx_pds.h"

/* Firmware include */
#include "sl_wfx_wf200_C0.h"

#include "sl_wfx_task.h"
#include "sl_wfx_host.h"
#include "sli_mem_pool.h"

#define SL_WFX_HOST_BUFFER_SIZE   1616
#define SL_WFX_HOST_BUFFER_NB     10

#ifdef SL_CATALOG_KERNEL_PRESENT
#include "cmsis_os2.h"
#include "sl_cmsis_os2_common.h"

osSemaphoreId_t sl_wfx_confirmation;
osSemaphoreId_t sl_wfx_wakeup_sem;
static osMutexId_t sl_wfx_mutex;

static uint8_t sl_wfx_confirmation_cb[osSemaphoreCbSize];
static uint8_t sl_wfx_wakeup_sem_cb[osSemaphoreCbSize];
static uint8_t sl_wfx_mutex_cb[osMutexCbSize];
#endif

SLI_MEM_POOL_DECLARE_BUFFER(sl_wfx_mem_pool, SL_WFX_HOST_BUFFER_SIZE, SL_WFX_HOST_BUFFER_NB);

struct {
  uint32_t wf200_firmware_download_progress;
  sli_mem_pool_handle_t buf_pool;
  int wf200_initialized;
  uint8_t waited_event_id;
  uint8_t posted_event_id;
} host_context;


#ifdef SL_WFX_USE_SDIO
#ifdef SLEEP_ENABLED
sl_status_t sl_wfx_host_enable_sdio (void);
sl_status_t sl_wfx_host_disable_sdio (void);
#endif
#endif

#ifdef SL_WFX_USE_SPI
#ifdef SLEEP_ENABLED
sl_status_t sl_wfx_host_enable_spi (void);
sl_status_t sl_wfx_host_disable_spi (void);
#endif
#endif

/**************************************************************************//**
 * Set up memory pools for WFX host interface
 *****************************************************************************/
sl_status_t sl_wfx_host_setup_memory_pools(void)
{
  sli_mem_pool_create(&host_context.buf_pool,
                     SL_WFX_HOST_BUFFER_SIZE,
                     SL_WFX_HOST_BUFFER_NB,
                     sl_wfx_mem_pool_buffer,
                     sizeof(sl_wfx_mem_pool_buffer));

  return SL_STATUS_OK;
}

/**************************************************************************//**
 * WFX FMAC driver host interface initialization
 *****************************************************************************/
sl_status_t sl_wfx_host_init(void)
{
  host_context.wf200_firmware_download_progress = 0;
  host_context.wf200_initialized = 0;

#ifdef SL_CATALOG_KERNEL_PRESENT
  osSemaphoreAttr_t sem_attr;
  osMutexAttr_t     mutex_attr;

  sem_attr.name = "WFX confirmation";
  sem_attr.cb_mem = sl_wfx_confirmation_cb;
  sem_attr.cb_size = osSemaphoreCbSize;
  sem_attr.attr_bits = 0;
  sl_wfx_confirmation = osSemaphoreNew(1, 0, &sem_attr);

  sem_attr.name = "WFX wakeup";
  sem_attr.cb_mem = sl_wfx_wakeup_sem_cb;
  sl_wfx_wakeup_sem = osSemaphoreNew(1, 0, &sem_attr);

  mutex_attr.name = "WFX host mutex";
  mutex_attr.cb_mem = sl_wfx_mutex_cb;
  mutex_attr.cb_size = osMutexCbSize;
  mutex_attr.attr_bits = 0;
  sl_wfx_mutex = osMutexNew(&mutex_attr);

  if ((sl_wfx_confirmation == NULL)
      || (sl_wfx_wakeup_sem == NULL)
      || (sl_wfx_mutex == NULL)) {
    printf("WFX Host init error\r\n");
    return SL_STATUS_FAIL;
  }
#endif

  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Get firmware data
 *****************************************************************************/
sl_status_t sl_wfx_host_get_firmware_data(const uint8_t** data, uint32_t data_size)
{
  *data = &sl_wfx_firmware[host_context.wf200_firmware_download_progress];
  host_context.wf200_firmware_download_progress += data_size;
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Get firmware size
 *****************************************************************************/
sl_status_t sl_wfx_host_get_firmware_size(uint32_t* firmware_size)
{
  *firmware_size = sizeof(sl_wfx_firmware);
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Get PDS data
 *****************************************************************************/
sl_status_t sl_wfx_host_get_pds_data(const char **pds_data, uint16_t index)
{
  *pds_data = sl_wfx_pds[index];
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Get PDS size
 *****************************************************************************/
sl_status_t sl_wfx_host_get_pds_size(uint16_t *pds_size)
{
  *pds_size = SL_WFX_ARRAY_COUNT(sl_wfx_pds);
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Deinit host interface
 *****************************************************************************/
sl_status_t sl_wfx_host_deinit(void)
{
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Allocate buffer
 *****************************************************************************/
sl_status_t sl_wfx_host_allocate_buffer(void **buffer,
                                        sl_wfx_buffer_type_t type,
                                        uint32_t buffer_size)
{
  (void)type;
  (void)buffer_size;

  *buffer = sli_mem_pool_alloc(&host_context.buf_pool);
  if (*buffer == NULL) {
#ifdef DEBUG
    printf("WFX Host buffer allocation error\r\n");
#endif
    return SL_STATUS_ALLOCATION_FAILED;
  }

  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Free host buffer
 *****************************************************************************/
sl_status_t sl_wfx_host_free_buffer(void* buffer, sl_wfx_buffer_type_t type)
{
  (void)type;
  sli_mem_pool_free(&host_context.buf_pool, buffer);

  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Set reset pin low
 *****************************************************************************/
sl_status_t sl_wfx_host_hold_in_reset(void)
{
  GPIO_PinOutClear(SL_WFX_HOST_PINOUT_RESET_PORT, SL_WFX_HOST_PINOUT_RESET_PIN);
  host_context.wf200_initialized = 0;
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Set wakeup pin status
 *****************************************************************************/
sl_status_t sl_wfx_host_set_wake_up_pin(uint8_t state)
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_ATOMIC();
  if ( state > 0 ) {
#ifdef SLEEP_ENABLED
#ifdef SL_WFX_USE_SDIO
    sl_wfx_host_enable_sdio();
#endif
#ifdef SL_WFX_USE_SPI
    sl_wfx_host_enable_spi();
#endif
#endif
    GPIO_PinOutSet(SL_WFX_HOST_PINOUT_WUP_PORT, SL_WFX_HOST_PINOUT_WUP_PIN);
  } else {
    GPIO_PinOutClear(SL_WFX_HOST_PINOUT_WUP_PORT, SL_WFX_HOST_PINOUT_WUP_PIN);
#ifdef SLEEP_ENABLED
#ifdef SL_WFX_USE_SDIO
    sl_wfx_host_disable_sdio();
#endif
#ifdef SL_WFX_USE_SPI
    sl_wfx_host_disable_spi();
#endif
#endif
  }
  CORE_EXIT_ATOMIC();
  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_reset_chip(void)
{
  // Pull it low for at least 1 ms to issue a reset sequence
  GPIO_PinOutClear(SL_WFX_HOST_PINOUT_RESET_PORT, SL_WFX_HOST_PINOUT_RESET_PIN);
  // Delay for 10ms
  sl_wfx_host_wait(10);
  // Hold pin high to get chip out of reset
  GPIO_PinOutSet(SL_WFX_HOST_PINOUT_RESET_PORT, SL_WFX_HOST_PINOUT_RESET_PIN);
  // Delay for 3ms
  sl_wfx_host_wait(3);

  host_context.wf200_initialized = 0;
  return SL_STATUS_OK;
}

#ifdef SL_CATALOG_KERNEL_PRESENT
sl_status_t sl_wfx_host_wait_for_wake_up(void)
{
  osStatus_t status;

  status = osSemaphoreAcquire(sl_wfx_wakeup_sem, 3);
  if (status != osOK)
  {
    return SL_STATUS_FAIL;
  }

  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_wait(uint32_t wait_time)
{
  osStatus_t status;

  status = osDelay(wait_time);
  if (status != osOK)
  {
    return SL_STATUS_FAIL;
  }

  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_wait_for_confirmation(uint8_t confirmation_id, uint32_t timeout, void** event_payload_out)
{
  osStatus_t  status;

  while (timeout > 0u) {
    timeout--;
    status = osSemaphoreAcquire(sl_wfx_confirmation, 1);
    if (status == osOK) {
      if (confirmation_id == host_context.posted_event_id) {
        if ( event_payload_out != NULL ) {
          *event_payload_out = sl_wfx_context->event_payload_buffer;
        }

        return SL_STATUS_OK;
      }
    } else if (status != osErrorTimeout) {
      printf("OS error: sl_wfx_host_wait_for_confirmation\r\n");
    }
  }

  return SL_STATUS_TIMEOUT;
}

/**************************************************************************//**
 * Called when the driver needs to lock its access
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_lock(void)
{
  osStatus_t status;

  status = osMutexAcquire(sl_wfx_mutex, 5000);
  if (status != osOK)
  {
    printf("WFX Host lock error (%d)\r\n", status);
    return SL_STATUS_FAIL;
  }

  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Called when the driver needs to unlock its access
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_unlock(void)
{
  osStatus_t status;

  status = osMutexRelease(sl_wfx_mutex);
  if (status != osOK)
  {
    printf("WFX Host unlock error (%d)\r\n", status);
    return SL_STATUS_FAIL;
  }

  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Called when the driver needs to post an event
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_post_event(sl_wfx_generic_message_t *event_payload)
{
  sl_status_t status;

#ifdef SL_WFX_USE_SECURE_LINK
  /* Execute actions not specific to the application */
  switch (event_payload->header.id) {
    case SL_WFX_SECURELINK_EXCHANGE_PUB_KEYS_IND_ID:
    {
      if (host_context.waited_event_id != SL_WFX_SECURELINK_EXCHANGE_PUB_KEYS_IND_ID) {
        memcpy((void*)&sl_wfx_context->secure_link_exchange_ind, (void*)event_payload, event_payload->header.length);
      }
      break;
    }
  }
#endif

  /* Forward the message to the application */
  status = sl_wfx_host_process_event(event_payload);

  if (host_context.waited_event_id == event_payload->header.id) {
    /* Post the event in the queue */
    memcpy(sl_wfx_context->event_payload_buffer,
           (void*) event_payload,
           event_payload->header.length);
    host_context.posted_event_id = event_payload->header.id;
    osSemaphoreRelease(sl_wfx_confirmation);
  }

  return status;
}
#endif /* SL_CATALOG_KERNEL_PRESENT */

sl_status_t sl_wfx_host_setup_waited_event(uint8_t event_id)
{
  host_context.waited_event_id = event_id;
  host_context.posted_event_id = 0;

  return SL_STATUS_OK;
}

uint8_t sl_wfx_host_get_waited_event(void)
{
  return host_context.waited_event_id;
}

/**************************************************************************//**
 * Function forwarding the messages to the application in order to process them.
 * Note: the application is in charge of redefining this function.
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
WEAK sl_status_t sl_wfx_host_process_event(sl_wfx_generic_message_t *event_payload)
{
  (void)event_payload;

  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Called when the driver needs to transmit a frame
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_transmit_frame(void* frame, uint32_t frame_len)
{
  return sl_wfx_data_write(frame, frame_len);
}

/**************************************************************************//**
 * Called when the driver is considering putting the WFx in
 * sleep mode
 * @returns SL_WIFI_SLEEP_GRANTED to let the WFx go to sleep,
 * SL_WIFI_SLEEP_NOT_GRANTED otherwise
 *****************************************************************************/
WEAK sl_status_t sl_wfx_host_sleep_grant(sl_wfx_host_bus_transfer_type_t type,
                                         sl_wfx_register_address_t address,
                                         uint32_t length)
{
  (void)(type);
  (void)(address);
  (void)(length);

  return SL_STATUS_WIFI_SLEEP_GRANTED;
}

#if SL_WFX_DEBUG_MASK
void sl_wfx_host_log(const char *string, ...)
{
  va_list valist;

  va_start(valist, string);
  vprintf(string, valist);
  va_end(valist);
}
#endif
