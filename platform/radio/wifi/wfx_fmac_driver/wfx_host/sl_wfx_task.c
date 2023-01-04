/***************************************************************************//**
 * @file sl_wfx_task.c
 * @brief WFX FMAC driver main bus communication task
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

#include "em_gpio.h"

#include "sl_wfx.h"
#include "sl_wfx_host_pinout.h"
#include "sl_wfx_task.h"
#include "sl_wfx_host.h"

#include "cmsis_os2.h"
#include "sl_cmsis_os2_common.h"

#define SL_WFX_BUS_TASK_PRIO             osPriorityRealtime
#define SL_WFX_BUS_STACK_SIZE            1024u

__ALIGNED(8) static uint8_t sl_wfx_bus_stack[(SL_WFX_BUS_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u];
__ALIGNED(4) static uint8_t sl_wfx_bus_task_cb[osThreadCbSize];

osMutexId_t               sl_wfx_tx_queue_mutex;
sl_wfx_packet_queue_t     sl_wfx_tx_queue_context;
osEventFlagsId_t          sl_wfx_bus_events;
sl_wfx_frame_q_item       sl_wfx_bus_tx_frame;
osSemaphoreId_t           sl_wfx_bus_tx_complete;

/// Flag to indicate receive frames is currently running.
static uint8_t  sl_wfx_bus_events_cb[osEventFlagsCbSize];
static uint8_t  sl_wfx_bus_tx_complete_cb[osSemaphoreCbSize];
static uint8_t  sl_wfx_bus_queue_mutex_cb[osMutexCbSize];

static void        sl_wfx_task_entry (void *p_arg);
static sl_status_t sl_wfx_rx_process (uint16_t control_register);
static sl_status_t sl_wfx_tx_process (void);

/**************************************************************************//**
 * Wfx process task entry
 *****************************************************************************/
static void sl_wfx_task_entry(void *p_arg)
{
  uint16_t control_register = 0;
  uint32_t flags = 0u;

  (void)p_arg;

  sl_wfx_host_setup_memory_pools();

  while (1) {
#ifdef SLEEP_ENABLED
#ifdef SL_WFX_USE_SPI
    if (GPIO_PinInGet(SL_WFX_HOST_PINOUT_SPI_WIRQ_PORT, SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN)) //wfx messages pending
#else
    if (GPIO_PinInGet(SL_WFX_HOST_PINOUT_GPIO_WIRQ_PORT, SL_WFX_HOST_PINOUT_GPIO_WIRQ_PIN))
#endif
    {
      osEventFlagsSet(sl_wfx_bus_events, SL_WFX_BUS_EVENT_FLAG_RX);
    }
#endif
    /*Wait for an event*/
    flags = osEventFlagsWait(sl_wfx_bus_events,
                             SL_WFX_BUS_EVENT_FLAG_RX | SL_WFX_BUS_EVENT_FLAG_TX,
                             osFlagsWaitAny,
                             osWaitForever);

    if (flags & SL_WFX_BUS_EVENT_FLAG_TX) {
      /* Process TX packets */
      sl_wfx_tx_process();
    }
    if (flags & SL_WFX_BUS_EVENT_FLAG_RX) {
      /* Process RX packets */
      sl_wfx_rx_process(control_register);
#ifdef SL_WFX_USE_SDIO
      /* Reenable interrupt (req for sdio)*/
      sl_wfx_host_enable_platform_interrupt();
#endif
    }
  }
}

/***************************************************************************//**
 * Receives frames from the WF200.
 ******************************************************************************/
static sl_status_t sl_wfx_rx_process(uint16_t control_register)
{
  sl_status_t result;

  result = sl_wfx_receive_frame(&control_register);
  if ((control_register & SL_WFX_CONT_NEXT_LEN_MASK) != 0) {
    /* if a packet is still available in the WF200, set an RX event */
    osEventFlagsSet(sl_wfx_bus_events, SL_WFX_BUS_EVENT_FLAG_RX);
  }

  return result;
}
/**************************************************************************//**
 * Wfx process tx queue
 *****************************************************************************/
static sl_status_t sl_wfx_tx_process(void)
{
  sl_status_t result;
  sl_wfx_packet_queue_item_t *item_to_free;

  if (sl_wfx_tx_queue_context.head_ptr == NULL) {
    return SL_STATUS_EMPTY;
  }

  /* Take TX queue mutex */
  osMutexAcquire(sl_wfx_tx_queue_mutex, osWaitForever);

  /* Send the packet */
  result = sl_wfx_send_ethernet_frame(&sl_wfx_tx_queue_context.head_ptr->buffer,
                                      sl_wfx_tx_queue_context.head_ptr->data_length,
                                      sl_wfx_tx_queue_context.head_ptr->interface,
                                      WFM_PRIORITY_BE0);

  if (result != SL_STATUS_OK) {
    /* If the packet is not successfully sent, set the associated event and return */
    osEventFlagsSet(sl_wfx_bus_events, SL_WFX_BUS_EVENT_FLAG_TX);
    osMutexRelease(sl_wfx_tx_queue_mutex);
    return SL_STATUS_FULL;
  }

  /* The packet has been successfully sent, free it  */
  item_to_free = sl_wfx_tx_queue_context.head_ptr;

  /* Move the queue pointer to process the next packet */
  sl_wfx_tx_queue_context.head_ptr = sl_wfx_tx_queue_context.head_ptr->next;

  /* The packet has been sent, release the packet  */
  sl_wfx_free_command_buffer((sl_wfx_generic_message_t*) item_to_free,
                             SL_WFX_SEND_FRAME_REQ_ID,
                             SL_WFX_TX_FRAME_BUFFER);

  /* If a packet is available, set the associated event */
  if (sl_wfx_tx_queue_context.head_ptr != NULL) {
    osEventFlagsSet(sl_wfx_bus_events, SL_WFX_BUS_EVENT_FLAG_TX);
  }

  /* Release TX queue mutex */
  osMutexRelease(sl_wfx_tx_queue_mutex);

  return result;
}

/***************************************************************************//**
 * Creates WF200 bus communication task.
 ******************************************************************************/
void sl_wfx_task_start()
{
  osThreadId_t       thread_id;
  osThreadAttr_t     thread_attr;
  osEventFlagsAttr_t event_attr;
  osSemaphoreAttr_t  sem_attr;
  osMutexAttr_t      mutex_attr;

  mutex_attr.name = "WFX bus tx queue mutex";
  mutex_attr.cb_mem = sl_wfx_bus_queue_mutex_cb;
  mutex_attr.cb_size = osMutexCbSize;
  mutex_attr.attr_bits = 0;

  sl_wfx_tx_queue_mutex = osMutexNew(&mutex_attr);
  EFM_ASSERT(sl_wfx_tx_queue_mutex != NULL);

  event_attr.name = "WFX bus events";
  event_attr.cb_mem = sl_wfx_bus_events_cb;
  event_attr.cb_size = osEventFlagsCbSize;
  event_attr.attr_bits = 0;

  sl_wfx_bus_events =  osEventFlagsNew(&event_attr);
  EFM_ASSERT(sl_wfx_bus_events != NULL);

  sem_attr.name = "WFX bus TX complete";
  sem_attr.cb_mem = sl_wfx_bus_tx_complete_cb;
  sem_attr.cb_size = osSemaphoreCbSize;
  sem_attr.attr_bits = 0;

  sl_wfx_bus_tx_complete = osSemaphoreNew(1, 0, &sem_attr);
  EFM_ASSERT(sl_wfx_bus_tx_complete != NULL);

  thread_attr.name = "WFX bus task";
  thread_attr.priority = SL_WFX_BUS_TASK_PRIO;
  thread_attr.stack_mem = sl_wfx_bus_stack;
  thread_attr.stack_size = SL_WFX_BUS_STACK_SIZE;
  thread_attr.cb_mem = sl_wfx_bus_task_cb;
  thread_attr.cb_size = osThreadCbSize;
  thread_attr.attr_bits = 0u;
  thread_attr.tz_module = 0u;

  thread_id = osThreadNew(sl_wfx_task_entry, NULL, &thread_attr);
  EFM_ASSERT(thread_id != NULL);
}
