/***************************************************************************//**
 * @file
 * @brief Core Wi-Fi Commissioning application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
// Define module name for Power Manager debuging feature.
#define CURRENT_MODULE_NAME    "APP_COMMON_EXAMPLE_WIFI_COMMISSIONING"

#include <stdio.h>
//#include "os.h"
//#include "io.h"
//#include "bsp_os.h"
//#include "common.h"
#include "em_common.h"
#include "sl_simple_led_instances.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_button_config.h"
#include "app_webpage.h"
#include "app_wifi_events.h"
#include "app_wifi_commissioning.h"
#include "sl_wfx_host.h"
#include "cmsis_os2.h"
#include "sl_cmsis_os2_common.h"
#include "sl_wfx_host_init.h"

#define START_APP_TASK_PRIO              30u
#define START_APP_TASK_STK_SIZE         600u
/// Start task stack.
//static CPU_STK start_app_task_stk[START_APP_TASK_STK_SIZE];
/// Start task TCB.
//static OS_TCB  start_app_task_tcb;
__ALIGNED(8) static uint8_t start_app_task_stk[(START_APP_TASK_STK_SIZE * sizeof(void *)) & 0xFFFFFFF8u];
__ALIGNED(4) static uint8_t start_app_task_tcb[osThreadCbSize];

static void    start_app_task(void *p_arg);

void sl_button_on_change(const sl_button_t *handle)
{
  if ((handle == &sl_button_btn0)
      && (sl_button_get_state(&sl_button_btn0) == SL_SIMPLE_BUTTON_POLARITY)) {
    sl_led_toggle(&sl_led_led0);
  } else if ((handle == &sl_button_btn1)
             && (sl_button_get_state(&sl_button_btn1) == SL_SIMPLE_BUTTON_POLARITY)) {
    sl_led_toggle(&sl_led_led1);
  }
}

static void start_app_task(void *p_arg)
{
//  RTOS_ERR  err;
//  PP_UNUSED_PARAM(p_arg); // Prevent compiler warning.

  osSemaphoreAcquire(sl_wfx_init_sem, osWaitForever);
  // Clear the console and buffer
  printf("Wi-Fi Commissioning Micrium  OS Example\r\n");

  app_wifi_events_start();
  webpage_start();

  // Delete the init thread.
//  OSTaskDel(0, &err);
  osThreadExit();
}
/**************************************************************************//**
 * Wi-Fi Commissioning application init.
 *****************************************************************************/
void app_wifi_commissioning_init(void)
{
  osThreadId_t       thread_id;
  osThreadAttr_t     thread_attr;
  thread_attr.name = "Start APP Task";
  thread_attr.priority = START_APP_TASK_PRIO;
  thread_attr.stack_mem = start_app_task_stk;
  thread_attr.stack_size = START_APP_TASK_STK_SIZE;
  thread_attr.cb_mem = start_app_task_tcb;
  thread_attr.cb_size = osThreadCbSize;
  thread_attr.attr_bits = 0u;
  thread_attr.tz_module = 0u;

  // notice
  thread_id = osThreadNew(start_app_task, NULL, &thread_attr);
  EFM_ASSERT(thread_id != NULL);
}
