/***************************************************************************//**
 * @file sl_wfx_host_init.c
 * @brief Core Wi-Fi internal application logic.
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

//#include "os.h"
//#include "io.h"
//#include "bsp_os.h"
//#include "common.h"
#include "em_common.h"
#include "em_gpio.h"
#include "sl_wfx_task.h"
#include "sl_wfx_host.h"
#include "sl_wfx_host_pinout.h"
#include "gpiointerrupt.h"
#include "sl_wfx_host_init.h"
#include "cmsis_os2.h"
#include "sl_cmsis_os2_common.h"

#ifdef SL_CATALOG_POWER_MANAGER_PRESENT
#include "sl_power_manager.h"
#endif

#define START_TASK_PRIO   osPriorityRealtime
#define START_TASK_SIZE   1024u

#ifdef SL_CATALOG_WFX_SECURE_LINK_PRESENT
extern void sl_wfx_securelink_task_start(void);
#endif

sl_wfx_context_t wifi;

#ifdef  SL_CATALOG_WFX_BUS_SPI_PRESENT
extern uint8_t wirq_irq_nb;
#endif

osSemaphoreId_t sl_wfx_init_sem;
static uint8_t  sl_wfx_init_sem_cb[osSemaphoreCbSize];

/// Start task stack.
__ALIGNED(8) static uint8_t start_task_stack[(START_TASK_SIZE * sizeof(void *)) & 0xFFFFFFF8u];
/// Start task TCB.
__ALIGNED(4) static uint8_t start_task_cb[osThreadCbSize];

static void    start_task(void *p_arg);

static void    sl_wfx_interrupt(uint8_t intNo)
{
  (void)intNo;
  if (sl_wfx_wakeup_sem != NULL) {
        osSemaphoreRelease(sl_wfx_wakeup_sem);
  }
#ifdef SL_CATALOG_WFX_BUS_SPI_PRESENT
  osEventFlagsSet(sl_wfx_bus_events, SL_WFX_BUS_EVENT_FLAG_RX);
#endif
#ifdef SL_CATALOG_WFX_BUS_SDIO_PRESENT
#ifdef SLEEP_ENABLED
  osEventFlagsSet(sl_wfx_bus_events, SL_WFX_BUS_EVENT_FLAG_RX);
#endif
#endif
}

/**************************************************************************//**
 * Configure the GPIO pins.
 *****************************************************************************/
static void gpio_setup(void)
{
  // Configure WF200 reset pin.
  GPIO_PinModeSet(SL_WFX_HOST_PINOUT_RESET_PORT, SL_WFX_HOST_PINOUT_RESET_PIN, gpioModePushPull, 0);
  // Configure WF200 WUP pin.
  GPIO_PinModeSet(SL_WFX_HOST_PINOUT_WUP_PORT, SL_WFX_HOST_PINOUT_WUP_PIN, gpioModePushPull, 0);
#ifdef  SL_CATALOG_WFX_BUS_SPI_PRESENT
  // GPIO used as IRQ.
  GPIO_PinModeSet(SL_WFX_HOST_PINOUT_SPI_WIRQ_PORT, SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN, gpioModeInputPull, 0);

  if (GPIO->IEN & (1 << SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN)) {
    uint8_t index_start = SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN - SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN % 4;
    uint8_t index_stop = index_start + 4;
    for (uint8_t i = index_start; i < index_stop; i++) {
      if ((GPIO->IEN & (1 << i)) == 0) {
        // Interrupt not used, use this EXTI line to reroute the interrupt
        wirq_irq_nb = i;
        break;
      }
    }

    if (wirq_irq_nb == SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN) {
      // No unused interrupt line found
      printf("WFx interrupt line shared, it can cause unknown behaviors\r\n");
    }
  }

  GPIOINT_CallbackRegister(wirq_irq_nb, (GPIOINT_IrqCallbackPtr_t)sl_wfx_interrupt);
#endif

#ifdef WGM160PX22KGA2
  GPIO_PinModeSet(SL_WFX_HOST_PINOUT_GPIO_WIRQ_PORT, SL_WFX_HOST_PINOUT_GPIO_WIRQ_PIN, gpioModeInputPull, 0);
  GPIOINT_CallbackRegister(SL_WFX_HOST_PINOUT_GPIO_WIRQ_PIN, (GPIOINT_IrqCallbackPtr_t)sl_wfx_interrupt);
  GPIO_PinModeSet(gpioPortD, 0, gpioModeDisabled, 1);
  GPIO_PinModeSet(gpioPortD, 1, gpioModeDisabled, 1);
  GPIO_PinModeSet(gpioPortD, 2, gpioModeDisabled, 1);
  GPIO_PinModeSet(gpioPortD, 3, gpioModeDisabled, 1);
  GPIO_PinModeSet(gpioPortD, 5, gpioModeDisabled, 1);
  GPIO_PinModeSet(SL_WFX_HOST_PINOUT_LP_CLK_PORT, SL_WFX_HOST_PINOUT_LP_CLK_PIN, gpioModePushPull, 0);
#endif

  (void)sl_wfx_interrupt;

  GPIOINT_Init();
}

/***************************************************************************//**
 * Initialize the WFX
 ******************************************************************************/
static void wifi_start(void)
{
  sl_status_t status;

  // Initialize the WF200
  status = sl_wfx_init(&wifi);

  osSemaphoreRelease(sl_wfx_init_sem);

  printf("\033\143");
  printf("\033[3J");
  printf("FMAC Driver version    %s\r\n", FMAC_DRIVER_VERSION_STRING);
  switch (status) {
    case SL_STATUS_OK:
      wifi.state = SL_WFX_STARTED;
      printf("WF200 Firmware version %d.%d.%d\r\n",
             wifi.firmware_major,
             wifi.firmware_minor,
             wifi.firmware_build);
      printf("WF200 initialization successful\r\n");
      break;
    case SL_STATUS_WIFI_INVALID_KEY:
      printf("Failed to init WF200: Firmware keyset invalid\r\n");
      break;
    case SL_STATUS_WIFI_FIRMWARE_DOWNLOAD_TIMEOUT:
      printf("Failed to init WF200: Firmware download timeout\r\n");
      break;
    case SL_STATUS_TIMEOUT:
      printf("Failed to init WF200: Poll for value timeout\r\n");
      break;
    case SL_STATUS_FAIL:
      printf("Failed to init WF200: Error\r\n");
      break;
    default:
      printf("Failed to init WF200: Unknown error %lu\r\n", status);
  }
  // Check error code.
//  APP_RTOS_ASSERT_DBG((status == SL_STATUS_OK), 1);

#ifdef SL_CATALOG_POWER_MANAGER_PRESENT
#ifdef SL_CATALOG_WFX_BUS_SDIO_PRESENT
  status = sl_wfx_host_switch_to_wirq();
  // Check error code.
  APP_RTOS_ASSERT_DBG((status == SL_STATUS_OK), 1);
#endif
#endif
}

static void start_task(void *p_arg)
{
//  RTOS_ERR  err;
//  PP_UNUSED_PARAM(p_arg); // Prevent compiler warning.

  // Initialize the IO module.
//  IO_Init(&err);
//  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  // Call common module initialization.
//  Common_Init(&err);
//  APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE,; );

  // Initialize the BSP.
//  BSP_OS_Init();

  gpio_setup();

  //start wfx bus communication task.
  sl_wfx_task_start();
#ifdef SL_CATALOG_WFX_SECURE_LINK_PRESENT
  sl_wfx_securelink_task_start(); // start securelink key renegotiation task
#endif //SL_CATALOG_WFX_SECURE_LINK_PRESENT

#ifdef SL_CATALOG_POWER_MANAGER_PRESENT
  // Prevent the program to go into EM2 sleep before the Wi-Fi
  // chip is initialized and as long the SoftAP is up.
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif

  wifi_start();

  // Delete the init thread.
  //OSTaskDel(0, &err);
  osThreadExit();
}

/**************************************************************************//**
 * Wi-Fi internal application init.
 *****************************************************************************/
void app_internal_wifi_init(void)
{
  osSemaphoreAttr_t  sem_attr;
  osThreadId_t       thread_id;
  osThreadAttr_t     thread_attr;

  sem_attr.name = "WFX init";
  sem_attr.cb_mem = sl_wfx_init_sem_cb;
  sem_attr.cb_size = osSemaphoreCbSize;
  sem_attr.attr_bits = 0;

  sl_wfx_init_sem = osSemaphoreNew(1, 0, &sem_attr);
  EFM_ASSERT(sl_wfx_init_sem != NULL);

  thread_attr.name = "Start Task";
  thread_attr.priority = START_TASK_PRIO;
  thread_attr.stack_mem = start_task_stack;
  thread_attr.stack_size = START_TASK_SIZE;
  thread_attr.cb_mem = start_task_cb;
  thread_attr.cb_size = osThreadCbSize;
  thread_attr.attr_bits = 0u;
  thread_attr.tz_module = 0u;

  thread_id = osThreadNew(start_task, NULL, &thread_attr);
  EFM_ASSERT(thread_id != NULL);
}
