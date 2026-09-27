/* add user code begin Header */
/**
  **************************************************************************
  * @file     at32f423_int.c
  * @brief    main interrupt service routines.
  **************************************************************************
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

/* includes ------------------------------------------------------------------*/
#include "at32f423_int.h"
/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */

#include "FreeRTOS.h"
#include "task.h"
#include "bsp.h"
#include "can.h"
#include "uart.h"

/* Internal driver hook kept out of the public UART application API. */
void uart_irq_handler(uart_id_t id);
void can_irq_handler(can_bus_t bus);

/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */

/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/* external variables ---------------------------------------------------------*/
/* add user code begin external variables */

/* add user code end external variables */

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
void NMI_Handler(void)
{
  /* add user code begin NonMaskableInt_IRQ 0 */

  /* add user code end NonMaskableInt_IRQ 0 */

  /* add user code begin NonMaskableInt_IRQ 1 */

  /* add user code end NonMaskableInt_IRQ 1 */
}

/**
  * @brief  this function handles hard fault exception.
  * @param  none
  * @retval none
  */
void HardFault_Handler(void)
{
  /* add user code begin HardFault_IRQ 0 */

  /* add user code end HardFault_IRQ 0 */
  /* go to infinite loop when hard fault exception occurs */
  while (1)
  {
    /* add user code begin W1_HardFault_IRQ 0 */

    /* add user code end W1_HardFault_IRQ 0 */
  }
}


/**
  * @brief  this function handles memory manage exception.
  * @param  none
  * @retval none
  */
void MemManage_Handler(void)
{
  /* add user code begin MemoryManagement_IRQ 0 */

  /* add user code end MemoryManagement_IRQ 0 */
  /* go to infinite loop when memory manage exception occurs */
  while (1)
  {
    /* add user code begin W1_MemoryManagement_IRQ 0 */

    /* add user code end W1_MemoryManagement_IRQ 0 */
  }
}

/**
  * @brief  this function handles bus fault exception.
  * @param  none
  * @retval none
  */
void BusFault_Handler(void)
{
  /* add user code begin BusFault_IRQ 0 */

  /* add user code end BusFault_IRQ 0 */
  /* go to infinite loop when bus fault exception occurs */
  while (1)
  {
    /* add user code begin W1_BusFault_IRQ 0 */

    /* add user code end W1_BusFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles usage fault exception.
  * @param  none
  * @retval none
  */
void UsageFault_Handler(void)
{
  /* add user code begin UsageFault_IRQ 0 */

  /* add user code end UsageFault_IRQ 0 */
  /* go to infinite loop when usage fault exception occurs */
  while (1)
  {
    /* add user code begin W1_UsageFault_IRQ 0 */

    /* add user code end W1_UsageFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles debug monitor exception.
  * @param  none
  * @retval none
  */
void DebugMon_Handler(void)
{
  /* add user code begin DebugMonitor_IRQ 0 */

  /* add user code end DebugMonitor_IRQ 0 */
  /* add user code begin DebugMonitor_IRQ 1 */

  /* add user code end DebugMonitor_IRQ 1 */
}

/**
  * @brief  this function handles systick handler.
  * @param  none
  * @retval none
  */
void SysTick_Handler(void)
{
  /* add user code begin SysTick_IRQ 0 */

  /* add user code end SysTick_IRQ 0 */

  /* add user code begin SysTick_IRQ 1 */

  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    xPortSysTickHandler();
  }

  /* add user code end SysTick_IRQ 1 */
}

/**
  * @brief  this function handles DMA1 Channel 1 handler.
  * @param  none
  * @retval none
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* add user code begin DMA1_Channel1_IRQ 0 */

  uint8_t dma_event = 0U;

  /* add user code end DMA1_Channel1_IRQ 0 */

  if(dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET)
  {   
    /* add user code begin DMA1_FDT1_FLAG */
    dma_flag_clear(DMA1_FDT1_FLAG);
    dma_event = 1U;
    /* add user code end DMA1_FDT1_FLAG */ 
  }

  if(dma_interrupt_flag_get(DMA1_HDT1_FLAG) != RESET)
  {   
    /* add user code begin DMA1_HDT1_FLAG */
    dma_flag_clear(DMA1_HDT1_FLAG);
    dma_event = 1U;
    /* add user code end DMA1_HDT1_FLAG */ 
  }

  if(dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET)
  {   
    /* add user code begin DMA1_DTERR1_FLAG */
    dma_flag_clear(DMA1_DTERR1_FLAG);
    dma_event = 1U;
    /* add user code end DMA1_DTERR1_FLAG */ 
  }

  /* add user code begin DMA1_Channel1_IRQ 1 */

  if (dma_event != 0U)
  {
    uart_irq_handler(UART_6);
  }

  /* add user code end DMA1_Channel1_IRQ 1 */
}

/**
  * @brief  this function handles CAN1 RX0 handler.
  * @param  none
  * @retval none
  */
void CAN1_RX0_IRQHandler(void)
{
  /* add user code begin CAN1_RX0_IRQ 0 */

  /* add user code end CAN1_RX0_IRQ 0 */

  /* add user code begin CAN1_RX0_IRQ 1 */

  can_irq_handler(CAN_BUS_1);

  /* add user code end CAN1_RX0_IRQ 1 */
}

/**
  * @brief  this function handles CAN1 SE handler.
  * @param  none
  * @retval none
  */
void CAN1_SE_IRQHandler(void)
{
  /* add user code begin CAN1_SE_IRQ 0 */

  /* add user code end CAN1_SE_IRQ 0 */

  /* add user code begin CAN1_SE_IRQ 1 */

  can_irq_handler(CAN_BUS_1);

  /* add user code end CAN1_SE_IRQ 1 */
}

/**
  * @brief  this function handles CAN2 RX0 handler.
  * @param  none
  * @retval none
  */
void CAN2_RX0_IRQHandler(void)
{
  /* add user code begin CAN2_RX0_IRQ 0 */

  /* add user code end CAN2_RX0_IRQ 0 */

  /* add user code begin CAN2_RX0_IRQ 1 */

  can_irq_handler(CAN_BUS_2);

  /* add user code end CAN2_RX0_IRQ 1 */
}

/**
  * @brief  this function handles CAN2 SE handler.
  * @param  none
  * @retval none
  */
void CAN2_SE_IRQHandler(void)
{
  /* add user code begin CAN2_SE_IRQ 0 */

  /* add user code end CAN2_SE_IRQ 0 */

  /* add user code begin CAN2_SE_IRQ 1 */

  can_irq_handler(CAN_BUS_2);

  /* add user code end CAN2_SE_IRQ 1 */
}

/**
  * @brief  this function handles USART6 handler.
  * @param  none
  * @retval none
  */
void USART6_IRQHandler(void)
{
  /* add user code begin USART6_IRQ 0 */

  /* add user code end USART6_IRQ 0 */

  /* add user code begin USART6_IRQ 1 */

  uart_irq_handler(UART_6);

  /* add user code end USART6_IRQ 1 */
}

/* add user code begin 1 */

/* add user code end 1 */
