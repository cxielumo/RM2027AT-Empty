/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_can.c
  * @brief    work bench config program
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

/* Includes ------------------------------------------------------------------*/
#include "wk_can.h"

/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief  init can1 function.
  * @param  none
  * @retval none
  */
void wk_can1_init(void)
{
  /* add user code begin can1_init 0 */

  /* add user code end can1_init 0 */
  
  gpio_init_type gpio_init_struct;
  can_base_type can_base_struct;
  can_baudrate_type can_baudrate_struct;
  can_filter_init_type can_filter_init_struct;

  /* add user code begin can1_init 1 */

  /* add user code end can1_init 1 */
  
  /*gpio-----------------------------------------------------------------------------*/ 
  gpio_default_para_init(&gpio_init_struct);

  /* configure the CAN1 TX pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE12, GPIO_MUX_9);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_12;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the CAN1 RX pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_9);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_11;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  /*can_base_init--------------------------------------------------------------------*/ 
  can_default_para_init(&can_base_struct);
  can_base_struct.mode_selection = CAN_MODE_COMMUNICATE;
  can_base_struct.ttc_enable = FALSE;
  can_base_struct.aebo_enable = TRUE;
  can_base_struct.aed_enable = TRUE;
  can_base_struct.prsf_enable = FALSE;
  can_base_struct.mdrsel_selection = CAN_DISCARDING_FIRST_RECEIVED;
  can_base_struct.mmssr_selection = CAN_SENDING_BY_ID;

  can_base_init(CAN1, &can_base_struct);

  /*can_baudrate_setting-------------------------------------------------------------*/ 
  /*set baudrate = pclk/(baudrate_div *(1 + bts1_size + bts2_size))------------------*/ 
  can_baudrate_struct.baudrate_div = 5;                       /*value: 1~0xFFF*/
  can_baudrate_struct.rsaw_size = CAN_RSAW_1TQ;                /*value: 1~4*/
  can_baudrate_struct.bts1_size = CAN_BTS1_11TQ;                /*value: 1~16*/
  can_baudrate_struct.bts2_size = CAN_BTS2_3TQ;                /*value: 1~8*/
  can_baudrate_set(CAN1, &can_baudrate_struct);

  /*can_filter_0_config--------------------------------------------------------------*/
  can_filter_init_struct.filter_activate_enable = TRUE;
  can_filter_init_struct.filter_number = 0;
  can_filter_init_struct.filter_fifo = CAN_FILTER_FIFO0;
  can_filter_init_struct.filter_bit = CAN_FILTER_16BIT;  
  can_filter_init_struct.filter_mode = CAN_FILTER_MODE_ID_MASK;  
  /*Standard identifier + Mask Mode + Data/Remote frame: id/mask 11bit --------------*/
  can_filter_init_struct.filter_id_high = 0x0 << 5;
  can_filter_init_struct.filter_id_low = 0x0 << 5;
  can_filter_init_struct.filter_mask_high = 0x0 << 5;
  can_filter_init_struct.filter_mask_low = 0x0 << 5;

  can_filter_init(CAN1, &can_filter_init_struct);

  /* add user code begin can1_init 2 */

  /* add user code end can1_init 2 */
}

/**
  * @brief  init can2 function.
  * @param  none
  * @retval none
  */
void wk_can2_init(void)
{
  /* add user code begin can2_init 0 */

  /* add user code end can2_init 0 */
  
  gpio_init_type gpio_init_struct;
  can_base_type can_base_struct;
  can_baudrate_type can_baudrate_struct;
  can_filter_init_type can_filter_init_struct;

  /* add user code begin can2_init 1 */

  /* add user code end can2_init 1 */
  
  /*gpio-----------------------------------------------------------------------------*/ 
  gpio_default_para_init(&gpio_init_struct);

  /* configure the CAN2 TX pin */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE13, GPIO_MUX_9);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_13;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure the CAN2 RX pin */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE12, GPIO_MUX_9);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_12;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /*can_base_init--------------------------------------------------------------------*/ 
  can_default_para_init(&can_base_struct);
  can_base_struct.mode_selection = CAN_MODE_COMMUNICATE;
  can_base_struct.ttc_enable = FALSE;
  can_base_struct.aebo_enable = TRUE;
  can_base_struct.aed_enable = TRUE;
  can_base_struct.prsf_enable = FALSE;
  can_base_struct.mdrsel_selection = CAN_DISCARDING_FIRST_RECEIVED;
  can_base_struct.mmssr_selection = CAN_SENDING_BY_ID;

  can_base_init(CAN2, &can_base_struct);

  /*can_baudrate_setting-------------------------------------------------------------*/ 
  /*set baudrate = pclk/(baudrate_div *(1 + bts1_size + bts2_size))------------------*/ 
  can_baudrate_struct.baudrate_div = 5;                       /*value: 1~0xFFF*/
  can_baudrate_struct.rsaw_size = CAN_RSAW_1TQ;                /*value: 1~4*/
  can_baudrate_struct.bts1_size = CAN_BTS1_11TQ;                /*value: 1~16*/
  can_baudrate_struct.bts2_size = CAN_BTS2_3TQ;                /*value: 1~8*/
  can_baudrate_set(CAN2, &can_baudrate_struct);

  /*can_filter_0_config--------------------------------------------------------------*/
  can_filter_init_struct.filter_activate_enable = TRUE;
  can_filter_init_struct.filter_number = 0;
  can_filter_init_struct.filter_fifo = CAN_FILTER_FIFO0;
  can_filter_init_struct.filter_bit = CAN_FILTER_16BIT;  
  can_filter_init_struct.filter_mode = CAN_FILTER_MODE_ID_MASK;  
  /*Standard identifier + Mask Mode + Data/Remote frame: id/mask 11bit --------------*/
  can_filter_init_struct.filter_id_high = 0x0 << 5;
  can_filter_init_struct.filter_id_low = 0x0 << 5;
  can_filter_init_struct.filter_mask_high = 0x0 << 5;
  can_filter_init_struct.filter_mask_low = 0x0 << 5;

  can_filter_init(CAN2, &can_filter_init_struct);

  /* add user code begin can2_init 2 */

  /* add user code end can2_init 2 */
}

/* add user code begin 1 */

/* add user code end 1 */
