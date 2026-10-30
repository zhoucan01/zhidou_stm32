/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_tmr.c
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
#include "wk_tmr.h"

/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief  init tmr1 function.
  * @param  none
  * @retval none
  */
void wk_tmr1_init(void)
{
  /* add user code begin tmr1_init 0 */

  /* add user code end tmr1_init 0 */


  /* add user code begin tmr1_init 1 */

  /* add user code end tmr1_init 1 */

  /* configure counter settings */
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);
  tmr_repetition_counter_set(TMR1, 0);
  tmr_period_buffer_enable(TMR1, FALSE);
  tmr_base_init(TMR1, 65535, 0);

  /* configure primary mode settings */
  tmr_sub_sync_mode_set(TMR1, FALSE);
  tmr_primary_mode_select(TMR1, TMR_PRIMARY_SEL_RESET);

  tmr_counter_enable(TMR1, TRUE);

  /* enable ovfien interrupt */
  tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);

  /* add user code begin tmr1_init 2 */

  /* add user code end tmr1_init 2 */
}

/**
  * @brief  init tmr2 function.
  * @param  none
  * @retval none
  */
void wk_tmr2_init(void)
{
  /* add user code begin tmr2_init 0 */

  /* add user code end tmr2_init 0 */

  gpio_init_type gpio_init_struct;
  tmr_output_config_type tmr_output_struct;

  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin tmr2_init 1 */

  /* add user code end tmr2_init 1 */

  /* configure the CH1 pin */
  gpio_init_struct.gpio_pins = hosePumpPWMA_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(hosePumpPWMA_GPIO_PORT, &gpio_init_struct);

  /* configure the CH2 pin */
  gpio_init_struct.gpio_pins = hosePumpPWMB_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(hosePumpPWMB_GPIO_PORT, &gpio_init_struct);

  /* configure the CH3 pin */
  gpio_init_struct.gpio_pins = waterPumpPWM_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(waterPumpPWM_GPIO_PORT, &gpio_init_struct);

  /* configure the CH4 pin */
  gpio_init_struct.gpio_pins = brinePumpPWM_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(brinePumpPWM_GPIO_PORT, &gpio_init_struct);

  /* configure counter settings */
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
  tmr_period_buffer_enable(TMR2, FALSE);
  tmr_base_init(TMR2, 14999, 0);

  /* configure primary mode settings */
  tmr_sub_sync_mode_set(TMR2, FALSE);
  tmr_primary_mode_select(TMR2, TMR_PRIMARY_SEL_RESET);

  /* configure channel 1 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
  tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_1, 0);
  tmr_output_channel_buffer_enable(TMR2, TMR_SELECT_CHANNEL_1, FALSE);

  tmr_output_channel_immediately_set(TMR2, TMR_SELECT_CHANNEL_1, FALSE);

  /* configure channel 2 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
  tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_2, 0);
  tmr_output_channel_buffer_enable(TMR2, TMR_SELECT_CHANNEL_2, FALSE);

  tmr_output_channel_immediately_set(TMR2, TMR_SELECT_CHANNEL_2, FALSE);

  /* configure channel 3 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
  tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, 0);
  tmr_output_channel_buffer_enable(TMR2, TMR_SELECT_CHANNEL_3, FALSE);

  tmr_output_channel_immediately_set(TMR2, TMR_SELECT_CHANNEL_3, FALSE);

  /* configure channel 4 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_4, &tmr_output_struct);
  tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_4, 0);
  tmr_output_channel_buffer_enable(TMR2, TMR_SELECT_CHANNEL_4, FALSE);

  tmr_output_channel_immediately_set(TMR2, TMR_SELECT_CHANNEL_4, FALSE);


  tmr_counter_enable(TMR2, TRUE);

  /* add user code begin tmr2_init 2 */

  /* add user code end tmr2_init 2 */
}

/**
  * @brief  init tmr3 function.
  * @param  none
  * @retval none
  */
void wk_tmr3_init(void)
{
  /* add user code begin tmr3_init 0 */

  /* add user code end tmr3_init 0 */

  gpio_init_type gpio_init_struct;
  tmr_input_config_type  tmr_input_struct;

  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin tmr3_init 1 */

  /* add user code end tmr3_init 1 */

  /* configure the CH1 pin */
  gpio_init_struct.gpio_pins = GPIO_PINS_6;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOC, &gpio_init_struct);

  /* configure the CH2 pin */
  gpio_init_struct.gpio_pins = GPIO_PINS_7;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOC, &gpio_init_struct);

  /* GPIO PIN remap */
  gpio_pin_remap_config(TMR3_GMUX_0011, TRUE); 

  /* configure counter settings */
  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR3, TMR_CLOCK_DIV1);
  tmr_period_buffer_enable(TMR3, FALSE);
  tmr_base_init(TMR3, 65535, 0);

  /* configure primary mode settings */
  tmr_sub_sync_mode_set(TMR3, FALSE);
  tmr_primary_mode_select(TMR3, TMR_PRIMARY_SEL_RESET);

  /* configure encoder mode */
  tmr_input_struct.input_channel_select = TMR_SELECT_CHANNEL_1;
  tmr_input_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
  tmr_input_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;
  tmr_input_struct.input_filter_value = 0;
  tmr_input_channel_init(TMR3, &tmr_input_struct, TMR_CHANNEL_INPUT_DIV_1);

  tmr_input_struct.input_channel_select = TMR_SELECT_CHANNEL_2;
  tmr_input_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
  tmr_input_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;
  tmr_input_struct.input_filter_value = 0;
  tmr_input_channel_init(TMR3, &tmr_input_struct, TMR_CHANNEL_INPUT_DIV_1);

  tmr_encoder_mode_config(TMR3, TMR_ENCODER_MODE_A, TMR_INPUT_RISING_EDGE, TMR_INPUT_RISING_EDGE);

  tmr_counter_enable(TMR3, TRUE);

  /* enable ovfien interrupt */
  tmr_interrupt_enable(TMR3, TMR_OVF_INT, TRUE);

  /* add user code begin tmr3_init 2 */

  /* add user code end tmr3_init 2 */
}

/**
  * @brief  init tmr4 function.
  * @param  none
  * @retval none
  */
void wk_tmr4_init(void)
{
  /* add user code begin tmr4_init 0 */

  /* add user code end tmr4_init 0 */

  gpio_init_type gpio_init_struct;
  tmr_output_config_type tmr_output_struct;

  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin tmr4_init 1 */

  /* add user code end tmr4_init 1 */

  /* configure the CH1 pin */
  gpio_init_struct.gpio_pins = riseMoterLPWMB_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(riseMoterLPWMB_GPIO_PORT, &gpio_init_struct);

  /* configure the CH2 pin */
  gpio_init_struct.gpio_pins = riseMoterLPWMA_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(riseMoterLPWMA_GPIO_PORT, &gpio_init_struct);

  /* configure the CH3 pin */
  gpio_init_struct.gpio_pins = pressMoterPWMB_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(pressMoterPWMB_GPIO_PORT, &gpio_init_struct);

  /* configure the CH4 pin */
  gpio_init_struct.gpio_pins = pressMoterPWMA_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(pressMoterPWMA_GPIO_PORT, &gpio_init_struct);

  /* configure counter settings */
  tmr_cnt_dir_set(TMR4, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR4, TMR_CLOCK_DIV1);
  tmr_period_buffer_enable(TMR4, TRUE);
  tmr_base_init(TMR4, 3749, 0);

  /* configure primary mode settings */
  /* sts=1: TRGOUT outputs OVF event to TMR5 (slave) while TRGOUT also triggers ADC1 preempt group */
  tmr_sub_sync_mode_set(TMR4, TRUE);
  tmr_primary_mode_select(TMR4, TMR_PRIMARY_SEL_OVERFLOW);

  /* configure channel 1 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR4, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
  tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_1, 0);
  tmr_output_channel_buffer_enable(TMR4, TMR_SELECT_CHANNEL_1, TRUE);

  tmr_output_channel_immediately_set(TMR4, TMR_SELECT_CHANNEL_1, FALSE);

  /* configure channel 2 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR4, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
  tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_2, 0);
  tmr_output_channel_buffer_enable(TMR4, TMR_SELECT_CHANNEL_2, TRUE);

  tmr_output_channel_immediately_set(TMR4, TMR_SELECT_CHANNEL_2, FALSE);

  /* configure channel 3 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR4, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
  tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_3, 0);
  tmr_output_channel_buffer_enable(TMR4, TMR_SELECT_CHANNEL_3, TRUE);

  tmr_output_channel_immediately_set(TMR4, TMR_SELECT_CHANNEL_3, FALSE);

  /* configure channel 4 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR4, TMR_SELECT_CHANNEL_4, &tmr_output_struct);
  tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_4, 0);
  tmr_output_channel_buffer_enable(TMR4, TMR_SELECT_CHANNEL_4, TRUE);

  tmr_output_channel_immediately_set(TMR4, TMR_SELECT_CHANNEL_4, FALSE);


  tmr_counter_enable(TMR4, TRUE);

  /* enable ovfien interrupt */
  tmr_interrupt_enable(TMR4, TMR_OVF_INT, TRUE);

  /* add user code begin tmr4_init 2 */

  /* add user code end tmr4_init 2 */
}

/**
  * @brief  init tmr5 function.
  * @param  none
  * @retval none
  */
void wk_tmr5_init(void)
{
  /* add user code begin tmr5_init 0 */

  /* add user code end tmr5_init 0 */

  gpio_init_type gpio_init_struct;
  tmr_output_config_type tmr_output_struct;

  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin tmr5_init 1 */

  /* add user code end tmr5_init 1 */

  /* configure the CH1 pin */
  gpio_init_struct.gpio_pins = riseMoterRPWMA_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(riseMoterRPWMA_GPIO_PORT, &gpio_init_struct);

  /* configure the CH2 pin */
  gpio_init_struct.gpio_pins = riseMoterRPWMB_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(riseMoterRPWMB_GPIO_PORT, &gpio_init_struct);

  /* GPIO PIN remap */
  gpio_pin_remap_config(TMR5_GMUX_001, TRUE); 

  /* configure counter settings */
  tmr_cnt_dir_set(TMR5, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR5, TMR_CLOCK_DIV1);
  tmr_period_buffer_enable(TMR5, TRUE);
  tmr_base_init(TMR5, 3749, 0);

  /* configure primary mode settings, same as TMR4 */
  tmr_sub_sync_mode_set(TMR5, FALSE);
  tmr_primary_mode_select(TMR5, TMR_PRIMARY_SEL_OVERFLOW);

  /* subordinate mode: reset TMR5 on every TMR4 overflow so both PWM carriers stay phase-locked.
     ITR3 = TMR4 output (same mapping as STM32F1, confirmed by AT32F415 official sync examples) */
  tmr_sub_mode_select(TMR5, TMR_SUB_RESET_MODE);
  tmr_trigger_input_select(TMR5, TMR_SUB_INPUT_SEL_IS3);

  /* configure channel 1 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR5, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
  tmr_channel_value_set(TMR5, TMR_SELECT_CHANNEL_1, 0);
  tmr_output_channel_buffer_enable(TMR5, TMR_SELECT_CHANNEL_1, TRUE);

  tmr_output_channel_immediately_set(TMR5, TMR_SELECT_CHANNEL_1, FALSE);

  /* configure channel 2 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR5, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
  tmr_channel_value_set(TMR5, TMR_SELECT_CHANNEL_2, 0);
  tmr_output_channel_buffer_enable(TMR5, TMR_SELECT_CHANNEL_2, TRUE);

  tmr_output_channel_immediately_set(TMR5, TMR_SELECT_CHANNEL_2, FALSE);

  tmr_counter_enable(TMR5, TRUE);

  /* add user code begin tmr5_init 2 */

  /* add user code end tmr5_init 2 */
}

/**
  * @brief  init tmr10 function.
  * @param  none
  * @retval none
  */
void wk_tmr10_init(void)
{
  /* add user code begin tmr10_init 0 */

  /* add user code end tmr10_init 0 */

  /* add user code begin tmr10_init 1 */

  /* add user code end tmr10_init 1 */

  /* configure counter settings */
  tmr_cnt_dir_set(TMR10, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR10, TMR_CLOCK_DIV1);
  tmr_period_buffer_enable(TMR10, FALSE);
  /* 150 MHz / 12: start at 4 ms, ramp down to 0.9 ms within 6 seconds. */
  tmr_base_init(TMR10, 49999, 11);

  tmr_counter_enable(TMR10, TRUE);

  /* enable ovfien interrupt */
  tmr_interrupt_enable(TMR10, TMR_OVF_INT, TRUE);

  /* add user code begin tmr10_init 2 */

  /* add user code end tmr10_init 2 */
}

/* add user code begin 1 */

/* add user code end 1 */
