/*!
    \file    main.c
    \brief   led spark with systick, USART print and key example

   \version 2024-12-20, V3.0.1, firmware for GD32F30x
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f30x.h"
#include <arm_math.h>
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "gd32f307c_eval.h"
#include "config.h"

uint32_t adc_value[8];

typedef int32_t int32_t;

//extern FlagStatus receive_flag;
extern can_receive_message_struct receive_message;
can_trasnmit_message_struct transmit_message;

void nvic_config(void);
void led_config(void);
void gpio_config(void);
void rcu_config(void);
void dma_config(void);
void adc_config(void);
void timer0_config(void); //PWM for Mosfet driver
void timer1_config(void); //PWM for triggering regular ADC
void timer2_config(void); // Input capture for hall sensors
ErrStatus can_networking(void);
void can_networking_init(void);
int32_t speed_PLL (int32_t ist, int32_t soll, uint8_t speedadapt);

uint16_t counter=0;
#define iabs(x) (((x) >= 0)?(x):-(x))
#define sign(x) (((x) >= 0)?(1):(-1))
MotorState_t MS;
MotorParams_t MP;
uint16_t ui16_timertics=0;
uint8_t ui8_hall_state=0;
uint8_t ui8_hall_state_old=0;
uint8_t ui8_hall_case=0;
uint32_t uint32_tics_filtered=32000;
uint8_t ui8_overflow_flag=0;
uint8_t ui8_SPEED_control_flag=0;
int32_t q31_rotorposition_hall=0;
int8_t i8_recent_rotor_direction=1;
int16_t i16_hall_order =1;
uint16_t uint16_full_rotation_counter=0;
uint16_t uint16_half_rotation_counter=0;
q31_t Hall_13 = 0;
int32_t Hall_32 = 0;
int32_t Hall_26 = 0;
int32_t Hall_64 = 0;
int32_t Hall_51 = 0;
int32_t Hall_45 = 0;
int32_t q31_PLL_error=0;
int32_t q31_rotorposition_PLL=0;
uint8_t ui_8_PLL_counter=0;
int32_t q31_angle_per_tic=0;
//Rotor angle scaled from degree to q31 for arm_math. -180Ã‚Â°-->-2^31, 0Ã‚Â°-->0, +180Ã‚Â°-->+2^31
const int32_t deg_30 = 357913941;



/*!
    \brief      toggle the led every 500ms
    \param[in]  none
    \param[out] none
    \retval     none
*/
void led_spark(void)
{
    static __IO uint32_t timingdelaylocal = 0U;

    if(timingdelaylocal){

        if(timingdelaylocal < 500U){
            gd_eval_led_on(LED2);
        }else{
            gd_eval_led_off(LED2);
        }

        timingdelaylocal--;
    }else{
        timingdelaylocal = 1000U;
    }
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

int main(void)
{
#ifdef __FIRMWARE_VERSION_DEFINE
     uint32_t fw_ver = 0;
#endif
    uint8_t i = 0;
    uint32_t timeout = 0xFFFF;
    uint8_t transmit_mailbox = 0;
    //receive_flag = RESET;
    SystemInit();
    /* system clocks configuration */
    rcu_config();
    /* configure systick */
    systick_config();
    /* initialize the LED */
    gd_eval_led_init(LED2);
    gd_eval_hall_init();
    //gd_eval_com_init(EVAL_COM0);
    nvic_config();
    gpio_config();
    /* TIMER configuration */
    timer0_config(); // PWM for Mosfet driver
    timer1_config(); //trigger regular ADC for testing
    timer2_config(); //for hall sensor handling
    /* DMA configuration */
    dma_config();
    /* ADC configuration */
    adc_config();

    /* initialize CAN and CAN filter */
    can_networking_init();

    /* enable CAN receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);

    /* initialize transmit message */
    transmit_message.tx_sfid = 0x7ab;
    transmit_message.tx_efid = 0x00;
    transmit_message.tx_ft = CAN_FT_DATA;
    transmit_message.tx_ff = CAN_FF_STANDARD;
    transmit_message.tx_dlen = 8;

#ifdef __FIRMWARE_VERSION_DEFINE
    fw_ver = gd32f30x_firmware_version_get();
    /* print firmware version */
    printf("\r\nGD32F30x series firmware version: V%d.%d.%d", (uint8_t)(fw_ver >> 24), (uint8_t)(fw_ver >> 16), (uint8_t)(fw_ver >> 8));
#endif /* __FIRMWARE_VERSION_DEFINE */

    while (1){

            if (counter > 2000){

            counter = 0;
            transmit_message.tx_data[0] = gpio_input_bit_get(GPIOA, GPIO_PIN_15);//(GPIO_ISTAT(GPIOC)>>6)&0x07;
            transmit_message.tx_data[1] = gpio_input_bit_get(GPIOA, GPIO_PIN_5); //ui16_timertics>>8;//(GPIO_ISTAT(GPIOA)>>8)&0xFF;
            transmit_message.tx_data[2] = gpio_input_bit_get(GPIOA, GPIO_PIN_6);
            transmit_message.tx_data[3] = gpio_input_bit_get(GPIOA, GPIO_PIN_10);
            transmit_message.tx_data[4] = (adc_value[2]>>8)&0xFF;;
            transmit_message.tx_data[5] = (adc_value[2])&0xFF;
            transmit_message.tx_data[6] = (adc_value[3]>>8)&0xFF;
            transmit_message.tx_data[7] = (adc_value[3])&0xFF;
//            printf("\r\n can0 transmit data:");
//            for(i = 0; i < transmit_message.tx_dlen; i++){
//                printf(" %02x", transmit_message.tx_data[i]);
//            }

            //TIMER_CTL1(TIMER7)=0b10000101;
            i=TIMER_CTL1(TIMER2);
            /* transmit message */
            transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
            /* waiting for transmit completed */
            timeout = 0xFFFF;
            while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
                timeout--;
            	}
            }


//            transmit_message.tx_data[2] = (GPIO_ISTAT(GPIOA)>>16)&0xFF;
//            transmit_message.tx_data[3] = GPIO_ISTAT(GPIOB)&0xFF;
//            transmit_message.tx_data[4] = (GPIO_ISTAT(GPIOB)>>8)&0xFF;
//            transmit_message.tx_data[5] = (GPIO_ISTAT(GPIOB)>>16)&0xFF;
//            transmit_message.tx_data[6] = GPIO_ISTAT(GPIOC)&0xFF;
//            transmit_message.tx_data[7] = (GPIO_ISTAT(GPIOC)>>8)&0xFF;
//            transmit_message.tx_data[0] = (adc_value[0]>>4)&0xFF;
//            transmit_message.tx_data[1] = (adc_value[1]>>4)&0xFF;
//            transmit_message.tx_data[2] = (adc_value[2]>>4)&0xFF;
//            transmit_message.tx_data[3] = (adc_value[3]>>4)&0xFF;
//            transmit_message.tx_data[4] = (adc_value[4]>>4)&0xFF;
//            transmit_message.tx_data[5] = (adc_value[5]>>4)&0xFF;
//            transmit_message.tx_data[6] = (adc_value[6]>>4)&0xFF;
//            transmit_message.tx_data[7] = (adc_value[7]>>4)&0xFF;
    }
}

/*!
    \brief      configure the different system clocks
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* enable GPIOC clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    /* enable GPIOA clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA0);
    /* enable TIMER0 clock */
    rcu_periph_clock_enable(RCU_TIMER0);
    /* enable ADC0 clock */
    rcu_periph_clock_enable(RCU_ADC0);
    /* enable ADC1 clock */
    rcu_periph_clock_enable(RCU_ADC1);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);
}

/*!
    \brief      initialize CAN and filter
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_networking_init(void)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;
    /* initialize CAN structures */
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    /* initialize CAN register */
    can_deinit(CAN0);

    /* initialize CAN */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_7TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
    /* baudrate 1Mbps */
    can_parameter.prescaler = 36;
    can_init(CAN0, &can_parameter);

    /* initialize filter */
    /* CAN0 filter number */
    can_filter.filter_number = 0;

    /* initialize filter */
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);
}

/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    /* enable can clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOD);
    /* configure CAN0 GPIO, CAN0_TX(PD1) and CAN0_RX(PD0) */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
    /* config the GPIO as analog mode */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7);

    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    //PB6: switch for DC/DC
    //PB5: switch for BatteryPlus display supply
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5|GPIO_PIN_6);

    GPIO_BOP(GPIOB) = GPIO_PIN_6; //DC/DC on
    //GPIO_BOP(GPIOB) = GPIO_PIN_5; // Display on



    /*configure PA8 PA9 PA10(TIMER0 CH0 CH1 CH2) as alternate function*/
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_8);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_10);

    /*configure PB13 PB14 PB15(TIMER0 CH0N CH1N CH2N) as alternate function*/
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_13);
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_14);
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_15);
}
/*!
    \brief      configure the DMA peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dma_config(void)
{
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;

    /* ADC_DMA_channel deinit */
    dma_deinit(DMA0, DMA_CH0);

    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr = (uint32_t)(adc_value);
    dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_32BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_32BIT;
    dma_data_parameter.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number = 8;
    dma_data_parameter.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);

    dma_circulation_enable(DMA0, DMA_CH0);

    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);
}
/*!
    \brief      configure the ADC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_config(void)
{
    /* configure the ADC sync mode */
    adc_mode_config(ADC_DAUL_REGULAL_FOLLOWUP_FAST);
    /* ADC scan mode function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC1, ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);

    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL,8);
    adc_channel_length_config(ADC1, ADC_REGULAR_CHANNEL,2);
    /* ADC regular channel config */
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_239POINT5); // PA0 Battery Current
    adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_6, ADC_SAMPLETIME_239POINT5); // PA6 Throttle?
    adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_7, ADC_SAMPLETIME_239POINT5); // PA7 Torque
    adc_regular_channel_config(ADC0, 3, ADC_CHANNEL_13, ADC_SAMPLETIME_239POINT5);// PC3 battery voltage
    adc_regular_channel_config(ADC0, 4, ADC_CHANNEL_13, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 5, ADC_CHANNEL_4, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 6, ADC_CHANNEL_7, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 7, ADC_CHANNEL_8, ADC_SAMPLETIME_239POINT5);

    adc_regular_channel_config(ADC1, 0, ADC_CHANNEL_2, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC1, 1, ADC_CHANNEL_3, ADC_SAMPLETIME_239POINT5);

    /* ADC external trigger enable */
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC1, ADC_REGULAR_CHANNEL, ENABLE);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_T1_CH1);
    adc_external_trigger_source_config(ADC1, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);

    /* enable ADC interface */
    adc_enable(ADC0);
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
    /* enable ADC interface */
    adc_enable(ADC1);
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC1);

    /* ADC DMA function enable */
    adc_dma_mode_enable(ADC0);
}

/*!
    \brief      configure the timer peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void timer0_config(void)
{
	/* -----------------------------------------------------------------------
	    TIMER0 configuration to:
	    generate 3 complementary PWM signals with 3 different duty cycles:
	    TIMER0CLK is fixed to systemcoreclock, the TIMER0 prescaler is equal to 6000 so the
	    TIMER0 counter clock used is 20KHz.
	    the three duty cycles are computed as the following description:
	    the channel 0 duty cycle is set to 25% so channel 1N is set to 75%.
	    the channel 1 duty cycle is set to 50% so channel 2N is set to 50%.
	    the channel 2 duty cycle is set to 75% so channel 3N is set to 25%.
	  ----------------------------------------------------------------------- */
	    timer_oc_parameter_struct timer_ocintpara;
	    timer_parameter_struct timer_initpara;
	    timer_break_parameter_struct timer_breakpara;
	    rcu_periph_clock_enable(RCU_TIMER0);

	    timer_deinit(TIMER0);

	    /* TIMER0 configuration */
	    timer_initpara.prescaler         = 0;
	    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_BOTH;
	    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
	    timer_initpara.period            = 5625; //for 32 kHz center aligned --> 16kHz PWM frequency
	    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
	    timer_initpara.repetitioncounter = 0;
	    timer_init(TIMER0,&timer_initpara);

	     /* CH1,CH2 and CH3 configuration in PWM mode */
	    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
	    timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
	    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
	    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_LOW;
	    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
	    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;

	    timer_channel_output_config(TIMER0,TIMER_CH_0,&timer_ocintpara);
	    timer_channel_output_config(TIMER0,TIMER_CH_1,&timer_ocintpara);
	    timer_channel_output_config(TIMER0,TIMER_CH_2,&timer_ocintpara);

	    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,2812);//grün
	    timer_channel_output_mode_config(TIMER0,TIMER_CH_0,TIMER_OC_MODE_PWM0);
	    timer_channel_output_shadow_config(TIMER0,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

	    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,2812+00);//gelb
	    timer_channel_output_mode_config(TIMER0,TIMER_CH_1,TIMER_OC_MODE_PWM0);
	    timer_channel_output_shadow_config(TIMER0,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

	    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,2812-00);//blau
	    timer_channel_output_mode_config(TIMER0,TIMER_CH_2,TIMER_OC_MODE_PWM0);
	    timer_channel_output_shadow_config(TIMER0,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);

	    /* automatic output enable, break, dead time and lock configuration*/
	    timer_breakpara.runoffstate      = TIMER_ROS_STATE_DISABLE;
	    timer_breakpara.ideloffstate     = TIMER_IOS_STATE_DISABLE ;
	    timer_breakpara.deadtime         = 16;
	    timer_breakpara.breakpolarity    = TIMER_BREAK_POLARITY_LOW;
	    timer_breakpara.outputautostate  = TIMER_OUTAUTO_ENABLE;
	    timer_breakpara.protectmode      = TIMER_CCHP_PROT_0;
	    timer_breakpara.breakstate       = TIMER_BREAK_DISABLE;
	    timer_break_config(TIMER0,&timer_breakpara);

	    timer_primary_output_config(TIMER0,ENABLE);

	    /* auto-reload preload enable */
	    timer_auto_reload_shadow_enable(TIMER0);
	    timer_enable(TIMER0);
}

void timer1_config(void)
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    /* TIMER0 configuration */
    timer_initpara.prescaler         = 2;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 9999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1, &timer_initpara);

    /* CH0 configuration in PWM mode0 */
    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocintpara);

    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 3999);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);



    /* TIMER0 primary output enable */
    timer_primary_output_config(TIMER1, ENABLE);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_UP);
        /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER1,TIMER_INT_FLAG_UP);

    /* enable TIMER0 */
    timer_enable(TIMER1);
}

void timer2_config(void)
{

    /* TIMER2 configuration: input capture mode */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER2);


    timer_deinit(TIMER2);
    /* hall mode config */
    timer_hall_mode_config(TIMER2, TIMER_HALLINTERFACE_ENABLE);



    /* TIMER2 configuration */
    timer_initpara.prescaler         = 119;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 0xFFFF;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2,&timer_initpara);

    /* TIMER2  configuration */
    /* TIMER2 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_ITS;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0;
    timer_input_capture_config(TIMER2,TIMER_CH_0,&timer_icinitpara);

    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_ITS;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0;
    timer_input_capture_config(TIMER2,TIMER_CH_1,&timer_icinitpara);

    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_ITS;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0;
    timer_input_capture_config(TIMER2,TIMER_CH_2,&timer_icinitpara);

    /* slave mode selection: TIMER2 */
    timer_input_trigger_source_select(TIMER2,TIMER_SMCFG_TRGSEL_CI0F_ED);
    timer_slave_mode_select(TIMER2,TIMER_SLAVE_MODE_RESTART);

    /* hall mode config */
    timer_hall_mode_config(TIMER2, TIMER_HALLINTERFACE_ENABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER2,TIMER_INT_FLAG_CH0);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER2,TIMER_INT_CH0);

    /* TIMER2 counter enable */
    timer_enable(TIMER2);

}
/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_config(void)
{
    /* configure CAN0 NVIC */
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn,0,0);

    //timer2 interrupt for Halls
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER1_IRQn, 0, 0);
    nvic_irq_enable(TIMER2_IRQn, 0, 0);

}

void TIMER2_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER2,TIMER_INT_FLAG_CH0)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER2,TIMER_INT_FLAG_CH0);


            /* read channel 0 capture value */
        	ui16_timertics= timer_channel_capture_value_register_read(TIMER2,TIMER_CH_0);
            //TIMER_CNT(TIMER2)=0;

                  	//Hall sensor event processing

            		ui8_hall_state = (GPIO_ISTAT(GPIOC)>>6)&0x07; //Mask input register with Hall 1 - 3 bits


            		ui8_hall_case=ui8_hall_state_old*10+ui8_hall_state;
            		if(MS.hall_angle_detect_flag){ //only process, if autodetect procedere is fininshed
            		ui8_hall_state_old=ui8_hall_state;
            		}

            			uint32_tics_filtered-=uint32_tics_filtered>>3;
            			uint32_tics_filtered+=ui16_timertics;

            		   ui8_overflow_flag=0;
            		   ui8_SPEED_control_flag=1;



            		switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
            				{
            			//6 cases for forward direction
            		//6 cases for forward direction
            		case 64:
            			q31_rotorposition_hall = Hall_64;

            			i8_recent_rotor_direction = -i16_hall_order;
            			uint16_full_rotation_counter = 0;
            			break;
            		case 45:
            			q31_rotorposition_hall = Hall_45;

            			i8_recent_rotor_direction = -i16_hall_order;
            			break;
            		case 51:
            			q31_rotorposition_hall = Hall_51;

            			i8_recent_rotor_direction = -i16_hall_order;
            			break;
            		case 13:
            			q31_rotorposition_hall = Hall_13;

            			i8_recent_rotor_direction = -i16_hall_order;
            			uint16_half_rotation_counter = 0;
            			break;
            		case 32:
            			q31_rotorposition_hall = Hall_32;

            			i8_recent_rotor_direction = -i16_hall_order;
            			break;
            		case 26:
            			q31_rotorposition_hall = Hall_26;

            			i8_recent_rotor_direction = -i16_hall_order;
            			break;

            			//6 cases for reverse direction
            		case 46:
            			q31_rotorposition_hall = Hall_64;

            			i8_recent_rotor_direction = i16_hall_order;
            			break;
            		case 62:
            			q31_rotorposition_hall = Hall_26;

            			i8_recent_rotor_direction = i16_hall_order;
            			break;
            		case 23:
            			q31_rotorposition_hall = Hall_32;

            			i8_recent_rotor_direction = i16_hall_order;
            			uint16_half_rotation_counter = 0;
            			break;
            		case 31:
            			q31_rotorposition_hall = Hall_13;

            			i8_recent_rotor_direction = i16_hall_order;
            			break;
            		case 15:
            			q31_rotorposition_hall = Hall_51;

            			i8_recent_rotor_direction = i16_hall_order;
            			break;
            		case 54:
            			q31_rotorposition_hall = Hall_45;

            			i8_recent_rotor_direction = i16_hall_order;
            			uint16_full_rotation_counter = 0;
            			break;

            		} // end case

            		if(MS.angle_est){
            			q31_PLL_error=q31_rotorposition_PLL-q31_rotorposition_hall;
            			if(iabs(q31_PLL_error) < deg_30){
            				if(ui_8_PLL_counter<12)ui_8_PLL_counter++;
            			}
            			else ui_8_PLL_counter=0;
            			q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall,0);
            		}

            	#ifdef SPEED_PLL
            		if(ui16_erps>30){   //360 interpolation at higher erps
            			if(ui8_hall_case==32||ui8_hall_case==23){
            				q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall, SPDSHFT*tics_higher_limit/(uint32_tics_filtered>>3));

            			}
            		}
            		else{

            			q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall, SPDSHFT*tics_higher_limit/(uint32_tics_filtered>>3));
            		}

            	#endif


    }

}

void TIMER1_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_UP)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_UP);


            /* read channel 0 capture value */
            counter ++;


    }
}

int32_t speed_PLL (int32_t ist, int32_t soll, uint8_t speedadapt)
  {
    int32_t q31_p;
    static int32_t q31_d_i = 0;
    static int32_t q31_d_dc = 0;
    //temp6 = soll-ist;
  //  temp5 = speedadapt;
    q31_p=(soll - ist)>>(P_FACTOR_PLL-speedadapt);   				//7 for Shengyi middrive, 10 for BionX IGH3
    q31_d_i+=(soll - ist)>>(I_FACTOR_PLL-speedadapt);				//11 for Shengyi middrive, 10 for BionX IGH3

    //clamp i part to twice the theoretical value from hall interrupts
    if (q31_d_i>((deg_30>>18)*500/ui16_timertics)<<16) q31_d_i = ((deg_30>>18)*500/ui16_timertics)<<16;
    if (q31_d_i<-((deg_30>>18)*500/ui16_timertics)<<16) q31_d_i =- ((deg_30>>18)*500/ui16_timertics)<<16;


    if (!ist&&!soll)q31_d_i=0;

    q31_d_dc=q31_p+q31_d_i;
    return (q31_d_dc);
  }

#ifdef GD_ECLIPSE_GCC
/* retarget the C library printf function to the USART, in Eclipse GCC environment */
int __io_putchar(int ch)
{
    usart_data_transmit(EVAL_COM0, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TBE));
    return ch;
}
#else
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM0, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TBE));

    return ch;
}
#endif /* GD_ECLIPSE_GCC */
