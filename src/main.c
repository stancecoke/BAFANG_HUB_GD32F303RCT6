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
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "gd32f307c_eval.h"

uint32_t adc_value[8];

//extern FlagStatus receive_flag;
extern can_receive_message_struct receive_message;
can_trasnmit_message_struct transmit_message;

void nvic_config(void);
void led_config(void);
void gpio_config(void);
void rcu_config(void);
void dma_config(void);
void adc_config(void);
void timer_config(void);
ErrStatus can_networking(void);
void can_networking_init(void);

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
    /* initialize the LEDs, USART and key */
    gd_eval_led_init(LED2);
    gd_eval_hall_init ();
    gd_eval_com_init(EVAL_COM0);
    adc_config();
    gpio_config();
    /* TIMER configuration */
    timer_config();
    /* DMA configuration */
    dma_config();
    /* ADC configuration */
    adc_config();

    
    /* print out the clock frequency of system, AHB, APB1 and APB2 */
    printf("\r\nCK_SYS is %d", rcu_clock_freq_get(CK_SYS));
    printf("\r\nCK_AHB is %d", rcu_clock_freq_get(CK_AHB));
    printf("\r\nCK_APB1 is %d", rcu_clock_freq_get(CK_APB1));
    printf("\r\nCK_APB2 is %d", rcu_clock_freq_get(CK_APB2));

    /* configure GPIO */
    gpio_config();

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

            delay_1ms(500);
            transmit_message.tx_data[0] = (GPIO_ISTAT(GPIOA))&0xFF;
            transmit_message.tx_data[1] = (adc_value[1]>>4)&0xFF;
            transmit_message.tx_data[2] = (adc_value[2]>>4)&0xFF;
            transmit_message.tx_data[3] = (adc_value[3]>>4)&0xFF;
            transmit_message.tx_data[4] = (adc_value[4]>>4)&0xFF;
            transmit_message.tx_data[5] = (adc_value[5]>>4)&0xFF;
            transmit_message.tx_data[6] = (adc_value[6]>>4)&0xFF;
            transmit_message.tx_data[7] = (adc_value[7]>>4)&0xFF;
            printf("\r\n can0 transmit data:");
            for(i = 0; i < transmit_message.tx_dlen; i++){
                printf(" %02x", transmit_message.tx_data[i]);
            }

            /* transmit message */
            transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
            /* waiting for transmit completed */
            timeout = 0xFFFF;
            while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
                timeout--;
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
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_0|GPIO_PIN_1);
    /*configure PA8(TIMER0 CH0) as alternate function*/
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    //Suche nach Pin zum Einschalten des Systems
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5|GPIO_PIN_6);

    GPIO_BOP(GPIOB) = GPIO_PIN_5|GPIO_PIN_6; //Pin ein
    //GPIO_BC(GPIOD) = GPIO_PIN_2;//|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3; //Pin aus

    //gpio_pin_remap_config(GPIO_CAN_FULL_REMAP,ENABLE);
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
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_1, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_2, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 3, ADC_CHANNEL_3, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 4, ADC_CHANNEL_4, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 5, ADC_CHANNEL_5, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 6, ADC_CHANNEL_6, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 7, ADC_CHANNEL_7, ADC_SAMPLETIME_239POINT5);

    adc_regular_channel_config(ADC1, 0, ADC_CHANNEL_2, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC1, 1, ADC_CHANNEL_3, ADC_SAMPLETIME_239POINT5);

    /* ADC external trigger enable */
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC1, ADC_REGULAR_CHANNEL, ENABLE);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_T0_CH0);
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
void timer_config(void)
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    /* TIMER0 configuration */
    timer_initpara.prescaler         = 8399;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 9999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0, &timer_initpara);

    /* CH0 configuration in PWM mode0 */
    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocintpara);

    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 3999);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    /* TIMER0 primary output enable */
    timer_primary_output_config(TIMER0, ENABLE);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER0);

    /* enable TIMER0 */
    timer_enable(TIMER0);
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
