#include "remote_control.h"
#include "dma.h"
#include "usart.h"
#include "bsp_usart.h"
#include "stdlib.h"
#include "struct_typedef.h"
#include "bsp_usart.h"

uint16_t rc_receive_num = 0;

Rc_ctrl ctl;

void remote_control_init(void)
{
    RC_Init(sbus_buf[0], sbus_buf[1], SBUS_RX_BUF_NUM);
}

void sbus_to_rc(volatile const uint8_t *sbus_buf, Rc_ctrl *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    ctl.rc.ch0 = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;                              //!< Channel 0
    ctl.rc.ch1 = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;                       //!< Channel 1
    ctl.rc.ch2 = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) &0x07ff;  //!< Channel 2
    ctl.rc.ch3 = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;                       //!< Channel 3
    ctl.rc.s2 = ((sbus_buf[5] >> 4) & 0x0003);                                             //!< Switch left
    ctl.rc.s1 = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                                        //!< Switch right
    
    ctl.mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    ctl.mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    ctl.mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    ctl.mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press
    ctl.mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press
    ctl.key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
	
    //中心死区
    if(ctl.rc.ch1 - RC_CH_VALUE_OFFSET < ZeroCh && ctl.rc.ch1 - RC_CH_VALUE_OFFSET > -ZeroCh){
	 	ctl.rc.ch1 = RC_CH_VALUE_OFFSET;
	}
	if(ctl.rc.ch0 - RC_CH_VALUE_OFFSET < ZeroCh && ctl.rc.ch0 - RC_CH_VALUE_OFFSET > -ZeroCh){
		ctl.rc.ch0 = RC_CH_VALUE_OFFSET;
	}
	if(ctl.rc.ch2 - RC_CH_VALUE_OFFSET < ZeroCh && ctl.rc.ch2 - RC_CH_VALUE_OFFSET > -ZeroCh){
		ctl.rc.ch2 = RC_CH_VALUE_OFFSET;
	}
	if(ctl.rc.ch3 - RC_CH_VALUE_OFFSET < ZeroCh && ctl.rc.ch3 - RC_CH_VALUE_OFFSET > -ZeroCh){
		ctl.rc.ch3 = RC_CH_VALUE_OFFSET;
	}

    rc_receive_num++;
}

//串口中断
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_buf[0], &ctl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_buf[1], &ctl);
            }
        }
    }
}

