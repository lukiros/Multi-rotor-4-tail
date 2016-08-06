/**********************************************************************
* $Id$		master.c 			   2011-01-09
*//**
* @file		I2C\I2C_Master\master.c 
* @brief		I2C master example
* @version	1.0
* @date		09.Jan.2011
* @author	NXP MCU SW Application Team
*
* Copyright(C) 2011, NXP Semiconductor
* All rights reserved.
*
***********************************************************************
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
* NXP Semiconductors assumes no responsibility or liability for the
* use of the software, conveys no license or title under any patent,
* copyright, or mask work right to the product. NXP Semiconductors
* reserves the right to make changes in the software without
* notification. NXP Semiconductors also make no representation or
* warranty that such application will be suitable for the specified
* use without further testing or modification.
* Permission to use, copy, modify, and distribute this software and its
* documentation is hereby granted, under NXP Semiconductors'
* relevant copyright in the software, without fee, provided that it
* is used in conjunction with NXP Semiconductors microcontrollers.  This
* copyright, permission, and disclaimer notice must appear in all copies of
* this code.
**********************************************************************/

#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc177x_8x_libcfg.h"
#else
#include "lpc177x_8x_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */
#include "lpc177x_8x_i2c.h"
#include "lpc177x_8x_pinsel.h"
#include "debug_frmwrk.h"
#include "LPC177x_8x.h"
#include "lpc177x_8x_clkpwr.h"
#include "GY-86.h"
#include "CMPS10 Compass.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "lpc177x_8x_pwm.h"
#include "lpc177x_8x_gpio.h"
#include "math.h"
#include "lpc177x_8x_adc.h"
#include "lpc177x_8x_wwdt.h"

/************************** PRIVATE DEFINITIONS *************************/
//****************I2x**********
#define I2CDEV_TRANSFER_POLLING        1  /*0: interrupt mode, 1: polling mode */
#define I2CDEV_M		(2)
#define UART_TEST_NUM		1
//****************************

//************************UART*************************
#if (UART_TEST_NUM == 0)
#define	_LPC_UART			UART_0
#define _UART_IRQ			UART0_IRQn
#define _UART_IRQHander		UART0_IRQHandler
#elif (UART_TEST_NUM == 1)
#define _LPC_UART			UART_1
#define _UART_IRQ			UART1_IRQn
#define _UART_IRQHander		UART1_IRQHandler
#elif (UART_TEST_NUM == 2)
#define _LPC_UART			UART_2
#define _UART_IRQ			UART2_IRQn
#define _UART_IRQHander		UART2_IRQHandler
#elif (UART_TEST_NUM == 3)
#define _LPC_UART			UART_3
#define _UART_IRQ			UART3_IRQn
#define _UART_IRQHander		UART3_IRQHandler
#elif (UART_TEST_NUM == 4)
#define _LPC_UART			UART_4
#define _UART_IRQ			UART4_IRQn
#define _UART_IRQHander		UART4_IRQHandler
#endif
/* buffer size definition */
#define UART_RING_BUFSIZE 512

/* Buf mask */
#define __BUF_MASK (UART_RING_BUFSIZE-1)
/* Check buf is full or not */
#define __BUF_IS_FULL(head, tail) ((tail&__BUF_MASK)==((head+1)&__BUF_MASK))
/* Check buf will be full in next receiving or not */
#define __BUF_WILL_FULL(head, tail) ((tail&__BUF_MASK)==((head+2)&__BUF_MASK))
/* Check buf is empty */
#define __BUF_IS_EMPTY(head, tail) ((head&__BUF_MASK)==(tail&__BUF_MASK))
/* Reset buf */
#define __BUF_RESET(bufidx)	(bufidx=0)
#define __BUF_INCR(bufidx)	(bufidx=(bufidx+1)&__BUF_MASK)
//******************************************************
#define _USING_PWM_NO					0
//******************************************************

#define PI 3.14159265

/************************** PRIVATE TYPES *************************/
/** @brief UART Ring buffer structure */
typedef struct
{
    __IO uint32_t tx_head;                /*!< UART Tx ring buffer head index */
    __IO uint32_t tx_tail;                /*!< UART Tx ring buffer tail index */
    __IO uint32_t rx_head;                /*!< UART Rx ring buffer head index */
    __IO uint32_t rx_tail;                /*!< UART Rx ring buffer tail index */
    __IO uint8_t  tx[UART_RING_BUFSIZE];  /*!< UART Tx data ring buffer */
    __IO uint8_t  rx[UART_RING_BUFSIZE];  /*!< UART Rx data ring buffer */
} UART_RING_BUFFER_T;

typedef struct
{
	float kp;
	float ki;
	float kd;
	
	float h;
	float a;
	float Tf;
	
	float u[2]; // output
	float e[3]; // error
	float ef[3]; // ?
	
} PID_e;

// UART Ring buffer
UART_RING_BUFFER_T rb;

// Current Tx Interrupt enable state
__IO FlagStatus TxIntStat;
__IO Bool complete;
__IO uint32_t match_cnt = 0;
extern MPU6050_sensor mpu6050;
extern HMC5883_sensor hmc5883;
extern CPMS10_sensor cmps10;


__IO PID_e ac_x;
__IO PID_e ac_y;
__IO PID_e gy_z;
__IO PID_e gy_y;
__IO PID_e gy_x;

__IO MPU6050_sensor sum_mpu6050;
__IO MPU6050_sensor total_mpu6050;
__IO float offset_GX=0;
__IO float offset_GY=0;
__IO float offset_GZ=0;

	float gain_acc=0;
__IO int speed = 0; // input speed
	int gx = 0;
	int gy = 0;
	int gz = 0;
__IO uint32_t channelVal = 59989;

	int igy_x = 0; // float to int total_mpu6050.GY_X
	int igy_y = 0; // float to int total_mpu6050.GY_Y
	int igy_z = 0; // float to int total_mpu6050.GY_Z

__IO float pitch=0;
__IO float roll=0;
__IO float yaw=0;

__IO float adc_v =0;
__IO float adc_c = 0;
__IO float offset_c =0;

int adc1=0;
int adc2=0;
int len=0;
int filter_time = 250;

	uint32_t report=0;
	uint32_t timeout=0;
	
	char buffer[512] = {0};
	unsigned char *new_pointer2 = (unsigned char*) buffer;
	char data[512] = {0};
	char bytes[512]= {0};
	unsigned char *new_pointer = (unsigned char*) bytes;
	int i = 0;

	char searh_dec[] ="0123456789";
// 	char searh_text[] ="zxcvbnmasdfghjklqwertyuiop %";
	char *str_start, *str_end;

uint16_t counter1=0;
uint16_t counter2=0;

/************************** PRIVATE FUNCTIONS *************************/
/* Interrupt service routines */
void _UART_IRQHander(void);
void UART_IntErr(uint8_t bLSErrType);
void UART_IntTransmit(void);
void UART_IntReceive(void);
void UART_init(void);
	
#if (_USING_PWM_NO == 1)
void PWM1_IRQHandler(void);
#elif (_USING_PWM_NO == 0)
void PWM0_IRQHandler(void);
#endif

uint32_t UARTReceive(UART_ID_Type UartID, uint8_t *rxbuf, uint32_t buflen);
uint32_t UARTSend(UART_ID_Type UartID, uint8_t *txbuf, uint32_t buflen);

void Report_status(void);
void INT_MPU6050_CPMS10(void);
/** These global variables below used in interrupt mode - Slave device ----------------*/

/************************** PRIVATE FUNCTIONS *************************/
void Buffer_Init(uint8_t type);

/* SysTick Counter */
volatile unsigned long SysTickCnt;

/************************** PRIVATE FUNCTIONS *************************/
void SysTick_Handler (void);
void Delay (unsigned long tick);

/*----------------- INTERRUPT SERVICE ROUTINES --------------------------*/
/*********************************************************************//**
 * @brief		UART0 interrupt handler sub-routine
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void _UART_IRQHander(void)
{
	uint32_t intsrc, tmp, tmp1;

	/* Determine the interrupt source */
	intsrc = UART_GetIntId(_LPC_UART);
	tmp = intsrc & UART_IIR_INTID_MASK;

	// Receive Line Status
	if (tmp == UART_IIR_INTID_RLS){
		// Check line status
		tmp1 = UART_GetLineStatus(_LPC_UART);
		// Mask out the Receive Ready and Transmit Holding empty status
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
				| UART_LSR_BI | UART_LSR_RXFE);
		// If any error exist
		if (tmp1) {
				UART_IntErr(tmp1);
		}
	}

	// Receive Data Available or Character time-out
	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)){
			UART_IntReceive();
	}

	// Transmit Holding Empty
	if (tmp == UART_IIR_INTID_THRE){
			UART_IntTransmit();
	}

}

/********************************************************************//**
 * @brief 		UART receive function (ring buffer used)
 * @param[in]	None
 * @return 		None
 *********************************************************************/
void UART_IntReceive(void)
{
	uint8_t tmpc;
	uint32_t rLen;

	while(1){
		// Call UART read function in UART driver
		rLen = UART_Receive(_LPC_UART, &tmpc, 1, NONE_BLOCKING);
		// If data received
		if (rLen){
			/* Check if buffer is more space
			 * If no more space, remaining character will be trimmed out
			 */
			if (!__BUF_IS_FULL(rb.rx_head,rb.rx_tail)){
				rb.rx[rb.rx_head] = tmpc;
				__BUF_INCR(rb.rx_head);
			}
		}
		// no more data
		else {
			break;
		}
	}
}

/********************************************************************//**
 * @brief 		UART transmit function (ring buffer used)
 * @param[in]	None
 * @return 		None
 *********************************************************************/
void UART_IntTransmit(void)
{
    // Disable THRE interrupt
    UART_IntConfig(_LPC_UART, UART_INTCFG_THRE, DISABLE);

	/* Wait for FIFO buffer empty, transfer UART_TX_FIFO_SIZE bytes
	 * of data or break whenever ring buffers are empty */
	/* Wait until THR empty */
    while (UART_CheckBusy(_LPC_UART) == SET);

	while (!__BUF_IS_EMPTY(rb.tx_head,rb.tx_tail))
    {
        /* Move a piece of data into the transmit FIFO */
    	if (UART_Send(_LPC_UART, (uint8_t *)&rb.tx[rb.tx_tail], 1, NONE_BLOCKING)){
        /* Update transmit ring FIFO tail pointer */
        __BUF_INCR(rb.tx_tail);
    	} else {
    		break;
    	}
    }

    /* If there is no more data to send, disable the transmit
       interrupt - else enable it or keep it enabled */
	if (__BUF_IS_EMPTY(rb.tx_head, rb.tx_tail)) {
    	UART_IntConfig(_LPC_UART, UART_INTCFG_THRE, DISABLE);
    	// Reset Tx Interrupt state
    	TxIntStat = RESET;
    }
    else{
      	// Set Tx Interrupt state
		TxIntStat = SET;
    	UART_IntConfig(_LPC_UART, UART_INTCFG_THRE, ENABLE);
    }
}


/*********************************************************************//**
 * @brief		UART Line Status Error
 * @param[in]	bLSErrType	UART Line Status Error Type
 * @return		None
 **********************************************************************/
void UART_IntErr(uint8_t bLSErrType)
{
	// Loop forever
	while (1){
		// For testing purpose
	}
}

/*-------------------------PRIVATE FUNCTIONS------------------------------*/
/*********************************************************************//**
 * @brief		UART transmit function for interrupt mode (using ring buffers)
 * @param[in]	UARTPort	Selected UART peripheral used to send data,
 * 				should be UART0
 * @param[out]	txbuf Pointer to Transmit buffer
 * @param[in]	buflen Length of Transmit buffer
 * @return 		Number of bytes actually sent to the ring buffer
 **********************************************************************/
uint32_t UARTSend(UART_ID_Type UartID, uint8_t *txbuf, uint32_t buflen)
{
    uint8_t *data = (uint8_t *) txbuf;
    uint32_t bytes = 0;

	/* Temporarily lock out UART transmit interrupts during this
	   read so the UART transmit interrupt won't cause problems
	   with the index values */
    UART_IntConfig(UartID, UART_INTCFG_THRE, DISABLE);

	/* Loop until transmit run buffer is full or until n_bytes
	   expires */
	while ((buflen > 0) && (!__BUF_IS_FULL(rb.tx_head, rb.tx_tail)))
	{
		/* Write data from buffer into ring buffer */
		rb.tx[rb.tx_head] = *data;
		data++;

		/* Increment head pointer */
		__BUF_INCR(rb.tx_head);

		/* Increment data count and decrement buffer size count */
		bytes++;
		buflen--;
	}

	/*
	 * Check if current Tx interrupt enable is reset,
	 * that means the Tx interrupt must be re-enabled
	 * due to call UART_IntTransmit() function to trigger
	 * this interrupt type
	 */
	if (TxIntStat == RESET) {
		UART_IntTransmit();
	}
	/*
	 * Otherwise, re-enables Tx Interrupt
	 */
	else {
		UART_IntConfig(UartID, UART_INTCFG_THRE, ENABLE);
	}

    return bytes;
}


/*********************************************************************//**
 * @brief		UART read function for interrupt mode (using ring buffers)
 * @param[in]	UARTPort	Selected UART peripheral used to send data,
 * 				should be UART0
 * @param[out]	rxbuf Pointer to Received buffer
 * @param[in]	buflen Length of Received buffer
 * @return 		Number of bytes actually read from the ring buffer
 **********************************************************************/
uint32_t UARTReceive(UART_ID_Type UartID, uint8_t *rxbuf, uint32_t buflen)
{
    uint8_t *data = (uint8_t *) rxbuf;
    uint32_t bytes = 0;

	/* Temporarily lock out UART receive interrupts during this
	   read so the UART receive interrupt won't cause problems
	   with the index values */
	UART_IntConfig(UartID, UART_INTCFG_RBR, DISABLE);

	/* Loop until receive buffer ring is empty or
		until max_bytes expires */
	while ((buflen > 0) && (!(__BUF_IS_EMPTY(rb.rx_head, rb.rx_tail))))
	{
		/* Read data from ring buffer into user buffer */
		*data = rb.rx[rb.rx_tail];
		data++;

		/* Update tail pointer */
		__BUF_INCR(rb.rx_tail);

		/* Increment data count and decrement buffer size count */
		bytes++;
		buflen--;
	}

	/* Re-enable UART interrupts */
	UART_IntConfig(UartID, UART_INTCFG_RBR, ENABLE);

    return bytes;
}

void SysTick_Handler (void)
{
	SysTickCnt++;
}

#if (_USING_PWM_NO == 1)
void PWM1_IRQHandler(void)
#elif (_USING_PWM_NO == 0)
void PWM0_IRQHandler(void)
#endif
{
	/* Check whether if match flag for channel 0 is set or not */
	if (PWM_GetIntStatus(_USING_PWM_NO, PWM_INTSTAT_MR0))
	{
		match_cnt++;

		/* Clear the interrupt flag */
		PWM_ClearIntPending(_USING_PWM_NO, PWM_INTSTAT_MR0);
	}
}

void Delay (unsigned long tick)
{
	unsigned long systickcnt;

	systickcnt = SysTickCnt;
	while ((SysTickCnt - systickcnt) < tick);
}

void UART_init(void)
{
	// UART Configuration structure variable
	UART_CFG_Type UARTConfigStruct;
	// UART FIFO configuration Struct variable
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	
	#if (UART_TEST_NUM == 0)
	/*
	 * Initialize UART0 pin connect
	 * P0.2: U0_TXD
	 * P0.3: U0_RXD
	 */
	PINSEL_ConfigPin(0,2,1);
	PINSEL_ConfigPin(0,3,1);
#elif (UART_TEST_NUM == 1)
	/*
	 * Initialize UART1 pin connect
	 * P0.15: U1_TXD
	 * P0.16: U1_RXD
	 */
	PINSEL_ConfigPin(0,15,1);
	PINSEL_ConfigPin(0,16,1);
#elif (UART_TEST_NUM == 2)
	/*
	 * Initialize UART2 pin connect
	 * P0.10: U2_TXD
	 * P0.11: U2_RXD
	 */
	PINSEL_ConfigPin(0,10,1);
	PINSEL_ConfigPin(0,11,1);
#elif (UART_TEST_NUM == 3)
	/*
	 * Initialize UART2 pin connect
	 * P0.2: U3_TXD
	 * P0.3: U3_RXD
	 */
	PINSEL_ConfigPin(0,2,2);
	PINSEL_ConfigPin(0,3,2);
#elif (UART_TEST_NUM == 4)
	/*
	 * Initialize UART2 pin connect
	 * P0.22: U4_TXD
	 * P2.9: U4_RXD
	 */
	PINSEL_ConfigPin(0,22,3);
	PINSEL_ConfigPin(2,9,3);
#endif
	
	UART_ConfigStructInit(&UARTConfigStruct);
	UART_Init(_LPC_UART, &UARTConfigStruct);
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
	UART_FIFOConfig(_LPC_UART, &UARTFIFOConfigStruct);
	UART_TxCmd(_LPC_UART, ENABLE);
	UART_IntConfig(_LPC_UART, UART_INTCFG_RBR, ENABLE);
	UART_IntConfig(_LPC_UART, UART_INTCFG_RLS, ENABLE);
	/*
	 * Do not enable transmit interrupt here, since it is handled by
	 * UART_Send() function, just to reset Tx Interrupt state for the
	 * first time
	 */
	TxIntStat = RESET;

	// Reset ring buf head and tail idx
	__BUF_RESET(rb.rx_head);
	__BUF_RESET(rb.rx_tail);
	__BUF_RESET(rb.tx_head);
	__BUF_RESET(rb.tx_tail);
	
  NVIC_SetPriority(_UART_IRQ, ((0x01<<3)|0x01));
  NVIC_EnableIRQ(_UART_IRQ);
	
	debug_frmwrk_init();
	
}
/*-------------------------PRIVATE FUNCTIONS-----------------------------*/

#if (I2CDEV_TRANSFER_POLLING == 0)
/*********************************************************************//**
 * @brief		I2C Interrupt Handler
 * @param[in]	None
 *
 * @return 		None
 **********************************************************************/
#if ((I2CDEV_M == 0))
void I2C0_IRQHandler(void)
#elif ((I2CDEV_M == 1))
void I2C1_IRQHandler(void)
#elif ((I2CDEV_M == 2))
void I2C2_IRQHandler(void)
#else
void I2C_IRQHandler(void)
#endif
{
	 I2C_MasterHandler((en_I2C_unitId)I2CDEV_M);
	if (I2C_MasterTransferComplete((en_I2C_unitId)I2CDEV_M)){
		  complete = TRUE;
	}   
}
#endif

void PID_control(void)
{
// 	float kp;
// 	float ki;
// 	float kd;
// 	
// 	float h;
// 	float a;
// 	float Tf;
// 	
// 	float u[2]; // output
// 	float e[3]; // error
// 	float ef[3]; // ?
// __IO PID_e ac_x;
// __IO PID_e ac_y;
// __IO PID_e gy_z;
// __IO PID_e gy_y;
// __IO PID_e gy_x;
	
	
	
	ac_x.Tf = 0.1 * ac_x.kd;
	ac_x.h = 0.02 * counter2;
	ac_x.a =  ac_x.h / (ac_x.Tf + ac_x.h);
	ac_x.u[1] = ac_x.u[0];
	ac_x.ef[2] = ac_x.ef[1];
	ac_x.ef[1] = ac_x.ef[0];
	ac_x.e[2] = ac_x.e[1];
	ac_x.e[1] = ac_x.e[0];
	ac_x.e[0] = roll - 0;//roll - setpoint
	ac_x.ef[0] = ((1-ac_x.a) * ac_x.ef[1]) + (ac_x.a * ac_x.e[0]);
	ac_x.u[0] = ac_x.u[1] + ((ac_x.kp * (ac_x.e[0] - ac_x.e[1])) + ((ac_x.kp * ac_x.h / ac_x.ki) * ac_x.e[0]) + ((ac_x.kp * ac_x.kd / ac_x.h) * (ac_x.ef[0] - (2 * ac_x.ef[1]) + ac_x.ef[2])));
	if ((ac_x.u[0] > 100) || ac_x.u[0] < -100)
	{
		if (ac_x.u[0] > 100) ac_x.u[0] = 100;
		if (ac_x.u[0] < -100) ac_x.u[0] = -100;
	}
	
	ac_y.Tf = 0.1 * ac_y.kd;
	ac_y.h = 0.02 * counter2;
	ac_y.a =  ac_y.h / (ac_y.Tf + ac_y.h);
	ac_y.u[1] = ac_y.u[0];
	ac_y.ef[2] = ac_y.ef[1];
	ac_y.ef[1] = ac_y.ef[0];
	ac_y.e[2] = ac_y.e[1];
	ac_y.e[1] = ac_y.e[0];
	ac_y.e[0] = pitch - 0;//pitch - setpoint
	ac_y.ef[0] = ((1-ac_y.a) * ac_y.ef[1]) + (ac_y.a * ac_y.e[0]);
	ac_y.u[0] = ac_y.u[1] + ((ac_y.kp * (ac_y.e[0] - ac_y.e[1])) + ((ac_y.kp * ac_y.h / ac_y.ki) * ac_y.e[0]) + ((ac_y.kp * ac_y.kd / ac_y.h) * (ac_y.ef[0] - (2 * ac_y.ef[1]) + ac_y.ef[2])));
	if ((ac_y.u[0] > 100) || ac_y.u[0] < -100)
	{
		if (ac_y.u[0] > 100) ac_y.u[0] = 100;
		if (ac_y.u[0] < -100) ac_y.u[0] = -100;
	}
	
		ac_x.Tf = 0.1 * ac_x.kd;
	ac_x.h = 0.02 * counter2;
	ac_x.a =  ac_x.h / (ac_x.Tf + ac_x.h);
	ac_x.u[1] = ac_x.u[0];
	ac_x.ef[2] = ac_x.ef[1];
	ac_x.ef[1] = ac_x.ef[0];
	ac_x.e[2] = ac_x.e[1];
	ac_x.e[1] = ac_x.e[0];
	ac_x.e[0] = roll - 0;//roll - setpoint
	ac_x.ef[0] = ((1-ac_x.a) * ac_x.ef[1]) + (ac_x.a * ac_x.e[0]);
	ac_x.u[0] = ac_x.u[1] + ((ac_x.kp * (ac_x.e[0] - ac_x.e[1])) + ((ac_x.kp * ac_x.h / ac_x.ki) * ac_x.e[0]) + ((ac_x.kp * ac_x.kd / ac_x.h) * (ac_x.ef[0] - (2 * ac_x.ef[1]) + ac_x.ef[2])));
	if ((ac_x.u[0] > 100) || ac_x.u[0] < -100)
	{
		if (ac_x.u[0] > 100) ac_x.u[0] = 100;
		if (ac_x.u[0] < -100) ac_x.u[0] = -100;
	}
	
	gy_x.Tf = 0.1 * gy_x.kd;
	gy_x.h = 0.02 * counter2;
	gy_x.a =  gy_x.h / (gy_x.Tf + gy_x.h);
	gy_x.u[1] = gy_x.u[0];
	gy_x.ef[2] = gy_x.ef[1];
	gy_x.ef[1] = gy_x.ef[0];
	gy_x.e[2] = gy_x.e[1];
	gy_x.e[1] = gy_x.e[0];
	gy_x.e[0] = (float)(igy_x - gx);//pitch - setpoint
	gy_x.ef[0] = ((1-gy_x.a) * gy_x.ef[1]) + (gy_x.a * gy_x.e[0]);
	gy_x.u[0] = ((gy_x.kp * gy_x.e[0]) + ((gy_x.kp * gy_x.h / gy_x.ki) * gy_x.e[0]) + ((gy_x.kp * gy_x.kd / gy_x.h) * (gy_x.ef[0] - (2 * gy_x.ef[1]) + gy_x.ef[2])));
	if ((gy_x.u[0] > 100) || gy_x.u[0] < -100)
	{
		if (gy_x.u[0] > 100) gy_x.u[0] = 100;
		if (gy_x.u[0] < -100) gy_x.u[0] = -100;
	}
	
	gy_y.Tf = 0.1 * gy_y.kd;
	gy_y.h = 0.02 * counter2;
	gy_y.a =  gy_y.h / (gy_y.Tf + gy_y.h);
	gy_y.u[1] = gy_y.u[0];
	gy_y.ef[2] = gy_y.ef[1];
	gy_y.ef[1] = gy_y.ef[0];
	gy_y.e[2] = gy_y.e[1];
	gy_y.e[1] = gy_y.e[0];
	gy_y.e[0] = (float)(igy_y  - gy);//pitch - setpoint
	gy_y.ef[0] = ((1-gy_y.a) * gy_y.ef[1]) + (gy_y.a * gy_y.e[0]);
	gy_y.u[0] = ((gy_y.kp * gy_y.e[0]) + ((gy_y.kp * gy_y.h / gy_y.ki) * gy_y.e[0]) + ((gy_y.kp * gy_x.kd / gy_y.h) * (gy_y.ef[0] - (2 * gy_y.ef[1]) + gy_y.ef[2])));
	if ((gy_y.u[0] > 100) || gy_y.u[0] < -100)
	{
		if (gy_y.u[0] > 100) gy_y.u[0] = 100;
		if (gy_y.u[0] < -100) gy_y.u[0] = -100;
	}
	
	
	
}

void INT_MPU6050_CPMS10(void)
{
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
	ADC_StartCmd(LPC_ADC, ADC_START_NOW);
	//Wait conversion complete
	while (!(ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE)));
	adc1 += ADC_ChannelGetData(LPC_ADC, 0);
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, DISABLE);
	
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, ENABLE);
	ADC_StartCmd(LPC_ADC, ADC_START_NOW);
	//Wait conversion complete
	while (!(ADC_ChannelGetStatus(LPC_ADC, 1, ADC_DATA_DONE)));
	adc2 += ADC_ChannelGetData(LPC_ADC, 1);
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, DISABLE);
	
	if (MPU6050_ReadAll(&mpu6050) != 0)	
	{
		counter1++;
		sum_mpu6050.AC_X += mpu6050.AC_X;
		sum_mpu6050.AC_Y += mpu6050.AC_Y;
		sum_mpu6050.AC_Z += mpu6050.AC_Z;
		sum_mpu6050.Temp += mpu6050.Temp;
		sum_mpu6050.GY_X += mpu6050.GY_X;
		sum_mpu6050.GY_Y += mpu6050.GY_Y;
		sum_mpu6050.GY_Z += mpu6050.GY_Z;
	}
	if (counter2 >= filter_time)
	{
		if (HMC5883_ReadAll(&hmc5883) != 0)
		{
			
		}
		
		adc_v = (3.3 / 4095 * 10 * (adc1 / counter1));
		adc_c = (((3.3 / 4096 * (adc2 / counter1) * 2) - 2.5) / 0.01) - offset_c;
		
		if (adc_c > 130)
			GPIO_OutputValue(4, (1<<27), 0);
			
		//CMPS10_RaadAll(&cmps10);
		
		if (gain_acc == 0)
		{
		total_mpu6050.AC_X = (sum_mpu6050.AC_X / (float)counter1);
		total_mpu6050.AC_Y = (sum_mpu6050.AC_Y / (float)counter1);
		total_mpu6050.AC_Z = (sum_mpu6050.AC_Z / (float)counter1);
		total_mpu6050.Temp = sum_mpu6050.Temp / (float)counter1;
		gain_acc = 1;
		}
		else
		{
			total_mpu6050.AC_X = (sum_mpu6050.AC_X / (float)counter1) * gain_acc;
			total_mpu6050.AC_Y = (sum_mpu6050.AC_Y / (float)counter1) * gain_acc;
			total_mpu6050.AC_Z = (sum_mpu6050.AC_Z / (float)counter1) * gain_acc;
			total_mpu6050.Temp = sum_mpu6050.Temp / (float)counter1;
		}
		
		total_mpu6050.GY_X = (sum_mpu6050.GY_X / (float)counter1) - offset_GX;
		total_mpu6050.GY_Y = (sum_mpu6050.GY_Y / (float)counter1) - offset_GY;
		total_mpu6050.GY_Z = (sum_mpu6050.GY_Z / (float)counter1) - offset_GZ;
		
		igy_x = total_mpu6050.GY_X;
		igy_y = total_mpu6050.GY_Y;
		igy_z = total_mpu6050.GY_Z;
				
		pitch = atan(total_mpu6050.AC_Y / sqrt((total_mpu6050.AC_X * total_mpu6050.AC_X) + (total_mpu6050.AC_Z * total_mpu6050.AC_Z))) * 180 / PI;
		roll = atan(total_mpu6050.AC_X / sqrt((total_mpu6050.AC_Y * total_mpu6050.AC_Y) + (total_mpu6050.AC_Z * total_mpu6050.AC_Z))) * 180 / PI;
		
// 		pitch = atan(total_mpu6050.AC_Y / sqrt((total_mpu6050.AC_X * total_mpu6050.AC_X) + (total_mpu6050.AC_Z * total_mpu6050.AC_Z) + (total_mpu6050.AC_Y * total_mpu6050.AC_Y))) * 180 / PI;
// 		roll = atan(total_mpu6050.AC_X / sqrt((total_mpu6050.AC_Y * total_mpu6050.AC_Y) + (total_mpu6050.AC_Z * total_mpu6050.AC_Z)) + (total_mpu6050.AC_X * total_mpu6050.AC_X)) * 180 / PI;

		if ((offset_GX == 0) & (offset_GY == 0) & (offset_GZ == 0))
		{
			offset_GX = total_mpu6050.GY_X;
			offset_GY = total_mpu6050.GY_Y;
			offset_GZ = total_mpu6050.GY_Z;
			filter_time = 3;
			offset_c = adc_c;
		}
		//****************PID loop*********************
		else
		{
			PID_control();
		}
			
		sum_mpu6050.AC_X = 0;
		sum_mpu6050.AC_Y = 0;
		sum_mpu6050.AC_Z = 0;
		sum_mpu6050.Temp = 0;
		sum_mpu6050.GY_X = 0;
		sum_mpu6050.GY_Y = 0;
		sum_mpu6050.GY_Z = 0;
		
		adc1=0;
		adc2=0;
		
		counter1 = 0;
		counter2 = 0;
		
	}
}

void Report_status(void)
{
	//memset(bytes, 0, sizeof bytes);
// 	i = sprintf(bytes, "AX %.1f AY %.1f AZ %.1f\r\nTemp %.1f\r\nGX %.1f GY %.1f GZ %.1f\r\n pitch %.1f Roll %.1f\r\nCompass %d Pitch %d Roll %.1d \r\nAX %.1f AY %.1f AZ %.1f V %.1f C %.1f\r\n",
// 	total_mpu6050.AC_X, total_mpu6050.AC_Y, total_mpu6050.AC_Z, total_mpu6050.Temp, total_mpu6050.GY_X, total_mpu6050.GY_Y, total_mpu6050.GY_Z,pitch,roll, 
// 	cmps10.Compass, cmps10.Pitch, cmps10.Roll, cmps10.AC_X, cmps10.AC_Y, cmps10.AC_Z, adc_v, adc_c);
// 	UARTSend(_LPC_UART, new_pointer, i);
	i = sprintf(bytes,
	"Temp %.1f pitch %.1f Roll %.1f GX %d GY %d GZ %d\r\nCompass %d\r\nV %.1f C %.1f\r\n Uax %.1f Uay %.1f Ugx % .1f Ugy %.1f\r\nSP %d GX %d GY %d\r\n\n",
	total_mpu6050.Temp, pitch, roll, igy_x, igy_y, igy_z, cmps10.Compass, adc_v, adc_c, ac_x.u[0], ac_y.u[0], gy_x.u[0], gy_y.u[0], speed, gx, gy);
	UARTSend(_LPC_UART, new_pointer, i);
//, hmc5883.MAG_X, hmc5883.MAG_Y, hmc5883.MAG_Z M %d %d %d\r\n
// 	i = sprintf(bytes, "M %d %d %d \r\n\n", hmc5883.MAG_X, hmc5883.MAG_Y, hmc5883.MAG_Z);
// 	UARTSend(_LPC_UART, new_pointer, i);
}

void Get_data(void)
{
			memset(buffer, 0, sizeof buffer);
   		len = UARTReceive(_LPC_UART, new_pointer2, sizeof(buffer));
			if (len != 0)
			{
				strcat(data, buffer);
				str_start = strstr(data, "CMD");
				str_end = strstr(data, "END");
				if (str_start > str_end)
					return;
				
				if ((str_start != 0) & (str_end != 0))
				{
					if (*(str_start+3) == '0') //mode 0 Control via gyro data(2B GY_X),(2B GY_Y),(2B GY_Z),(2B Speed)
					{
						str_start = str_start+3;
						str_end = strstr(str_start, "GX");
						if ((str_end != 0) & (str_start != 0))
						{
							memset(buffer, 0, sizeof buffer);
							memcpy(buffer , str_start+1, str_end - str_start);
							gx = atoi(buffer);
						}
						
						str_start = str_end + 2;
						str_end = strstr(str_start, "GY");
						if ((str_end != 0) & (str_start != 0))
						{
							memset(buffer, 0, sizeof buffer);
							memcpy(buffer , str_start, str_end - str_start);
							gy = atoi(buffer);
						}
						
						str_start = str_end + 2;
						str_end = strstr(str_start, "GZ");
						if ((str_end != 0) & (str_start != 0))
						{
							memset(buffer, 0, sizeof buffer);
							memcpy(buffer , str_start, str_end - str_start);
							gz = atoi(buffer);
						}
						
						str_start = str_end + 2;
						str_end = strstr(str_start, "SP");
						if ((str_end != 0) & (str_start != 0))
						{
							memset(buffer, 0, sizeof buffer);
							memcpy(buffer , str_start, str_end - str_start);
							speed = atoi(buffer);
						}
							
					}
					else if (*(str_start+3) == '1') //mode 1 Control via Accelerometer (2B Roll),(2B Picth),(2B Compass(Heading)),(2BSpeed)
					{
						
					}
					else if (*(str_start+3) == '2') //mode 2 Control via GPS
					{
						
					}
					else if (*(str_start+3) == '3') //mode 3 Turn on driver
						GPIO_OutputValue(4, (1<<27), 1);
					else if (*(str_start+3) == '4') //mode 4 Turn off driver
						GPIO_OutputValue(4, (1<<27), 0);
					else if (*(str_start+3) == '5') //mode 5 Turn off controler
						GPIO_OutputValue(4, (1<<26), 0);
					else if (*(str_start+3) == '6') //turn pid control
					{
						
					}
					else
					{
						
					}
					report++;
					memset(data, 0, sizeof data);
					timeout = 0;
					if (report > 0)
					{
						report = 0;
						Report_status();
					}
				}
			}
}

/*-------------------------MAIN FUNCTION------------------------------*/
/*********************************************************************//**
 * @brief		c_entry: Main program body
 * @param[in]	None
 * @return 		int
 **********************************************************************/
int c_entry(void)
{
	PWM_TIMERCFG_Type PWMCfgDat;
	PWM_MATCHCFG_Type PWMMatchCfgDat;
	uint32_t speed_1 = 0;
	uint32_t speed_2 = 0;
	uint32_t speed_3 = 0;
	uint32_t speed_4 = 0;
	
	uint8_t pwmChannel = 0;
	
	uint32_t cclk = CLKPWR_GetCLK(CLKPWR_CLKTYPE_CPU);
	
	//Init WatchDog
	WWDT_Init(1000000);
	WWDT_Enable(ENABLE);
	WWDT_SetMode(WWDT_RESET_MODE, ENABLE);
	WWDT_Start(1000000);

	// Init GPIO
	GPIO_Init();
	PINSEL_ConfigPin(4, 26, 0);
	PINSEL_ConfigPin(4, 27, 0);
	GPIO_SetDir(4, (1<<26), GPIO_DIRECTION_OUTPUT);
	GPIO_SetDir(4, (1<<27), GPIO_DIRECTION_OUTPUT);
	GPIO_OutputValue(4, (1<<26), 1);
	
	//Init ADC
	PINSEL_ConfigPin (0, 23, 1);
	PINSEL_ConfigPin (0, 24, 1);
	PINSEL_SetAnalogPinMode(0, 23, ENABLE);
	PINSEL_SetAnalogPinMode(0, 24, ENABLE);
	PINSEL_SetPinMode(0 , 23, PINSEL_BASICMODE_PLAINOUT);
	PINSEL_SetPinMode(0 , 24, PINSEL_BASICMODE_PLAINOUT);
	ADC_Init(LPC_ADC, 400000);
	
	// Initialize PWM pin connect
	#if (_USING_PWM_NO == 1)
	for (pwmChannel = 0; pwmChannel <= 6; pwmChannel++)
	{
		PINSEL_ConfigPin (2, pwmChannel, 1);
	}
#elif (_USING_PWM_NO == 0)
	PINSEL_ConfigPin (1, 2, 3);//PWM0.1
	PINSEL_ConfigPin (1, 3, 3);//PWM0.2
	PINSEL_ConfigPin (1, 5, 3);//PWM0.3
	PINSEL_ConfigPin (1, 6, 3);//PWM0.4
	PINSEL_ConfigPin (1, 7, 3);//PWM0.5
	PINSEL_ConfigPin (1, 11, 3);//PWM0.6
#else
	return 0;
#endif
	PWMCfgDat.PrescaleOption = PWM_TIMER_PRESCALE_USVAL;
	PWMCfgDat.PrescaleValue = 1;
	PWM_Init(_USING_PWM_NO, PWM_MODE_TIMER, (void *) &PWMCfgDat);
	PWM_MatchUpdate(_USING_PWM_NO, 0, 20000, PWM_MATCH_UPDATE_NOW); // 1 Clock = 16.67E^-9 (sec)
	PWMMatchCfgDat.IntOnMatch = ENABLE;
	PWMMatchCfgDat.MatchChannel = 0;
	PWMMatchCfgDat.ResetOnMatch = ENABLE;
	PWMMatchCfgDat.StopOnMatch = DISABLE;
	PWM_ConfigMatch(_USING_PWM_NO, &PWMMatchCfgDat);
	
	for (pwmChannel = 1; pwmChannel < 7; pwmChannel++)
	{
		PWM_ChannelConfig(_USING_PWM_NO, pwmChannel, PWM_CHANNEL_SINGLE_EDGE);
	}
		/* Setting interrupt for PWM ---------------------------------------------- */
    /* Disable PWM interrupt */
#if (_USING_PWM_NO == 1)
    NVIC_DisableIRQ(PWM1_IRQn);
	/* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(PWM1_IRQn, ((0x01<<3)|0x01));
#elif (_USING_PWM_NO == 0)
	NVIC_DisableIRQ(PWM0_IRQn);
	/* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(PWM0_IRQn, ((0x01<<3)|0x01));
#endif
	
	speed = 0;
	channelVal = 1000 + (1000 * speed / 100);
	for (pwmChannel = 1; pwmChannel < 7; pwmChannel++)
	{
		PWM_MatchUpdate(_USING_PWM_NO, pwmChannel, channelVal, PWM_MATCH_UPDATE_NOW);
		PWMMatchCfgDat.IntOnMatch = DISABLE;
		PWMMatchCfgDat.MatchChannel = pwmChannel;
		PWMMatchCfgDat.ResetOnMatch = DISABLE;
		PWMMatchCfgDat.StopOnMatch = DISABLE;
		PWM_ConfigMatch(_USING_PWM_NO, &PWMMatchCfgDat);
		PWM_ChannelCmd(_USING_PWM_NO, pwmChannel, ENABLE);
	}
	
	/* Enable PWM interrupt */
#if (_USING_PWM_NO == 1)
	NVIC_EnableIRQ(PWM1_IRQn);
#elif (_USING_PWM_NO == 0)
	NVIC_EnableIRQ(PWM0_IRQn);
#endif
	
	PWM_ResetCounter(_USING_PWM_NO);
	PWM_CounterCmd(_USING_PWM_NO, ENABLE);
	PWM_Cmd(_USING_PWM_NO, ENABLE);
	
	/* Generate interrupt each 1 ms   */
	SysTick_Config(cclk/1000 - 1);
	
	//************WDT check last status
	if (WWDT_GetStatus(WWDT_TIMEOUT_FLAG))
	{
		// Clear WDT TimeOut
		WWDT_ClrTimeOutFlag();
		
		ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
		ADC_StartCmd(LPC_ADC, ADC_START_NOW);
		//Wait conversion complete
		while (!(ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE)));
		adc1 = ADC_ChannelGetData(LPC_ADC, 0);
		ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, DISABLE);
		adc_v = (3.3 / 4095 * 10 * (adc1));
		
		if (adc_V > 1)
		{
			GPIO_OutputValue(4, (1<<27), 1);
		}
		
		_DBG_("Last MCU reset caused by WDT TimeOut!\n\r");
	}
	else
		GPIO_OutputValue(4, (1<<27), 0);
	
	Delay(200);
	GY87_init(); //GY87_init return bit error bit0=MPU6050 bit1=HMC5883 bit2=BMP180
	match_cnt = 0;
	WWDT_Feed();
	
	//PID config
	//PID accelometer
	ac_x.kp = 2;
	ac_x.ki = 0.5;
	ac_x.kd = 1.5;
	
	ac_y.kp = ac_x.kp;
	ac_y.ki = ac_x.ki;
	ac_y.kd = ac_x.kd;
	//PID Gyro
	gy_x.kp = 0.5;
	gy_x.ki = 0.5;
	gy_x.kd = 0;
	
	gy_y.kp = gy_x.kp;
	gy_y.ki = gy_x.ki;
	gy_y.kd = gy_x.kd;
	
	while(1)
	{
		INT_MPU6050_CPMS10();

		//match_cnt 1 unit = 20mS
		if (match_cnt > 0)
		{
			match_cnt = 0;
			timeout++;
			counter2++;
			WWDT_Feed();
			
			//Timeout function(Lost comunication)*************
			if (timeout > 150) // 20ms * 50 = 1s
			{
				speed = 0;
				channelVal = 1000;
				for (pwmChannel = 1; pwmChannel < 7; pwmChannel++)
				{
					PWM_MatchUpdate(_USING_PWM_NO, pwmChannel, channelVal, PWM_MATCH_UPDATE_NEXT_RST);
				}
				timeout = 0;
			}
			
			// PMW motor control
			else
			{
				if (speed > 10)
				{
// 					if (ac_x.u[0] > speed) ac_x.u[0] = speed;
// 					else if (ac_x.u[0] < (speed * -1)) ac_x.u[0] = speed * -1;
// 					if (ac_y.u[0] > speed) ac_y.u[0] = speed;
// 					else if (ac_y.u[0] < (speed * -1)) ac_y.u[0] = speed * -1;
					
					if (ac_x.u[0] > (speed / 3)) ac_x.u[0] = (speed / 3);
					else if (ac_x.u[0] < ((speed / 3) * -1)) ac_x.u[0] = (speed / 3) * -1;
					if (ac_y.u[0] > (speed / 3)) ac_y.u[0] = (speed / 3);
					else if (ac_y.u[0] < ((speed / 3) * -1)) ac_y.u[0] = (speed / 3) * -1;
					
					if (speed < 30)
					{
						ac_x.u[0] = 0; ac_y.u[0] = 0; gy_x.u[0] = 0; gy_y.u[0] = 0;
					}
					
					speed_1 = 1000 + (10 * speed / 3) + ((ac_x.u[0] - ac_y.u[0] + gy_x.u[0] -  gy_y.u[0]) * speed / 10);
					speed_2 = 1000 + (10 * speed / 3) + ((-ac_x.u[0] - ac_y.u[0] - gy_x.u[0] -  gy_y.u[0])* speed / 10);
					speed_3 = 1000 + (10 * speed / 3) + ((-ac_x.u[0] + ac_y.u[0] - gy_x.u[0] +  gy_y.u[0])* speed / 10);
					speed_4 = 1000 + (10 * speed / 3) + ((ac_x.u[0] + ac_y.u[0] + gy_x.u[0] +  gy_y.u[0]) * speed / 10);
					
					if (speed_1 > 2000) speed_1 = 2000;
					else if (speed_1 < 1000) speed_1 = 1000;
					if (speed_2 > 2000) speed_2 = 2000;
					else if (speed_2 < 1000) speed_2 = 1000;
					if (speed_3 > 2000) speed_3 = 2000;
					else if (speed_3 < 1000) speed_3 = 1000;
					if (speed_4 > 2000) speed_4 = 2000;
					else if (speed_4 < 1000) speed_4 = 1000;
					
					PWM_MatchUpdate(_USING_PWM_NO, 1, speed_1, PWM_MATCH_UPDATE_NEXT_RST);
					PWM_MatchUpdate(_USING_PWM_NO, 2, speed_2, PWM_MATCH_UPDATE_NEXT_RST);
					PWM_MatchUpdate(_USING_PWM_NO, 3, speed_3, PWM_MATCH_UPDATE_NEXT_RST);
					PWM_MatchUpdate(_USING_PWM_NO, 4, speed_4, PWM_MATCH_UPDATE_NEXT_RST);
					
				}
				else
				{
					channelVal = 1000;
					for (pwmChannel = 1; pwmChannel < 7; pwmChannel++)
					{
						PWM_MatchUpdate(_USING_PWM_NO, pwmChannel, channelVal, PWM_MATCH_UPDATE_NEXT_RST);
					}
					speed = 0;
					ac_x.u[0] = 0; ac_y.u[0] = 0; gy_x.u[0] = 0; gy_y.u[0] = 0;
				}
			}
			
			//Report staus function*********************
			report++;
			if (report > 2)
			{
				//Report_status();
				Get_data();
				report = 0;
			}
			
			//Recive CMD function***********************
			//Get_data();
		}
		
	}
	
}

/* With ARM and GHS toolsets, the entry point is main() - this will
   allow the linker to generate wrapper code to setup stacks, allocate
   heap area, and initialize and copy code and data segments. For GNU
   toolsets, the entry point is through __start() in the crt0_gnu.asm
   file, and that startup code will setup stacks and data */
int main(void)
{
	UART_init();
  return c_entry();
}

/*
 * @}
 */
