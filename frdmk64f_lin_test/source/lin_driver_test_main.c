/*
 * lin_driver_test_main.c
 * Created on: Sep 15, 2018
 *     Author: Nico
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"
#include "fsl_port.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "driver_lin/lin1d3_driver.h"
#include "FreeRTOSConfig.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define USE_MASTER
#define USE_SLAVE1
//#define USE_SLAVE2
//#define USE_SLAVE3

/* Use to indicate the slave if the header received has to transmit or receive */
#define SlaveTransmit 0
#define SlaveReceive 1

/* UART instance and clock */
#define MASTER_UART UART3
#define MASTER_UART_CLKSRC UART3_CLK_SRC
#define MASTER_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define MASTER_UART_RX_TX_IRQn UART3_RX_TX_IRQn

/* UART instance and clock */
#define LOCAL_SLAVE_UART UART3
#define LOCAL_SLAVE_UART_CLKSRC UART3_CLK_SRC
#define LOCAL_SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define LOCAL_SLAVE_UART_RX_TX_IRQn UART3_RX_TX_IRQn

/* UART instance and clock */
#define SLAVE_UART UART4
#define SLAVE_UART_CLKSRC UART4_CLK_SRC
#define SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART4_CLK_SRC)
#define SLAVE_UART_RX_TX_IRQn UART4_RX_TX_IRQn

/* Task priorities. */
#define init_task_PRIORITY (configMAX_PRIORITIES - 2)
#define test_task_heap_size_d	(1024)

#define app_message_id_1_d (0x01<<2|message_size_2_bytes_d)
#define app_message_id_2_d (0x02<<2|message_size_4_bytes_d)
#define app_message_id_3_d (0x03<<2|message_size_8_bytes_d)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void test_task(void *pvParameters);

static void	message_1_callback_slave(void* message);
static void	message_2_callback_slave(void* message);
static void	message_3_callback_slave(void* message);
/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool button1_pressed = 0;
volatile bool button2_pressed = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Interrupt service function of switch SW3.
 *
 */
void BOARD_SW2_IRQ_HANDLER(void)
{
	if (button1_pressed == 0) {
		button1_pressed = 1;
	}
	else {
		button1_pressed = 0;
	}

    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW2_GPIO, 1U << BOARD_SW2_GPIO_PIN);
}

/*!
 * @brief Interrupt service function of switch SW3.
 *
 */
void BOARD_SW3_IRQ_HANDLER(void)
{
	button2_pressed = 1;
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW3_GPIO, 1U << BOARD_SW3_GPIO_PIN);
}

void BOARD_InitGPIOInterrupts (void)
{
    /* Define the init structure for the input switch pin */
    gpio_pin_config_t sw3_config = {
        kGPIO_DigitalInput,
        0,
    };
    gpio_pin_config_t sw2_config = {
    	kGPIO_DigitalInput,
		0,
    };

    PORT_SetPinInterruptConfig(BOARD_SW3_PORT, BOARD_SW3_GPIO_PIN, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(BOARD_SW2_PORT, BOARD_SW2_GPIO_PIN, kPORT_InterruptFallingEdge);

	EnableIRQ(BOARD_SW3_IRQ);
	EnableIRQ(BOARD_SW2_IRQ);

	GPIO_PinInit(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, &sw3_config);
	GPIO_PinInit(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, &sw2_config);


    //Set PORTA Interrupt level to 3 (higher than SYSCALL), configMAX_SYSCALL_INTERRUPT_PRIORITY priority is 2.
    (void) NVIC_GetPriority(PORTA_IRQn);
	NVIC_SetPriority(PORTA_IRQn,3);        //PORTA vector is 5
	(void) NVIC_GetPriority(PORTC_IRQn);
	NVIC_SetPriority(PORTC_IRQn,4);

	/* Define the init structure for the output LED pin*/
	gpio_pin_config_t led_config = {
		kGPIO_DigitalOutput,
		0,
	};
	/* PORTB2 is configured as PTB2 */
	PORT_SetPinMux(PORTB, 2U, kPORT_MuxAsGpio);
	/* PORTB3 is configured as PTB3 */
	PORT_SetPinMux(PORTB, 3U, kPORT_MuxAsGpio);
	/* PORTB10 is configured as PTB10 */
	PORT_SetPinMux(PORTB, 10U, kPORT_MuxAsGpio);
	/* PORTB11 is configured as PTB11 */
	PORT_SetPinMux(PORTB, 11U, kPORT_MuxAsGpio);

	GPIO_PinInit(GPIOB, 2,  &led_config);
	GPIO_PinInit(GPIOB, 3,  &led_config);
	GPIO_PinInit(GPIOB, 10, &led_config);
	GPIO_PinInit(GPIOB, 11, &led_config);
}

/***********************************************************
 *  LEDs
 * ********************************************************/
void BOARD_InitLEDsPins(void)
{
    /* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Port E Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortE);

    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };

    /* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);

    /* RED*/
    PORT_SetPinMux(PORTB, 22U, kPORT_MuxAsGpio);
    /* GREEN */
	PORT_SetPinMux(PORTE, 26U, kPORT_MuxAsGpio);
    /* BLUE */
    PORT_SetPinMux(PORTB, 21U, kPORT_MuxAsGpio);


    /* RED*/
    GPIO_PinInit(GPIOB, 22U,  &led_config);
    /* GREEN */
    GPIO_PinInit(GPIOE, 26U, &led_config);
    /* BLUE */
    GPIO_PinInit(GPIOB, 21U,  &led_config);

    GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
    GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u <<BOARD_LED_GREEN_GPIO_PIN);
    GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
}


/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    NVIC_SetPriority(MASTER_UART_RX_TX_IRQn, 5);
    NVIC_SetPriority(SLAVE_UART_RX_TX_IRQn, 5);

    BOARD_InitGPIOInterrupts();
    BOARD_InitLEDsPins();

    if (xTaskCreate(test_task, "test_task", test_task_heap_size_d, NULL, init_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Init Task creation failed!.\r\n");
        while (1)
            ;
    }
    PRINTF(" *** LIN driver demo ***\r\n");
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for loopback.
 */
static void test_task(void *pvParameters)
{
	int error;
	lin1d3_nodeConfig_t node_config;
	lin1d3_handle_t* master_handle;
	lin1d3_handle_t* slave_handle;
	lin1d3_handle_t* local_slave_handle;

#if defined(USE_MASTER)
	node_config.type = lin1d3_master_nodeType;
	node_config.bitrate = 19200;
	node_config.uartBase = MASTER_UART;
	node_config.srcclk = MASTER_UART_CLK_FREQ;
	node_config.skip_uart_init = 0;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	/* Init Master node */
	master_handle = lin1d3_InitNode(node_config);
#endif

#if defined(USE_SLAVE1)
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 19200;
	node_config.uartBase = SLAVE_UART;
	node_config.srcclk = SLAVE_UART_CLK_FREQ;
	node_config.skip_uart_init = 0;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].rx = SlaveTransmit;
	node_config.messageTable[0].handler = message_1_callback_slave;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].rx = SlaveReceive;
	node_config.messageTable[1].handler = message_2_callback_slave;
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].rx = SlaveReceive;
	node_config.messageTable[2].handler = message_3_callback_slave;
	/* Init Slave Node*/
	slave_handle = lin1d3_InitNode(node_config);
#endif

#if defined(USE_SLAVE2)
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = SLAVE_UART;
	node_config.srcclk = SLAVE_UART_CLK_FREQ;
	node_config.skip_uart_init = 0;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].rx = SlaveReceive;
	node_config.messageTable[0].handler = message_1_callback_slave;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].rx = SlaveTransmit;
	node_config.messageTable[1].handler = message_2_callback_slave;
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].rx = SlaveReceive;
	node_config.messageTable[2].handler = message_3_callback_slave;
	/* Init Slave Node*/
	slave_handle = lin1d3_InitNode(node_config);
#endif

#if defined(USE_SLAVE3)
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = SLAVE_UART;
	node_config.srcclk = SLAVE_UART_CLK_FREQ;
	node_config.skip_uart_init = 0;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].rx = SlaveReceive;
	node_config.messageTable[0].handler = message_1_callback_slave;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].rx = SlaveReceive;
	node_config.messageTable[1].handler = message_2_callback_slave;
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].rx = SlaveTransmit;
	node_config.messageTable[2].handler = message_3_callback_slave;
	/* Init Slave Node*/
	slave_handle = lin1d3_InitNode(node_config);
#endif

	if((NULL == slave_handle)){
		PRINTF(" Init failed!! \r\n");
		error = kStatus_Fail;
	}
	else {
		error = kStatus_Success;
	}

	while (kStatus_Success == error)
    {
    	vTaskDelay(500);
    	lin1d3_masterSendMessage(master_handle, app_message_id_1_d);
    	//vTaskDelay(2000);
    	//lin1d3_masterSendMessage(master_handle, app_message_id_2_d);
    	//vTaskDelay(2000);
    	//lin1d3_masterSendMessage(master_handle, app_message_id_3_d);
    }

    vTaskSuspend(NULL);
}

static void	message_1_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 1 request\r\n");

#if defined(USE_SLAVE1)
	if (button1_pressed) {
		message_data[0] = 1;
		LED_RED_ON();
	}
	else {
		message_data[0] = 0;
		LED_RED_OFF();
	}
	message_data[1] = 0;
#endif

#if defined(USE_SLAVE2)
	if (message_data[0] == 1) {
		LED_RED_ON();
	}
	else {
		LED_RED_OFF();
	}
#endif

#if defined(USE_SLAVE3)
	if (message_data[0] == 1) {
		LED_RED_ON();
	}
	else {
		LED_RED_OFF();
	}
#endif
}

static void	message_2_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 2 request\r\n");

#if defined(USE_SLAVE1)
	if (message_data[0] == 1) {
		LED_GREEN_ON();
	}
	else {
		LED_GREEN_OFF();
	}
#endif

#if defined(USE_SLAVE2)
	if (button1_pressed) {
		message_data[0] = 1;
		LED_GREEN_ON();
	}
	else {
		message_data[0] = 0;
		LED_GREEN_OFF();
	}
	message_data[1] = 0;
	message_data[2] = 0;
	message_data[3] = 0;
#endif

#if defined(USE_SLAVE3)
	if (message_data[0] == 1) {
		LED_GREEN_ON();
	}
	else {
		LED_GREEN_OFF();
	}
#endif
}

static void	message_3_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 3 request\r\n");

#if defined(USE_SLAVE1)
	if (message_data[0] == 1) {
		LED_BLUE_ON();
	}
	else {
		LED_BLUE_OFF();
	}
#endif

#if defined(USE_SLAVE2)
	if (message_data[0] == 1) {
		LED_BLUE_ON();
	}
	else {
		LED_BLUE_OFF();
	}
#endif

#if defined(USE_SLAVE3)
	if (button1_pressed) {
		message_data[0] = 1;
		LED_BLUE_ON();
	}
	else {
		message_data[0] = 0;
		LED_BLUE_OFF();
	}
	message_data[1] = 0;
	message_data[2] = 0;
	message_data[3] = 0;
	message_data[4] = 0;
	message_data[5] = 0;
	message_data[6] = 0;
	message_data[7] = 0;
#endif
}
