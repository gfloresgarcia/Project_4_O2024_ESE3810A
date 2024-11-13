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
#define init_task_PRIORITY (configMAX_PRIORITIES - 3)
#define test_task_heap_size_d	(1024)

#define app_message_id_1_d (0x01<<2|message_size_2_bytes_d)
#define app_message_id_2_d (0x02<<2|message_size_4_bytes_d)
#define app_message_id_3_d (0x03<<2|message_size_8_bytes_d)
//#define app_message_id_4_d (0x04<<2|message_size_2_bytes_d)
//#define app_message_id_5_d (0x05<<2|message_size_4_bytes_d)
//#define app_message_id_6_d (0x06<<2|message_size_8_bytes_d)



/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void test_task(void *pvParameters);

static void	message_4_callback_local_slave(void* message);
static void	message_5_callback_local_slave(void* message);
static void	message_6_callback_local_slave(void* message);
static void	message_1_callback_local_slave(void* message);

static void	message_1_callback_slave(void* message);
static void	message_2_callback_slave(void* message);
static void	message_3_callback_slave(void* message);
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
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
	node_config.bitrate = 9600;
	node_config.uartBase = MASTER_UART;
	node_config.srcclk = MASTER_UART_CLK_FREQ;
	node_config.skip_uart_init = 0;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	/* Init Master node */
	master_handle = lin1d3_InitNode(node_config);
#endif

#if defined(USE_SLAVE1)
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = LOCAL_SLAVE_UART;
	node_config.srcclk = LOCAL_SLAVE_UART_CLK_FREQ;
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
    	vTaskDelay(2000);
    	lin1d3_masterSendMessage(master_handle, app_message_id_1_d);
    	vTaskDelay(2000);
    	lin1d3_masterSendMessage(master_handle, app_message_id_2_d);
    	vTaskDelay(2000);
    	lin1d3_masterSendMessage(master_handle, app_message_id_3_d);
    }

    vTaskSuspend(NULL);
}


static void	message_4_callback_local_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Local Slave got message 4 request\r\n");
	message_data[0] = 1;
	message_data[1] = 2;
}

static void	message_5_callback_local_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Local Slave got message 5 request\r\n");
	message_data[0] = 1;
	message_data[1] = 2;
	message_data[2] = 3;
	message_data[3] = 4;
}

static void	message_6_callback_local_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Local Slave got message 6 request\r\n");
	message_data[0] = 1;
	message_data[1] = 2;
	message_data[2] = 3;
	message_data[3] = 4;
	message_data[4] = 5;
	message_data[5] = 6;
	message_data[6] = 7;
	message_data[7] = 8;
}

static void	message_1_callback_local_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Local Slave got response to message 1 %d,%d\r\n", message_data[0], message_data[1]);
}

static void	message_1_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 1 request\r\n");
	message_data[0] = 79;
	message_data[1] = 80;
}

static void	message_2_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 2 request\r\n");
	message_data[0] = 79;
	message_data[1] = 80;
	message_data[2] = 81;
	message_data[3] = 82;
}

static void	message_3_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 3 request\r\n");
	message_data[0] = 79;
	message_data[1] = 80;
	message_data[2] = 81;
	message_data[3] = 82;
	message_data[4] = 83;
	message_data[5] = 84;
	message_data[6] = 85;
	message_data[7] = 86;
}
