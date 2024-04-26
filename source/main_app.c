/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "fsl_lpuart_freertos.h"
#include "fsl_lpuart.h"
#include "stdio.h"
#include "board.h"
#include "bldc_hall_drv.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define main_task_PRIORITY     (configMAX_PRIORITIES - 2)
#define uart_task_PRIORITY     (configMAX_PRIORITIES - 1)
#define HALL_IRQHandler         GPIOB_INT0_IRQHandler
#define TX_BUFFER_SIZE			500
#define cursup                  "\033[A"
#define curspu_len              strlen(cursup)
//#define KEEP_TRACE
/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
typedef enum app_State_tag {
	idle_state_c,
	set_initial_position_state_c,
	set_rpm_cmd_state_c,
	set_position_cmd_state_c,
	running_state_c,
} app_State_t;

/************************************************************************************
*************************************************************************************
* Private functions prototypes
*************************************************************************************
************************************************************************************/
static void main_task(void *pvParameters);
static void uart_task(void *pvParameters);
static void MainApp_InitMotor(void);
static void MainApp_InitUserInterface(void);
static void MainApp_PrintMainCmdMenu(void);
static void MainApp_PrintInitPositionSelectMenu(void);
static void MainApp_PrintPositionSelectMenu(void);
static void MainApp_PrintSpeedSelectMenu(void);
static void MainApp_PrintRunningMenu(void);
static void MainApp_PrintIdleSateInfo(void);
static void MainApp_PrintRunningStateInfo(void);
#ifndef KEEP_TRACE
static void MainApp_MoveCursorUp(uint8_t line_number);
#endif
static void MainApp_PrintPromptInput(void);
static uint8_t MainApp_WaitForUserInput(void);

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
static QueueHandle_t gControlEventQueue = NULL;
lpuart_rtos_handle_t handle;
struct _lpuart_handle t_handle;
uint8_t background_buffer[32];
uint8_t recv_buffer[8];
char *welcomeMessage        = "\r\n#####################################################\r\n##### BLDC Motor Control App (FreeRTOS) started #####\r\n#####################################################\r\n";
char *send_ring_overrun     = "\r\nRing buffer overrun!\r\n";
char *send_hardware_overrun = "\r\nHardware buffer overrun!\r\n";
char *send_timeout          = "\r\nTimeout expired!\r\n";
char *runningMenuText = "\r\n  ========== Commands menu ==========\r\n    0 - Stop motor\r\n    1 - Update targeted position\r\n    2 - Update targeted speed\r\n\n";
char *mainMenuText = "\r\n  ========== Commands menu ==========\r\n    0 - Set initial motor position\r\n    1 - Set targeted position\r\n    2 - Set targeted speed\r\n    3 - Start Motor\r\n\n";
char *promptText = " Enter your value here > ";
char txBuffer[TX_BUFFER_SIZE] = "";
int32_t rpmTable[9] = {-4000, -3000, -2000, -1000, 0, 1000, 2000, 3000, 4000};
int32_t posTable[9] = {-500, -400, -200, -100, 0, 100, 200, 400, 500};

lpuart_rtos_config_t lpuart_config = {
    .baudrate                 = 115200,
    .parity                   = kLPUART_ParityDisabled,
    .stopbits                 = kLPUART_OneStopBit,
    .buffer                   = background_buffer,
    .buffer_size              = sizeof(background_buffer),
    .rx_timeout_constant_ms   = 1,
    .rx_timeout_multiplier_ms = 1,
    .tx_timeout_constant_ms   = 20,
    .tx_timeout_multiplier_ms = 1,
};

int64_t currentPos = INFINITY_RUN_CLOCKWISE;
int64_t posCmd = INFINITY_RUN_CLOCKWISE;
int32_t speedCmd = 0;
/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
 * \brief        GPIOs interrupt handler function for hall sensors.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
void HALL_IRQHandler(void)
{
	BLDC_HallIrqHandler();
	SDK_ISR_EXIT_BARRIER
}


/*! *********************************************************************************
 * \brief        Application entry point.
 *
 * \param[in]    void
 *
 ********************************************************************************** */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

	/* Initialize message queue  */
    gControlEventQueue = xQueueCreate( 5, sizeof(uint8_t) );
	if(gControlEventQueue == NULL){
		PRINTF("error creating Uart queue\r\n");
		while(1)
			;
	}
    vQueueAddToRegistry(gControlEventQueue, "Ctrl Event Queue");


	/* Init uart consol */
	MainApp_InitUserInterface();

    /* Create Uart Task */
    if (xTaskCreate(uart_task, "Uart_task", configMINIMAL_STACK_SIZE + 1000, NULL, uart_task_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

    /* Create Main Task */
    if (xTaskCreate(main_task, "Main_task", configMINIMAL_STACK_SIZE + 1000, NULL, main_task_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }


    vTaskStartScheduler();
    for (;;)
        ;
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
 * \brief        Main demo Task - Print the messages and send commands to the motor driver.
 *
 * \param[in]    pvParameters    Not used
 *
 * \return       void
 ********************************************************************************** */
static void main_task(void *pvParameters)
{
	app_State_t appState = idle_state_c;
	app_State_t prevAppState = idle_state_c;
	uint8_t eventInput = 0xFF;
	char eventInputTxt[6] = "";


	/* Init hardware drivers */
	BOARD_InitTPMPins();
	BOARD_InitHallPins();
	BOARD_InitADCPin();
	BOARD_InitLPCMPin();

	/* Init motor driver and set initial position to 0 */
	MainApp_InitMotor();
	BLDC_SetInitPos(0);
	currentPos = 0;

    /* Send introduction message. */
    if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)welcomeMessage, strlen(welcomeMessage)))
    {
        vTaskSuspend(NULL);
    }

    for (;;)
    {
    	switch(appState)
    	{
			case idle_state_c:
			{
				MainApp_PrintIdleSateInfo();
				MainApp_PrintMainCmdMenu();
				eventInput = 0xFF;

				if(pdTRUE == xQueueReceive(gControlEventQueue, &eventInput, portMAX_DELAY))
				{
			    	sprintf(eventInputTxt, "%d\r\n", eventInput);
					if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)eventInputTxt, strlen(eventInputTxt)))
					{
						vTaskSuspend(NULL);
					}

					prevAppState = idle_state_c;
					char msg[150] = "";

					switch(eventInput)
					{
						case 0:
						{
							appState = set_initial_position_state_c;
						} break;
						case 1:
						{
							appState = set_position_cmd_state_c;
						} break;
						case 2:
						{
							appState = set_rpm_cmd_state_c;
						} break;
						case 3:
						{
							if(currentPos == INFINITY_RUN_CLOCKWISE)
							{
								sprintf(msg, "\r\n  >> Initial position must be set !\r\n");
					    	}
					    	else if(speedCmd == 0)
					    	{
								sprintf(msg, "\r\n  >> Speed cmd is 0 - motor will not start\r\n");
					    	}
							else
							{
					    		if(posCmd == INFINITY_RUN_CLOCKWISE)
					    		{
					    			if(speedCmd > 0)
					    				sprintf(msg, "\r\n  >> Motor will run indefinitely at %d rpm (clockwise)\r\n", speedCmd);
					    			else
					    				sprintf(msg, "\r\n  >> Motor will run indefinitely at %d rpm (counterclockwise)\r\n", speedCmd);
						    		BLDC_Start(posCmd, speedCmd);
					    		}
					    		else
					    		{
					    			if(speedCmd > 0)
					    				sprintf(msg, "\r\n  >> Motor will run to reach %lld at %d rpm maximum\r\n", posCmd, speedCmd);
					    			else
					    				sprintf(msg, "\r\n  >> Motor will run to reach %lld at %d rpm maximum\r\n", posCmd, (uint32_t)-speedCmd);
						    		BLDC_Start(posCmd*6, speedCmd);
					    		}
					    		appState = running_state_c;
					    	}
					    } break;
    					case 0xF2:
    					{
    						sprintf(msg, "\r\n  /!\\ Current overshoot !! Motor stopped /!\\\r\n");
    					} break;
    					case 0xF3:
    					{
    						sprintf(msg, "\r\n  /!\\ DC bus voltage overshoot !! Motor stopped /!\\\r\n");
    					} break;
    					case 0xF4:
    					{
    						sprintf(msg, "\r\n  /!\\ DC bus voltage dropped !! Motor stopped /!\\\r\n");
    					} break;
    					case 0xF5:
    					{
    						sprintf(msg, "\r\n  /!\\ State Error !! Motor stopped /!\\\r\n");
    					} break;
					    default:
					    {
					    	; /* Nothing to do */
					    } break;
					}
    				if(strlen(msg) > 0)
    				{
    					if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)msg, strlen(msg)))
    					{
    						vTaskSuspend(NULL);
    					}
    				}
    				if(appState == running_state_c)
    				{
			    		MainApp_PrintRunningMenu();
			    		MainApp_PrintRunningStateInfo();
			    		MainApp_PrintPromptInput();
    				}
				}
			} break;
			case set_initial_position_state_c:
			{
				MainApp_PrintInitPositionSelectMenu();
				eventInput = 0xFF;
				if(pdTRUE == xQueueReceive(gControlEventQueue, &eventInput, portMAX_DELAY))
				{
			    	sprintf(eventInputTxt, "%d\r\n", eventInput);
					if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)eventInputTxt, strlen(eventInputTxt)))
					{
						vTaskSuspend(NULL);
					}
    				if(eventInput < 9)
    				{
    					currentPos = (int64_t)posTable[eventInput];
    					BLDC_SetInitPos(currentPos*6);
    					appState = prevAppState;
    				}
				}
			} break;
			case set_rpm_cmd_state_c:
			{
				MainApp_PrintSpeedSelectMenu();
				eventInput = 0xFF;
				if(pdTRUE == xQueueReceive(gControlEventQueue, &eventInput, portMAX_DELAY))
				{
			    	sprintf(eventInputTxt, "%d\r\n", eventInput);
					if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)eventInputTxt, strlen(eventInputTxt)))
					{
						vTaskSuspend(NULL);
					}
					speedCmd = rpmTable[eventInput];
					BLDC_UpdateRpmCmd(speedCmd);
					appState = prevAppState;
				}
				if(appState == running_state_c)
				{
		    		MainApp_PrintRunningMenu();
		    		MainApp_PrintRunningStateInfo();
		    		MainApp_PrintPromptInput();
				}
			} break;
			case set_position_cmd_state_c:
			{
				MainApp_PrintPositionSelectMenu();
				eventInput = 0xFF;
				if(pdTRUE == xQueueReceive(gControlEventQueue, &eventInput, portMAX_DELAY))
				{
			    	sprintf(eventInputTxt, "%d\r\n", eventInput);
					if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)eventInputTxt, strlen(eventInputTxt)))
					{
						vTaskSuspend(NULL);
					}
    				if(eventInput < 9)
    				{
    					posCmd = posTable[eventInput];
    					BLDC_UpdatePosCmd(posCmd*6);
    					appState = prevAppState;
    				}
				}
				if(appState == running_state_c)
				{
		    		MainApp_PrintRunningMenu();
		    		MainApp_PrintRunningStateInfo();
		    		MainApp_PrintPromptInput();
				}
			} break;
			case running_state_c:
			{
				eventInput = 0xFF;
				if(pdTRUE == xQueueReceive(gControlEventQueue, &eventInput, pdMS_TO_TICKS(200)))
				{
			    	sprintf(eventInputTxt, "%d\r\n", eventInput);
					if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)eventInputTxt, strlen(eventInputTxt)))
					{
						vTaskSuspend(NULL);
					}
					char msg[50] = "";
					prevAppState = running_state_c;
    				switch(eventInput)
    				{
    					case 0:
    					{
    						BLDC_Stop();
    						appState = idle_state_c;
    					} break;
    					case 1:
    					{
    						appState = set_position_cmd_state_c;
    					} break;
    					case 2:
    					{
    						appState = set_rpm_cmd_state_c;
    					} break;
    					case 0xF1:
    					{
    						sprintf(msg, "\r\n  >> Motor stopped normally\r\n");
    						appState = idle_state_c;
    					} break;
    					case 0xF2:
    					{
    						sprintf(msg, "\r\n  /!\\ Current overshoot !! Motor stopped /!\\\r\n");
    					    appState = idle_state_c;
    					} break;
    					case 0xF3:
    					{
    						sprintf(msg, "\r\n  /!\\ DC bus voltage overshoot !! Motor stopped /!\\\r\n");
    					    appState = idle_state_c;
    					} break;
    					case 0xF4:
    					{
    						sprintf(msg, "\r\n  /!\\ DC bus voltage dropped !! Motor stopped /!\\\r\n");
    					    appState = idle_state_c;
    					} break;
    					case 0xF5:
    					{
    						sprintf(msg, "\r\n  /!\\ State Error !! Motor stopped /!\\\r\n");
    					    appState = idle_state_c;
    					} break;
    					default:
    					{
    						; /* Nothing to do */
    					} break;
    				}
    				if(strlen(msg) > 0)
    				{
    					if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)msg, strlen(msg)))
    					{
    						vTaskSuspend(NULL);
    					}
    				}
				}
				else
				{
					/* Simply update the state info */
#ifndef KEEP_TRACE
					MainApp_MoveCursorUp(10);
#endif
					MainApp_PrintRunningStateInfo();
					MainApp_PrintPromptInput();
				}
			} break;
    	}
    }
}


/*! *********************************************************************************
 * \brief        Uart Task - Received the User's commands and notify the main task.
 *
 * \param[in]    pvParameters    Not used
 *
 * \return       void
 ********************************************************************************** */
static void uart_task(void *pvParameters)
{
	uint8_t userInput = 0xFF;
    for (;;)
    {
    	userInput = MainApp_WaitForUserInput();
		/* Notify main task */
		if(pdTRUE != xQueueSend(gControlEventQueue, &userInput, pdMS_TO_TICKS(10)))
		{
			PRINTF("ERROR WHILE SENDING DATA TO QUEUE\r\n");
		}
    }

}

/*! *********************************************************************************
 * \brief        callback function to handle BLDC driver events.
 *
 * \param[in]    event    Event raised by the BLDC driver
 *
 * \return       void
 ********************************************************************************** */
static void bldcEvtCallbackHandler(bldcEvt_t event)
{
	uint8_t data = 0xFF;
	switch (event)
	{
		case bldc_MotorStopped_c:
		{
			/* Notify main task */
			data = 0xF1;
		} break;
		case bdlc_ErrorCurrentOvershoot_c:
		{
			data = 0xF2;
		} break;
		case bldc_ErrorVoltageOvershoot_c:
		{
			data = 0xF3;
		} break;
		case bldc_ErrorVoltageDrop_c:
		{
			data = 0xF4;
		} break;
		default:
		{
			data = 0xF5;
		} break;
	}

	if(pdTRUE != xQueueSend(gControlEventQueue, &data, pdMS_TO_TICKS(10)))
	{
		PRINTF("ERROR WHILE SENDING DATA TO QUEUE\r\n");
	}
}


/*! *********************************************************************************
 * \brief        Initialize the motor driver and the PID controller.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
static void MainApp_InitMotor(void)
{
	hw_config_t hwCfg = {
		.adc_channel = 2U,
		.lpit_channel = 0U,
		.lpcmp_cfg = {
			.instance = 0U,
			.conn_inst = 0U,
		},
		.tpm_instance = 0U,
		.hall_A_cfg = {
			.pin = 0U,
			.port = portB,
		},
		.hall_B_cfg = {
			.pin = 1U,
			.port = portB,
		},
		.hall_C_cfg = {
			.pin = 2U,
			.port = portB,
		},
	};
	motor_spec_t motorCfg = {
		.pwmFreq = 20000, /* 20kHz */
		.rpmMin = 200,
		.rpmMax = 5000,
		.dcBusVoltageMin = 8U,
		.dcBusVoltageMax = 16U,
		.busCurrentMax = 4U, /* Up to 5A */
	};

	/* From LVBLDC-SCH:
	 * DCB Voltage sense Gain = 0.219
	 * DCB Current sense Gain = 0.2 + 1.65V offset
	 *--------------
	 * Voltage ADC:
	 * ADC_VMAX_IN  --> adcMax (= 0xFFFF) [ADC_VMAX_IN = VDD_ANA = 2.9 depending on the configuration --> TODO: check]
	 * UdcBus*0.219 --> UdcBus*0.219*adcMax/ADC_VMAX_IN = ADC_ReadValue
	 * -------------
	 * Conclusion:
	 * UdcBus = ADC_ReadValue*ADC_VMAX_IN/(0.219*adcMax) = ADC_ReadValue*2.9/(0.219*65535) = ADC_ReadValue*2.02060e-4
	*/
	driver_config_t driverCfg = {
		.dcBusVoltageScaleFactor = 2.02060e-4f,
		.busCurrentScaleFactor = 0.2,
		.busCurrentOffset = 1.65,
	};

	PID_t positionPid = {
			.PID_P = 22,
			.PID_I = 0.0001,
			.PID_D = 0,
			.PID_MaxLim = motorCfg.rpmMax,
	};

	PID_t speedPid = {
			.PID_P = 7.5e-5f,
			.PID_I = 2e-5f,
			.PID_D = 0,
			.PID_MaxLim = 0.9f, /* Max duty cycle = 90% */
	};

	BLDC_Init(&hwCfg, &motorCfg, &driverCfg,bldcEvtCallbackHandler);
	BLDC_SetupPIDs(&positionPid, &speedPid);

}


/*! *********************************************************************************
 * \brief        Initialize the Uart console.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
static void MainApp_InitUserInterface(void)
{
    lpuart_config.srcclk = BOARD_DEBUG_UART_CLK_FREQ;
    lpuart_config.base   = (LPUART_Type *)BOARD_DEBUG_UART_BASEADDR;

    if (kStatus_Success != LPUART_RTOS_Init(&handle, &t_handle, &lpuart_config))
    {
        vTaskSuspend(NULL);
    }

    LPUART_RTOS_SetRxTimeout(&handle, 0xFFFFFFFF, 1);
    LPUART_RTOS_SetTxTimeout(&handle, 0xFFFFFFFF, 1);
}


/*! *********************************************************************************
 * \brief        Print the message to for user input.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
static void MainApp_PrintPromptInput(void)
{
    if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)promptText, strlen(promptText)))
    {
        vTaskSuspend(NULL);
    }
}


/*! *********************************************************************************
 * \brief        Print the main command menu.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
static void MainApp_PrintMainCmdMenu(void)
{
    if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)mainMenuText, strlen(mainMenuText)))
    {
        vTaskSuspend(NULL);
    }
	MainApp_PrintPromptInput();
}


/*! *********************************************************************************
 * \brief        Print menu to set initial motor position.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
static void MainApp_PrintInitPositionSelectMenu(void)
{
	memset(txBuffer, 0, TX_BUFFER_SIZE);
	sprintf(txBuffer, "\r\n  ========== Select the position ==========\r\n");
	for(uint8_t i=0; i<9; i++)
	{
		sprintf(txBuffer, "%s    %d - (%d)\r\n", txBuffer, i, posTable[i]);
	}

    if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)txBuffer, strlen(txBuffer)))
    {
        vTaskSuspend(NULL);
    }
	MainApp_PrintPromptInput();
}


/*! *********************************************************************************
 * \brief        Print menu to select the targeted motor position.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
static void MainApp_PrintPositionSelectMenu(void)
{
	memset(txBuffer, 0, TX_BUFFER_SIZE);
	sprintf(txBuffer, "\r\n  ========== Select the targeted position (cycle) ==========\r\n");
	for(uint8_t i=0; i<9; i++)
	{
		sprintf(txBuffer, "%s    %d - (%d)\r\n", txBuffer, i, posTable[i]);
	}

    if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)txBuffer, strlen(txBuffer)))
    {
        vTaskSuspend(NULL);
    }
	MainApp_PrintPromptInput();
}


/*! *********************************************************************************
 * \brief        Print menu to select the targeted motor speed.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
static void MainApp_PrintSpeedSelectMenu(void)
{
	memset(txBuffer, 0, TX_BUFFER_SIZE);
	sprintf(txBuffer, "\r\n  ========== Select the targeted speed (RPM) ==========\r\n");
	for(uint8_t i=0; i<9; i++)
	{
		sprintf(txBuffer, "%s    %d - (%d)\r\n", txBuffer, i, rpmTable[i]);
	}

    if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)txBuffer, strlen(txBuffer)))
    {
        vTaskSuspend(NULL);
    }
	MainApp_PrintPromptInput();
}


/*! *********************************************************************************
 * \brief        Print the command menu while motor is running.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
static void MainApp_PrintRunningMenu(void)
{
    if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)runningMenuText, strlen(runningMenuText)))
    {
        vTaskSuspend(NULL);
    }
}


/*! *********************************************************************************
 * \brief        Print the motor information in IDLE state.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
static void MainApp_PrintIdleSateInfo(void)
{
	memset(txBuffer, 0, TX_BUFFER_SIZE);
	sprintf(txBuffer, "\r\n  ========== IDLE state ==========\r\n");
	if(currentPos == INFINITY_RUN_CLOCKWISE)
		sprintf(txBuffer, "%s    Current position: Init position Not set\r\n", txBuffer);
	else
		sprintf(txBuffer, "%s    Current position: %lld    \r\n", txBuffer, currentPos);
	sprintf(txBuffer, "%s    Targeted speed (rmp): %d    \r\n", txBuffer, speedCmd);
	if(posCmd == INFINITY_RUN_CLOCKWISE)
		sprintf(txBuffer, "%s    Targeted position (rotation): Not set - run indefinitely\r\n", txBuffer);
	else
		sprintf(txBuffer, "%s    Targeted position (rotation): %lld    \r\n", txBuffer, posCmd);
	sprintf(txBuffer, "%s  ===================================\r\n", txBuffer);

	if(strlen(txBuffer) > 0)
	{
        if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)txBuffer, strlen(txBuffer)))
        {
            vTaskSuspend(NULL);
        }
	}
}


/*! *********************************************************************************
 * \brief        Print the motor information while is running.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
static void MainApp_PrintRunningStateInfo(void)
{
	memset(txBuffer, 0, TX_BUFFER_SIZE);
	sprintf(txBuffer, "\r\n  ========== Running state ==========\r\n");
	int32_t speed_cmd = 0;
	uint8_t duty_cycle_cmd = 0;
	int64_t sector_count = BLDC_GetSectorCount();
	if(sector_count >= 0)
		currentPos = (sector_count - sector_count%6)/6;  /* 6 sectors per rotation */
	else
		currentPos = (sector_count + sector_count%6)/6;  /* 6 sectors per rotation */

	BLDC_GetPidInfo(&speed_cmd, &duty_cycle_cmd);

	sprintf(txBuffer, "%s    Targeted speed (rmp): %d    \r\n", txBuffer, speedCmd);
	if(posCmd == INFINITY_RUN_CLOCKWISE)
		sprintf(txBuffer, "%s    Targeted position (rotation): Not set - run indefinitely\r\n", txBuffer);
	else
		sprintf(txBuffer, "%s    Targeted position (number of rotation): %lld    \r\n", txBuffer, posCmd);
	sprintf(txBuffer, "%s    Current position (number of rotation): %lld    \r\n", txBuffer, currentPos);
	sprintf(txBuffer, "%s    Speed command (rmp): %d    \r\n", txBuffer, speed_cmd);
	sprintf(txBuffer, "%s    Current speed (rmp): %d    \r\n", txBuffer, BLDC_GetSpeed());
	sprintf(txBuffer, "%s    DutyCycle command (%%): %d    \r\n", txBuffer, duty_cycle_cmd);
	sprintf(txBuffer, "%s  ===================================\r\n\n", txBuffer);

	if(strlen(txBuffer) > 0)
	{
        if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)txBuffer, strlen(txBuffer)))
        {
            vTaskSuspend(NULL);
        }
	}
}


#ifndef KEEP_TRACE
/*! *********************************************************************************
 * \brief        Move the cursor up in the console.
 *
 * \param[in]    line_number    number of line to move
 *
 * \return       void
 ********************************************************************************** */
static void MainApp_MoveCursorUp(uint8_t line_number)
{
	char *pBuffer = NULL;
	pBuffer = (char *)malloc(curspu_len*line_number + 1);
	if(pBuffer == NULL)
	{
		return;
	}
	memset(pBuffer, 0, curspu_len*line_number + 1);
	for(uint8_t i = 0; i<line_number; i++)
	{
		memcpy(&pBuffer[i*curspu_len], cursup, curspu_len);
	}
    if (kStatus_Success !=
        LPUART_RTOS_Send(&handle, (uint8_t *)pBuffer, strlen(pBuffer)))
    {
        vTaskSuspend(NULL);
    }
    free(pBuffer);
}
#endif

/*! *********************************************************************************
 * \brief        Wait for UART Rx byte.
 *
 * \param[in]    void
 *
 * \return       uint8_t value received
 ********************************************************************************** */
static uint8_t MainApp_WaitForUserInput(void)
{
    size_t n = 0;
	volatile uint8_t user_input = 0xFFU;

	int error = LPUART_RTOS_Receive(&handle, recv_buffer, 1, &n);
    if (error == kStatus_LPUART_RxHardwareOverrun)
    {
        /* Notify about hardware buffer overrun */
        if (kStatus_Success !=
            LPUART_RTOS_Send(&handle, (uint8_t *)send_hardware_overrun, strlen(send_hardware_overrun)))
        {
        	while(1);
            vTaskSuspend(NULL);
        }
    }
    if (error == kStatus_LPUART_RxRingBufferOverrun)
    {
        /* Notify about ring buffer overrun */
        if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)send_ring_overrun, strlen(send_ring_overrun)))
        {
        	while(1);
            vTaskSuspend(NULL);
        }
    }
    if (error == kStatus_Timeout)
    {
        /* Notify about timeout */
        if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)send_timeout, strlen(send_timeout)))
        {
        	while(1);
            vTaskSuspend(NULL);
        }
    }
    if (n > 0)
    {
    	user_input = recv_buffer[0] - 0x30U;
    }
    return (user_input);
}

