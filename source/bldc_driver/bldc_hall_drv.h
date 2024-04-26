/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef BLDC_HALL_DRV_H_
#define BLDC_HALL_DRV_H_

#define INFINITY_RUN_CLOCKWISE           (0x7FFFFFFFFFFFFFFF)
#define INFINITY_RUN_COUNTERCLOCKWISE    (0x8000000000000000)

#define HALL_GPIO_ISR_PRIORITY 			 (3U)
#define LOOP_CONTROL_PERIOD_MS 			 (10U)
#define MOTOR_PAUSED_DETECTION_PERIOD_MS (100U)
#define MOTOR_SWITCHING_DIR_DELAY_MS     (100U)
#define MOTOR_STOPED_DETECTION_PERIOD_MS (500U)
#define VDD_IO_ABC						 (3.0f)
#define LPCMP_DMA_CH					 (0U)

typedef enum bldcEvt_tag {
	bldc_MotorStopped_c,
	bdlc_ErrorCurrentOvershoot_c,
	bldc_ErrorVoltageOvershoot_c,
	bldc_ErrorVoltageDrop_c,
	bldc_ErrorFaultState
}bldcEvt_t;

typedef enum bldc_status_tag {
	bldcStatus_Success_c,
	bldcStatus_InvalidArgument_c,
	bldcStatus_OutOfRange_c,
	bldcStatus_InvalidState_c,
	bldcStatus_UnknownError_c,
}bldc_status_t;

typedef void (*pfCallback_t)(bldcEvt_t event);


typedef enum io_port_tag
{
	portA = 0U,
	portB = 1U,
	portC = 2U,
	portD = 3U,
}io_port_t;

typedef struct io_cfg_tag
{
	uint32_t  pin;
	io_port_t port;
}io_cfg_t;


typedef struct lpcmp_cfg_tag {
	uint8_t instance;
	uint8_t conn_inst;
}lpcmp_cfg_t;

typedef struct hw_config_tag {
	uint8_t      adc_channel;
	uint8_t      lpit_channel;
	lpcmp_cfg_t  lpcmp_cfg;
	uint8_t      tpm_instance;
	io_cfg_t     hall_A_cfg;
	io_cfg_t     hall_B_cfg;
	io_cfg_t     hall_C_cfg;
} hw_config_t;

typedef struct motor_spec_tag {
	int16_t  pwmFreq;
	uint16_t rpmMin;
	uint16_t rpmMax;
	int16_t  dcBusVoltageMin;
	int16_t  dcBusVoltageMax;
	int16_t  busCurrentMax;
}motor_spec_t;

typedef struct driver_config_tag {
	float dcBusVoltageScaleFactor;
	float busCurrentScaleFactor;
	float busCurrentOffset;
}driver_config_t;

typedef struct PID_tag  {
	float	 PID_P;
	float	 PID_I;
	float	 PID_D;
	float    PID_MaxLim;
	float    PID_Pout;
	float    PID_Iout;
	float    PID_Dout;
	float    PID_PIDout;
} PID_t;

bldc_status_t BLDC_Init(hw_config_t *pHwConfig, motor_spec_t *pMotorConfig, driver_config_t *pDriverConfig, pfCallback_t pfApplicationCallback);
bldc_status_t BLDC_SetupPIDs(PID_t *pPositionPID, PID_t *pSpeedPid);
bldc_status_t BLDC_Start(int64_t sector_count, int32_t speedRpm);
bldc_status_t BLDC_Stop(void);
bldc_status_t BLDC_SetInitPos(int64_t sector_count);
bldc_status_t BLDC_UpdateRpmCmd(int32_t speed_rpm);
bldc_status_t BLDC_UpdatePosCmd(int64_t sector_count);
int64_t BLDC_GetSectorCount(void);
int32_t BLDC_GetSpeed(void);
void BLDC_GetPidInfo(int32_t *pSpeedCmd, uint8_t *pDutyCycle);
void BLDC_HallIrqHandler(void);

#endif /* BLDC_HALL_DRV_H_ */
