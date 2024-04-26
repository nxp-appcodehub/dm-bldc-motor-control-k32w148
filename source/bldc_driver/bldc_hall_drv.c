/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_tpm.h"
#include "fsl_lpit.h"
#include "fsl_vref.h"
#include "fsl_lpadc.h"
#include "fsl_trgmux.h"
#include "fsl_tstmr.h"
#include "fsl_lpcmp.h"
#include "bldc_hall_drv.h"
#include "fsl_edma.h"


/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define BIT(x)				1<<(x)
#define HALL_SECTOR_A		BIT(0)
#define HALL_SECTOR_B		BIT(1)
#define HALL_SECTOR_C		BIT(2)
#define HALL_SECTOR_BC		HALL_SECTOR_B | HALL_SECTOR_C
#define HALL_SECTOR_AC		HALL_SECTOR_A | HALL_SECTOR_C
#define HALL_SECTOR_AB		HALL_SECTOR_A | HALL_SECTOR_B

/* ADC */
#define ADC_IRQHandler		 		ADC0_IRQHandler
#define ADC_CMDID       			1U

/* LPCMP */
#define LPCMP_IRQHandler            LPCMP0_IRQHandler

#define LPCMP_DAC_CHANNEL      		7U

#define CLOCKWISE_DIR				0
#define COUNTERCLOCKWISE_DIR		1

#if defined(FSL_FEATURE_TSTMR_CLOCK_FREQUENCY_1MHZ) && FSL_FEATURE_TSTMR_CLOCK_FREQUENCY_1MHZ
#define TSTMR_CLOCK_FREQ			1000000
#else
#define TSTMR_CLOCK_FREQ			0
#endif


/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
typedef struct
{
	int64_t sectorCount;
	int64_t sectorCmd;
	int64_t sectorErr;
	PID_t   positionPID;
	float   SpeedMeasure;
	int32_t	SpeedCmd;
	float	SpeedErr;
	PID_t   SpeedPID;
	float	UdcBus_Float;
	uint8_t DIR;
	bool    switchDIR;
	uint16_t mStopedPeriodCount;
	uint16_t mSwitchingPeriodCount;
} BldcCtrl_t;

typedef struct
{
	int64_t sectorNumber;
	int32_t Speed;
} UserCmd_t;

typedef enum
{
    AppFault = 0,
    AppInit  = 1,
    AppStop  = 2,
    AppRun   = 3,
} AppState_t;

typedef struct
{
	uint8_t  previousTimeIndex;
	uint64_t previousTime;
	uint64_t timestamps[7];
}TimingInfo_t;

typedef struct
{
	io_cfg_t     hall_A_cfg;
	io_cfg_t     hall_B_cfg;
	io_cfg_t     hall_C_cfg;
} HallIoCfg_t;


/************************************************************************************
*************************************************************************************
* Private functions prototypes
*************************************************************************************
************************************************************************************/
static status_t BLDC_HallInitPin(io_cfg_t *ioCfg);
static status_t BLDC_AdcInit(uint8_t adc_channel);
static status_t BLDC_LpitInit(uint8_t lpit_channel);
static status_t BLDC_LpcmpInit(uint8_t instance, uint8_t in_chnl_index);
static status_t BLDC_TpmInit(uint8_t instance);
static void BLDC_ResetControlLoopData(void);
static void BLDC_ResetPid(void);
static void BLDC_ResetTimmingInfo(void);
static void BLDC_ControlLoopRun(void);
static void BLDC_UpdateSectors(uint8_t sector);
static void BLDC_Commutation(void);
static void BLDC_SetPwmDuty(uint8_t duty);
static uint8_t BLDC_ReadHall(void);
static void BLDC_LpCmpDmaCallback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);


/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
static GPIO_Type *const s_GpioPort[] = GPIO_BASE_PTRS;
static const IRQn_Type s_GpioIrqNo[] = GPIO_IRQS;
static LPCMP_Type *const s_LpcmpInst[] = LPCMP_BASE_PTRS;
static TPM_Type *const s_TpmInst[] = TPM_BASE_PTRS;
static const lpit_chnl_t s_LpitChnl[] = {kLPIT_Chnl_0, kLPIT_Chnl_1, kLPIT_Chnl_2, kLPIT_Chnl_3};
static const lpit_trigger_select_t s_LpitTrg[] = {kLPIT_Trigger_TimerChn0, kLPIT_Trigger_TimerChn1, kLPIT_Trigger_TimerChn2, kLPIT_Trigger_TimerChn3};
static const trgmux_source_t s_LpitToAdcTrg[] = {kTRGMUX_SourceLpit0Channel0, kTRGMUX_SourceLpit0Channel1, kTRGMUX_SourceLpit0Channel2, kTRGMUX_SourceLpit0Channel3};
static const IRQn_Type s_LpcmpIrqNo[] = {LPCMP0_IRQn, LPCMP1_IRQn};
static const clock_ip_name_t s_TpmClocks[] = {kCLOCK_Tpm0, kCLOCK_Tpm1};

AT_NONCACHEABLE_SECTION_INIT(uint32_t g_stopTpmDmaValue[1]) = {0x00};

/* Chnl_0.CnSC, Chnl_1.CnSC, Chnl_2.CnSC, Chnl_3.CnSC, Chnl_4.CnSC, Chnl_5.SnSC*/
static uint8_t sectorCtrl[2][6*7] =
{
		[CLOCKWISE_DIR] = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* STOP */
			0x24, 0x20,	0x00, 0x00, 0x24, 0x20,	/* SECTOR A */
			0x24, 0x20, 0x24, 0x20, 0x00, 0x00, /* SECTOR B */
			0x00, 0x00,	0x24, 0x20, 0x24, 0x20, /* SECTOR AB */
			0x00, 0x00, 0x24, 0x20, 0x24, 0x20,	/* SECTOR C */
			0x24, 0x20, 0x24, 0x20, 0x00, 0x00, /* SECTOR CA */
			0x24, 0x20, 0x00, 0x00, 0x24, 0x20, /* SECTOR CB */
		},
		[COUNTERCLOCKWISE_DIR] = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* STOP */
			0x24, 0x20,	0x00, 0x00, 0x24, 0x20, /* SECTOR A */
			0x24, 0x20, 0x24, 0x20, 0x00, 0x00,	/* SECTOR B */
			0x00, 0x00, 0x24, 0x20, 0x24, 0x20, /* SECTOR AB */
			0x00, 0x00,	0x24, 0x20, 0x24, 0x20, /* SECTOR C */
			0x24, 0x20, 0x24, 0x20, 0x00, 0x00, /* SECTOR CA */
			0x24, 0x20, 0x00, 0x00, 0x24, 0x20, /* SECTOR CB */
		}
};

/* Chnl_0.CnV, Chnl_1.CnV, Chnl_2.CnV, Chnl_3.CnV, Chnl_4.CnV, Chnl_5.CnV */
static uint32_t sectorCtrlVal[2][6*7] =
{
		[CLOCKWISE_DIR] = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     		/* STOP */
			0x02, 0x97, 0x00, 0x00, 0xFFFFFFFF, 0xFFFFFFFF, /* SECTOR A */
			0xFFFFFFFF, 0xFFFFFFFF,	0x02, 0x97, 0x00, 0x00,	/* SECTOR B */
			0x00, 0x00,	0x02, 0x97, 0xFFFFFFFF, 0xFFFFFFFF, /* SECTOR AB */
			0x00, 0x00,	0xFFFFFFFF, 0xFFFFFFFF, 0x02, 0x97, /* SECTOR C */
			0x02, 0x97, 0xFFFFFFFF, 0xFFFFFFFF, 0x00, 0x00, /* SECTOR CA */
			0xFFFFFFFF, 0xFFFFFFFF,	0x00, 0x00, 0x02, 0x97,	/* SECTOR CB */
		},
		[COUNTERCLOCKWISE_DIR] = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00,    		 	/* STOP */
			0xFFFFFFFF, 0xFFFFFFFF,	0x00, 0x00, 0x02, 0x97, /* SECTOR A */
			0x02, 0x97, 0xFFFFFFFF, 0xFFFFFFFF, 0x00, 0x00,	/* SECTOR B */
			0x00, 0x00,	0xFFFFFFFF, 0xFFFFFFFF, 0x02, 0x97, /* SECTOR AB */
			0x00, 0x00,	0x02, 0x97, 0xFFFFFFFF, 0xFFFFFFFF, /* SECTOR C */
			0xFFFFFFFF, 0xFFFFFFFF,	0x02, 0x97, 0x00, 0x00, /* SECTOR CA */
			0x02, 0x97,	0x00, 0x00, 0xFFFFFFFF, 0xFFFFFFFF, /* SECTOR CB */
		}
};
static pfCallback_t mpfAppCallback = NULL;
static BldcCtrl_t mBldcCtrl;
static UserCmd_t  mUserCmd;
static AppState_t mAppState = AppStop;
static int64_t mTmpCommutationSectorAcc = 0;
static uint8_t  mTpmInstance = 0xFF;
static uint8_t  mLpitChnl = 0xFF;
static uint8_t  mLpCmpInstance = 0xFF;
static uint8_t  mLpCmpChannel = 0xFF;
static HallIoCfg_t mHallIoCfg;
static motor_spec_t mMotorSpec;
static driver_config_t mDriverSpec;
static edma_handle_t g_lpCmpEdmaHandle;

static TimingInfo_t mTimingInfo = {
	.timestamps ={
		0xFFFFFFFF,
		0xFFFFFFFF,
		0xFFFFFFFF,
		0xFFFFFFFF,
		0xFFFFFFFF,
		0xFFFFFFFF,
		0xFFFFFFFF
	},
};

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
 * \brief        ADC interrupt handler function.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
void ADC_IRQHandler(void)
{
	lpadc_conv_result_t g_LpadcResultConfigStruct;
    if (LPADC_GetConvResult(ADC0, &g_LpadcResultConfigStruct, 0U))
    {
    	mBldcCtrl.UdcBus_Float = (float)(g_LpadcResultConfigStruct.convValue*mDriverSpec.dcBusVoltageScaleFactor);

    	/* Bus voltage protect */
        if(mBldcCtrl.UdcBus_Float<mMotorSpec.dcBusVoltageMin)
        {
        	mAppState = AppFault;
        	if(mpfAppCallback)
        		mpfAppCallback(bldc_ErrorVoltageDrop_c);
        }
        else if(mBldcCtrl.UdcBus_Float>mMotorSpec.dcBusVoltageMax)
        {
        	mAppState = AppFault;
        	if(mpfAppCallback)
        		mpfAppCallback(bldc_ErrorVoltageOvershoot_c);
        }
        else
        {
        	BLDC_ControlLoopRun();
        }
    }
    SDK_ISR_EXIT_BARRIER;
}


/*! *********************************************************************************
 * \brief        LPCMP (Low Power Comparator) interrupt handler function.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
void LPCMP_IRQHandler(void)
{
    LPCMP_ClearStatusFlags(LPCMP0, kLPCMP_OutputRisingEventFlag);
	mAppState = AppFault;
	if(mpfAppCallback)
		mpfAppCallback(bdlc_ErrorCurrentOvershoot_c);
    SDK_ISR_EXIT_BARRIER;
}


/*! *********************************************************************************
 * \brief        GPIOs interrupt handler function for hall sensors (called by application layer).
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
void BLDC_HallIrqHandler(void)
{
	GPIO_PinClearInterruptFlag(s_GpioPort[mHallIoCfg.hall_A_cfg.port], mHallIoCfg.hall_A_cfg.pin);
	GPIO_PinClearInterruptFlag(s_GpioPort[mHallIoCfg.hall_B_cfg.port], mHallIoCfg.hall_B_cfg.pin);
	GPIO_PinClearInterruptFlag(s_GpioPort[mHallIoCfg.hall_C_cfg.port], mHallIoCfg.hall_C_cfg.pin);
    BLDC_Commutation();
}


/*! *********************************************************************************
 * \brief        BLDC driver initialization function.
 *
 * \param[in]    pHwConfig              Hardware configuration of the setup (channels, IOs, peripherals instances, etc)
 * \param[in]    pMotorConfig           Spec of the motor (PWM frequency, speed limits, current limit, and voltage limits)
 * \param[in]    pDriverConfig          Driver shield spec (scale factors and offset)
 * \param[in]    pfApplicationCallback  Application callback
 *
 * \return       status of the operation
 ********************************************************************************** */
bldc_status_t BLDC_Init(hw_config_t *pHwConfig, motor_spec_t *pMotorConfig, driver_config_t *pDriverConfig, pfCallback_t pfApplicationCallback)
{
	bldc_status_t res = bldcStatus_UnknownError_c;

	do {

		if ((pHwConfig == NULL) || (pMotorConfig == NULL) || (pDriverConfig == NULL))
		{
			res = bldcStatus_InvalidArgument_c;
			break;
		}

		if(mAppState == AppRun)
		{
			res = bldcStatus_InvalidState_c;
			break;
		}
		mAppState = AppInit;
		mpfAppCallback = pfApplicationCallback;
		mTpmInstance = pHwConfig->tpm_instance;

		memcpy(&mMotorSpec, pMotorConfig, sizeof(motor_spec_t));
		memcpy(&mDriverSpec, pDriverConfig, sizeof(driver_config_t));

		/* Init PID controller */
		BLDC_ResetPid();

		/* Init ADC */
		BLDC_AdcInit(pHwConfig->adc_channel);

		/* Init LPIT */
		BLDC_LpitInit(pHwConfig->lpit_channel);

		/* Set LPIT as trigger source for ADC */
		if (kStatus_Success !=TRGMUX_SetTriggerSource(TRGMUX0, kTRGMUX_Trgmux0AdcGp0, kTRGMUX_TriggerInput0, s_LpitToAdcTrg[pHwConfig->lpit_channel]))
		{
			break;
		}

		/* Init IOs (Hall sensor) */
		BLDC_HallInitPin(&pHwConfig->hall_A_cfg);
		BLDC_HallInitPin(&pHwConfig->hall_B_cfg);
		BLDC_HallInitPin(&pHwConfig->hall_C_cfg);
		memcpy(&mHallIoCfg.hall_A_cfg, &pHwConfig->hall_A_cfg, sizeof(io_cfg_t));
		memcpy(&mHallIoCfg.hall_B_cfg, &pHwConfig->hall_B_cfg, sizeof(io_cfg_t));
		memcpy(&mHallIoCfg.hall_C_cfg, &pHwConfig->hall_C_cfg, sizeof(io_cfg_t));

		/* Init TPM0 (PWM) */
		BLDC_TpmInit(pHwConfig->tpm_instance);

		/* Init LPCMP */
		BLDC_LpcmpInit(pHwConfig->lpcmp_cfg.instance, pHwConfig->lpcmp_cfg.conn_inst);

		/* Start Low Power Interrupt Timer for control loop */
		LPIT_StartTimer(LPIT0, s_LpitChnl[mLpitChnl]);

	} while (0);

	return (res);
}


/*! *********************************************************************************
 * \brief        Set the parameters of the PIDs used for position and speed control.
 *
 * \param[in]    pPositionPID   Parameters of the PID for position control
 * \param[in]    pPositionPID   Parameters of the PID for speed control
 *
 * \return       status of the operation
 ********************************************************************************** */
bldc_status_t BLDC_SetupPIDs(PID_t *pPositionPID, PID_t *pSpeedPid)
{
	bldc_status_t res = bldcStatus_InvalidState_c;

	if(mAppState == AppInit)
	{
		if(pPositionPID == NULL || pSpeedPid == NULL)
		{
			res = bldcStatus_InvalidArgument_c;
		}
		else
		{
			mBldcCtrl.positionPID.PID_P = pPositionPID->PID_P;
			mBldcCtrl.positionPID.PID_I = pPositionPID->PID_I;
			mBldcCtrl.positionPID.PID_D = pPositionPID->PID_D;
			mBldcCtrl.positionPID.PID_MaxLim = pPositionPID->PID_MaxLim;
			mBldcCtrl.SpeedPID.PID_P = pSpeedPid->PID_P;
			mBldcCtrl.SpeedPID.PID_I = pSpeedPid->PID_I;
			mBldcCtrl.SpeedPID.PID_D = pSpeedPid->PID_D;
			mBldcCtrl.SpeedPID.PID_MaxLim = pSpeedPid->PID_MaxLim;
		}
	}
	return (res);
}


/*! *********************************************************************************
 * \brief        Start the motor with defined number of sectors and speed.
 *
 * \param[in]    sector_count   Targeted number of sector to reach (1 sector = 1/6 of cycle)
 * \param[in]    speedRpm       Targeted speed to reach (in rpm)
 *
 * \return       status of the operation
 ********************************************************************************** */
bldc_status_t BLDC_Start(int64_t sector_count, int32_t speedRpm)
{
	bldc_status_t res = bldcStatus_UnknownError_c;
	if(mAppState == AppInit)
	{
		mTmpCommutationSectorAcc = 0;
		BLDC_ResetControlLoopData();
		BLDC_LpcmpInit(mLpCmpInstance, mLpCmpChannel); /* Restart LP CMP */
		mAppState=AppRun;

		mUserCmd.sectorNumber = sector_count;
		mUserCmd.Speed = speedRpm;

		/* Start PWM at 45% duty cycle to get enough power */
		BLDC_SetPwmDuty(45);
		/* Start Low Power Interrupt Timer for control loop */
		LPIT_StartTimer(LPIT0, s_LpitChnl[mLpitChnl]);

		/* Update the PWM sectors using commutation table */
		uint8_t startSector = BLDC_ReadHall();
		BLDC_UpdateSectors(startSector);
		TPM_StartTimer(s_TpmInst[mTpmInstance], kTPM_SystemClock);
		res = bldcStatus_Success_c;
	}
	else
	{
		res = bldcStatus_InvalidState_c;
	}

	return (res);
}


/*! *********************************************************************************
 * \brief        Stop the motor.
 *
 * \param[in]    void
 *
 * \return       status of the operation
 ********************************************************************************** */
bldc_status_t BLDC_Stop(void)
{
	bldc_status_t res = bldcStatus_UnknownError_c;
	if(mAppState == AppRun || mAppState == AppFault)
	{
		mAppState=AppStop;
		TPM_StopTimer(s_TpmInst[mTpmInstance]);
		res = bldcStatus_Success_c;
	}
	else
	{
		res = bldcStatus_InvalidState_c;
	}
	return (res);
}


/*! *********************************************************************************
 * \brief        Set the current sector value
 *
 * \param[in]    sector_count   Current sector count to be set (initial motor position)
 *
 * \return       status of the operation
 ********************************************************************************** */
bldc_status_t BLDC_SetInitPos(int64_t sector_count)
{
	bldc_status_t res = bldcStatus_InvalidState_c;
	if(mAppState == AppInit)
	{
		mBldcCtrl.sectorCount = sector_count;
		res = bldcStatus_Success_c;
	}

	return (res);
}


/*! *********************************************************************************
 * \brief        Update the targeted speed command.
 *
 * \param[in]    speed_rpm   Targeted speed to reach (in rpm)
 *
 * \return       status of the operation
 ********************************************************************************** */
bldc_status_t BLDC_UpdateRpmCmd(int32_t speed_rpm)
{
	bldc_status_t res = bldcStatus_Success_c;

	if((speed_rpm > 0) && ((speed_rpm > mMotorSpec.rpmMax) || (speed_rpm < mMotorSpec.rpmMin)))
	{
		res = bldcStatus_OutOfRange_c;
	}
	else if((speed_rpm < 0) && ((speed_rpm < -mMotorSpec.rpmMax) || (speed_rpm > -mMotorSpec.rpmMin)))
	{
		res = bldcStatus_OutOfRange_c;
	}
	else
	{
		mUserCmd.Speed = speed_rpm;
	}

	return (res);
}


/*! *********************************************************************************
 * \brief        Update the targeted sector count command.
 *
 * \param[in]    sector_count   Targeted number of sector to reach (1 sector = 1/6 of cycle)
 *
 * \return       status of the operation
 ********************************************************************************** */
bldc_status_t BLDC_UpdatePosCmd(int64_t sector_count)
{
	bldc_status_t res = bldcStatus_Success_c;

	mUserCmd.sectorNumber = sector_count;

	return (res);
}


/*! *********************************************************************************
 * \brief        Get the current sector value.
 *
 * \param[in]    void
 *
 * \return       current sector value (in number of sector = 1/6 of cycle)
 ********************************************************************************** */
int64_t BLDC_GetSectorCount(void)
{
	return (mBldcCtrl.sectorCount);
}


/*! *********************************************************************************
 * \brief        Get the current speed value.
 *
 * \param[in]    void
 *
 * \return       current speed value (in rpm)
 ********************************************************************************** */
int32_t BLDC_GetSpeed(void)
{
	return ((int32_t)mBldcCtrl.SpeedMeasure);
}


/*! *********************************************************************************
 * \brief        Get the current speed PID information.
 *
 * \param[out]   pSpeedCmd    Speed command (output of the position control PID)
 * \param[out]   pDutyCycle   Duty cycle (output of the speed control PID)
 *
 * \return       current sector value
 ********************************************************************************** */
void BLDC_GetPidInfo(int32_t *pSpeedCmd, uint8_t *pDutyCycle)
{
	*pSpeedCmd = (int32_t)mBldcCtrl.SpeedCmd;
	if(mBldcCtrl.SpeedPID.PID_PIDout > 0)
		*pDutyCycle = (uint8_t)(100*mBldcCtrl.SpeedPID.PID_PIDout);
	else
		*pDutyCycle = (uint8_t)(-100*mBldcCtrl.SpeedPID.PID_PIDout);
}


/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
 * \brief        Initialization of Low-power Periodic Interrupt Timer.
 *
 * \param[in]    lpit_channel     channel to be used
 *
 * \return       status of the operation
 ********************************************************************************** */
static status_t BLDC_LpitInit(uint8_t lpit_channel)
{
    /* Structure of initialize LPIT */
	status_t res = kStatus_Success;
    lpit_config_t lpitConfig;
    lpit_chnl_params_t lpitChannelConfig;

    do {
    	if(lpit_channel > 3)
    	{
			res = kStatus_OutOfRange;
			break;
    	}

    	mLpitChnl = lpit_channel;
		/* Set the source for the LPIT module */
		CLOCK_SetIpSrc(kCLOCK_Lpit0, kCLOCK_IpSrcFro6M);

		/*
		 * lpitConfig.enableRunInDebug = false;
		 * lpitConfig.enableRunInDoze = false;
		 */
		LPIT_GetDefaultConfig(&lpitConfig);

		/* Init lpit module */
		LPIT_Init(LPIT0, &lpitConfig);

		lpitChannelConfig.chainChannel          = false;
		lpitChannelConfig.enableReloadOnTrigger = false;
		lpitChannelConfig.enableStartOnTrigger  = false;
		lpitChannelConfig.enableStopOnTimeout   = false;
		lpitChannelConfig.timerMode             = kLPIT_PeriodicCounter;
		/* Set default values for the trigger source */
		lpitChannelConfig.triggerSelect = s_LpitTrg[lpit_channel];
		lpitChannelConfig.triggerSource = kLPIT_TriggerSource_External;

		/* Init lpit channel */
		res = LPIT_SetupChannel(LPIT0, s_LpitChnl[lpit_channel], &lpitChannelConfig);
		if(res != kStatus_Success)
		{
			break;
		}

		/* Set timer period for channel */
		LPIT_SetTimerPeriod(LPIT0, s_LpitChnl[lpit_channel], USEC_TO_COUNT(1000*LOOP_CONTROL_PERIOD_MS, CLOCK_GetFreq(kCLOCK_ScgSircClk)));

    } while (0);

    return(res);
}


/*! *********************************************************************************
 * \brief        Initialization of Low-power Comparator.
 *
 * \param[in]    instance          lpcmp instance to be used
 * \param[in]    in_chnl_index     channel to be used
 *
 * \return       status of the operation
 ********************************************************************************** */
static status_t BLDC_LpcmpInit(uint8_t instance, uint8_t in_chnl_index)
{
    uint8_t dac_value = 0xFF;
    status_t res = kStatus_Success;
    edma_transfer_config_t transferConfig;
    edma_config_t dmaConfig;
    lpcmp_config_t mLpcmpConfigStruct;
    lpcmp_dac_config_t mLpcmpDacConfigStruct;

    do {
		if ((instance > 1) || (in_chnl_index > 7))
		{
			res = kStatus_OutOfRange;
			break;
		}

		mLpCmpInstance = instance;
		mLpCmpChannel = in_chnl_index;
		float Umax = mMotorSpec.busCurrentMax*mDriverSpec.busCurrentScaleFactor + mDriverSpec.busCurrentOffset;
		float dac_out = VDD_IO_ABC;

		/* Get optimal dac value */
		do
		{
			dac_value--;
			dac_out = (VDD_IO_ABC/256)*(dac_value + 1);   	/* DAC outout = (Vin/256)*(DAC_DATA + 1) */
		} while(dac_out >= Umax && dac_value > 0);

		if(dac_value == 0)
		{
			res = kStatus_OutOfRange;
			break;
		}

		CLOCK_EnableClock(kCLOCK_Dma0);
		LPCMP_GetDefaultConfig(&mLpcmpConfigStruct);
		mLpcmpConfigStruct.powerMode = kLPCMP_HighSpeedPowerMode;

		/* Init the LPCMP module. */
		LPCMP_Init(s_LpcmpInst[instance], &mLpcmpConfigStruct);

		/* Configure the internal DAC to output half of reference voltage. */
		mLpcmpDacConfigStruct.enableLowPowerMode     = false;
		mLpcmpDacConfigStruct.referenceVoltageSource = kLPCMP_VrefSourceVin1; /* VREF_INT = VDD_IO_ABC */
		mLpcmpDacConfigStruct.DACValue = dac_value;
		LPCMP_SetDACConfig(s_LpcmpInst[instance], &mLpcmpDacConfigStruct);

		/* Configure LPCMP input channels. */
		LPCMP_SetInputChannels(s_LpcmpInst[instance], in_chnl_index, LPCMP_DAC_CHANNEL);

		/* Create EDMA transfer */
	    EDMA_GetDefaultConfig(&dmaConfig);
	    EDMA_Init(DMA0, &dmaConfig);
	    EDMA_CreateHandle(&g_lpCmpEdmaHandle, DMA0, LPCMP_DMA_CH);
	    EDMA_SetChannelMux(DMA0, LPCMP_DMA_CH, (54U + instance));
	    EDMA_EnableAsyncRequest(DMA0, LPCMP_DMA_CH, true);

		LPCMP_DisableInterrupts(s_LpcmpInst[instance], kLPCMP_OutputRisingInterruptEnable|kLPCMP_OutputFallingInterruptEnable);
		EnableIRQ(s_LpcmpIrqNo[instance]);
	    EDMA_SetCallback(&g_lpCmpEdmaHandle, BLDC_LpCmpDmaCallback, NULL);

	    uint32_t pRegisterAddr = (uint32_t)&s_TpmInst[mTpmInstance]->SC;
	    EDMA_PrepareTransfer(&transferConfig, g_stopTpmDmaValue, sizeof(uint32_t), (uint32_t *)pRegisterAddr, sizeof(uint32_t), sizeof(g_stopTpmDmaValue), sizeof(g_stopTpmDmaValue), kEDMA_MemoryToMemory);
	    EDMA_SubmitTransfer(&g_lpCmpEdmaHandle, (const edma_transfer_config_t *)(uint32_t)&transferConfig);
	    EDMA_StartTransfer(&g_lpCmpEdmaHandle);

		/* Enable the interrupt. */
	    LPCMP_EnableDMA(s_LpcmpInst[instance], true);
		LPCMP_EnableInterrupts(s_LpcmpInst[instance], kLPCMP_OutputRisingEventFlag);

    } while (0);

    return(res);
}


/*! *********************************************************************************
 * \brief        Initialization of Timer/PWM Module.
 *
 * \param[in]    instance          tpm instance to be used
 *
 * \return       status of the operation
 ********************************************************************************** */
static status_t BLDC_TpmInit(uint8_t instance)
{
	status_t res = kStatus_Success;
	tpm_config_t tpmInfo;
	tpm_chnl_pwm_signal_param_t tpmParam[3];

	do {
		if(instance > 1)
		{
			res = kStatus_OutOfRange;
			break;
		}

		/* TPM Clock Gate Control: Clock enabled */
		CLOCK_EnableClock(s_TpmClocks[instance]);
		/* Set the source for the LPIT module */
		CLOCK_SetIpSrc(s_TpmClocks[instance], kCLOCK_IpSrcFro6M);

		/* Fill in the TPM config struct with the default settings */
		TPM_GetDefaultConfig(&tpmInfo);
		/* Calculate the clock division based on the PWM frequency to be obtained */
		tpmInfo.prescale = TPM_CalculateCounterClkDiv(s_TpmInst[instance], mMotorSpec.pwmFreq, CLOCK_GetIpFreq(s_TpmClocks[instance]));
		/* Initialize TPM module */
		TPM_Init(s_TpmInst[instance], &tpmInfo);

		/* Configure tpm params with frequency 24kHZ */
		tpmParam[0].chnlNumber            = (tpm_chnl_t)kTPM_Chnl_0;
		tpmParam[0].pauseLevel            = kTPM_ClearOnPause;
		tpmParam[0].secPauseLevel         = kTPM_ClearOnPause;
		tpmParam[0].level                 = kTPM_HighTrue;
		tpmParam[0].dutyCyclePercent      = 1;
		tpmParam[0].firstEdgeDelayPercent = 1;
		tpmParam[0].enableComplementary   = true;
		tpmParam[0].deadTimeValue[0]      = 2;
		tpmParam[0].deadTimeValue[1]      = 2;

		tpmParam[1].chnlNumber            = (tpm_chnl_t)kTPM_Chnl_1; //Warning : Chnl2 will be set by driver because in combine mode
		tpmParam[1].pauseLevel            = kTPM_ClearOnPause;
		tpmParam[1].secPauseLevel         = kTPM_ClearOnPause;
		tpmParam[1].level                 = kTPM_HighTrue;
		tpmParam[1].dutyCyclePercent      = 1;
		tpmParam[1].firstEdgeDelayPercent = 1;
		tpmParam[1].enableComplementary   = true;
		tpmParam[1].deadTimeValue[0]      = 2;
		tpmParam[1].deadTimeValue[1]      = 2;

		tpmParam[2].chnlNumber            = (tpm_chnl_t)kTPM_Chnl_2; //Warning : Chnl4 will be set by driver because in combine mode
		tpmParam[2].pauseLevel            = kTPM_ClearOnPause;
		tpmParam[2].secPauseLevel         = kTPM_ClearOnPause;
		tpmParam[2].level                 = kTPM_HighTrue;
		tpmParam[2].dutyCyclePercent      = 1;
		tpmParam[2].firstEdgeDelayPercent = 1;
		tpmParam[2].enableComplementary   = true;
		tpmParam[2].deadTimeValue[0]      = 2;
		tpmParam[2].deadTimeValue[1]      = 2;

		/* In combine mode, the Channels 3/4/5 will be automatically configured */
		res = TPM_SetupPwm(s_TpmInst[instance], tpmParam, 3U, kTPM_CombinedPwm, mMotorSpec.pwmFreq, CLOCK_GetIpFreq(s_TpmClocks[instance]));

	} while(0);

	return(res);
}


/*! *********************************************************************************
 * \brief        Initialization of Analog to Digital Converter.
 *
 * \param[in]    adc_channel       channel to be used
 *
 * \return       status of the operation
 ********************************************************************************** */
static status_t BLDC_AdcInit(uint8_t adc_channel)
{
	status_t res = kStatus_Success;
    lpadc_config_t mLpadcConfigStruct;
    lpadc_conv_trigger_config_t mLpadcTriggerConfigStruct;
    lpadc_conv_command_config_t mLpadcCommandConfigStruct;
    vref_config_t vrefConfig;

    do{
		if(adc_channel > 31)
		{
			res = kStatus_OutOfRange;
			break;
		}

		CLOCK_SetIpSrc(kCLOCK_Lpadc0, kCLOCK_IpSrcFro192M);
		CLOCK_SetIpSrcDiv(kCLOCK_Lpadc0, kSCG_SysClkDivBy10);

		/* Init VREF */
		VREF_GetDefaultConfig(&vrefConfig);
		/* Initialize VREF module, the VREF module provides reference voltage and bias current for LPADC. */
		VREF_Init(VREF0, &vrefConfig);
		/* Get a 1.8V reference voltage. */
		VREF_SetTrim21Val(VREF0, 8U);

		LPADC_GetDefaultConfig(&mLpadcConfigStruct);
		mLpadcConfigStruct.enableAnalogPreliminary = true;
		mLpadcConfigStruct.referenceVoltageSource  = kLPADC_ReferenceVoltageAlt1; //Use VDD_ANA = V_BOARD
		mLpadcConfigStruct.conversionAverageMode = kLPADC_ConversionAverage16;
		mLpadcConfigStruct.powerLevelMode = kLPADC_PowerLevelAlt4; /*!< Highest power setting. */
		LPADC_Init(ADC0, &mLpadcConfigStruct);

		/* Request offset calibration. */
		LPADC_DoOffsetCalibration(ADC0);
		/* Request gain calibration. */
		LPADC_DoAutoCalibration(ADC0);

		/* Set conversion CMD configuration. */
		LPADC_GetDefaultConvCommandConfig(&mLpadcCommandConfigStruct);
		mLpadcCommandConfigStruct.channelNumber = adc_channel;
		mLpadcCommandConfigStruct.conversionResolutionMode = kLPADC_ConversionResolutionHigh; /* 16-bits */
		LPADC_SetConvCommandConfig(ADC0, ADC_CMDID, &mLpadcCommandConfigStruct);

		/* Set trigger configuration. */
		LPADC_GetDefaultConvTriggerConfig(&mLpadcTriggerConfigStruct);
		mLpadcTriggerConfigStruct.targetCommandId       = ADC_CMDID; /* CMD1 is executed. */
		mLpadcTriggerConfigStruct.enableHardwareTrigger = true;
		LPADC_SetConvTriggerConfig(ADC0, 0U, &mLpadcTriggerConfigStruct); /* Configuring the trigger0. */

		/* Enable the watermark interrupt. */
		LPADC_EnableInterrupts(ADC0, kLPADC_FIFO0WatermarkInterruptEnable);
		NVIC_SetPriority(ADC0_IRQn, 255U);
		res = EnableIRQ(ADC0_IRQn);
    } while(0);

    return(res);
}


/*! *********************************************************************************
 * \brief        Initialization of Hall GPIOs.
 *
 * \param[in]    ioCfg     IOs configuration (pin and port)
 *
 * \return       status of the operation
 ********************************************************************************** */
static status_t BLDC_HallInitPin(io_cfg_t *ioCfg)
{
	status_t res = kStatus_Success;
    gpio_pin_config_t in_config = {
        kGPIO_DigitalInput,
        0,
    };

    GPIO_SetPinInterruptConfig(s_GpioPort[ioCfg->port], ioCfg->pin, kGPIO_InterruptEitherEdge);
    GPIO_PinInit(s_GpioPort[ioCfg->port], ioCfg->pin, &in_config);
    NVIC_SetPriority(s_GpioIrqNo[ioCfg->port], HALL_GPIO_ISR_PRIORITY);
    res = EnableIRQ(s_GpioIrqNo[ioCfg->port]);

    return(res);
}


/*! *********************************************************************************
 * \brief        Reset all Control Loop  output info 0.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
static void BLDC_ResetControlLoopData(void)
{
	BLDC_ResetPid();
	BLDC_ResetTimmingInfo();
	mBldcCtrl.switchDIR = false;
	mBldcCtrl.mStopedPeriodCount = 0;
	mBldcCtrl.mSwitchingPeriodCount = 0;
}


/*! *********************************************************************************
 * \brief        Reset all PID output to 0.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
static void BLDC_ResetPid(void)
{
	mBldcCtrl.positionPID.PID_Iout = 0;
	mBldcCtrl.positionPID.PID_Dout = 0;
	mBldcCtrl.positionPID.PID_Pout = 0;
	mBldcCtrl.positionPID.PID_PIDout = 0;
	mBldcCtrl.SpeedPID.PID_Iout = 0;
	mBldcCtrl.SpeedPID.PID_Dout = 0;
	mBldcCtrl.SpeedPID.PID_Pout = 0;
	mBldcCtrl.SpeedPID.PID_PIDout = 0;
}


/*! *********************************************************************************
 * \brief        Reset all timestamping information to 0.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
static void BLDC_ResetTimmingInfo(void)
{
	mTimingInfo.previousTime = TSTMR_ReadTimeStamp(TSTMR0);
	mTimingInfo.timestamps[0] = 0xFFFFFFFF;
	mTimingInfo.timestamps[1] = 0xFFFFFFFF;
	mTimingInfo.timestamps[2] = 0xFFFFFFFF;
	mTimingInfo.timestamps[3] = 0xFFFFFFFF;
	mTimingInfo.timestamps[4] = 0xFFFFFFFF;
	mTimingInfo.timestamps[5] = 0xFFFFFFFF;
	mTimingInfo.timestamps[6] = 0xFFFFFFFF;
}


/*! *********************************************************************************
 * \brief        Control loop implementation, to be called periodically.
 *
 * \param[in]    void
 *
 * \return       void
********************************************************************************** */
static void BLDC_ControlLoopRun(void)
{
	switch (mAppState)
	{
		case AppFault:
		{
			if(mpfAppCallback)
					mpfAppCallback(bldc_ErrorFaultState);
			BLDC_Stop();
		} break;
		case AppInit:
		{
			BLDC_ResetPid();
		} break;
		case AppRun:
		{
			bool applyNow = false;
			int32_t tmpPreviousSectorCount = mBldcCtrl.sectorCount;
			mBldcCtrl.sectorCmd   = mUserCmd.sectorNumber;
			mBldcCtrl.SpeedCmd	  = mUserCmd.Speed;
			mBldcCtrl.sectorCount += mTmpCommutationSectorAcc;
			mTmpCommutationSectorAcc = 0;

			/* Position control loop */
			if (mBldcCtrl.sectorCmd != INFINITY_RUN_CLOCKWISE && mBldcCtrl.sectorCmd != INFINITY_RUN_COUNTERCLOCKWISE)
			{
				int64_t tmpSectorError = mBldcCtrl.sectorCmd - mBldcCtrl.sectorCount;
				mBldcCtrl.positionPID.PID_Pout   = mBldcCtrl.positionPID.PID_P*tmpSectorError;
				mBldcCtrl.positionPID.PID_Iout   = mBldcCtrl.positionPID.PID_I*tmpSectorError + mBldcCtrl.positionPID.PID_Iout;
				mBldcCtrl.positionPID.PID_Dout   = mBldcCtrl.positionPID.PID_D*(tmpSectorError - mBldcCtrl.sectorErr);
				mBldcCtrl.positionPID.PID_PIDout = mBldcCtrl.positionPID.PID_Pout + mBldcCtrl.positionPID.PID_Iout + mBldcCtrl.positionPID.PID_Dout;
				mBldcCtrl.sectorErr              = tmpSectorError;

				/* Limit velocity using Speed command */
				int32_t tempAbsSpeed = mBldcCtrl.SpeedCmd;
				if(mBldcCtrl.SpeedCmd < 0)
				{
					tempAbsSpeed = -tempAbsSpeed;
				}
				if(mBldcCtrl.positionPID.PID_PIDout > tempAbsSpeed)
				{
					mBldcCtrl.positionPID.PID_PIDout = tempAbsSpeed;
				}
				else if(mBldcCtrl.positionPID.PID_PIDout < -tempAbsSpeed)
				{
					mBldcCtrl.positionPID.PID_PIDout = -tempAbsSpeed;
				}
				else
				{
					; /* MISRA */
				}

				mBldcCtrl.SpeedCmd = mBldcCtrl.positionPID.PID_PIDout;
			}

			mBldcCtrl.SpeedMeasure = (float)(60*TSTMR_CLOCK_FREQ)/(mTimingInfo.timestamps[1]+mTimingInfo.timestamps[2]+mTimingInfo.timestamps[3]+mTimingInfo.timestamps[4]+mTimingInfo.timestamps[5]+mTimingInfo.timestamps[6]);//calculate speed(rpm)
			if(mBldcCtrl.DIR == COUNTERCLOCKWISE_DIR)
				mBldcCtrl.SpeedMeasure = -mBldcCtrl.SpeedMeasure;

			/* Switching direction - stop motor before switching */
			if((mBldcCtrl.SpeedMeasure > 1 && mBldcCtrl.SpeedCmd < 0) ||(mBldcCtrl.SpeedMeasure < -1 && mBldcCtrl.SpeedCmd > 0))
			{
				mBldcCtrl.SpeedCmd = 0;
				mBldcCtrl.switchDIR = true;
			}

			/* Motor is considered stopped for switching - restart it */
			if(mBldcCtrl.switchDIR && (mBldcCtrl.SpeedMeasure < 1 && mBldcCtrl.SpeedMeasure > -1))
			{
				mBldcCtrl.mSwitchingPeriodCount ++;
				if(mBldcCtrl.mSwitchingPeriodCount < MOTOR_SWITCHING_DIR_DELAY_MS/LOOP_CONTROL_PERIOD_MS)
				{
					/* Wait in stop mode for a N cycles */
					mBldcCtrl.SpeedCmd = 0;
				}
				else
				{
					/* Restart motor in opposite direction */
					applyNow = true;
					mBldcCtrl.mSwitchingPeriodCount = 0;
					mBldcCtrl.switchDIR = false;
 				}
			}
			if(mBldcCtrl.SpeedCmd <0)
			{
				mBldcCtrl.DIR = COUNTERCLOCKWISE_DIR;
			}
			else if (mBldcCtrl.SpeedCmd >0)
			{
				mBldcCtrl.DIR = CLOCKWISE_DIR;
			}
			else
			{
				; /* MISRA */
			}

			/* Speed control loop */
			float tmpSpeedError           = mBldcCtrl.SpeedCmd - mBldcCtrl.SpeedMeasure;
			mBldcCtrl.SpeedPID.PID_Pout	  = mBldcCtrl.SpeedPID.PID_P*tmpSpeedError;
			mBldcCtrl.SpeedPID.PID_Iout	  = mBldcCtrl.SpeedPID.PID_I*tmpSpeedError + mBldcCtrl.SpeedPID.PID_Iout;
			mBldcCtrl.SpeedPID.PID_Dout   = mBldcCtrl.SpeedPID.PID_D*(tmpSpeedError - mBldcCtrl.SpeedErr);
			mBldcCtrl.SpeedPID.PID_PIDout = mBldcCtrl.SpeedPID.PID_Pout + mBldcCtrl.SpeedPID.PID_Iout + mBldcCtrl.SpeedPID.PID_Dout;
			mBldcCtrl.SpeedErr		      = tmpSpeedError;

			/* Limit PID output */
			if(mBldcCtrl.switchDIR)
			{
				if(mBldcCtrl.SpeedMeasure > 0 && mBldcCtrl.SpeedPID.PID_PIDout < 0)
				{
					mBldcCtrl.SpeedPID.PID_PIDout = 0;
				}
				else if(mBldcCtrl.SpeedMeasure > 0 && mBldcCtrl.SpeedPID.PID_PIDout > 0)
				{
					mBldcCtrl.SpeedPID.PID_PIDout = 0;
				}
				else
				{
					; /* MISRA */
				}
			}
			else
			{
				if(mBldcCtrl.SpeedPID.PID_PIDout>mBldcCtrl.SpeedPID.PID_MaxLim)
				{
					mBldcCtrl.SpeedPID.PID_PIDout = mBldcCtrl.SpeedPID.PID_MaxLim;
				}
				else if(mBldcCtrl.SpeedPID.PID_PIDout<-mBldcCtrl.SpeedPID.PID_MaxLim)
				{
					mBldcCtrl.SpeedPID.PID_PIDout = -mBldcCtrl.SpeedPID.PID_MaxLim;
				}
				else
				{
					; /* MISRA */
				}

			}

			uint8_t tmpAbsDuty = mBldcCtrl.SpeedPID.PID_PIDout > 0 ? (uint8_t)(mBldcCtrl.SpeedPID.PID_PIDout*100): (uint8_t)(-mBldcCtrl.SpeedPID.PID_PIDout*100);

			/* Update PWM using PID output*/
			BLDC_SetPwmDuty(tmpAbsDuty);

			/* Motor has stop running */
			if(tmpPreviousSectorCount == mBldcCtrl.sectorCount)
			{
				uint32_t tmpAbsSpeedCmd = mBldcCtrl.SpeedCmd > 0 ? mBldcCtrl.SpeedCmd : -mBldcCtrl.SpeedCmd;
				mBldcCtrl.mStopedPeriodCount ++;
				if(mBldcCtrl.mStopedPeriodCount == MOTOR_STOPED_DETECTION_PERIOD_MS/LOOP_CONTROL_PERIOD_MS)
				{
					mBldcCtrl.mStopedPeriodCount = 0;
					BLDC_ResetTimmingInfo();
					if(mpfAppCallback)
							mpfAppCallback(bldc_MotorStopped_c);
					BLDC_Stop();
				}
				if(mBldcCtrl.mStopedPeriodCount == MOTOR_PAUSED_DETECTION_PERIOD_MS/LOOP_CONTROL_PERIOD_MS)
				{
					BLDC_ResetTimmingInfo();
				}
				else if(tmpAbsDuty > 0.1 && tmpAbsSpeedCmd >= mMotorSpec.rpmMin)
				{
					/* PWM might be low so motor didn't start, update register here */
					applyNow = true;
				}
				else
				{
					; /* MISRA */
				}
			}
			else
			{
				mBldcCtrl.mStopedPeriodCount = 0;
			}

			if(applyNow)
			{
				/* Force new PWM update */
				BLDC_UpdateSectors(BLDC_ReadHall());
			}
		} break;
		case AppStop:
		{
			mAppState = AppInit;
			BLDC_ResetControlLoopData();
			mBldcCtrl.SpeedMeasure = 0;
			LPIT_StopTimer(LPIT0, s_LpitChnl[mLpitChnl]);
		} break;
	}
}


/*! *********************************************************************************
 * \brief        Update the sector control table with new duty cycle for PWM.
 *
 * \param[in]    duty    New duty cycle
 *
 * \return       void
 ********************************************************************************** */
static void BLDC_SetPwmDuty(uint8_t duty)
{
	uint32_t cnvFirstEdgeValue = 2;
	uint32_t cnvValue = 0;
	uint32_t mod = s_TpmInst[mTpmInstance]->MOD;

	if (duty > 100)
		duty = 100;

	cnvValue = (mod * duty) / 100U;
	if (cnvValue == mod)
	{
		/* 100% duty cycle */
		cnvValue = mod + 1U;
	}
	else if (cnvValue == 0U)
	{
		/* 0% duty cycle */
		cnvFirstEdgeValue = mod + 1U;
	}

	/* Update sectorCtrl table - warning : if commutation table changes, index must change accordingly */
	sectorCtrlVal[CLOCKWISE_DIR][6] = cnvFirstEdgeValue;
	sectorCtrlVal[CLOCKWISE_DIR][7] = cnvFirstEdgeValue + cnvValue;
	sectorCtrlVal[CLOCKWISE_DIR][14] = cnvFirstEdgeValue;
	sectorCtrlVal[CLOCKWISE_DIR][15] = cnvFirstEdgeValue + cnvValue;
	sectorCtrlVal[CLOCKWISE_DIR][20] = cnvFirstEdgeValue;
	sectorCtrlVal[CLOCKWISE_DIR][21] = cnvFirstEdgeValue + cnvValue;
	sectorCtrlVal[CLOCKWISE_DIR][28] = cnvFirstEdgeValue;
	sectorCtrlVal[CLOCKWISE_DIR][29] = cnvFirstEdgeValue + cnvValue;
	sectorCtrlVal[CLOCKWISE_DIR][30] = cnvFirstEdgeValue;
	sectorCtrlVal[CLOCKWISE_DIR][31] = cnvFirstEdgeValue + cnvValue;
	sectorCtrlVal[CLOCKWISE_DIR][40] = cnvFirstEdgeValue;
	sectorCtrlVal[CLOCKWISE_DIR][41] = cnvFirstEdgeValue + cnvValue;

	sectorCtrlVal[COUNTERCLOCKWISE_DIR][10] = cnvFirstEdgeValue;
	sectorCtrlVal[COUNTERCLOCKWISE_DIR][11] = cnvFirstEdgeValue + cnvValue;
	sectorCtrlVal[COUNTERCLOCKWISE_DIR][12] = cnvFirstEdgeValue;
	sectorCtrlVal[COUNTERCLOCKWISE_DIR][13] = cnvFirstEdgeValue + cnvValue;
	sectorCtrlVal[COUNTERCLOCKWISE_DIR][22] = cnvFirstEdgeValue;
	sectorCtrlVal[COUNTERCLOCKWISE_DIR][23] = cnvFirstEdgeValue + cnvValue;
	sectorCtrlVal[COUNTERCLOCKWISE_DIR][26] = cnvFirstEdgeValue;
	sectorCtrlVal[COUNTERCLOCKWISE_DIR][27] = cnvFirstEdgeValue + cnvValue;
	sectorCtrlVal[COUNTERCLOCKWISE_DIR][32] = cnvFirstEdgeValue;
	sectorCtrlVal[COUNTERCLOCKWISE_DIR][33] = cnvFirstEdgeValue + cnvValue;
	sectorCtrlVal[COUNTERCLOCKWISE_DIR][36] = cnvFirstEdgeValue;
	sectorCtrlVal[COUNTERCLOCKWISE_DIR][37] = cnvFirstEdgeValue + cnvValue;
}


/*! *********************************************************************************
 * \brief        Update the each PWM command according the sector control table and current position.
 *
 * \param[in]    sector    Current occupied sector
 *
 * \return       void
 ********************************************************************************** */
static void BLDC_UpdateSectors(uint8_t sector)
{
	for(uint8_t chnl=0; chnl<6; chnl++)
	{
		do
		{
			s_TpmInst[mTpmInstance]->CONTROLS[chnl].CnSC = sectorCtrl[mBldcCtrl.DIR][sector*6 + chnl];
		} while ((uint8_t)(s_TpmInst[mTpmInstance]->CONTROLS[chnl].CnSC & 0x7F)!= (uint8_t) sectorCtrl[mBldcCtrl.DIR][sector*6 + chnl]);
		do
		{
			s_TpmInst[mTpmInstance]->CONTROLS[chnl].CnV = sectorCtrlVal[mBldcCtrl.DIR][sector*6 + chnl];
		} while (s_TpmInst[mTpmInstance]->CONTROLS[chnl].CnV != sectorCtrlVal[mBldcCtrl.DIR][sector*6 + chnl]);
	}
}


/*! *********************************************************************************
 * \brief        function called at each Hall sensor interrupt to apply the next PWM command on expected sector.
 *
 * \param[in]    void
 *
 * \return       void
 ********************************************************************************** */
static void BLDC_Commutation(void)
{
	uint8_t sector = BLDC_ReadHall();
	if(mAppState==AppRun)
	{
		/* Update the PWM sectors using commutation table */
		BLDC_UpdateSectors(sector);

		/* Speed count */
		mTimingInfo.timestamps[mTimingInfo.previousTimeIndex] = TSTMR_ReadTimeStamp(TSTMR0) - mTimingInfo.previousTime;
		mTimingInfo.previousTime = TSTMR_ReadTimeStamp(TSTMR0);
		mTimingInfo.previousTimeIndex = sector;
		if (mBldcCtrl.DIR == CLOCKWISE_DIR)
			mTmpCommutationSectorAcc ++;
		else
			mTmpCommutationSectorAcc --;
	}
}


/*! *********************************************************************************
 * \brief        read the hall sensor value to get current position.
 *
 * \param[in]    void
 *
 * \return       bit field corresponding to current position
 ********************************************************************************** */
static uint8_t BLDC_ReadHall(void)
{
    uint8_t sector_temp=0;
    if(GPIO_PinRead(s_GpioPort[mHallIoCfg.hall_A_cfg.port], mHallIoCfg.hall_A_cfg.pin))sector_temp|=HALL_SECTOR_A;
    if(GPIO_PinRead(s_GpioPort[mHallIoCfg.hall_B_cfg.port], mHallIoCfg.hall_B_cfg.pin))sector_temp|=HALL_SECTOR_B;
    if(GPIO_PinRead(s_GpioPort[mHallIoCfg.hall_C_cfg.port], mHallIoCfg.hall_C_cfg.pin))sector_temp|=HALL_SECTOR_C;
    return sector_temp;
}


/*! *********************************************************************************
 * \brief        DMA transfer callback.
 *
 * \param[in]    handle           dma transfer handle
 * \param[in]    param            not used
 * \param[in]    transferDone     status of the transfer
 * \param[in]    tcds             not used
 *
 * \return       void
 ********************************************************************************** */
static void BLDC_LpCmpDmaCallback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    if (transferDone)
    {
    	mAppState = AppFault;
    	LPCMP_Enable(s_LpcmpInst[mLpCmpInstance], false);
    	uint32_t flag = LPCMP_GetStatusFlags(s_LpcmpInst[mLpCmpInstance]);
    	LPCMP_ClearStatusFlags(s_LpcmpInst[mLpCmpInstance], flag);
    	BLDC_Stop();
    	if(mpfAppCallback)
    		mpfAppCallback(bdlc_ErrorCurrentOvershoot_c);
    }
}
