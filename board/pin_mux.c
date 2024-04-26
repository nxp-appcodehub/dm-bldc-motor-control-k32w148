/*
 * Copyright 2021-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v12.0
processor: KW45B41Z83xxxA
package_id: KW45B41Z83AFTA
mcu_data: ksdk2_0
processor_version: 0.12.6
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"
#include "fsl_gpio.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm33, enableClock: 'true'}
- pin_list:
  - {pin_num: '39', peripheral: LPUART1, signal: RX, pin_signal: PTC2/WUU0_P9/LPSPI1_SOUT/LPUART1_RX/LPI2C1_SCLS/TPM1_CH2/I3C0_PUR/FLEXIO0_D18, pull_select: down,
    pull_enable: disable, slew_rate: fast, open_drain: disable, drive_strength: low}
  - {pin_num: '40', peripheral: LPUART1, signal: TX, pin_signal: PTC3/LPSPI1_SCK/LPUART1_TX/LPI2C1_SDAS/TPM1_CH3/FLEXIO0_D19, pull_select: down, pull_enable: disable,
    slew_rate: fast, open_drain: disable, drive_strength: low}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
    /* Clock Configuration: Peripheral clocks are enabled; module does not stall low power mode entry */
    CLOCK_EnableClock(kCLOCK_PortC);

    const port_pin_config_t portc2_pin39_config = {/* Internal pull-up/down resistor is disabled */
                                                   (uint16_t)kPORT_PullDisable,
                                                   /* Low internal pull resistor value is selected. */
                                                   (uint16_t)kPORT_LowPullResistor,
                                                   /* Fast slew rate is configured */
                                                   (uint16_t)kPORT_FastSlewRate,
                                                   /* Passive input filter is disabled */
                                                   (uint16_t)kPORT_PassiveFilterDisable,
                                                   /* Open drain output is disabled */
                                                   (uint16_t)kPORT_OpenDrainDisable,
                                                   /* Low drive strength is configured */
                                                   (uint16_t)kPORT_LowDriveStrength,
                                                   /* Normal drive strength is configured */
                                                   (uint16_t)kPORT_NormalDriveStrength,
                                                   /* Pin is configured as LPUART1_RX */
                                                   (uint16_t)kPORT_MuxAlt3,
                                                   /* Pin Control Register fields [15:0] are not locked */
                                                   (uint16_t)kPORT_UnlockRegister};
    /* PORTC2 (pin 39) is configured as LPUART1_RX */
    PORT_SetPinConfig(PORTC, 2U, &portc2_pin39_config);

    const port_pin_config_t portc3_pin40_config = {/* Internal pull-up/down resistor is disabled */
                                                   (uint16_t)kPORT_PullDisable,
                                                   /* Low internal pull resistor value is selected. */
                                                   (uint16_t)kPORT_LowPullResistor,
                                                   /* Fast slew rate is configured */
                                                   (uint16_t)kPORT_FastSlewRate,
                                                   /* Passive input filter is disabled */
                                                   (uint16_t)kPORT_PassiveFilterDisable,
                                                   /* Open drain output is disabled */
                                                   (uint16_t)kPORT_OpenDrainDisable,
                                                   /* Low drive strength is configured */
                                                   (uint16_t)kPORT_LowDriveStrength,
                                                   /* Normal drive strength is configured */
                                                   (uint16_t)kPORT_NormalDriveStrength,
                                                   /* Pin is configured as LPUART1_TX */
                                                   (uint16_t)kPORT_MuxAlt3,
                                                   /* Pin Control Register fields [15:0] are not locked */
                                                   (uint16_t)kPORT_UnlockRegister};
    /* PORTC3 (pin 40) is configured as LPUART1_TX */
    PORT_SetPinConfig(PORTC, 3U, &portc3_pin40_config);
}

void BOARD_InitTPMPins(void)
{
    /* Clock Configuration: Peripheral clocks are enabled; module does not stall low power mode entry */
    CLOCK_EnableClock(kCLOCK_PortA);

    const port_pin_config_t porta21_pin18_config = {/* Internal pull-up/down resistor is disabled */
                                                    (uint16_t)kPORT_PullDown, //kPORT_PullDisable,
                                                    /* Low internal pull resistor value is selected. */
                                                    (uint16_t)kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    (uint16_t)kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    (uint16_t)kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    (uint16_t)kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    (uint16_t)kPORT_LowDriveStrength,
                                                    /* Normal drive strength is configured */
                                                    (uint16_t)kPORT_NormalDriveStrength,
                                                    /* Pin is configured as TPM0_CH0 */
                                                    (uint16_t)kPORT_MuxAlt5,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    (uint16_t)kPORT_UnlockRegister};
    /* PORTA21 (pin 18) is configured as TPM0_CH0 */
    PORT_SetPinConfig(PORTA, 21U, &porta21_pin18_config);

    const port_pin_config_t porta20_pin17_config = {/* Internal pull-up/down resistor is disabled */
                                                    (uint16_t)kPORT_PullDown,
                                                    /* Low internal pull resistor value is selected. */
                                                    (uint16_t)kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    (uint16_t)kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    (uint16_t)kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    (uint16_t)kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    (uint16_t)kPORT_LowDriveStrength,
                                                    /* Normal drive strength is configured */
                                                    (uint16_t)kPORT_NormalDriveStrength,
                                                    /* Pin is configured as TPM0_CH1 */
                                                    (uint16_t)kPORT_MuxAlt5,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    (uint16_t)kPORT_UnlockRegister};
    /* PORTA20 (pin 17) is configured as TPM0_CH1 */
    PORT_SetPinConfig(PORTA, 20U, &porta20_pin17_config);


    /* PORTA19 (pin 14) is configured as TPM0_CH2 */
    const port_pin_config_t porta19_pin14_config = {/* Internal pull-up/down resistor is disabled */
                                                    (uint16_t)kPORT_PullDown,
                                                    /* Low internal pull resistor value is selected. */
                                                    (uint16_t)kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    (uint16_t)kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    (uint16_t)kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    (uint16_t)kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    (uint16_t)kPORT_LowDriveStrength,
                                                    /* Normal drive strength is configured */
                                                    (uint16_t)kPORT_NormalDriveStrength,
                                                    /* Pin is configured as TPM0_CH2 */
                                                    (uint16_t)kPORT_MuxAlt5,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    (uint16_t)kPORT_UnlockRegister};
    /* PORTA19 (pin 17) is configured as TPM0_CH2 */
    PORT_SetPinConfig(PORTA, 19U, &porta19_pin14_config);


    /* PORTA18 (pin 13) is configured as TPM0_CH3 */
    const port_pin_config_t porta18_pin13_config = {/* Internal pull-up/down resistor is disabled */
                                                    (uint16_t)kPORT_PullDown,
                                                    /* Low internal pull resistor value is selected. */
                                                    (uint16_t)kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    (uint16_t)kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    (uint16_t)kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    (uint16_t)kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    (uint16_t)kPORT_LowDriveStrength,
                                                    /* Normal drive strength is configured */
                                                    (uint16_t)kPORT_NormalDriveStrength,
                                                    /* Pin is configured as TPM0_CH3 */
                                                    (uint16_t)kPORT_MuxAlt5,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    (uint16_t)kPORT_UnlockRegister};
    /* PORTA18 (pin 13) is configured as TPM0_CH3 */
    PORT_SetPinConfig(PORTA, 18U, &porta18_pin13_config);

    /* PORTA16 (pin 11) is configured as TPM0_CH4 */
    const port_pin_config_t porta16_pin11_config = {/* Internal pull-up/down resistor is disabled */
                                                    (uint16_t)kPORT_PullDown,
                                                    /* Low internal pull resistor value is selected. */
                                                    (uint16_t)kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    (uint16_t)kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    (uint16_t)kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    (uint16_t)kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    (uint16_t)kPORT_LowDriveStrength,
                                                    /* Normal drive strength is configured */
                                                    (uint16_t)kPORT_NormalDriveStrength,
                                                    /* Pin is configured as TPM0_CH4 */
                                                    (uint16_t)kPORT_MuxAlt5,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    (uint16_t)kPORT_UnlockRegister};
    /* PORTA16 (pin 11) is configured as TPM0_CH4 */
    PORT_SetPinConfig(PORTA, 16U, &porta16_pin11_config);

    /* PORTA17 (pin 12) is configured as TPM0_CH5 */
    const port_pin_config_t porta17_pin12_config = {/* Internal pull-up/down resistor is disabled */
                                                    (uint16_t)kPORT_PullDown,
                                                    /* Low internal pull resistor value is selected. */
                                                    (uint16_t)kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    (uint16_t)kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    (uint16_t)kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    (uint16_t)kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    (uint16_t)kPORT_LowDriveStrength,
                                                    /* Normal drive strength is configured */
                                                    (uint16_t)kPORT_NormalDriveStrength,
                                                    /* Pin is configured as TPM0_CH5 */
                                                    (uint16_t)kPORT_MuxAlt5,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    (uint16_t)kPORT_UnlockRegister};
    /* PORTA17 (pin 12) is configured as TPM0_CH5 */
    PORT_SetPinConfig(PORTA, 17U, &porta17_pin12_config);
}

void BOARD_InitHallPins(void)
{
    /* Clock Configuration: Peripheral clocks are enabled; module does not stall low power mode entry */
    CLOCK_EnableClock(kCLOCK_PortB);

    gpio_pin_config_t input_config = {
         .pinDirection = kGPIO_DigitalInput,
         .outputLogic = 0U
     };
     GPIO_PinInit(PTB0_GPIO, PTB0_PIN, &input_config);
     GPIO_PinInit(PTB1_GPIO, PTB1_PIN, &input_config);
     GPIO_PinInit(PTB2_GPIO, PTB2_PIN, &input_config);

     const port_pin_config_t inputPinCfg = {/* Internal pull-up resistor is enabled */
                                    (uint16_t)kPORT_PullDown,
                                    /* Low internal pull resistor value is selected. */
                                    (uint16_t)kPORT_LowPullResistor,
                                    /* Fast slew rate is configured */
                                    (uint16_t)kPORT_FastSlewRate,
                                    /* Passive input filter is disabled */
                                    (uint16_t)kPORT_PassiveFilterDisable,
                                    /* Open drain output is disabled */
                                    (uint16_t)kPORT_OpenDrainDisable,
                                    /* Low drive strength is configured */
                                    (uint16_t)kPORT_LowDriveStrength,
                                    /* Normal drive strength is configured */
                                    (uint16_t)kPORT_NormalDriveStrength,
                                    /* Pin is configured as GPIO */
                                    (uint16_t)kPORT_MuxAsGpio,
                                    /* Pin Control Register fields [15:0] are not locked */
                                    (uint16_t)kPORT_UnlockRegister};
     PORT_SetPinConfig(PTB0_PORT, PTB0_PIN, &inputPinCfg);
     PORT_SetPinConfig(PTB1_PORT, PTB1_PIN, &inputPinCfg);
     PORT_SetPinConfig(PTB2_PORT, PTB2_PIN, &inputPinCfg);
}

void BOARD_InitADCPin(void)
{
    /* Clock Configuration: Peripheral clocks are enabled; module does not stall low power mode entry */
    CLOCK_EnableClock(kCLOCK_Lpadc0);

    const port_pin_config_t portd2_pin25_config = {/* Internal pull-up/down resistor is disabled */
                                                   (uint16_t)kPORT_PullDisable,
                                                   /* Low internal pull resistor value is selected. */
                                                   (uint16_t)kPORT_LowPullResistor,
                                                   /* Fast slew rate is configured */
                                                   (uint16_t)kPORT_FastSlewRate,
                                                   /* Passive input filter is disabled */
                                                   (uint16_t)kPORT_PassiveFilterDisable,
                                                   /* Open drain output is disabled */
                                                   (uint16_t)kPORT_OpenDrainDisable,
                                                   /* Low drive strength is configured */
                                                   (uint16_t)kPORT_LowDriveStrength,
                                                   /* Normal drive strength is configured */
                                                   (uint16_t)kPORT_NormalDriveStrength,
                                                   /* Pin is configured as ADC0_A6 */
                                                   (uint16_t)kPORT_PinDisabledOrAnalog,
                                                   /* Pin Control Register fields [15:0] are not locked */
                                                   (uint16_t)kPORT_UnlockRegister};
    /* PORTD2 (pin 25) is configured as ADC0_A6 */
    PORT_SetPinConfig(PORTD, 2U, &portd2_pin25_config);


}

void BOARD_InitLPCMPin(void)
{
    /* Clock Configuration: Peripheral clocks are enabled; module does not stall low power mode entry */
    CLOCK_EnableClock(kCLOCK_PortA);

    const port_pin_config_t porta4_pin10_config = {/* Internal pull-down resistor is enabled */
                                                   (uint16_t)kPORT_PullDown,
                                                   /* Low internal pull resistor value is selected. */
                                                   (uint16_t)kPORT_LowPullResistor,
                                                   /* Fast slew rate is configured */
                                                   (uint16_t)kPORT_FastSlewRate,
                                                   /* Passive input filter is disabled */
                                                   (uint16_t)kPORT_PassiveFilterDisable,
                                                   /* Open drain output is disabled */
                                                   (uint16_t)kPORT_OpenDrainDisable,
                                                   /* Low drive strength is configured */
                                                   (uint16_t)kPORT_LowDriveStrength,
                                                   /* Normal drive strength is configured */
                                                   (uint16_t)kPORT_NormalDriveStrength,
                                                   /* Pin is configured as CMP0_IN0 */
                                                   (uint16_t)kPORT_PinDisabledOrAnalog,
                                                   /* Pin Control Register fields [15:0] are not locked */
                                                   (uint16_t)kPORT_UnlockRegister};
    /* PORTA4 (pin 10) is configured as CMP0_IN0 */
    PORT_SetPinConfig(PORTA, 4U, &porta4_pin10_config);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
