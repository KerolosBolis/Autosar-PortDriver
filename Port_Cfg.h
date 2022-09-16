 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Mohamed Tarek
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_ON)
   
/* Pre-compile option for Version Info API */ 
#define PORT_SET_PIN_DIRECTION_API           (STD_ON)     

/* Number of the configured Dio Channels */
#define PORT_CONFIGURED_CHANNLES              (43U)

/* Channel Index in the array of structures in Port_PBcfg.c */
#define PortConf_CHANNEL_PORT_A_PIN_0__INDEX      (uint8)0x00
#define PortConf_CHANNEL_PORT_A_PIN_1__INDEX      (uint8)0x01
#define PortConf_CHANNEL_PORT_A_PIN_2__INDEX      (uint8)0x02
#define PortConf_CHANNEL_PORT_A_PIN_3__INDEX      (uint8)0x03
#define PortConf_CHANNEL_PORT_A_PIN_4__INDEX      (uint8)0x04
#define PortConf_CHANNEL_PORT_A_PIN_5__INDEX      (uint8)0x05
#define PortConf_CHANNEL_PORT_A_PIN_6__INDEX      (uint8)0x06
#define PortConf_CHANNEL_PORT_A_PIN_7__INDEX      (uint8)0x07
/*-----------------------------------------------------------*/
#define PortConf_CHANNEL_PORT_B_PIN_0__INDEX      (uint8)0x08
#define PortConf_CHANNEL_PORT_B_PIN_1__INDEX      (uint8)0x09
#define PortConf_CHANNEL_PORT_B_PIN_2__INDEX      (uint8)0x0A
#define PortConf_CHANNEL_PORT_B_PIN_3__INDEX      (uint8)0x0B
#define PortConf_CHANNEL_PORT_B_PIN_4__INDEX      (uint8)0x0C
#define PortConf_CHANNEL_PORT_B_PIN_5__INDEX      (uint8)0x0D
#define PortConf_CHANNEL_PORT_B_PIN_6__INDEX      (uint8)0x0E
#define PortConf_CHANNEL_PORT_B_PIN_7__INDEX      (uint8)0x0F
/*-----------------------------------------------------------*/
#define PortConf_CHANNEL_PORT_C_PIN_0__INDEX      (uint8)0x10
#define PortConf_CHANNEL_PORT_C_PIN_1__INDEX      (uint8)0x11
#define PortConf_CHANNEL_PORT_C_PIN_2__INDEX      (uint8)0x12
#define PortConf_CHANNEL_PORT_C_PIN_3__INDEX      (uint8)0x13
#define PortConf_CHANNEL_PORT_C_PIN_4__INDEX      (uint8)0x14
#define PortConf_CHANNEL_PORT_C_PIN_5__INDEX      (uint8)0x15
#define PortConf_CHANNEL_PORT_C_PIN_6__INDEX      (uint8)0x16
#define PortConf_CHANNEL_PORT_C_PIN_7__INDEX      (uint8)0x17
/*-----------------------------------------------------------*/
#define PortConf_CHANNEL_PORT_D_PIN_0__INDEX      (uint8)0x18
#define PortConf_CHANNEL_PORT_D_PIN_1__INDEX      (uint8)0x19
#define PortConf_CHANNEL_PORT_D_PIN_2__INDEX      (uint8)0x1A
#define PortConf_CHANNEL_PORT_D_PIN_3__INDEX      (uint8)0x1B
#define PortConf_CHANNEL_PORT_D_PIN_4__INDEX      (uint8)0x1C
#define PortConf_CHANNEL_PORT_D_PIN_5__INDEX      (uint8)0x1D
#define PortConf_CHANNEL_PORT_D_PIN_6__INDEX      (uint8)0x1E
#define PortConf_CHANNEL_PORT_D_PIN_7__INDEX      (uint8)0x1F
/*-----------------------------------------------------------*/
#define PortConf_CHANNEL_PORT_E_PIN_0__INDEX      (uint8)0x20
#define PortConf_CHANNEL_PORT_E_PIN_1__INDEX      (uint8)0x21
#define PortConf_CHANNEL_PORT_E_PIN_2__INDEX      (uint8)0x22
#define PortConf_CHANNEL_PORT_E_PIN_3__INDEX      (uint8)0x23
#define PortConf_CHANNEL_PORT_E_PIN_4__INDEX      (uint8)0x24
#define PortConf_CHANNEL_PORT_E_PIN_5__INDEX      (uint8)0x25
/*-----------------------------------------------------------*/
#define PortConf_CHANNEL_PORT_F_PIN_0__INDEX      (uint8)0x26
#define PortConf_CHANNEL_PORT_F_PIN_1__INDEX      (uint8)0x27
#define PortConf_CHANNEL_PORT_F_PIN_2__INDEX      (uint8)0x28
#define PortConf_CHANNEL_PORT_F_PIN_3__INDEX      (uint8)0x29
#define PortConf_CHANNEL_PORT_F_PIN_4__INDEX      (uint8)0x2A
/*-----------------------------------------------------------*/

/* Number of Configured Modes */
#define PORT_CONFIGURED_MODES  (10U)
/* Default Mode Of Configuration */
#define PortConf_Mode_ADC        (Port_PinModeType)9   /* ADC Mode */
#define PortConf_Mode_GPIO       (Port_PinModeType)0   /* GPIO Mode */
#define PortConf_Mode_UART       (Port_PinModeType)1   /* UART Mode */
#define PortConf_Mode_SSI        (Port_PinModeType)2   /* SSI Mode */
#define PortConf_Mode_I2C        (Port_PinModeType)3   /* I2C Mode */
#define PortConf_Mode_M0PWM      (Port_PinModeType)4   /*M0PWM Mode */
#define PortConf_Mode_M0FAULT    (Port_PinModeType)4   /*Motion Control Module  Mode */
#define PortConf_Mode_M1PWM      (Port_PinModeType)5   /*M1PWM Mode */
#define PortConf_Mode_IDX_PHASE  (Port_PinModeType)6   /* IDX AND PHASE Mode */
#define PortConf_Mode_TIMER      (Port_PinModeType)7   /* TIMER  Mode */
#define PortConf_Mode_CAN        (Port_PinModeType)8   /* CAN Mode */
#define PortConf_Mode_USB        (Port_PinModeType)8   /* USB Mode */
#define PortConf_Mode_NMI        (Port_PinModeType)8   /*Non-Maskable Interrupt*/

/*Number of ports*/
#define PORTS_CONFIGURED                     (6U)
/* Configured Port ID's  */
#define PortConf_PORT_A_NUM                  (Port_PortType)0 /* PORTA */
#define PortConf_PORT_B_NUM                  (Port_PortType)1 /* PORTB */
#define PortConf_PORT_C_NUM                  (Port_PortType)2 /* PORTC */
#define PortConf_PORT_D_NUM                  (Port_PortType)3 /* PORTD */
#define PortConf_PORT_E_NUM                  (Port_PortType)4 /* PORTC */
#define PortConf_PORT_F_NUM                  (Port_PortType)5 /* PORTF */

/* PORT Configured Channel ID's */
#define PortConf_PIN0_NUM                    (Port_PinType)0 /* Pin 1 */
#define PortConf_PIN1_NUM                    (Port_PinType)1 /* Pin 2 */
#define PortConf_PIN2_NUM                    (Port_PinType)2 /* Pin 3 */
#define PortConf_PIN3_NUM                    (Port_PinType)3 /* Pin 4 */
#define PortConf_PIN4_NUM                    (Port_PinType)4 /* Pin 5 */
#define PortConf_PIN5_NUM                    (Port_PinType)5 /* Pin 6 */
#define PortConf_PIN6_NUM                    (Port_PinType)6 /* Pin 7 */
#define PortConf_PIN7_NUM                    (Port_PinType)7 /* Pin 8 */
#endif /* PORT_CFG_H */