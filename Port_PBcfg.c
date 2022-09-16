 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_PBcfg.c
 *
 * Description: Post Build Configuration Source file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Mohamed Tarek
 ******************************************************************************/

#include "Port.h"

/*
 * Module Version 1.0.0
 */
#define PORT_PBCFG_SW_MAJOR_VERSION              (1U)
#define PORT_PBCFG_SW_MINOR_VERSION              (0U)
#define PORT_PBCFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION     (3U)

/* AUTOSAR Version checking between Dio_PBcfg.c and Dio.h files */
#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of PBcfg.c does not match the expected version"
#endif

/* Software Version checking between Dio_PBcfg.c and Dio.h files */
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of PBcfg.c does not match the expected version"
#endif
   
   
/* PB structure used with Dio_Init API */
const Port_ConfigType Port_Configuration = {
                                             /*----------------------------------------------------------------PORT A----------------------------------------------------------------------*/
                                             PortConf_PORT_A_NUM,PortConf_PIN0_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_A_NUM,PortConf_PIN1_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_A_NUM,PortConf_PIN2_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_A_NUM,PortConf_PIN4_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_A_NUM,PortConf_PIN5_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_A_NUM,PortConf_PIN6_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_A_NUM,PortConf_PIN7_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             /*----------------------------------------------------------------PORT B----------------------------------------------------------------------*/
                                             PortConf_PORT_B_NUM,PortConf_PIN0_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_B_NUM,PortConf_PIN1_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_B_NUM,PortConf_PIN2_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_B_NUM,PortConf_PIN3_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_B_NUM,PortConf_PIN4_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_B_NUM,PortConf_PIN5_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_B_NUM,PortConf_PIN6_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_B_NUM,PortConf_PIN7_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             /*----------------------------------------------------------------PORT C----------------------------------------------------------------------*/
                                             PortConf_PORT_C_NUM,PortConf_PIN0_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_C_NUM,PortConf_PIN1_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_C_NUM,PortConf_PIN2_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_C_NUM,PortConf_PIN3_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_C_NUM,PortConf_PIN4_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_C_NUM,PortConf_PIN5_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_C_NUM,PortConf_PIN6_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_C_NUM,PortConf_PIN7_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             /*----------------------------------------------------------------PORT D----------------------------------------------------------------------*/
                                             PortConf_PORT_D_NUM,PortConf_PIN0_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_D_NUM,PortConf_PIN1_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_D_NUM,PortConf_PIN2_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_D_NUM,PortConf_PIN3_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_D_NUM,PortConf_PIN4_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_D_NUM,PortConf_PIN5_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_D_NUM,PortConf_PIN6_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_D_NUM,PortConf_PIN7_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             /*----------------------------------------------------------------PORT E----------------------------------------------------------------------*/
                                             PortConf_PORT_E_NUM,PortConf_PIN0_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_E_NUM,PortConf_PIN1_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_E_NUM,PortConf_PIN2_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_E_NUM,PortConf_PIN3_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_E_NUM,PortConf_PIN4_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_E_NUM,PortConf_PIN5_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             /*----------------------------------------------------------------PORT F----------------------------------------------------------------------*/
                                             PortConf_PORT_F_NUM,PortConf_PIN0_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_F_NUM,PortConf_PIN1_NUM,PORT_PIN_OUT,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_NOT_CHANGEABLE,PortConf_PIN_MODE_NOT_CHANGEABLE, // Red Led
                                             PortConf_PORT_F_NUM,PortConf_PIN2_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_F_NUM,PortConf_PIN3_NUM,PORT_PIN_IN,OFF,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_CHANGEABLE,PortConf_PIN_MODE_CHANGEABLE,
                                             PortConf_PORT_F_NUM,PortConf_PIN4_NUM,PORT_PIN_IN,PULL_UP,PortConf_Mode_GPIO,STD_LOW,PortConf_PIN_DIRECTION_NOT_CHANGEABLE,PortConf_PIN_MODE_NOT_CHANGEABLE, // SW1
                                             
                                             
				             
				         };
