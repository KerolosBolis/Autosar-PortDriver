 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Tarek
 ******************************************************************************/


#include "Port.h"
#include "Port_Regs.h"
#include "tm4c123gh6pm_registers.h"
#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif
   #endif

STATIC const Port_ConfigChannel * Port_PortChannels = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;
/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Provide initial value for o/p pin
*              - Setup the internal resistor for i/p pin
************************************************************************************/
void Port_Init(const Port_ConfigType * ConfigPtr )
{
    #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
	if (NULL_PTR == ConfigPtr)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
		     PORT_E_PARAM_CONFIG);
	}
	else
        #endif
        {
        /*
		 * Set the module state to initialized and point to the PB configuration structure using a global pointer.
		 * This global pointer is global to be used by other functions to read the PB configuration structures
		 */
		Port_Status       = PORT_INITIALIZED;
		Port_PortChannels = ConfigPtr->Channels; /* address of the first Channels structure --> Channels[0] */
        }
        volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
	volatile uint32 delay = 0;
	volatile uint8 number_of_pins= 0 ;
    for(number_of_pins=0;number_of_pins<PORT_CONFIGURED_CHANNLES;number_of_pins++){
	switch(Port_PortChannels[number_of_pins].port_type)
      {
        case  PortConf_PORT_A_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break;
	case  PortConf_PORT_B_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case  PortConf_PORT_C_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case  PortConf_PORT_D_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case  PortConf_PORT_E_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case  PortConf_PORT_F_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
    }
    
    /* Enable clock for PORT and allow time for clock to start*/
    SYSCTL_REGCGC2_REG |= (1<<Port_PortChannels[number_of_pins].port_type);
    delay = SYSCTL_REGCGC2_REG;
    
    if(((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_D_NUM) && (Port_PortChannels[number_of_pins].pin_type == PortConf_PIN7_NUM)) || ((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_F_NUM) && (Port_PortChannels[number_of_pins].pin_type == PortConf_PIN0_NUM))) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_C_NUM) && (Port_PortChannels[number_of_pins].pin_type <= PortConf_PIN3_NUM) ) /* PC0 to PC3 */
    {
         continue;
        /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
     /*----------------------------------------------------------------Setting Up Pin Mode----------------------------------------------------------------------*/
    if (Port_PortChannels[number_of_pins].pin_mode == PortConf_Mode_ADC) /*ADC Mode Select */
    {
     SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);/*SET Analog */
     CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);/* Clear Digital */
    }
    else
    {
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);/*Clear Analog */
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);   /* Set Digital */
    }
    if ( Port_PortChannels[number_of_pins].pin_mode== PortConf_Mode_GPIO )
    {
     CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type); /* Clear Alternate Function To Select GPIO*/
     }
     else
     {
     SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type); /* Set Alternate Function To Use Other Function Of Pin */
     }
    switch (Port_PortChannels[number_of_pins].pin_mode)
	        {
	        case PortConf_Mode_GPIO:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[number_of_pins].pin_type* 4));
	        	break;
	        case PortConf_Mode_ADC:
	        	/* Do Nothing */
	        	break;
	        case PortConf_Mode_UART:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConf_Mode_UART << (Port_PortChannels[number_of_pins].pin_type * 4));
	        	break;
	        case PortConf_Mode_SSI:
	        	if ( (Port_PortChannels[number_of_pins].port_type == PortConf_PORT_D_NUM ) && ( Port_PortChannels[number_of_pins].pin_type <= PortConf_PIN3_NUM))
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConf_Mode_UART << (Port_PortChannels[number_of_pins].pin_type * 4));
	        	}
	        	else
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConf_Mode_SSI << (Port_PortChannels[number_of_pins].pin_type * 4));
	        	}
	        	break;
	        case PortConf_Mode_I2C:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConf_Mode_I2C << (Port_PortChannels[number_of_pins].pin_type * 4));
	        	break;
	        case PortConf_Mode_M0PWM:
	        	if(((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_D_NUM ) && ( Port_PortChannels[number_of_pins].pin_type <= PortConf_PIN6_NUM)) ||
	        		((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_F_NUM ) && ( Port_PortChannels[number_of_pins].pin_type <= PortConf_PIN2_NUM)))
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConf_Mode_M0FAULT << (Port_PortChannels[number_of_pins].pin_type * 4));
	        	}
	        	else
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConf_Mode_M0PWM << (Port_PortChannels[number_of_pins].pin_type * 4));
	        	}
	        	break;
	        case PortConf_Mode_M1PWM:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConf_Mode_M1PWM << (Port_PortChannels[number_of_pins].pin_type * 4));
	        	break;
	        case PortConf_Mode_IDX_PHASE:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConf_Mode_IDX_PHASE << (Port_PortChannels[number_of_pins].pin_type * 4));
	        	break;
	        case PortConf_Mode_TIMER:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConf_Mode_TIMER << (Port_PortChannels[number_of_pins].pin_type * 4));
	        	break;
	        case PortConf_Mode_CAN:
	        	if (((Port_PortChannels[number_of_pins].pin_type == PortConf_PORT_F_NUM ) && ( Port_PortChannels[number_of_pins].pin_type <= PortConf_PIN0_NUM)) ||
	        			((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_F_NUM ) && ( Port_PortChannels[number_of_pins].pin_type <= PortConf_PIN3_NUM)))
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConf_Mode_I2C << (Port_PortChannels[number_of_pins].pin_type * 4));
	        	}
	        	else if (((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_F_NUM ) && ( Port_PortChannels[number_of_pins].pin_type <= PortConf_PIN0_NUM)) ||
	        			((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_D_NUM ) && ( Port_PortChannels[number_of_pins].pin_type <= PortConf_PIN7_NUM)))
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConf_Mode_NMI << (Port_PortChannels[number_of_pins].pin_type * 4));
	        	}
	        	else if (((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_F_NUM ) && ( Port_PortChannels[number_of_pins].pin_type <= PortConf_PIN4_NUM)) ||
	        			((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_D_NUM ) && ( Port_PortChannels[number_of_pins].pin_type <= PortConf_PIN3_NUM))  ||
						((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_D_NUM ) && ( Port_PortChannels[number_of_pins].pin_type <= PortConf_PIN2_NUM))  ||
						((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_C_NUM ) && ( Port_PortChannels[number_of_pins].pin_type <= PortConf_PIN6_NUM))  ||
						((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_C_NUM ) && ( Port_PortChannels[number_of_pins].pin_type <= PortConf_PIN7_NUM)))

	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConf_Mode_USB << (Port_PortChannels[number_of_pins].pin_type * 4));
	        	}
	        	else
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConf_Mode_CAN << (Port_PortChannels[number_of_pins].pin_type * 4));
	        	}
	        	break;
	        }
    /*----------------------------------------------------------------Setting Up Pin Direction----------------------------------------------------------------------*/
     if(Port_PortChannels[number_of_pins].pin_direction == PORT_PIN_OUT)
     {
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
      /*----------------------------------------------------------------Setting Up Output Pin Initial Value----------------------------------------------------------------------*/
      if(Port_PortChannels[number_of_pins].pin_level == STD_HIGH)
      {
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
      }
      else
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
      }
      }
      /*----------------------------------------------------------------Setting Up Input Pin Resistor Type----------------------------------------------------------------------*/
      else if(Port_PortChannels[number_of_pins].pin_direction == PORT_PIN_IN)
	 {
	   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
           if(Port_PortChannels[number_of_pins].resistor == PULL_UP)
	      {
	         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
	        }
	          else if(Port_PortChannels[number_of_pins].resistor == PULL_DOWN)
	           {
	            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
	             }
	              else
	                {
	                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
	                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
	                }
	            }
	            else
	            {
	                /* Do Nothing */
	            }

    
}
}


/************************************************************************************
* Service Name: Port_SetPinDirection
* Syntax: void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction )
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): 
                  -Pin      -Port Pin ID number
                  -Direction-Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction
************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction )
{
	boolean error = FALSE;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
	/* Check if the used channel is within the valid range */
	if (PORT_CONFIGURED_CHANNLES <= Pin)
	{

		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
        /* Check if the used channel direction is changeable or not */
	if (Port_PortChannels[Pin].direction_changeable == STD_OFF)
	{

		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
        
#endif
        volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
	volatile uint32 delay = 0;
        /* In-case there are no errors */
	if (error == FALSE)
	{
		switch(Port_PortChannels[Pin].port_type)
		{
		case  PortConf_PORT_A_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		break;
		case  PortConf_PORT_B_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		break;
		case  PortConf_PORT_C_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		break;
		case  PortConf_PORT_D_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		break;
		case  PortConf_PORT_E_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		break;
		case  PortConf_PORT_F_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		break;
		}
		/* Enable clock for PORT and allow time for clock to start*/
	    SYSCTL_REGCGC2_REG |= (1<<Port_PortChannels[Pin].port_type);
	    delay =SYSCTL_REGCGC2_REG ;

	    if( ((Port_PortChannels[Pin].port_type == PortConf_PORT_D_NUM) && (Port_PortChannels[Pin].pin_type == PortConf_PIN7_NUM)) || ((Port_PortChannels[Pin].port_type == PortConf_PORT_F_NUM) && (Port_PortChannels[Pin].pin_type == PortConf_PIN0_NUM)) ) /* PD7 or PF0 */
	    {
	     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */
	     SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PortChannels[Pin].pin_type);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
	     }
	     else if( (Port_PortChannels[Pin].port_type == PortConf_PORT_C_NUM) && (Port_PortChannels[Pin].pin_type <= PortConf_PIN3_NUM) ) /* PC0 to PC3 */
	     {
	     /* Do Nothing ...  this is the JTAG pins */
             
	     return;
	     }
	     else
	     {
	     /* Do Nothing ... No need to unlock the commit register for this pin */
	     }

		if (Direction == PORT_PIN_OUT)
		{
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[Pin].pin_type);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
		   }
		else if (Direction == PORT_PIN_IN)
		{
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[Pin].pin_type);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
		}
		else
		{
			/* Do Nothing */
		}

	}
	else
	{
		/* Do Nothing */
	}
        
}
#endif

/************************************************************************************
* Service Name: Port_SetPinMode
* Syntax: void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode )
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): 
                  -Pin      -Port Pin ID number
                  -Mode     -New Port Pin mode to be set on port pin.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode.
************************************************************************************/
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode )
{
	boolean error = FALSE;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
	/* Check if the used channel is within the valid range */
	if (PORT_CONFIGURED_CHANNLES <= Pin)
	{

		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
        /* Check if the used channel direction is changeable or not */
	if (Port_PortChannels[Pin].mode_changeable == STD_OFF)
	{

		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
        /* Check if the used channel direction is changeable or not */
	if (PORT_CONFIGURED_MODES <= Mode)
	{

		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
#endif
        volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
	volatile uint32 delay = 0;
        /* In-case there are no errors */
	if(FALSE == error){
          switch(Port_PortChannels[Pin].port_type)
      {
        case  PortConf_PORT_A_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break;
	case  PortConf_PORT_B_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case  PortConf_PORT_C_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case  PortConf_PORT_D_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case  PortConf_PORT_E_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case  PortConf_PORT_F_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
    }
    
    /* Enable clock for PORT and allow time for clock to start*/
    SYSCTL_REGCGC2_REG |= (1<<Port_PortChannels[Pin].port_type);
    delay = SYSCTL_REGCGC2_REG;
    
    if(((Port_PortChannels[Pin].port_type == PortConf_PORT_D_NUM) && (Port_PortChannels[Pin].pin_type == PortConf_PIN7_NUM)) || ((Port_PortChannels[Pin].port_type == PortConf_PORT_F_NUM) && (Port_PortChannels[Pin].pin_type == PortConf_PIN0_NUM))) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PortChannels[Pin].pin_type);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if((Port_PortChannels[Pin].port_type == PortConf_PORT_C_NUM) && (Port_PortChannels[Pin].pin_type <= PortConf_PIN3_NUM) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
      return;
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
    if (Port_PortChannels[Pin].pin_mode == PortConf_Mode_ADC) /*ADC Mode Select */
    {
     SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].pin_type);/*SET Analog */
     CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].pin_type);/* Clear Digital */
    }
    else
    {
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].pin_type);/*Clear Analog */
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].pin_type);   /* Set Digital */
    }
    if ( Port_PortChannels[Pin].pin_mode== PortConf_Mode_GPIO )
    {
     CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].pin_type); /* Clear Alternate Function To Select GPIO*/
     }
     else
     {
     SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].pin_type); /* Set Alternate Function To Use Other Function Of Pin */
     }
    switch (Mode)
{
 case PortConf_Mode_GPIO:
 *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[Pin].pin_type* 4));
 break;
 case PortConf_Mode_ADC:
 /* Do Nothing */
	        	break;
	        case PortConf_Mode_UART:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConf_Mode_UART << (Port_PortChannels[Pin].pin_type * 4));
	        	break;
	        case PortConf_Mode_SSI:
	        	if ( (Port_PortChannels[Pin].port_type == PortConf_PORT_D_NUM ) && ( Port_PortChannels[Pin].pin_type <= PortConf_PIN3_NUM))
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConf_Mode_UART << (Port_PortChannels[Pin].pin_type * 4));
	        	}
	        	else
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConf_Mode_SSI << (Port_PortChannels[Pin].pin_type * 4));
	        	}
	        	break;
	        case PortConf_Mode_I2C:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConf_Mode_I2C << (Port_PortChannels[Pin].pin_type * 4));
	        	break;
	        case PortConf_Mode_M0PWM:
	        	if(((Port_PortChannels[Pin].port_type == PortConf_PORT_D_NUM ) && ( Port_PortChannels[Pin].pin_type <= PortConf_PIN6_NUM)) ||
	        		((Port_PortChannels[Pin].port_type == PortConf_PORT_F_NUM ) && ( Port_PortChannels[Pin].pin_type <= PortConf_PIN2_NUM)))
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConf_Mode_M0FAULT << (Port_PortChannels[Pin].pin_type * 4));
	        	}
	        	else
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConf_Mode_M0PWM << (Port_PortChannels[Pin].pin_type * 4));
	        	}
	        	break;
	        case PortConf_Mode_M1PWM:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConf_Mode_M1PWM << (Port_PortChannels[Pin].pin_type * 4));
	        	break;
	        case PortConf_Mode_IDX_PHASE:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConf_Mode_IDX_PHASE << (Port_PortChannels[Pin].pin_type * 4));
	        	break;
	        case PortConf_Mode_TIMER:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConf_Mode_TIMER << (Port_PortChannels[Pin].pin_type * 4));
	        	break;
	        case PortConf_Mode_CAN:
	        	if (((Port_PortChannels[Pin].pin_type == PortConf_PORT_F_NUM ) && ( Port_PortChannels[Pin].pin_type <= PortConf_PIN0_NUM)) ||
	        			((Port_PortChannels[Pin].port_type == PortConf_PORT_F_NUM ) && ( Port_PortChannels[Pin].pin_type <= PortConf_PIN3_NUM)))
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConf_Mode_I2C << (Port_PortChannels[Pin].pin_type * 4));
	        	}
	        	else if (((Port_PortChannels[Pin].port_type == PortConf_PORT_F_NUM ) && ( Port_PortChannels[Pin].pin_type <= PortConf_PIN0_NUM)) ||
	        			((Port_PortChannels[Pin].port_type == PortConf_PORT_D_NUM ) && ( Port_PortChannels[Pin].pin_type <= PortConf_PIN7_NUM)))
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConf_Mode_NMI << (Port_PortChannels[Pin].pin_type * 4));
	        	}
	        	else if (((Port_PortChannels[Pin].port_type == PortConf_PORT_F_NUM ) && ( Port_PortChannels[Pin].pin_type <= PortConf_PIN4_NUM)) ||
	        			((Port_PortChannels[Pin].port_type == PortConf_PORT_D_NUM ) && ( Port_PortChannels[Pin].pin_type <= PortConf_PIN3_NUM))  ||
						((Port_PortChannels[Pin].port_type == PortConf_PORT_D_NUM ) && ( Port_PortChannels[Pin].pin_type <= PortConf_PIN2_NUM))  ||
						((Port_PortChannels[Pin].port_type == PortConf_PORT_C_NUM ) && ( Port_PortChannels[Pin].pin_type <= PortConf_PIN6_NUM))  ||
						((Port_PortChannels[Pin].port_type == PortConf_PORT_C_NUM ) && ( Port_PortChannels[Pin].pin_type <= PortConf_PIN7_NUM)))

	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConf_Mode_USB << (Port_PortChannels[Pin].pin_type * 4));
	        	}
	        	else
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConf_Mode_CAN << (Port_PortChannels[Pin].pin_type * 4));
	        	}
	        	break;
	        }

    
        }
        
}

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Syntax: void Port_RefreshPortDirection( void )
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): Non
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshs The Direction Of Pin.
************************************************************************************/
void Port_RefreshPortDirection( void ){ 
boolean error = FALSE;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_REFRESH_PORT_DIRECTION_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
#endif
        volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
	volatile uint32 delay = 0;
        volatile uint8 number_of_pins=0;
        /* In-case there are no errors */
	if(FALSE == error){
  for(number_of_pins=0;number_of_pins<PORT_CONFIGURED_CHANNLES;number_of_pins++){
	switch(Port_PortChannels[number_of_pins].port_type)
      {
        case  PortConf_PORT_A_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break;
	case  PortConf_PORT_B_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case  PortConf_PORT_C_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case  PortConf_PORT_D_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case  PortConf_PORT_E_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case  PortConf_PORT_F_NUM: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
    }
    
    /* Enable clock for PORT and allow time for clock to start*/
    SYSCTL_REGCGC2_REG |= (1<<Port_PortChannels[number_of_pins].port_type);
    delay = SYSCTL_REGCGC2_REG;
    
    if(((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_D_NUM) && (Port_PortChannels[number_of_pins].pin_type == PortConf_PIN7_NUM)) || ((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_F_NUM) && (Port_PortChannels[number_of_pins].pin_type == PortConf_PIN0_NUM))) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if((Port_PortChannels[number_of_pins].port_type == PortConf_PORT_C_NUM) && (Port_PortChannels[number_of_pins].pin_type <= PortConf_PIN3_NUM) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
      continue;
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
    if (Port_PortChannels[number_of_pins].direction_changeable == PortConf_PIN_DIRECTION_NOT_CHANGEABLE)
			       {
			    	   /* Do Nothing */
			       }
			       else
			       {
			    	   if(Port_PortChannels[number_of_pins].pin_direction == PORT_PIN_OUT)
			    	   	{
			    		   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
			    	   	}
			    	   	else if(Port_PortChannels[number_of_pins].pin_direction == PORT_PIN_IN)
			    	   	{
			    	   		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[number_of_pins].pin_type);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
			    	   	}
			    	   	else
			    	   	{
			    	   			 /* Do Nothing */
			    	   	}
			       }

			 }
	}
	else
	{
		/* Do Nothing */
	}
    
    
 
}



/************************************************************************************
* Service name:Port_GetVersionInfo
* Syntax: void Port_GetVersionInfo( Std_VersionInfoType* versioninfo )
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo -Pointer to where to store the version information of this module.
* Return value: None
* Description: Returns the version information of this module
************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo( Std_VersionInfoType* versioninfo )
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if input pointer is not Null pointer */
	if(NULL_PTR == versioninfo)
	{
		/* Report to DET  */
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
	}
	else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
	{
		/* Copy the vendor Id */
		versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
		/* Copy the module Id */
		versioninfo->moduleID = (uint16)PORT_MODULE_ID;
		/* Copy Software Major Version */
		versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
		/* Copy Software Minor Version */
		versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
		/* Copy Software Patch Version */
		versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
	}
}
#endif
