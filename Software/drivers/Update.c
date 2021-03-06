/***************************************************************************//**
 * @file
 * @brief	Update Manager
 * @author	Peter Loes
 * @version	2021-06-25
 *
 * This is the Update Manager module.
 *
 ****************************************************************************//*
Revision History:
2020-06-25,rage - PowerUp() must be called to set Port Pin D0 and enable FET T1.
		- UpdateKeyHandler Pushbutton 1 and Pushbutton 2 
                - UpdateCheck added flags for
                  Pushbutton1 DATA LOGGER is ON or off
                  Pushbutton2 COLLECT DATA is ON or off
          	  Probing Sensor1 and Sensor2.
                  LogInfo about microcontroller voltage                                                 
2015-06-22,rage	Initial version, derived from SNB_Heaven.
*/

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "em_device.h"
#include "em_assert.h"
#include "Keys.h"
#include "AlarmClock.h"
#include "Update.h"
#include "SensorMon.h"
#include "Logging.h"


/*=============================== Definitions ================================*/

    //@brief Set level of the power enable pin.
#define SET_POWER_PIN(level)  IO_Bit(GPIO->P[HOLD_POWER_PORT].DOUT,	\
				     HOLD_POWER_PIN) = (level)

/*=============================== External Data ==============================*/

    /* external routine to switch Power Logger LED1 on or off */
extern void ShowPowerLogger_LED1 (bool enable);

/*================================ Local Data ================================*/

    /*!@brief Flag to trigger Power Off. */
static volatile bool    l_flgPowerOff = false;

static volatile bool    flgPowerOffActive = true;

    /*!@brief Flag to trigger Power Collecting Data off. */
static volatile bool	 l_flgSensorPowerOff = true;
 
/*!@brief Current state if collecting data: true means ON, false mean OFF. */
static volatile bool	 l_flgDataCollectIsOn;

/*!@brief Current state if disk is removed: true means ON, false mean OFF. */
static volatile bool	 l_flgisDiskRemoved;

/***************************************************************************//**
 *
 * @brief	Power Up the Device
 *
 * This routine initializes and sets Port Pin D1 to enable FET T1.
 *
 ******************************************************************************/
void  PowerUp (void)
{
    /* Configure PD1 to hold power, set FET input to HIGH */
    GPIO_PinModeSet (HOLD_POWER_PORT, HOLD_POWER_PIN, gpioModePushPull, 1);
}


/***************************************************************************//**
 *
 * @brief	Update Key Handler
 *
 * This handler receives the translated key codes from the interrupt-driven
 * key handler, including autorepeat keys.  That is, whenever the user asserts
 * a key (push button), the resulting code is sent to this function.  The main
 * purpose of the handler is log messages and set start_stop flags.
 *
 * The following keys are recognized:
 * - <b>POWER</b> Power up the Data Logger. Keep asserted to switch off the
 *   device.
 * - <b>START_STOP</b> START or STOP data collecting form sensors.
 *
 * @warning
 * 	This function is called in interrupt context!
 *
 * @param[in] keycode
 *	Translated key code of type KEYCODE.
 *
 ******************************************************************************/
void	UpdateKeyHandler (KEYCODE keycode)
{
    switch (keycode)
    {
	case KEYCODE_POWER_ASSERT:	// POWER was asserted
             break;
         
	case KEYCODE_POWER_REPEAT:	// repeated device POWER was asserted
             /* Note: l_flgPowerOff is set below */ 
             break;		 
          
	case KEYCODE_START_STOP_ASSERT:	// START_STOP was asserted
             break;	
             
	case KEYCODE_START_STOP_REPEAT:	// repeated START_STOP was asserted
             /* Note: l_flgSensorPowerOff is set below */ 
             break;

	case KEYCODE_POWER_RELEASE:	 // POWER was released
        case KEYCODE_START_STOP_RELEASE: // START_STOP was released
          
           return;
            
	default:	// ignore all other key codes
	    return;
    }
    
    /* POWER_REPEAT should switch the device off (when button is released) */
    l_flgPowerOff = (keycode == KEYCODE_POWER_REPEAT ? true : false);
    
     /* START_STOP_REPEAT should switch collecting data off (when button is released) */
    l_flgSensorPowerOff = (keycode == KEYCODE_START_STOP_REPEAT ? true : false);
}

    
/***************************************************************************//**
 *
 * @brief	Update Check
 *
 * This function checks if the information needs to be updated.
 * It also handles power-off from data logger and sensors controller.
 *
 * @note
 * 	This function may be called from standard program, usually the loop
 * 	in module "main.c" - it must not be called from interrupt routines!
 * 	In this application it is called every second, triggered from the
 * 	one second interrupt of the RTC.
 *
 ******************************************************************************/
void	UpdateCheck (void)
{
static bool flgPowerOffActive;
static bool flgSensorPowerOffActive;
       
    /*
     * Check if Data Logger should be powered off.
     */
    if (l_flgPowerOff)
    {
        if(flgPowerOffActive)
        {  
            flgPowerOffActive = false;
            Log ("DATA LOGGER is off");
            msDelay(200);
            
            /*  Switch red LED1 off */  
            ShowPowerLogger_LED1 (false);
            SET_POWER_PIN(0);	// set FET input to HIGH
            return;    // INHIBIT ALL OTHER ACTIONS
        }
    } 
    else
    {
	if (! flgPowerOffActive)
	{
	    flgPowerOffActive = true;
            Log ("DATA LOGGER is ON");
            
            /*  Switch red LED1 ON */  
            ShowPowerLogger_LED1 (true);
            SET_POWER_PIN(1);	// set FET input to HIGH 
     	}
    }
    
    /*
     * Key START_STOP is pressed and collecting data is set.
     * l_flgDataCollectIsOn is active.
     */
 
    if (l_flgSensorPowerOff)
    {
        if (! flgSensorPowerOffActive)
	{
	    flgSensorPowerOffActive = true;
            Log ("COLLECTING DATA is ON");              
            
            /* Switch red LED1 off */ 
            ShowPowerLogger_LED1 (false);
            l_flgDataCollectIsOn = true;
        }
    }
    else
    {
	if (flgSensorPowerOffActive)
	{
	    flgSensorPowerOffActive = false;
                 
            Log ("COLLECTING DATA is off");
            
            /* Switch red LED1 ON */ 
            ShowPowerLogger_LED1 (true);
            l_flgDataCollectIsOn = false;
     	}
    }
}


/***************************************************************************//**
 *
 * @brief	Determine if Collecting Data is on
 *
 * After Press Button 2 Collecting Data the SENSOR_POWER will be switched
 * on and off by setting alarm times and interval times.  This routine
 * is used to determine collecting data via Button 2.
 *
 * @return
 * 	The value <i>true</i> if data collecting is on, <i>false</i> if not.
 *
 ******************************************************************************/
bool	IsDataCollectOn (void)
{      
    return l_flgDataCollectIsOn;
}