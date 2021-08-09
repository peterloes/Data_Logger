/***************************************************************************//**
 * @file
 * @brief	Sequence Control
 * @author	Ralf Gerhauser
 * @version	2018-10-10
 *
 * This is the automatic sequence control module.  It controls the power
 * outputs and the measurement of their voltage and current.  Calibration
 * routines and the ability to write data to an @ref EEPROM area makes it
 * possible to store board-specific factors, so, voltage and current can be
 * calculated and logged .<br>
 * This module also defines the configuration variables for the file
 * <a href="../../CONFIG.TXT"><i>CONFIG.TXT</i></a>.
 *
 ****************************************************************************//*
Revision History:
2019-02-09,rage	- Added configuration variable SCAN_DURATION which specifies the
		  measurement duration of a single channel in [ms].  Valid range
		  is 52ms to 2200ms.  There are four channels: U1, I1, U2, I2.
2018-10-10,rage	Implemented Power Cycling for each output during the on-times.
2018-03-26,rage	Initial version, based on MAPRDL.
*/

/*=============================== Header Files ===============================*/

#include "em_cmu.h"
#include "em_int.h"
#include "em_gpio.h"
#include "ExtInt.h"
#include "Logging.h"
#include "AlarmClock.h"
#include "SensorMon.h"
#include "CfgData.h"
#include "Control.h"
#include "microsd.h"


/*=============================== Definitions ================================*/

    /*!@brief Structure to define power outputs. */
typedef struct
{
    __IO uint32_t *BitBandAddr;	// Bit band address of GPIO power enable pin
    bool	   HighActive;	// true: The power enable pin is high-active
} PWR_OUT_DEF;

    /*!@brief Macro to calculate a GPIO bit address for a port and pin. */
#define GPIO_BIT_ADDR(port, pin)					\
	IO_BIT_ADDR((&(GPIO->P[(port)].DOUT)), (pin))

/*!@brief Macro to extract the port number from a GPIO bit address.  */
#define GPIO_BIT_ADDR_TO_PORT(bitAddr)		(GPIO_Port_TypeDef)	\
	(((((uint32_t)(bitAddr) - BITBAND_PER_BASE) >> 5)		\
	 + PER_MEM_BASE - GPIO_BASE) / sizeof(GPIO_P_TypeDef))

    /*!@brief Macro to extract the pin number from a GPIO bit address.  */
#define GPIO_BIT_ADDR_TO_PIN(bitAddr)					\
	(((uint32_t)(bitAddr) >> 2) & 0x1F)

/*================================ Global Data ===============================*/

    /*!@brief CFG_VAR_TYPE_ENUM_2: Enum names for Power Outputs. */
const char *g_enum_PowerOutput[] = { "SENSOR1", "SENSOR2", NULL };

    /*!@brief Power Cycle Interval for SENSOR1 and SENSOR2, 0=disable */
int32_t		g_PwrInterval[NUM_PWR_OUT];

    /*!@brief Power Cycle ON duration for SENSOR1 and SENSOR2, 0=disable */
int32_t		g_On_Duration[NUM_PWR_OUT];

    /*!@brief date and time structure. */
struct tm    DateTime;	// not intended to be used by other modules


/*================================ Local Data ================================*/

    /*!@brief Timer handles for SENSOR1 and SENSOR2 Power Cycle Interval */
static TIM_HDL		l_hdlPwrInterval[NUM_PWR_OUT] = { NONE, NONE };


    /*!@brief Power output port and pin assignment - keep in sync with enums
     * @ref PWR_OUT and string array @ref g_enum_PowerOutput !
     */
static const PWR_OUT_DEF  l_PwrOutDef[NUM_PWR_OUT] =
{   //     BitBandAddr,         HighActive          
    { GPIO_BIT_ADDR(gpioPortE, 9), true  }, // PE9 enables PWR_OUT_VDD_SENSOR1
    { GPIO_BIT_ADDR(gpioPortC, 9), true  }, // PC9 enables PWR_OUT_VDD_SENSOR2
};


    /*!@brief Assume DateStamp from config.txt */
static int32_t	 g_DateStampDAY,  g_DateStampMONTH,  g_DateStampYEAR;

    /*!@brief Assume TimeStamp from config.txt */
static int32_t	 g_TimeStampHOUR, g_TimeStampMINUTE, g_TimeStampSECOND;

     /*!@brief List of configuration variables.
     * Alarm times, i.e. @ref CFG_VAR_TYPE_TIME must be defined first, because
     * the array index is used to specify the alarm number \<alarmNum\>,
     * starting with @ref SENSOR1_ON_TIME_1, when calling AlarmSet().
     */
static const CFG_VAR_DEF l_CfgVarList[] =
{
    // Alarm times (must be consecutive)
    { "SENSOR1_ON_TIME_1",    CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR1_ON_TIME_2",    CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR1_ON_TIME_3",    CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR1_ON_TIME_4",    CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR1_ON_TIME_5",    CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR2_ON_TIME_1",    CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR2_ON_TIME_2",    CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR2_ON_TIME_3",    CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR2_ON_TIME_4",    CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR2_ON_TIME_5",    CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR1_OFF_TIME_1",   CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR1_OFF_TIME_2",   CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR1_OFF_TIME_3",   CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR1_OFF_TIME_4",   CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR1_OFF_TIME_5",   CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR2_OFF_TIME_1",   CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR2_OFF_TIME_2",   CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR2_OFF_TIME_3",   CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR2_OFF_TIME_4",   CFG_VAR_TYPE_TIME,	NULL	      },
    { "SENSOR2_OFF_TIME_5",   CFG_VAR_TYPE_TIME,	NULL	      },
    // Power Cycling Intervals for SENSOR1 and SENSOR2
    { "SENSOR1_INTERVAL",     CFG_VAR_TYPE_DURATION, &g_PwrInterval[PWR_OUT_SENSOR1]   },
    { "SENSOR1_ON_DURATION",  CFG_VAR_TYPE_DURATION, &g_On_Duration[PWR_OUT_SENSOR1]   },
    { "SENSOR2_INTERVAL",     CFG_VAR_TYPE_DURATION, &g_PwrInterval[PWR_OUT_SENSOR2]   },
    { "SENSOR2_ON_DURATION",  CFG_VAR_TYPE_DURATION, &g_On_Duration[PWR_OUT_SENSOR2]   },
    // Configuration for  SENSOR1 and SENSOR2
    { "SENSOR1_TYPE",	      CFG_VAR_TYPE_ENUM_1,	&g_SENSOR1_Type      },
    { "SENSOR2_TYPE",	      CFG_VAR_TYPE_ENUM_1,	&g_SENSOR2_Type      },
    // Configuration of date
    { "DAY",  	              CFG_VAR_TYPE_INTEGER,	&g_DateStampDAY      },
    { "MONTH",                CFG_VAR_TYPE_INTEGER,	&g_DateStampMONTH    },
    { "YEAR",		      CFG_VAR_TYPE_INTEGER,	&g_DateStampYEAR     },
    // Configuration of time
    { "HOUR",		      CFG_VAR_TYPE_INTEGER,	&g_TimeStampHOUR     },
    { "MINUTE",		      CFG_VAR_TYPE_INTEGER,	&g_TimeStampMINUTE   },
    { "SECOND",		      CFG_VAR_TYPE_INTEGER,	&g_TimeStampSECOND   },
    {  NULL,                  END_CFG_VAR_TYPE,         NULL                 }
};

    /*!@brief List of all enum definitions. */
static const ENUM_DEF l_EnumList[] =
{
       g_enum_SENSOR1_Type,		// CFG_VAR_TYPE_ENUM_1  
       g_enum_SENSOR2_Type,		// CFG_VAR_TYPE_ENUM_1  
       g_enum_PowerOutput,		// CFG_VAR_TYPE_ENUM_2
};


/*=========================== Forward Declarations ===========================*/

static void	AlarmPowerControl (int alarmNum);
static void	IntervalPowerControl (TIM_HDL hdl);
static void	DateTimeSynchronize (struct tm *pDateTime);


/***************************************************************************//**
 *
 * @brief	Initialize control module
 *
 * This routine initializes the sequence control module.
 *
 ******************************************************************************/
void	ControlInit (void)
{
int	i;

    /* Introduce variable list to configuration data module */
    CfgDataInit (l_CfgVarList, l_EnumList);

    for (i = 0;  i < NUM_PWR_OUT;  i++)
    {
	if (l_hdlPwrInterval[i] == NONE)
	    l_hdlPwrInterval[i] = sTimerCreate (IntervalPowerControl);
    }
    
    /* Initialize power output enable pins */
    for (i = 0;  i < NUM_PWR_OUT;  i++)
    {
	/* Configure Power Enable Pin, switch it OFF per default */
	GPIO_PinModeSet (GPIO_BIT_ADDR_TO_PORT(l_PwrOutDef[i].BitBandAddr),
			 GPIO_BIT_ADDR_TO_PIN (l_PwrOutDef[i].BitBandAddr),
			 gpioModePushPull, 0);
    }
   
     /* Use same routine for all power-related alarms */
    for (i = FIRST_POWER_ALARM;  i <= LAST_POWER_ALARM;  i++)
	AlarmAction (i, AlarmPowerControl);

    /* Initialize configuration with default values */
    ClearConfiguration();
}


/***************************************************************************//**
 *
 * @brief	Clear Configuration
 *
 * This routine disables all alarm times and switches the corresponding power
 * outputs off.  It then sets all configuration variables to default values.
 * It must be executed <b>before</b> calling CfgRead() to to ensure the correct
 * settings for variables which are <b>not</b> set within a new configuration.
 *
 ******************************************************************************/
void	ClearConfiguration (void)
{
int	i;

       /* Disable all power-related alarms */
    for (i = ALARM_SENSOR1_ON_TIME_1;  i <= LAST_POWER_ALARM;  i++)
	AlarmDisable(i);
          
    /* Disable Power Cycle Interval */
    for (i = 0;  i < NUM_PWR_OUT;  i++)
    {
	g_PwrInterval[i] = 0;
	g_On_Duration[i] = 0;
    }
    
   /* Disable SENSOR1 and SENSOR2 functionality */
    g_SENSOR1_Type = SENSOR1_TYPE_NONE;
    g_SENSOR2_Type = SENSOR2_TYPE_NONE;
    g_SENSOR_Power = PWR_OUT_NONE;
}

/***************************************************************************//**
 *
 * @brief	Verify Configuration
 *
 * This routine verifies if the new configuration values are valid.  It must
 * be executed <b>after</b> calling CfgRead().
 *
 ******************************************************************************/
void	VerifyConfiguration (void)
{
#define MIN_VAL_INTERVAL	10	//<! Minimum Power Cycle Interval in [s]
#define MIN_VAL_ON_DURATION	 5	//<! Minimum ON Duration in [s]
#define MIN_VAL_OFF_DURATION	 5	//<! Minimum OFF Duration in [s]
int	i;
bool	error;
int32_t	interval, duration;

    /* Verify Power Cycle Interval */
    for (i = 0;  i < NUM_PWR_OUT;  i++)
    {
	error = false;
	interval = g_PwrInterval[i];
	duration = g_On_Duration[i];

	if (interval <= 0)
	    continue;			// this channel is inactive

	if (interval < MIN_VAL_INTERVAL)
	{
	    LogError ("Config File - %s_INTERVAL: Value %ds is too small,"
		      " minimum is %ds",
		      g_enum_PowerOutput[i], interval, MIN_VAL_INTERVAL);
	    error = true;
	}
	else if (duration < MIN_VAL_ON_DURATION)
	{
	    LogError ("Config File - %s_ON_DURATION: Value %ds is too small,"
		      " minimum is %ds",
		      g_enum_PowerOutput[i], duration, MIN_VAL_ON_DURATION);
	    error = true;
	}
	else if (interval - duration < MIN_VAL_OFF_DURATION)
	{
	    LogError ("Config File - %s_ON_DURATION: Off duration of %ds is"
		      " too small, limit the On duration!",
		      g_enum_PowerOutput[i], interval - duration);
	    error = true;
	}

	if (error)
	{
	    g_PwrInterval[i] = (-1);
	    g_On_Duration[i] = (-1);
	}
    }    
    
    DateTime.tm_year = g_DateStampYEAR;
    DateTime.tm_mon  = g_DateStampMONTH -1;
    DateTime.tm_mday = g_DateStampDAY;
    DateTime.tm_hour = g_TimeStampHOUR;
    DateTime.tm_min  = g_TimeStampMINUTE;
    DateTime.tm_sec  = g_TimeStampSECOND;
    
    /* set date and time on the microcontroller */
   DateTimeSynchronize(&DateTime);

}


/******************************************************************************
 *
 * @brief	Switch the specified power output on or off
 *
 * This routine enables or disables the specified power output.
 *
 * @param[in] output
 *	Power output to be changed.
 *
 * @param[in] enable
 *	If true (PWR_ON), the power output will be enabled, false (PWR_OFF)
 *	disables it.
 *
 *****************************************************************************/
void	PowerOutput (PWR_OUT output, bool enable)
{
    /*!@brief data collect is activate. */
bool   isDataCollectOn;	
bool   isDiskRemoved;

    /* Get current state of DataCollectOn (PB2 is ON) */
    isDataCollectOn = IsDataCollectOn();
    isDiskRemoved = IsDiskRemoved(); 

    /* Parameter check */
    if (output == PWR_OUT_NONE)
	return;		// power output not assigned, nothing to be done

    if ((PWR_OUT)0 > output  ||  output >= NUM_PWR_OUT)
    {
#ifdef LOGGING
	/* Generate Error Log Message */
	LogError ("PowerOutput(%d, %d): Invalid output parameter",
		  output, enable);
#endif
	return;
    }

    /* See if Power Output is already in the right state */
    if ((bool)*l_PwrOutDef[output].BitBandAddr == enable)
	return;		// Yes - nothing to be done

    /* Switch power output on or off */
    *l_PwrOutDef[output].BitBandAddr = enable;
    
    /* Data Collection is off and Disk is not removed */ 
    if(!isDataCollectOn || isDiskRemoved)
       return;   
    
#ifdef LOGGING
    Log ("Power Output %s %sabled",
   	 g_enum_PowerOutput[output], enable ? "en":"dis");
#endif
    
    g_flgIRQ = true;	// keep on running
   
    /* Data Collection is off and Disk is not removed */ 
//    if(!isDataCollectOn && !isDiskRemoved)
//       return; 

   /* If Power Output enabled get Message */ 
   if(enable)
   {   
      if (output == PWR_OUT_SENSOR1)
        LogSensorInfo (BAT_LOG_INFO_VERBOSE1);
      
      if (output == PWR_OUT_SENSOR2)
         LogSensorInfo (BAT_LOG_INFO_VERBOSE2);
   }
}


/******************************************************************************
 *
 * @brief	Determine if the specified power output is switched on
 *
 * This routine determines the current state of a power output.
 *
 * @param[in] output
 *	Power output to be checked.
 *
 *****************************************************************************/
bool	IsPowerOutputOn (PWR_OUT output)
{
  /* Parameter check */
    if (output == PWR_OUT_NONE)
	return false;	// power output not assigned, return false (off)

    EFM_ASSERT (PWR_OUT_SENSOR1 <= output  &&  output <= PWR_OUT_SENSOR2);
 
    
    /* Determine the current state of this power output */
    return (*l_PwrOutDef[output].BitBandAddr ? true : false);
}


/***************************************************************************//**
 *
 * @brief	Alarm routine for Power Control
 *
 * This routine is called when one of the power alarm times has been reached.
 * The alarm number is an enum value between @ref FIRST_POWER_ALARM and
 * @ref LAST_POWER_ALARM.<br>
 *
 ******************************************************************************/
static void	AlarmPowerControl (int alarmNum)
{
PWR_OUT	 pwrOut;
int	 pwrState;

    /* Parameter check */
    EFM_ASSERT (FIRST_POWER_ALARM <= alarmNum && alarmNum <= LAST_POWER_ALARM);
    
    /* Determine switching state */
    pwrState = (alarmNum >= ALARM_SENSOR1_OFF_TIME_1 ? PWR_OFF : PWR_ON);

    /* Determine Power Output and switching state */
    if (alarmNum >= ALARM_SENSOR2_OFF_TIME_1)
    {
	pwrOut = PWR_OUT_SENSOR2;
    }
    else if (alarmNum >= ALARM_SENSOR1_OFF_TIME_1)
    {
	pwrOut = PWR_OUT_SENSOR1;
    }
    else if (alarmNum >= ALARM_SENSOR2_ON_TIME_1)
    {
	pwrOut = PWR_OUT_SENSOR2;
    }
    else 
    {
	pwrOut = PWR_OUT_SENSOR1;
    }    

    /* If configured, initiate Power Interval */
    if (pwrState == PWR_OFF)
    {   // inhibit further power switching
	sTimerCancel (l_hdlPwrInterval[pwrOut]);
    }
    else
    {   // start new power interval
	if (g_On_Duration[pwrOut] >= MIN_VAL_ON_DURATION)
	    sTimerStart(l_hdlPwrInterval[pwrOut], g_On_Duration[pwrOut]);
    }
    
    /* Standard Power Output - call PowerOutput() directly */
    PowerOutput (pwrOut, pwrState);
 
    g_flgIRQ = true;	// keep on running
}


/***************************************************************************//**
 *
 * @brief	Interval Timer routine for Power Control
 *
 * This routine is called by one of the interval timers.  The timer handle is
 * used to identify the Power Output, which can be @ref PWR_OUT_SENSOR1 or @ref
 * PWR_OUT_SENSOR2.  The power interval consists of a
 * <b>XXX_ON_DURATION</b> seconds on phase and a <b>XXX_INTERVAL</b> minus
 * <b>XXX_ON_DURATION</b> off phase.
 *
 ******************************************************************************/
static void	IntervalPowerControl (TIM_HDL hdl)
{
PWR_OUT	 pwrOut;
int	 pwrState;

  /*!@brief data collect is activate. */
bool   isDataCollectOn;	
bool   isDiskRemoved;

    /* Get current state of DataCollectOn (PB2 is ON) */
    isDataCollectOn = IsDataCollectOn();
    isDiskRemoved = IsDiskRemoved(); 

    /* Determine Power Output */
    if (hdl == l_hdlPwrInterval[PWR_OUT_SENSOR1])
    {
	pwrOut = PWR_OUT_SENSOR1;
    }
    else if (hdl == l_hdlPwrInterval[PWR_OUT_SENSOR2])
    {
	pwrOut = PWR_OUT_SENSOR2;
    }
    else
    {
	LogError ("IntervalPowerControl(%d): Invalid timer handle", hdl);
	return;		// Invalid handle - abort
    }

    /* Check of Power Cycling is active for this Output */
    if (g_PwrInterval[pwrOut] < MIN_VAL_INTERVAL)
	return;		// No - immediately return

    /* Determine switching state and change phase */
    if (IsPowerOutputOn(pwrOut))
    {
	// Power is currently ON - switch it OFF for a while
	pwrState = PWR_OFF;
	sTimerStart(l_hdlPwrInterval[pwrOut],
		    g_PwrInterval[pwrOut] - g_On_Duration[pwrOut]);
    }
    else
    {
	// Power is currently OFF - switch it ON for a while
	pwrState = PWR_ON;
	sTimerStart(l_hdlPwrInterval[pwrOut], g_On_Duration[pwrOut]);
    }

    /* Data Collection is off and Disk is not removed */ 
    if(!isDataCollectOn && !isDiskRemoved)
       return;
    
   
    /* Standard Power Output - call PowerOutput() directly */
    PowerOutput (pwrOut, pwrState);
    
    g_flgIRQ = true;	// keep on running
}


/***************************************************************************//**
 *
 * @brief	DateTime Synchronization
 *
 * This function is called from VerifyConfiguration.
 * It sets the system clock via ClockSet() with the new time.
 * If this happens, all configured alarm times will be adjusted accordingly.
 *
 * @note
 * Be aware, this function take the TIMESTAMP Varibles from SD Card. 
 *
 * @param[in] pTime
 *	Pointer to a <b>tm</b> structure that holds the current time.
 *
 ******************************************************************************/
static void DateTimeSynchronize (struct tm *pDateTime)
{
   Log("");
   
   Log ("DATE and TIME read from config.txt 20%02d%02d%02d-%02d%02d%02d",
	 g_DateStampYEAR, g_DateStampMONTH, g_DateStampDAY, g_TimeStampHOUR,  
         g_TimeStampMINUTE, g_TimeStampSECOND);

     /* set system clock to date/ time */
    g_CurrDateTime = *pDateTime;

    /* Set System Clock also in UNIX time and check initially alarm times */
    ClockSet (&g_CurrDateTime, true);	// set milliseconds to zero
    
   /* Show time on display (if applicable) */
    ClockUpdate (false);	// g_CurrDateTime is already up to date
}