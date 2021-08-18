/***************************************************************************//**
 * @file
 * @brief	Sensor Monitoring
 * @author	Peter Loes
 * @version	2021-05-03
 * This module can be used to read status information from the sensor sht31-d
 * (sensirion) via its SMBus interface. It also provides function 
 * ReadVdd() to read the voltage of the local supply battery.
 *
 * Parts of the code are based on the example code of AN0021
 * "Analog to Digital Converter" from Energy Micro AS / Silicon Labs.
 *
 ***************************************************************************//**
 *
 * Parts are Copyright 2013 Energy Micro AS, http://www.energymicro.com
 *
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ****************************************************************************//*
Revision History:
2021-06-03,rage	Implemented alarms to read battery status 2 times per day.
		BugFix: <SMB_Status> must be declared volatile.
		Removed unused variable <g_BattRunDays>.
		Do not install interval timer if BAT_MON_INTERVAL is 0.
2020-06-23,rage	SensorMon_Init: Added SENSOR_PWR_OUT structure to specify the
		power outputs for the two SENSOR readers.
2020-01-13,rage	Implemented probing for a connected sensor controller type.
		Make information available via variables g_SensorCtrlAddr,
		g_SensorCtrlType, and g_SensorCtrlName.
		Reading the voltage of the CR2032 battery is now possible before
		calling BatteryMonInit().  This is used to calculate the LCD
		contrast value depending on the CR2032 voltage.
2018-03-25,rage	Set interrupt priority for SMB_IRQn.
		Added BatteryInfoReq() and BatteryInfoGet().
		BatteryCheck() also handles read requests from BatteryInfoReq().
		LogBatteryInfo() calls drvLEUART_sync() to prevent overflows.
2015-10-13,rage	BugFix: <SMB_Status> must be declared "volatile".
2015-06-24,rage	Added support for local battery voltage measurement (CR3032).
2015-06-22,rage	Initial version, derived from RFID_MS alias SNB_Heaven.
*/

/*=============================== Header Files ===============================*/

#include <string.h>
#include "em_cmu.h"
#include "em_i2c.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_adc.h"
#include "AlarmClock.h"		// msDelay()
#include "LEUART.h"
#include "Logging.h"
#include "microsd.h"
#include "SensorMon.h"

/*=============================== Definitions ================================*/

    /*!@name Hardware Configuration: SMBus controller and pins. */
//@{
#define SMB_GPIOPORT2		gpioPortD	//!< Port SMBus interface
#define SMB_SDA_PIN2		6		//!< Pin PA0 for SDA signal
#define SMB_SCL_PIN2		7		//!< Pin PA1 for SCL signal

#define SMB_GPIOPORT1		gpioPortA	//!< Port SMBus interface
#define SMB_SDA_PIN1		0		//!< Pin PA0 for SDA signal
#define SMB_SCL_PIN1		1		//!< Pin PA1 for SCL signal
#define SMB_I2C_CTRL		I2C0		//!< I2C controller to use
#define SMB_I2C_CMUCLOCK	cmuClock_I2C0	//!< Enable clock for I2C
#define SMB_LOC1		I2C_ROUTE_LOCATION_LOC0 //!< Use location 0
#define SMB_LOC2		I2C_ROUTE_LOCATION_LOC1 //!< Use location 1
#define SMB_IRQn		I2C0_IRQn	//!< I2C controller interrupt
#define SMB_IRQHandler		I2C0_IRQHandler	//!< SMBus interrupt handler
    /* PD1 is Hold_Power and ADC0_CH1 */
#define MEASURE_VD_PORT		gpioPortD	//!< Port to enable voltage divider
#define MEASURE_VD_PIN		1		//!< Pin for voltage divider
#define MEASURE_ADC_PORT	gpioPortD	//!< Port to enable voltage divider
#define MEASURE_ADC_PIN		1		//!< Pin for voltage divider
//@}

    /*!@brief I2C Transfer Timeout (500ms) in RTC ticks */
#define I2C_XFER_TIMEOUT	(RTC_COUNTS_PER_SEC / 2)

    /*!@brief I2C Recovery Timeout (5s) in RTC ticks */
#define I2C_RECOVERY_TIMEOUT	(RTC_COUNTS_PER_SEC * 5)

    /*!@brief Structure to hold Information about a Sensor Controller */
typedef struct
{
    uint8_t	 addr;		//!< SMBus address of the sensor controller
    BC_TYPE	 type;		//!< Corresponding controller type
    const char	*name;		//!< ASCII name of the controller
} BC_INFO;

    /*!@brief Format identifiers.
     *
     * These enumerations specify various data formats.  They are used as an
     * element in structure @ref ITEM to specify the data representation of
     * an item.  They are handled by a switch() statement in ItemDataString().
     */
typedef enum
{
    FRMT_FW_VERSION,	//!<   1: Firmware Version
    FRMT_SENS_CTRL,	//!<   2: Battery controller SMBus address and type  FRMT_STRING
    FRMT_VDD,   	//!<   3: Supply Voltage of controller
    FRMT_STRING,	//!<   4: 0-terminated string
    FRMT_HEX,		//!<   5: H1,2,3,4 Hexadecimal data representation
    FRMT_HEXDUMP,	//!<   6: Show Hexdump for more than 4 bytes
    FRMT_INTEGER,	//!<   7: Integer value
    FRMT_SERNUM,	//!<   8: Serial Number
    FRMT_PERCENT,	//!<   9: U1 Amount in percent [%]
    FRMT_DURATION,	//!<  10: U2 Duration in [min]
    FRMT_OC_REATIME,	//!<  11: Overcurrent Reaction Time in 1/2[ms] units
    FRMT_HC_REATIME,	//!<  12: Highcurrent Reaction Time in 2[ms] units
    FRMT_VOLT,		//!<  13: U2 Voltage in [V]
    FRMT_MILLIVOLT,	//!<  14: U2 Voltage in [mV]
    FRMT_MILLIAMP,	//!<  15: I2 Current in [±mA], +:charging, -:discharging
    FRMT_MILLIAMPH,	//!<  16: U2 Capacity in [mAh]
    FRMT_MICROOHM,	//!<  17: Resistance in [uOhm]
    FRMT_DATE,		//!<  18: Date [15:9=Year|8:5=Month|4:0=Day]
    FRMT_TEMP,		//!<  19: Temperature [°C]
    FRMT_RH,		//!<  20: Relative Humidity [%RH]
} FRMT_TYPE;

/*================================== Macros ==================================*/

#ifndef LOGGING		// define as UART output, if logging is not enabled
    #define LogError(str)	drvLEUART_puts(str "\n")
#endif

/*========================= Global Data and Routines =========================*/

    /*!@brief I2C Device Address of the Sensor Controller */
uint8_t g_SensorCtrlAddr;

    /*!@brief Sensor Controller Type */
BC_TYPE g_SensorCtrlType = BCT_UNKNOWN;

    /*!@brief ASCII Name of the Sensor Controller, or "" if no one found */
const char *g_SensorCtrlName;

    /*!@brief SENSOR type from config.txt file.
     * SENSOR_TYPE [VCNL, HT, CO2, C]
     * - VCNL is  Fully Integrated Proximity and
     *   Ambient Light Sensor with Infrared Emitter,
     * - HT is Humidity and Temperature Sensor, 
     * - CO2 is C02, humidity and temperature sensor 
     * - C is Event Recorder tells the User that events occurred (Counter)
     */
SENSOR1_TYPE g_SENSOR1_Type;

/*!@brief SENSOR power output. */
PWR_OUT   g_SENSOR_Power =  PWR_OUT_NONE;


    /*!@brief SENSOR type from config.txt file.
     * SENSOR_TYPE [VCNL, HT, CO2, C]
     * - VCNL is  Fully Integrated Proximity and
     *   Ambient Light Sensor with Infrared Emitter,
     * - HT is Humidity and Temperature Sensor, 
     * - CO2 is C02, humidity and temperature sensor 
     * - C is Event Recorder tells the User that events occurred (Counter)
     */
SENSOR2_TYPE g_SENSOR2_Type;
 
    /*!@brief Enum names for the SENSOR1 type. */
const char *g_enum_SENSOR1_Type[] = { "VCNL", "HT", "CO2", "C", NULL };

    /*!@brief Enum names for the SENSOR2 type. */
const char *g_enum_SENSOR2_Type[] = { "VCNL", "HT", "CO2", "C", NULL };


/*================================ Local Data ================================*/

    /*! Local structure to SENSOR type and power output configuration */
static SENSOR_CONFIG l_pSENSOR_Cfg;

    /*!@brief Probe List of supported Sensor Controllers */
static const BC_INFO l_ProbeList[] =
{  //  addr	type		name (maximum 10 characters!)
    {  0x88,	BCT_SHT3X_DFLT,	"SHT3X-DFLT"	},  // 0x44 Address 7 MSBs 0x88
    {  0x8A,	BCT_SHT3X_ALT,	"SHT3X-ALT"	},  // 0x45 Address 7 MSBs 0x8A
    {  0xC0,    BCT_VCNL,       "VCNL4040"      },  // 0x60 Address 7 MSBs 0xC0
    {  0xC2,    BCT_CO2,        "C02"           },  // 0x61 Address 7 MSBs 0xC2
    {  0x00,	BCT_UNKNOWN,	""		}   // End of the list};
};
    
    /* Defining the SMBus initialization data */
static I2C_Init_TypeDef smbInit =
{
  .enable   = true,		// Enable controller after initialization
  .master   = true,		// Configure for MASTER mode
  .refFreq  = 0,		// Get clock frequency from clock source
  .freq     = 10000,		// Low frequency because of long SMBus wires
  .clhr     = i2cClockHLRStandard,	// Set to use 4:4 low/high duty cycle
};

    /* Status of the last SMBus transaction */
static volatile I2C_TransferReturn_TypeDef SMB_Status;

    /*!@brief Flag to trigger Sensor Controller Probing. */
static volatile bool	 l_flgSensorCtrlProbe = true;

    /*!@brief Flag to trigger battery monitoring measurement. */
static volatile bool	 l_flgBatMonTrigger;

#if BAT_MON_INTERVAL > 0
    /* Timer handle for the battery monitoring interval */
static TIM_HDL	l_thBatMon = NONE;
#endif

    /* Battery Info structure - may hold up to two info requests */
static BAT_INFO  l_BatInfo;

/*=========================== Forward Declarations ===========================*/

#if BAT_MON_INTERVAL > 0
static void	BatMonTrigger(TIM_HDL hdl);
#endif

static void	BatMonTriggerAlarm(int alarmNum);

static void	ADC_Config(void);


/***************************************************************************//**
 *
 * @brief	Initialize the Sensor1 monitoring module
 *
 * This routine initializes the board-specific SMBus (I2C) interface, which
 * is connected to the Sensor1 and all other parts.
 * But Sensor2MoniInit (void) initializes specific SMBus (I2C) interface.
 *
 ******************************************************************************/
void	 Sensor1MonInit (void)
{
  /*!@brief data collect is activate. */
bool   isDataCollectOn;	
  
    /* Be sure to enable clock to GPIO (should already be done) */
    CMU_ClockEnable (cmuClock_GPIO, true);

    /* Enable clock for I2C controller*/
    CMU_ClockEnable(SMB_I2C_CMUCLOCK, true);

#if 0	//RAGE: We currently use the internal Vdd/3 channel, see ReadVdd()
    /* Configure GPIO to enable voltage divider for local 3V measurements */
    GPIO_PinModeSet (MEASURE_VD_PORT, MEASURE_VD_PIN, gpioModePushPull, 0);

    /* Configure GPIO as analog input for local 3V measurements */
    GPIO_PinModeSet (MEASURE_ADC_PORT, MEASURE_ADC_PIN, gpioModePushPull, 0);
#endif

    /* Configure GPIOs for SMBus (I2C) functionality with Pull-Ups */
    GPIO_PinModeSet (SMB_GPIOPORT1, SMB_SCL_PIN1, gpioModeWiredAndPullUp, 1);
    GPIO_PinModeSet (SMB_GPIOPORT1, SMB_SDA_PIN1, gpioModeWiredAndPullUp, 1);
       
   /* Route SMB signals to the respective pins */
   SMB_I2C_CTRL->ROUTE = I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN | SMB_LOC1;
   

   /* Initialize SMBus (I2C) controller */
   I2C_Init (SMB_I2C_CTRL, &smbInit);
   
    /* Clear and enable SMBus interrupt */
    NVIC_SetPriority(SMB_IRQn, INT_PRIO_SMB);
    NVIC_ClearPendingIRQ (SMB_IRQn);
    NVIC_EnableIRQ (SMB_IRQn);
    
    /* Get a timer handle for the battery monitoring interval */
#if BAT_MON_INTERVAL > 0
    if (l_thBatMon == NONE)
    {
	l_thBatMon = sTimerCreate (BatMonTrigger);
	if (l_thBatMon != NONE)
	    sTimerStart (l_thBatMon, BAT_MON_INTERVAL);
    }
#endif

    /* Set up alarm times when to log the battery status */
    AlarmAction (ALARM_BATTERY_MON_1, BatMonTriggerAlarm);
    AlarmSet (ALARM_BATTERY_MON_1, ALARM_BAT_MON_TIME_1);
    AlarmEnable (ALARM_BATTERY_MON_1);

    AlarmAction (ALARM_BATTERY_MON_2, BatMonTriggerAlarm);
    AlarmSet (ALARM_BATTERY_MON_2, ALARM_BAT_MON_TIME_2);
    AlarmEnable (ALARM_BATTERY_MON_2);
  
    /* Initialize Battery Info structure */
    l_BatInfo.Req_1 = l_BatInfo.Req_2 = SBS_NONE;

 
    /* Get current state of DataCollectOn (PB2 is ON) */
    isDataCollectOn = IsDataCollectOn();

    /* Data Collection is off */ 
    if(!isDataCollectOn)
    { 
       /* Build new structure based on the configuration variables */
       l_pSENSOR_Cfg.SENSOR1_Type   = g_SENSOR1_Type;
 
       if (g_SENSOR1_Type == SENSOR1_TYPE_NONE)
       {
          /* There is no Sensor1 Type */
          Log ("ERROR: SENSOR1 Type is NONE");
          Log ("Please use first VDD_SENSOR1");
          return;     
       }
       else
       {
#ifdef LOGGING
       Log ("SENSOR1 of Type %s for VDD_%s read from config.txt",
           g_enum_SENSOR1_Type[g_SENSOR1_Type], g_enum_PowerOutput[PWR_OUT_SENSOR1]);
 #endif
        } 
      
        /* Build new structure based on the configuration variables */
        l_pSENSOR_Cfg.SENSOR2_Type   = g_SENSOR2_Type;

        if (g_SENSOR2_Type == SENSOR2_TYPE_NONE)
        {
           /* There is no Sensor2 Type */
           Log ("SENSOR2 Type is NONE");
        }
        else
        {
#ifdef LOGGING
       Log ("SENSOR2 of Type %s for VDD_%s read from config.txt",
           g_enum_SENSOR2_Type[g_SENSOR2_Type], g_enum_PowerOutput[PWR_OUT_SENSOR2]);
#endif
        }   
     
        LogSensorInfo (BAT_LOG_INFO_SHORT1);
  
        Log ("COLLECT DATA with Button 2 ");
    }
  
}

/***************************************************************************//**
 *
 * @brief	Initialize the Sensor2 monitoring module
 *
 * This routine initializes the board-specific SMBus (I2C) interface, which
 * is connected to the Sensor2.
 *
 ******************************************************************************/
void	 Sensor2MonInit (void)
{
bool   isDataCollectOn;	 ///data collect is activate
  
    /* Be sure to enable clock to GPIO (should already be done) */
    CMU_ClockEnable (cmuClock_GPIO, true);

    /* Enable clock for I2C controller*/
    CMU_ClockEnable(SMB_I2C_CMUCLOCK, true);
    
      /* Configure GPIOs for SMBus (I2C) functionality with Pull-Ups */
    GPIO_PinModeSet (SMB_GPIOPORT2, SMB_SCL_PIN2, gpioModeWiredAndPullUp, 1);
    GPIO_PinModeSet (SMB_GPIOPORT2, SMB_SDA_PIN2, gpioModeWiredAndPullUp, 1);
       
   /* Route SMB signals to the respective pins */
   SMB_I2C_CTRL->ROUTE = I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN | SMB_LOC2;
   

   /* Initialize SMBus (I2C) controller */
   I2C_Init (SMB_I2C_CTRL, &smbInit);
   
    /* Clear and enable SMBus interrupt */
    NVIC_SetPriority(SMB_IRQn, INT_PRIO_SMB);
    NVIC_ClearPendingIRQ (SMB_IRQn);
    NVIC_EnableIRQ (SMB_IRQn);
    
    
     /* Get current state COLLECTING DATA is ON or off */
    isDataCollectOn = IsDataCollectOn();
    
    if (!isDataCollectOn)
    { 
       LogSensorInfo (BAT_LOG_INFO_SHORT2);
    }
}


    
/***************************************************************************//**
 *
 * @brief	De-Initialize the sensor monitoring module
 *
 * This routine brings the SMBus (I2C) interface, which is connected to the
 * sensor, into a quiescent state.
 *
 ******************************************************************************/
void	 SensorMonDeinit (void)
{
    /* Set Power Enable Pin for the SENSOR receiver to OFF */
    //PowerOutput (l_pSENSOR_Cfg.SENSOR_PwrOut, PWR_OFF);    
    
    /* Disable interrupt in ADC and NVIC */
    ADC0->IEN = 0;
    NVIC_DisableIRQ(ADC0_IRQn);
    
    /* Make sure conversion is not in progress */
    ADC0->CMD = ADC_CMD_SINGLESTOP | ADC_CMD_SCANSTOP;

    /* Reset ADC */
    ADC_Reset (ADC0);

    /* Disable clock for ADC */
    CMU_ClockEnable(cmuClock_ADC0, false);
        
    /* ADC is no longer active, clear bit in bit mask */
    Bit(g_EM1_ModuleMask, EM1_MOD_ADC) = 0;
    
    /* Disable SMBus interrupt */
    NVIC_DisableIRQ (SMB_IRQn);

    /* Reset SMBus controller */
    I2C_Reset (SMB_I2C_CTRL);

    /* Disable clock for I2C controller */
    CMU_ClockEnable(SMB_I2C_CMUCLOCK, false);
    
    /* Reset variables */
    g_SensorCtrlAddr = 0x00;
    g_SensorCtrlName = "";
    g_SensorCtrlType = BCT_UNKNOWN;
}


/***************************************************************************//**
 *
 * @brief	Probe for Controller Type
 *
 * This routine probes the type of sensor controller.  This is done by checking
 * dedicated I2C-bus addresses on the SMBus.  The following addresses and their
 * corresponding controller type are supported:
 * - 0x88 MSBs (0x44) in case of SHT3X-ADDR-DFLT and
 * - 0x8A MSBs (0x45) for the SHT3X-ADDR-ALT.
 * The address is stored in @ref g_SensorCtrlAddr, its ASCII name in @ref
 * g_SensorCtrlName and the controller type is stored as bit definition
 * @ref BC_TYPE in @ref g_SensorCtrlType.
 *
 ******************************************************************************/
static void SensorCtrlProbe (void)
{
int	i;
int	status;

    for (i = 0;  l_ProbeList[i].addr != 0x00;  i++)
    {
	g_SensorCtrlAddr = l_ProbeList[i].addr;	// try this address
   	status = SensorRegReadValue(SBS_READ_SERIAL_NUMBER, NULL);
        if (status >= 0)
	{
	    /* Response from controller - sensor found */
	    break;
	}
	else
	{
	    if (status != i2cTransferNack)
		Log ("SensorCtrlProbe: Unexpected error %d\n",
				status);
	}
    }

    g_SensorCtrlAddr = l_ProbeList[i].addr;
    g_SensorCtrlName = l_ProbeList[i].name;
    g_SensorCtrlType = l_ProbeList[i].type;

#if 0
    /*
     * RAGE WORKAROUND: There may be some Sensor Packs with new 
     * controller out in the field, that use I2C-bus address 0x44.  These
     * would be detected as "SHT3X-ADDR-DFLT" devices, which is wrong.
     * Therefore this workaround probes for register SBS_READ_STATUS
     * (0xF32D) which (only) exists in the SHT3X-D controller.
     */
    status = SensorRegReadValue(SBS_READ_STATUS, NULL);
    if (status >= 0)
    {
	/* Register exists - must be Sensor controller */
	g_SensorCtrlName = l_ProbeList[1].name;
	g_SensorCtrlType = l_ProbeList[1].type;
    }
#endif
}


/***************************************************************************//**
 *
 * @brief	SMBus Interrupt Handler
 *
 * This handler is executed for each byte transferred via the SMBus interface.
 * It calls the driver function I2C_Transfer() to prepare the next data byte,
 * or generate a STOP condition at the end of a transfer.
 *
 ******************************************************************************/
void	 SMB_IRQHandler (void)
{

    /* Update <SMB_Status> */
    SMB_Status = I2C_Transfer (SMB_I2C_CTRL);

}


/***************************************************************************//**
 *
 * @brief	SMBus Reset
 *
 * This internal routine aborts the current I2C-bus transfer and tries to
 * recover from a state where SCL is driven low by the battery controller.
 * It should be called if there occurs a timeout of a transfer.
 *
 ******************************************************************************/
static void  SMB_Reset (void)
{
    LogError("SMB_Reset: Try to recover from invalid state");

    /* abort the current transfer */
    SMB_I2C_CTRL->CMD = I2C_CMD_ABORT;
    msDelay(100);

    /* check if SCL is still low */
    if ((GPIO->P[SMB_GPIOPORT1].DIN & (1 << SMB_SCL_PIN1)) == 0)
    {
	/* drive SDA low */
	GPIO->P[SMB_GPIOPORT1].DOUTCLR = (1 << SMB_SDA_PIN1);
	SMB_I2C_CTRL->ROUTE = I2C_ROUTE_SCLPEN | SMB_LOC1;

	/* wait until SCL returns to high */
	uint32_t start = RTC->CNT;
	while ((GPIO->P[SMB_GPIOPORT1].DIN & (1 << SMB_SCL_PIN1)) == 0)
	{
	    /* check for timeout */
	    if (((RTC->CNT - start) & 0x00FFFFFF) > I2C_RECOVERY_TIMEOUT)
	    {
		LogError("SMB_Reset: Recovery failed, giving up");
		break;
	    }
	}

	/* re-configure GPIO as SDA signal */
	SMB_I2C_CTRL->ROUTE = I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN | SMB_LOC1;
    }
}


/***************************************************************************//**
 *
 * @brief	Read Word Register from the Sensor
 *
 * This routine reads two bytes from the register address specified by @p cmd,
 * assembles them to a signed 16bit value, and returns this.  If an error
 * occurred, a negative status code is returned instead.
 *
 * @param[in] cmd
 *	SBS command, i.e. the register address to be read.
 *
 * @return
 *	Requested 16bit signed data value, or a negative error code of type
 *	@ref I2C_TransferReturn_TypeDef.  Additionally to those codes, there is
 *	another error code defined, named @ref i2cTransferTimeout.
 *
 * @see
 *	SensorRegReadBlock()
 *
 ******************************************************************************/
int	 SensorRegReadWord (SBS_CMD cmd)
{
uint32_t value;
int	 status;

    status = SensorRegReadValue (cmd, &value);

    if (status < 0)
	return status;

    return (int16_t)value;
}


/***************************************************************************//**
 *
 * @brief	Read Register Value from the Sensor
 *
 * This routine reads two bytes from the register address specified by @p cmd,
 * assembles them to a 16bit value, and returns this.  If an error occurred,
 * a negative status code is returned instead.
 *
 * @param[in] cmd
 *	SBS command, i.e. the register address to be read.
 *
 * @param[in] pValue
 *	Address of 32bit variable where to store the value read from the
 *	register.  May be set to NULL, if value is omitted.
 *
 * @return
 *	Status code @ref i2cTransferDone (0), or a negative error code of type
 *	@ref I2C_TransferReturn_TypeDef.  Additionally to those codes, there is
 *	another error code defined, named @ref i2cTransferTimeout.
 *
 * @see
 *	SensorRegReadBlock()
 *
 ******************************************************************************/
int	 SensorRegReadValue (SBS_CMD cmd, uint32_t *pValue)
{
uint8_t  dataBuf[6];			// buffer for data read from register
uint32_t value = 0;
int	 status;
int	 i;


    /* Call block command to transfer data bytes into buffer */
    status = SensorRegReadBlock (cmd, dataBuf, sizeof(dataBuf));

    if (status == i2cTransferDone  &&  pValue != NULL)
    {
	/* build value from data buffer (always little endian) */
	for (i = SBS_CMD_SIZE(cmd) - 1;  i >= 0;  i--)
	    value = (value << 8) | dataBuf[i];

	*pValue = value;
    }

    return status;
}


/***************************************************************************//**
 *
 * @brief	Read Data Block from the Sensor Controller
 *
 * This routine reads an amount of bytes from the sensor controller, as
 * specified by parameter cmd.  This contains the register address and number
 * of bytes to read.
 *
 * @param[in] cmd
 *	SBS command, i.e. the register address and number of bytes to read.
 *
 * @param[out] pBuf
 *	Address of a buffer where to store the data.
 *
 * @param[in] rdCnt
 *	Number of bytes to read.
 *
 * @return
 *	Status code @ref i2cTransferDone (0), or a negative error code of type
 *	@ref I2C_TransferReturn_TypeDef.  Additionally to those codes, there is
 *	another error code defined, named @ref i2cTransferTimeout.
 *
 * @see
 *	SensorRegReadValue()
 *
 ******************************************************************************/
int	SensorRegReadBlock (SBS_CMD cmd, uint8_t *pBuf, size_t rdCnt)
{
I2C_TransferSeq_TypeDef smbXfer;	// SMBus transfer data
uint8_t addrBuf[2];			// buffer for command address

    /* Check parameters */
    EFM_ASSERT (SBS_CMD_SIZE(cmd) != 0);// size field must not be 0
    EFM_ASSERT (pBuf != NULL);		// buffer address
    EFM_ASSERT (rdCnt >= SBS_CMD_SIZE(cmd));	// buffer size

   if (rdCnt < SBS_CMD_SIZE(cmd))	// if EFM_ASSERT() is empty
	return i2cInvalidParameter;

     /* Set up SMBus transfer S-Wr-Cmd-Sr-Rd-data1-P */
    smbXfer.addr  = g_SensorCtrlAddr; // I2C address of the Sensor
    smbXfer.flags = I2C_FLAG_WRITE_READ;    // write address, then read data
    smbXfer.buf[0].data = addrBuf;    // first buffer (data to write)
    addrBuf[0] = cmd >> 8;        // register address (high byte)
    addrBuf[1] = cmd & 0xFF;      // register address (low byte)
    smbXfer.buf[0].len  = 2;		 // 2 bytes for command
    smbXfer.buf[1].data = pBuf;		 // second buffer to store bytes read
    smbXfer.buf[1].len  = rdCnt;	 // number of bytes to read

    /* Start I2C Transfer */
    SMB_Status = I2C_TransferInit (SMB_I2C_CTRL, &smbXfer);

    /* Check early status */
    if (SMB_Status < 0)
	return SMB_Status;		// return error code

    /* Wait until data is complete or time out */
    uint32_t start = RTC->CNT;
    while (SMB_Status == i2cTransferInProgress)
    {
	/* Enter EM1 while waiting for I2C interrupt */
	EMU_EnterEM1();

	/* check for timeout */
	if (((RTC->CNT - start) & 0x00FFFFFF) > I2C_XFER_TIMEOUT)
	{
	    SMB_Reset();
	    SMB_Status = (I2C_TransferReturn_TypeDef)i2cTransferTimeout;
	}
    }

    /* Return final status */
    return SMB_Status;
}


/***************************************************************************//**
 *
 * @brief	Item Data String
 *
 * This routine returns a formatted data string of the specified item data.
 * It uses SensorRegReadValue() and SensorRegReadBlock() to read the data
 * directly from the battery controller.
 *
 * @param[in] cmd
 *	SBS command, i.e. the register address and number of bytes to read.
 *
 * @param[in] frmt
 *	Format specifier for data representation.
 *
 * @return
 * 	Static buffer that contains the formatted data string of the item,
 * 	or error message if there was an error, e.g. a read error from the
 * 	battery controller.
 *
 * @warning
 *	This routine is not MT-save (which should not be a problem for this
 *	application)!
 *
 ******************************************************************************/
static const char *ItemDataString (SBS_CMD cmd, FRMT_TYPE frmt)
{
static char	 strBuf[120];	// static buffer to return string into
uint8_t		 dataBuf[40];	// buffer for I2C data, read from the controller
uint32_t	 value;		// unsigned data variable
int		 data = 0;	// generic signed integer data variable
int		 data1 = 0;	// generic signed integer data variable
int		 d, h, m;	// FRMT_DURATION: days, hours, minutes


    /* Prepare check for string buffer overflow */
    strBuf[sizeof(strBuf)-1] = 0x11;

    if (cmd != SBS_NONE)
    {
	/* See how many bytes we need to read */
	data = SBS_CMD_SIZE(cmd);	// get object size
	if (data > 4)
	{
	    /* More than 32 bits - must be a block, e.g. a string */
	    EFM_ASSERT(data < (int)sizeof(dataBuf));

	    if (SensorRegReadBlock (cmd, dataBuf, data) < 0)
		return NULL;	// READ ERROR
	}
	else
	{
	    /* Read data word - may be 1, 2, 3, or 4 bytes long */
	    if (SensorRegReadValue (cmd, &value) < 0)
		return NULL;	// READ ERROR

	    data = (int)value;
	}
    }

    /* Variable <data> contains 16bit raw value, build formatted string */
    switch (frmt)
    {
	case FRMT_FW_VERSION:	// Firmware Version
	   // sprintf (strBuf, "V%s %s", prjVersion, prjDate);
	    break;

	case FRMT_SENS_CTRL:	// Battery controller SMBus address and type
	    if (g_SensorCtrlAddr == 0)
		strcpy (strBuf, "N O T  F O U N D");
	    else
		sprintf (strBuf, "0x%02X: %s", g_SensorCtrlAddr,
			 g_SensorCtrlName);
	    break;

	case FRMT_VDD:	// Supply Voltage of controller
	    data = ReadVdd();
	    sprintf (strBuf, " %d.%03dV", data / 1000, data % 1000);
	    break;

	case FRMT_STRING:	// return string to be displayed
	    /*
	     * The first byte contains the number of ASCII characters WITHOUT
	     * a trailing 0 as EndOfString marker.  However, in some cases the
	     * specified byte count is larger than the string - then an EOS
	     * marker exists in the data read from the controller.
	     */
	    data = dataBuf[0];
	    EFM_ASSERT(data < (int)(sizeof(strBuf)-1));
	    strncpy (strBuf, (char *)dataBuf+1, data);
	    strBuf[data] = EOS;		// terminate string
	    break;

	case FRMT_HEXDUMP:	// prepare data as hexdump
	    data = dataBuf[0];
	    for (d = 0;  d < data;  d++)	// data = number of bytes
		sprintf (strBuf + 3*d, "%02X ", dataBuf[d+1]);
	    strBuf[3*d - 1] = EOS;
	    break;

	case FRMT_HEX:		// HEX Digits (8, 16, 24, or 32bit)
	    switch (SBS_CMD_SIZE(cmd))
	    {
		case 1:
		    sprintf (strBuf, "0x%02X", data);
		    break;

		case 2:
		    sprintf (strBuf, "0x%04X", data);
		    break;

		case 3:
		    sprintf (strBuf, "0x%06X", data);
		    break;

		case 4:
		default:
		    sprintf (strBuf, "0x%08lX", value);
		    break;
	    }
	    break;

	case FRMT_INTEGER:	// Integer Value
	    sprintf (strBuf, "%d", data);
	    break;

	case FRMT_SERNUM:	// 5-Digit Integer Value
	    sprintf (strBuf, "%0d", data);
	    break;

	case FRMT_PERCENT:	// Amount in percent
	    sprintf (strBuf, "%d%%", data);
	    break;

	case FRMT_DURATION:	// Duration in [min]
	    if (data > 65534)		// > 45d
	    {
		strcpy (strBuf, "> 45 days");
	    }
	    else
	    {
		d = data / 60 / 24;
		data -= (d * 60 * 24);
		h = data / 60;
		data -= (h * 60);
		m = data;
		sprintf (strBuf, "%2dd %2dh %2dm", d, h, m);
	    }
	    break;

	case FRMT_OC_REATIME:	// Overcurrent Reaction Time in 1/2[ms] units
	    sprintf (strBuf, "%dms", data/2);
	    break;

	case FRMT_HC_REATIME:	// Highcurrent Reaction Time in 2[ms] units
	    sprintf (strBuf, "%dms", data*2);
	    break;

        case FRMT_VOLT:		// Voltage in [V]
	    sprintf (strBuf, "%d.%03dV", data / 1000, data % 1000);
	    break;
            
	case FRMT_MILLIVOLT:	// Voltage in [mV]
	    sprintf (strBuf, "%dmV", data);
	    break;

	case FRMT_MILLIAMP:	// Current in [Â±mA], +:charging, -:discharging
	    sprintf (strBuf, "%dmA", (int16_t)data);
	    break;

	case FRMT_MILLIAMPH:	// Capacity in [mAh]
	    sprintf (strBuf, "%dmAh", data);
	    break;

	case FRMT_MICROOHM:	// Resistance in [uOhm]
	    sprintf (strBuf, "%duOhm", data);
	    break;

	case FRMT_DATE:		// Date [15:9=Year|8:5=Month|4:0=Day]
	    sprintf (strBuf, "%04d-%02d-%02d", 1980 + (data >> 9),
		     (data >> 5) & 0xF, data & 0x1F);
	    break;

       case FRMT_TEMP:	  // Temperature convert to [Â°C]
           {
                /* The first and second byte contains the temperature.
                *  The third byte contains the checksum. */
               
               // combine the two bytes to a 16-bit value
               data = (dataBuf[0] << 8) | dataBuf[1];
               float degC =  (175 * (float)data / 65535.0f) - 45.0f;
               sprintf (strBuf, "%04.02f C", degC);
           }
           break;
           
        case FRMT_RH:	 // Relative Humidity, convert to [%RH]
            {
               /* The fourth and fifth byte contains the relative humidity.
                * The sixth byte contains the checksum. */
        
               // combine the two bytes to a 16-bit value
               data1 = (dataBuf[3] << 8) | dataBuf[4];
               float rh = ((float)data1 / 65535.0f) * 100.0f;
               sprintf (strBuf, "%04.02f RH", rh);
            }
            break;

	default:		// unsupported format
	    return NULL;

    }	// switch (pItem->Frmt)

    /* Perform check for string buffer overflow */
    if (strBuf[sizeof(strBuf)-1] != 0x11)
    {
    sprintf (strBuf, "ERROR strBuf Overflow, cmd=%d frmt=%d", cmd, frmt);
    }

    return strBuf;
}


/***************************************************************************//**
 *
 * @brief	Log Sensors and Battery Information
 *
 * Depending on the @ref BAT_LOG_INFO_LVL, this routine reads miscellaneous
 * information from the sensor1 and sensor2 via the SMBus and 
 * microcontroller voltage:
 *
 * - Voltage of Microcontroller
 * - Sensor1 Type
 * - Sensor1 Address
 * - Sensor1 Serial Number
 * - Sensor1 Set high repeatability mps - measurement per second 
 * - Sensor1 Measure temperature and humidity
 *
 * - Sensor2 Type
 * - Sensor2 Address
 * - Sensor2 Serial Number
 * - Sensor2 Measure temperature and humidity
 *
 * @param[in] infoLvl
 *	Enum of type @ref BAT_LOG_INFO_LVL which specifies the level of
 *	information.
 *
 ******************************************************************************/
void	LogSensorInfo (BAT_LOG_INFO_LVL infoLvl)
{
#ifdef LOGGING
    
    if (infoLvl == BAT_LOG_ONLY)
    {
       /* Read microcontroller voltage */
       Log ("VOLTAGE Minimum 2.8V <= %s",
       ItemDataString(SBS_NONE, FRMT_VDD));
    }
  
    if (infoLvl == BAT_LOG_INFO_SHORT1)// Read by Bat_Alarm
    {
        /* Check if the Sensor Probe routine should be called (again) */
        if (l_flgSensorCtrlProbe)
        { 
	    l_flgSensorCtrlProbe = false;
            SensorCtrlProbe(); 
                   
            Log ("SENSOR1 Type is \"%s\" at address 0x%02X",
	    g_SensorCtrlName, (g_SensorCtrlAddr >> 1));
        }
        
        switch (l_pSENSOR_Cfg.SENSOR1_Type)
        {
            case SENSOR1_TYPE_VCNL:
               Log ("SENSOR1 Serial Number     : %s",	// display as Hex value now
	       ItemDataString(SBS_READ_SERIAL_NUMBER_VCNL, FRMT_HEX));
                  
               /* Switch off SENSOR POWER if Alarm is off */
               PowerOutput (PWR_OUT_SENSOR1, PWR_OFF);
                    
               /* Enable Probe SENSOR2 */
               l_flgSensorCtrlProbe = true;                     
               break;
                    
            case SENSOR1_TYPE_HT:
               Log ("SENSOR1 Serial Number     : %s",	// display as Hex value now
	       ItemDataString(SBS_READ_SERIAL_NUMBER, FRMT_HEX));
                  
               /* Switch off SENSOR POWER if Alarm is off */
               PowerOutput (PWR_OUT_SENSOR1, PWR_OFF);
                    
               /* Enable Probe SENSOR2 */
               l_flgSensorCtrlProbe = true;                     
               break;
               
            case SENSOR1_TYPE_CO2:
               Log ("SENSOR1 CO2 not yet programmed!");
               break;
                    
            case SENSOR1_TYPE_C:
               Log ("C Counter(not I2C) not yet programmed!(SENSOR1)");
               break;
            
             case SENSOR1_TYPE_NONE:
                Log ("SENSOR1 TYPE NONE: Please use first VDD_SENSOR1!");
                return;
                            
              default:		// unknown SENSOR1 Type
                  // restart state machine
	         break;
         }
            
           /* De-Initialize the Sensor1 monitoring module */
           SensorMonDeinit ();
        
          /* Initialize the Sensor2 monitoring module */
          Sensor2MonInit();
    }
       
    if (infoLvl == BAT_LOG_INFO_SHORT2)// Read by Bat_Alarm??
    {
    
        /* Check if the Sensor Probe routine should be called (again) */
        if (l_flgSensorCtrlProbe)
        { 
	     l_flgSensorCtrlProbe = false;
                      
             SensorCtrlProbe(); 
                   
             Log ("SENSOR2 Type is \"%s\" at address 0x%02X",
	     g_SensorCtrlName, (g_SensorCtrlAddr >> 1));
        }
             
        switch (l_pSENSOR_Cfg.SENSOR2_Type)
        { 
            case SENSOR2_TYPE_VCNL:
               Log ("SENSOR2 VCNL not yet programmed!");
               break;
                    
            case SENSOR2_TYPE_HT:
               Log ("SENSOR2 Serial Number     : %s",	// display as Hex value now
	       ItemDataString(SBS_READ_SERIAL_NUMBER, FRMT_HEX));
               
               /* Switch off SENSOR POWER if Alarm is off */
               PowerOutput (PWR_OUT_SENSOR2, PWR_OFF);
               break;
               
            case SENSOR2_TYPE_CO2:
               Log ("SENSOR2 Serial Number     : %s",	// display as Hex value now
	       ItemDataString(SBS_FIRMWARE_VERSION_CO2, FRMT_HEX));
               
               /* Switch off SENSOR POWER if Alarm is off */
               PowerOutput (PWR_OUT_SENSOR2, PWR_OFF);
               break;
                    
            case SENSOR2_TYPE_C:
               Log ("C Counter(not I2C) not yet programmed!(SENSOR2)");
               break;
            
            case SENSOR2_TYPE_NONE:
               Log ("SENSOR2 TYPE NONE: Please use first VDD_SENSOR1!");
               break;
                            
            default:		// unknown SENSOR1 Type
                // restart state machine
	       break;
        }
        /* De-Initialize the Sensor1 monitoring module */
        SensorMonDeinit ();
    }
    
    drvLEUART_sync();	// to prevent UART buffer overflow
 
    /* No get only measurement information */
    if (infoLvl == BAT_LOG_INFO_VERBOSE1)
    {
        /* Initialize the Sensor1 monitoring module */
        Sensor1MonInit();
        
       SensorCtrlProbe();
      
        switch (l_pSENSOR_Cfg.SENSOR1_Type)
        {
           case SENSOR1_TYPE_VCNL:
              Log ("SENSOR1 VCNL not yet programmed!");
              break;
                    
           case SENSOR1_TYPE_HT:
              msDelay(50); 
              /* Set Measurement to 4 per second */
              ItemDataString(SBS_REPEATABILITY_CLOCK_MPS_4_HIGH, FRMT_HEX);
              msDelay(50); 
              Log ("SENSOR1 Temperature     : %s",	// display as Hex value now
              ItemDataString(SBS_FETCH_DATA, FRMT_TEMP));
              msDelay(500);
              Log ("SENSOR1 Relative Humidity : %s",  // display as Hex value now
              ItemDataString(SBS_FETCH_DATA, FRMT_RH));
              break;
               
            case SENSOR1_TYPE_CO2:
                Log ("SENSOR1 CO2 not yet programmed!");
                break;
                    
            case SENSOR1_TYPE_C:
               Log ("Counter(not I2C) not yet programmed!(SENSOR1)");
            break;
            
            case SENSOR1_TYPE_NONE:
              Log ("SENSOR1 TYPE NONE: Please use first VDD_SENSOR1!");
              return;
                            
         default:		// unknown RFID reader type
             // restart state machine
	     break;
            }
           
           /* De-Initialize the Sensor1 monitoring module */
           SensorMonDeinit ();
    }
  
    /* No get only measurement information */
    if (infoLvl == BAT_LOG_INFO_VERBOSE2)
    {
        
        /* Initialize Sensor2 Monitor according to (new) configuration */
        Sensor2MonInit();
        
        SensorCtrlProbe();
        
        switch (l_pSENSOR_Cfg.SENSOR2_Type)
        {
          
           case SENSOR2_TYPE_VCNL:
              Log ("SENSOR2 VCNL not yet programmed!");
              break;
                    
           case SENSOR2_TYPE_HT:
              msDelay(50); 
              /* Set Measurement to 4 per second */
              ItemDataString(SBS_REPEATABILITY_CLOCK_MPS_4_HIGH, FRMT_HEX);
              msDelay(50); 
              Log ("SENSOR2 Temperature     : %s",	// display as Hex value now
              ItemDataString(SBS_FETCH_DATA, FRMT_TEMP));
              msDelay(500);
              Log ("SENSOR2 Relative Humidity : %s",  // display as Hex value now
              ItemDataString(SBS_FETCH_DATA, FRMT_RH));
              break;
              
           case SENSOR2_TYPE_CO2:
              Log ("SENSOR2 CO2 not yet programmed!");
              break;
                    
           case SENSOR2_TYPE_C:
              Log ("Counter(not I2C) not yet programmed!(SENSOR2)");
              break;
            
           case SENSOR2_TYPE_NONE:
              //Log ("SENSOR2 TYPE NONE:  Please use first VDD_SENSOR1!");
               break;
                            
           default:		// unknown Sensor type
              // restart state machine
	      break;
          }
           
           /* De-Initialize the Sensor1 monitoring module */
           SensorMonDeinit (); 
     }    
    drvLEUART_sync();	// to prevent UART buffer overflow
    
#endif
}


/***************************************************************************//**
 *
 * @brief	Battery Check
 *
 * This routine is called periodically to check the status of the battery.
 * It reads only the microcontroller voltage.<br>
 * It also handles the requests for BatteryInfoReq(), resp. BatteryInfoGet().
 *
 ******************************************************************************/
void	BatteryCheck (void)
{
static bool isDiskRemoved;

    /* Refresh flag isDiskRemoved */  
     isDiskRemoved = IsDiskRemoved(); 

    if(!isDiskRemoved)
    {  
    /* Check if the Sensor Controller Probe routine should be called (again) */
    if (l_flgSensorCtrlProbe)
    {
	l_flgSensorCtrlProbe = false;
        
        /* Set Power Enable Pin for the SENSOR receiver to OFF */
        SensorCtrlProbe();    
    }
    
    /* Check for Battery Information Request */
    if (l_BatInfo.Req_1 != SBS_NONE)
    {
	if (SBS_CMD_SIZE(l_BatInfo.Req_1) > 4)
	    SensorRegReadBlock (l_BatInfo.Req_1, l_BatInfo.Buffer, 18);
	else
	   l_BatInfo.Data_1 = SensorRegReadWord (l_BatInfo.Req_1);
	   l_BatInfo.Req_1  = SBS_NONE;
    }

    if (l_BatInfo.Req_2 != SBS_NONE)
    {
	msDelay(100);		// to prevent hang-up of battery controller

	if (SBS_CMD_SIZE(l_BatInfo.Req_2) > 4)
	    SensorRegReadBlock (l_BatInfo.Req_2, l_BatInfo.Buffer, 18);
	else
	   l_BatInfo.Data_2 = SensorRegReadWord (l_BatInfo.Req_2);
	   l_BatInfo.Req_2  = SBS_NONE;
    }
   
    l_BatInfo.Done = true;
    }  
    
    /* see if to log battery status */
    if (l_flgBatMonTrigger)
    {
	l_flgBatMonTrigger = false;

	/* Log verbose information, if Battery Pack has changed */
	LogSensorInfo (BAT_LOG_ONLY);
    }
}


#if BAT_MON_INTERVAL > 0
/***************************************************************************//**
 *
 * @brief	Battery Monitoring Trigger
 *
 * This routine is called when the battery monitoring interval is over.  This
 * interval can be adjusted by @ref BAT_MON_INTERVAL.
 *
 ******************************************************************************/
static void	BatMonTrigger(TIM_HDL hdl)
{
    (void) hdl;		// suppress compiler warning "unused parameter"

    /* Restart the timer */
    if (l_thBatMon != NONE)
	sTimerStart (l_thBatMon, BAT_MON_INTERVAL);

    /* Set trigger flag */
    l_flgBatMonTrigger = true;

    g_flgIRQ = true;	// keep on running
}
#endif


/***************************************************************************//**
 *
 * @brief	Battery Monitoring Trigger Alarm
 *
 * This routine is called when a battery monitoring alarm time has been reached.
 * There are 2 alarm times: @ref ALARM_BAT_MON_TIME_1 and @ref
 * ALARM_BAT_MON_TIME_2.
 *
 ******************************************************************************/
static void	BatMonTriggerAlarm(int alarmNum)
{
    (void) alarmNum;	// suppress compiler warning "unused parameter"

    /* Set trigger flag */
    l_flgBatMonTrigger = true;

    g_flgIRQ = true;	// keep on running
}


/***************************************************************************//**
 *
 * @brief	Battery Information Request
 *
 * This routine can be called from interrupt context to introduce up to two
 * requests for battery information.  The requests will be handled by function
 * BatteryCheck().  Use BatteryInfoGet() to poll for finished requests and
 * get the data.  If only one request is used, set <i>req_2</i> to SBS_NONE.
 *
 ******************************************************************************/
void	 BatteryInfoReq (SBS_CMD req_1, SBS_CMD req_2)
{
    l_BatInfo.Req_1 = req_1;
    l_BatInfo.Req_2 = req_2;
    strcpy ((char *)l_BatInfo.Buffer, "ERROR");
    l_BatInfo.Done  = false;
}


/***************************************************************************//**
 *
 * @brief	Battery Information Get
 *
 * This routine can be called from interrupt context to get the results of
 * a battery information request.  The requests will be handled by function
 * BatteryCheck().  Use BatteryInfoReq() to introduce information requests.
 * The information request is finished, when both request elements are set
 * to SBS_NONE.
 *
 ******************************************************************************/
BAT_INFO *BatteryInfoGet (void)
{
    return &l_BatInfo;
}


/***************************************************************************//**
 *
 * @brief	ADC Configuration
 *
 * This routine configures the ADC to measure the internal VDD/3 voltage,
 * see AN0021 for more information.
 *
 ******************************************************************************/
static void ADC_Config(void)
{
ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

    /* ADC requires EM1, set bit in bit mask */
    Bit(g_EM1_ModuleMask, EM1_MOD_ADC) = 1;

    /* Enable clock for ADC */
    CMU_ClockEnable(cmuClock_ADC0, true);

    /* Init common settings for both single conversion and scan mode */
    init.timebase = ADC_TimebaseCalc(0);
    /* Might as well finish conversion as quickly as possibly since polling */
    /* for completion. */
    /* Set ADC clock to 7 MHz, use default HFPERCLK */
    init.prescale = ADC_PrescaleCalc(7000000, 0);

    /* WARMUPMODE must be set to Normal according to ref manual before */
    /* entering EM2. In this example, the warmup time is not a big problem */
    /* due to relatively infrequent polling. Leave at default NORMAL, */

    ADC_Init(ADC0, &init);

    /* Init for single conversion use, measure VDD/3 with 1.25 reference. */
    singleInit.reference  = adcRef1V25;
    singleInit.input      = adcSingleInpVDDDiv3;
    singleInit.resolution = adcRes12Bit;

    /* The datasheet specifies a minimum aquisition time when sampling vdd/3 */
    /* 32 cycles should be safe for all ADC clock frequencies */
    singleInit.acqTime = adcAcqTime32;

    ADC_InitSingle(ADC0, &singleInit);
    
    /* Enable interrupt for Scan Mode in ADC and NVIC */
    NVIC_SetPriority(ADC0_IRQn, INT_PRIO_ADC);
    ADC0->IEN = ADC_IEN_SCAN;
    NVIC_EnableIRQ(ADC0_IRQn);  
}


/***************************************************************************//**
 *
 * @brief	Read VDD
 *
 * This routine measures the internal VDD/3 channel via ADC0, to obtain the
 * voltage of the local CR3032 supply battery.  The value is converted to
 * milli volts [mV].
 *
 * @return
 *	VDD value in [mV].
 *
 ******************************************************************************/
uint32_t ReadVdd (void)
{
static bool initADC = true;
uint32_t    value;

    if (initADC)
    {
	/* Routine has been called for the first time - initialize ADC */
	ADC_Config();
	initADC = false;
    }

    ADC_Start(ADC0, adcStartSingle);

    /* Wait while conversion is active */
    while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;

    /* Get ADC result */
    value = ADC_DataSingleGet(ADC0);

    /* Calculate supply voltage relative to 1.25V reference */
    value = (value * 1250 * 3) / 4096;

    return value;
}
