/***************************************************************************//**
 * @file
 * @brief	SL Sensor_Logger
 * @author	Peter Loes
 * @version	2021-06-21
 *
 * This application consists of the following modules:
 * - main.c - Initialization code and main execution loop.
 * - DMA_ControlBlock.c - Control structures for the DMA channels.
 * - Control.c - Sequence Control module.
 * - CfgData.c - Handling of configuration data.
 * - ExtInt.c - External interrupt handler.
 * - Keys.c - Key interrupt handling and translation.
 * - AlarmClock.c - Alarm clock and timers facility.
 * - clock.c - An implementation of the POSIX time() function.
 * - LEUART.c - The Low-Energy UART can be used as monitoring and debugging
 *   connection to a host computer.
 * - microsd.c - Together with the files "diskio.c" and "ff.c", this module
 *   provides an implementation of a FAT file system on the @ref SD_Card.
 * - Logging.c - Logging facility to send messages to the LEUART and store
 *   them into a file on the SD-Card.
 * - Update.c - Hold PowerUp the Device. Manage KeyHandler.
 *   Update Data Logger functions.     
 * - SensorMon.c - Sensor monitor, allows to read the state of the
 *   sensor via the SMBus.
 *
 * Parts of the code are based on the example code of AN0006 "tickless calender"
 * and AN0019 "eeprom_emulation" from Energy Micro AS.
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
2021-06-11,rage	Call CheckAlarmTimes() after CONFIG.TXT has been read.
2021-06-11,rage	Moved DMA related variables to module "DMA_ControlBlock.c".
		Calling VerifyConfiguration() ensures data is valid.
2021-06-07,rage	Initial version.
*/

/*!
 * @mainpage
 * <b>Description</b><br>
 * Sensor Data Logger is an application to get actual values of different 
 * sensors. For that purpose the Data Logger must be connected to the Sensor
 * controller via its SMBus.
 *
 * The system consists of the following components:
 *
 * <b>Microcontroller</b><br>
 * The heart of the board is an EFM32G230 microcontroller.  It provides two
 * different clock domains: All low-energy peripheral is clocked via a
 * 32.768kHz external XTAL.  The MCU and other high performance peripheral
 * uses a high-frequency clock.  The board can be configured to use the
 * internal RC-oscillator, or an external 32MHz XTAL for that purpose,
 * see define @ref USE_EXT_32MHZ_CLOCK.
 *
 * <b>Keys (Push Buttons)</b><br>
 * There exist 2 push buttons on the board (description of the keys is left to
 * right).  When asserted, the following action will be taken:
 * - <b>POWER</b> button.  It is used to switch the Logger Device on or off (for
 *   switching it off keep the button asserted until the text message
 *   <b>POWER OFF</b> is showed and the red LED is off).  
 *   When pressed for a short time, it shows the firmware version and date, and
 *   sensors for the Data Logger again.  This is useful after a different
 *   sensor has been connected to the Data Logger.
 * - <b>START_STOP</b> key.  It is used to start/stop collecting data from
 *   the connected sensors keep the button asserted until the text message
 *   <b>Collecting data is off</b> is shown or yellow LED is off.
 *   This key has a auto-repeat functionality when kept asserted.
 *
 * <b>LEDs</b><br>
 * There are two LEDs, one is red, the other one is yellow.
 * - The red LED is the Power-On LED.  It shows the current power state of the
 *   Sensor Logger. In normal condition the red LED will be continued flashing.
 * - The yellow LED is the Log Flush LED.  It flashes whenever data from sensors
 *   are collected.
 *
 * <b>Battery Monitor</b><br>
 * Rechargeable power pack or the coincell battery CR2032 is powering
 * the connected EFM32 microcontroller. This module provides function ReadVdd()
 * to read the voltage of the microcontroller configure by PD1
 * to hold power on a regular basis.
 * i.e. every BAT_MON_INTERVAL.
 *
 * <b>Firmware</b><br>
 * The firmware consists of an initialization part and a main loop, also called
 * service execution loop.  The initialization part sets up all modules, enables
 * devices and interrupts.  The service execution loop handles all tasks that
 * must not be executed in interrupt context.
 *
 * After power-up or reset the following actions are performed:
 * -# Basic initialization of MCU and clocks.
 * -# The Hold-Power pin is activated.
 * -# Further hardware initialization. (Keys, Interrupts, Alarm Clock)
 * -# The Sensor Monitor is initialized.
 * -# If a Sensor is connected via I²C-bus, the controller type is probed
 *    and displayed.
 *
 * The program then enters the Service Execution Loop which takes care of:
 * - Manual or timed Power-Off
 * - Sensor controller probing requests via POWER button
 * - Sensor monitoring
 * - Entering the right energy mode
 *
 * Power Sensors can be switched on or off at selectable times.  Up to
 * five <b>on</b> and <b>off</b> times can be defined via configuration
 * variables in file <a href="../../CONFIG.TXT"><i>CONFIG.TXT</i></a>:
 * - @ref SENSOR1_ON_TIME_1
 * - @ref SENSOR2_ON_TIME_1
 *
 * If an additional Power Cycle Interval is specified for a power output,
 * it will be switched on and off during its activation times, according to
 * the values of the following configuration variables:
 * - @ref SENSOR1_INTERVAL
 * - @ref SENSOR2_INTERVAL
 *
 * @subsection SENSOR1_ON_TIME_1 SENSOR1_ON_TIME_1~5, SENSOR1_OFF_TIME_1~5
 * These variables determine the on and off times of the SENSOR1 Power Output.
 * If the @ref SENSOR1 connected it will also be enabled or disabled.
 *
 * @subsection SENSOR2_ON_TIME_1 SENSOR2_ON_TIME_1~5, SENSOR2_OFF_TIME_1~5
 * These variables determine the on and off times of the SENSOR2 Power Output.
 * If the @ref SENSOR2 connected it will also be enabled or disabled.
 *
 * @subsection SENSOR1_INTERVAL SENSOR1_INTERVAL and SENSOR1_ON_DURATION
 * These variables allow you to specify a Power Cycle Interval during the ON
 * times of Power Output SENSOR1.  SENSOR1_INTERVAL is the length of the interval in
 * [s], SENSOR1_ON_DURATION the ON duration in [s] during this intervals.
 *
 * @subsection SENSOR2_INTERVAL SENSOR2_INTERVAL and SENSOR2_ON_DURATION
 * These variables allow you to specify a Power Cycle Interval during the ON
 * times of Power Output SENSOR2.  SENSOR2_INTERVAL is the length of the interval in
 * [s], SENSOR2_ON_DURATION the ON duration in [s] during this intervals.
 */                     
   
/*=============================== Header Files ===============================*/

#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "config.h"		// include project configuration parameters
#include "ExtInt.h"
#include "Keys.h"
#include "AlarmClock.h"
#include "LEUART.h"
#include "SensorMon.h"
#include "Logging.h"
#include "CfgData.h"
#include "Control.h"
#include "Update.h"

#include "ff.h"		// FS_FAT12/16/32
#include "diskio.h"	// DSTATUS
#include "microsd.h"

/*================================ Global Data ===============================*/

extern PRJ_INFO const  prj;		// Project Information


/*! @brief Flag to indicate that an Interrupt occurred in the meantime.
 *
 * This flag must be set <b>true</b> by any interrupt service routine that
 * requires actions in the service execution loop of main().  This prevents
 * the system from entering sleep mode, so the action can be taken before.
 */
volatile bool		g_flgIRQ;

/*! @brief Modules that require EM1.
 *
 * This global variable is a bit mask for all modules that require EM1.
 * Standard peripherals would stop working in EM2 because clocks, etc. are
 * disabled.  Therefore it is required for software modules that make use
 * of such devices, to set the appropriate bit in this mask, as long as they
 * need EM1.  This prevents the power management of this application to enter
 * EM2.  The enumeration @ref EM1_MODULES lists those modules.
 * Low-Power peripherals, e.g. the LEUART still work in EM1.
 *
 * Examples:
 *
   @code
   // Module RFID requires EM1, set bit in bit mask
   Bit(g_EM1_ModuleMask, EM1_MOD_RFID) = 1;
   ...
   // Module RFID is no longer active, clear bit in bit mask
   Bit(g_EM1_ModuleMask, EM1_MOD_RFID) = 0;
   @endcode
 */
volatile uint16_t	g_EM1_ModuleMask;

/*================================ Local Data ================================*/

    /*! EXTI initialization structure
     *
     * Connect the external interrupts of the push buttons to the key handler.
     */
static const EXTI_INIT  l_ExtIntCfg[] =
{   //	IntBitMask,	IntFct
    {	KEY_EXTI_MASK,	KeyHandler	},	// Keys
    {	0,		NULL		}
};

    /*!
     * Initialization structure to define the timings for the autorepeat (AR)
     * threshold and rate (in milliseconds), and a function to be called for each
	 * translated key.
     */
static const KEY_INIT  l_KeyInit =
{
    .AR_Threshold = AUTOREPEAT_THRESHOLD,
    .AR_Rate	= AUTOREPEAT_RATE,
    .KeyFct	= UpdateKeyHandler
};


/* Return code for CMU_Select_TypeDef as string */
static const char *CMU_Select_String[] =
{ "Error", "Disabled", "LFXO", "LFRCO", "HFXO", "HFRCO", "LEDIV2", "AUXHFRCO" };

/*=========================== Forward Declarations ===========================*/

static void cmuSetup(void);
static void Reboot(void);


/******************************************************************************
 * @brief  Main function
 *****************************************************************************/
int main( void )
{
    /* Initialize chip - handle erratas */
    CHIP_Init();
    
    /* EFM32 NVIC implementation provides 8 interrupt levels (0~7) */
    NVIC_SetPriorityGrouping (4);	// 8 priority levels, NO sub-priority
    
    /* Set up clocks */
    cmuSetup();
    
    /* Enable FET to power the device independent from the Power Button */
    PowerUp();

    /* Init Low Energy UART with 9600bd (this is the maximum) */
    drvLEUART_Init (9600);

#ifdef DEBUG
    dbgInit();
#endif

    /* Output version string to SWO or LEUART */
    drvLEUART_puts("\n***** Sensor Logger V");
    drvLEUART_puts(prj.Version);
    drvLEUART_puts(" *****\n\n");

    /* Configure PA2 to drive the red Power-On LED1 - show we are alive */
    GPIO_PinModeSet (POWER_LED_PORT, POWER_LED_PIN, gpioModePushPull, 1);

    /* Configure PA5 to drive the yellow Data collecting LED2 - show we are alive */
    GPIO_PinModeSet (LOG_FLUSH_LED_PORT, LOG_FLUSH_LED_PIN, gpioModePushPull, 1);
    
    /*
     * All modules that make use of external interrupts (EXTI) should be
     * initialized before calling ExtIntInit() because this enables the
     * interrupts, so IRQ handler may be executed immediately!
     */
    
    /* Initialize Logging (do this early) */
    LogInit();    
    
        /* Log Firmware Revision and Clock Info */
    Log ("Sensor Logger V%s (%s %s)", prj.Version, prj.Date, prj.Time);
    uint32_t freq = CMU_ClockFreqGet(cmuClock_HF);
    Log ("Using %s Clock at %ld.%03ldMHz",
	 CMU_Select_String[CMU_ClockSelectGet(cmuClock_HF)],
	 freq / 1000000L, (freq % 1000000L) / 1000L);

    /* Initialize key hardware */
    KeyInit (&l_KeyInit);

    /* Initialize SD-Card Interface */
    DiskInit();
    
    /* Introduce Power-Fail Handlers, configure Interrupt
     * not included, no Goldcap
     * see Microcontroller Voltage for coincell and batterypack!  */ 
    //PowerFailInit (l_PowerFailFct); */
    
    /*
     * Initialize (and enable) External Interrupts
     */
    ExtIntInit (l_ExtIntCfg);

    /* Initialize the Alarm Clock module */
    AlarmClockInit();
    
    /* Initialize control module */
    ControlInit();
   
    /* Switch Log Flush LED OFF */
    LOG_FLUSH_LED = 0;
    
    /* Enable all other External Interrupts */
    ExtIntEnableAll();
    
    /* Read microcontroller voltage */
    LogSensorInfo(BAT_LOG_ONLY);

    /* ============================================ *
     * ========== Service Execution Loop ========== *
     * ============================================ */
    while (1)
    {
            /* Check Data Logger State */
            //UpdateCheck();  
            
            /* Check if SD-Card has been inserted or removed */
	    if (DiskCheck())
	    {
		/* First check if an "*.UPD" file exists on this SD-Card */
		if (FindFile ("/", "*.UPD") != NULL)
		{
		    /*
		     * In this case the SD-Card contains update images.  We must
		     * pass control to the booter to perform a firmware upgrade.
		     */
		    Reboot();
		}

		/* New File System mounted - (re-)open Log File */
		LogFileOpen("BOX*.TXT", "BOX0999.TXT");

		/* Be sure to flush current log buffer so it is empty */
		LogFlush(true);	// keep SD-Card power on!

		/* Log information about the MCU and the battery */
		uint32_t uniquHi = DEVINFO->UNIQUEH;
		Log ("MCU: %s HW-ID: 0x%08lX%08lX",
		     PART_NUMBER, uniquHi, DEVINFO->UNIQUEL);
                
                /* Clear (previous) Configuration */
		ClearConfiguration();
                                     
                /* Read and parse configuration file */
		CfgRead("CONFIG.TXT");
                
                /* Verify new Configuration */
		VerifyConfiguration();

                /* Initialize Sensor1 Monitor according to (new) configuration */
                Sensor1MonInit();
                   
                /* Flush log buffer again and switch SD-Card power off */
		LogFlush(false);
            }
            
            /* Check Battery State */
	    BatteryCheck();
        
            /* Check if to flush the log buffer */
            LogFlushCheck();
            
            /* Check Data Logger State */
            UpdateCheck();  
           
                    
    	/*
	 * Check for current power mode:  If a minimum of one active module
	 * requires EM1, i.e. <g_EM1_ModuleMask> is not 0, this will be
	 * entered.  If no one requires EM1 activity, EM2 is entered.
	 */
	if (! g_flgIRQ)		// enter EM only if no IRQ occured
	{
	    if (g_EM1_ModuleMask)
		EMU_EnterEM1();		// EM1 - Sleep Mode
	     else
		EMU_EnterEM2(true);	// EM2 - Deep Sleep Mode
	}
	else
	{
	    g_flgIRQ = false;	// clear flag to enter EM the next time
	}
    }
}


/******************************************************************************
 * @brief   Configure Clocks
 *
 * This local routine is called once from main() to configure all required
 * clocks of the EFM32 device.
 *
 *****************************************************************************/
static void cmuSetup(void)
{
    /* Start LFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

#if USE_EXT_32MHZ_CLOCK
    /* Start HFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

    /* Select HFXO as clock source for HFCLK */
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

    /* Disable HFRCO */
    CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
#endif

    /* Route the LFXO clock to the RTC and set the prescaler */
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);	// RTC, LETIMER
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);	// LEUART0/1
    CMU_ClockEnable(cmuClock_RTC, true);

    /* Prescaler of 1 = 30 us of resolution and overflow each 8 min */
    CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);

    /* Enable clock to low energy modules */
    CMU_ClockEnable(cmuClock_CORELE, true);

    /* Enable clock for HF peripherals (ADC, DAC, I2C, TIMER, and USART) */
    CMU_ClockEnable(cmuClock_HFPER, true);

    /* Enable clock to GPIO */
    CMU_ClockEnable(cmuClock_GPIO, true);
}


/******************************************************************************
 * @brief   Reboot
 *
 * This local routine brings the system into a quiescent state and then
 * generates a reset.  It is typically used to transfer control from the
 * application to the booter for firmware upgrades.
 *
 *****************************************************************************/
static void Reboot(void)
{
int	n, i;

     /* Disable external interrupts */
    ExtIntDisableAll();

    /* Shut down peripheral devices */
    SensorMonDeinit();

    drvLEUART_puts ("Shutting down system for reboot\n");

    /*
     * Show LED Pattern before resetting:
     * 3x 5-short-pulses, separated by a pause,
     * finally a dimming LED from maximum brightness to off.
     */
    for (n = 0;  n < 3;  n++)		// 3x patterns
    {
	for (i = 0;  i < 5;  i++)	// a' 5 pulses
	{
	    POWER_LED = 1;
	    msDelay(100);
	    POWER_LED = 0;
	    msDelay(100);
	}
	msDelay(800);			// pause
    }

    for (n = 0;  n < 200;  n++)
    {
	POWER_LED = 1;
	for (i = 0;  i < (200 - n);  i++)
	    DelayTick();

	POWER_LED = 0;
	for (i = 0;  i < n;  i++)
	    DelayTick();
    }

    /* Perform RESET */
    NVIC_SystemReset();
}


/******************************************************************************
 * @brief   Shows the ShowDataCollect LED1
 *
 * This routine is called by the UpdateCheck to indicate the current state
 * of the red LED1 POWER LOGGER is ON (LED1 on) or is off(LED1 off).
 *
 *****************************************************************************/
void	ShowPowerLogger_LED1 (bool enable)
{
    /* Set once after Pushbutton1 is assert or release */  
    POWER_LED = enable;
}


/******************************************************************************
 * @brief   Shows the ShowDataCollect LED2
 *
 * This routine is called by the UpdateCheck to indicate the current state
 * of the LED2 DATA COLLECT is ON (LED2 on) or is off(LED2 off).
 *
 *****************************************************************************/
void	ShowDataCollect_LED2 (bool enable)
{
    /* Set once after Pushbutton2 is assert or release */  
    LOG_FLUSH_LED = enable;
}
