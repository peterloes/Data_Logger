/***************************************************************************//**
 * @file
 * @brief	Header file of module Control.c
 * @author	Ralf Gerhauser
 * @version	2018-10-10
 ****************************************************************************//*
Revision History:
2018-10-10,rage	Added prototype VerifyConfiguration(), removed unused prototypes.
		Added timing variables for Power Cycling.
2018-03-26,rage	Initial version, based on MAPRDL.
*/

#ifndef __INC_Control_h
#define __INC_Control_h

/*=============================== Header Files ===============================*/

#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

    /*!@brief Show module "Control" exists in this project. */
#define MOD_CONTROL_EXISTS

    /*!@brief Power output selection. */
typedef enum
{
    PWR_OUT_NONE = NONE,	// (-1) for no output at all
    PWR_OUT_SENSOR1,		// 0: 3V3 VDD_SENSOR1 at Pin X4-5
    PWR_OUT_SENSOR2,		// 1: 3V3 VDD_SENSOR2 at Pin X5-5
    NUM_PWR_OUT
} PWR_OUT;

    /*!@brief Power control. */
//@{
#define PWR_OFF		false	//!< Switch power output off (disable power)
#define PWR_ON		true	//!< Switch power output on  (enable power)
//@}

/*================================ Global Data ===============================*/

extern const char *g_enum_PowerOutput[];
extern int32_t     g_PwrInterval[NUM_PWR_OUT];
extern int32_t     g_On_Duration[NUM_PWR_OUT];

/*================================ Prototypes ================================*/

    /* Initialize control module */
void	ControlInit (void);

    /* Clear Configuration variables (set default values) */
void	ClearConfiguration (void);

    /* Verify Configuration values */
void	VerifyConfiguration (void);

    /* Perform miscellaneous control tasks */
void	Control (void);

    /* Switch power output on or off */
void	PowerOutput	(PWR_OUT output, bool enable);
bool	IsPowerOutputOn (PWR_OUT output);

    /* Determine if Data collect is on */
bool	IsDataCollectOn (void);

#endif /* __INC_Control_h */
