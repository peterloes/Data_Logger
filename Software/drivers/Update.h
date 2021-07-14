/***************************************************************************//**
 * @file
 * @brief	Header file of module Update.c
 * @author	Peter Loes      
 * @version	2021-06-25
 ****************************************************************************//*
Revision History:
2014-11-25,rage	Initial version.
*/

#ifndef __INC_Update_h
#define __INC_Update_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"

/*=============================== Definitions ================================*/

/*
 * Power Pin Definitions (drives FET T1)
 */
    /*!@brief GPIO Port of the Power-Pin. */
#define HOLD_POWER_PORT		gpioPortD
    /*!@brief GPIO Pin of the Power-Pin. */
#define HOLD_POWER_PIN		1

/*================================ Prototypes ================================*/

void	PowerUp (void);
void	UpdateKeyHandler (KEYCODE keycode);
void	UpdateCheck (void);
    /* Determine if Collecting Data is Activate */
bool	IsDataCollectOn (void);

#endif /* __INC_Update_h */
