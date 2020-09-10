/******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/
/*
 *      PROJECT:   ST25R391x firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file tick.h
 *
 *  \brief Tick implmentation
 *
 *  This modules conts the number of ticks in 1ms
 *  The increments are triggered by a Timer IRQ
 *
 *
 *  \author Gustavo Patricio
 *
 *
 */


#ifndef __TICK_H
#define __TICK_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"

 /*!
 *****************************************************************************
 * \brief  Ticke Get
 *
 * Gets the current value of the tick counter
 *
* \return the current tick counter value (in ms)
 *****************************************************************************
 */
uint32_t tickGet( void );


 /*!
 *****************************************************************************
 * \brief  Ticke Increment
 *
 * Increments the tick counter. To be triggered by the timer IRQ
 *
 *****************************************************************************
 */
void tickInc( void );

#endif /* __TICK_H */
