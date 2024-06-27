/**
 * Lin Host Application Header file
 *
 * @file lin_app.h
 * 
 * @defgroup linhost LIN Host Application
 *
 * @brief This header file provides the API for the Lin Host.
 *
 * @version Lin Host Driver Version 1.0.0
*/

/*
© [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

/**
  Section: Included Files
*/

#include "./lin_host.h"

#ifndef LIN_APP_H
#define	LIN_APP_H

/**
 * @ingroup linhost
 * @enum linhost_cmd_t.
 * @brief Enumeration of signalName and ID value.
*/
typedef enum {
    MasterSwitch=0,
    SerialAnal=1
}lin_cmd_t;

uint8_t MasterSwitch_Data[1];
uint8_t SerialAnal_Data[1];

const lin_cmd_packet_t scheduleTable[] = {
    //Command, Type, TX/RX Length, Timeout, Period, Data Address
   {MasterSwitch,TRANSMIT,1,5,10,MasterSwitch_Data},
   {SerialAnal,RECEIVE,1,5,20,SerialAnal_Data}
};
#define TABLE_SIZE  (sizeof(scheduleTable)/sizeof(lin_cmd_packet_t))

/**
 * @ingroup linhost
 * @brief Initializes the linhost module
 *        This routine must be called before any other linhost routine
 * @param None.
 * @return None. 
 */
void LIN_Host_Initialize(void);

/**
 * @ingroup linhost
 * @brief processLIN the linhost module
 *        This routine must be called before any other linhost routine
 * @param None.
 * @return None. 
 */
void processLIN(void);

#endif	/* LIN_APP_H */

