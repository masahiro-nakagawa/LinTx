/**
 * LIN Host Interface Header File
 *
 * @file lin_host.h
 * 
 * @ingroup  linhost
 * 
 * @brief This header file provides the API for the Lin Host App.
 *
 * @version Lin Host App Driver Version 1.0.0
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


#ifndef LIN_H
#define	LIN_H
/**
  Section: Included Files
*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/**
 * @ingroup linhost
 * @enum receive state.
 * @brief Enumeration of receive state.
*/
typedef enum {
    LIN_IDLE,
    LIN_TX_IP,
    LIN_RX_IP,
    LIN_RX_RDY
}lin_state_t;

/**
 * @ingroup linhost
 * @enum Type mode.
 * @brief Enumeration of receive state.
*/
typedef enum {
    TRANSMIT,
    RECEIVE
}lin_packet_type_t;

typedef struct {
    uint8_t cmd;
    lin_packet_type_t type;
    uint8_t length;
    uint8_t timeout;
    uint8_t period;
    uint8_t* data;
}lin_cmd_packet_t;

typedef union {
    struct {
        uint8_t PID;
        uint8_t data[8];
        uint8_t checksum;
        uint8_t length;
    };
    uint8_t rawPacket[11];
}lin_packet_t;

typedef union {
    struct {
        uint8_t cmd;
        uint8_t rxLength;
        uint8_t data[8];
        uint8_t checksum;
        uint8_t timeout;
    };
    uint8_t rawPacket[12];
}lin_rxpacket_t;

typedef union {
    struct {
        unsigned ID0: 1;
        unsigned ID1: 1;
        unsigned ID2: 1;
        unsigned ID3: 1;
        unsigned ID4: 1;
        unsigned ID5: 1;
        unsigned P0: 1;
        unsigned P1: 1;
    };
    uint8_t rawPID;
}lin_pid_t;

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This routine must be called from Initializes.
 * @param tableLength:unsigned 8-bit integer , Callback function of receive , Callback function of processData.
 * @return None. 
 */
void LIN_init(uint8_t tableLength, const lin_cmd_packet_t* const table, void (*processData)(void));

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This routine must be called  Initializes.
 * @param cmd:unsigned 8-bit integer.
 * @return None. 
 */
void LIN_queuePacket(uint8_t cmd, uint8_t* data);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This routine must be called  received data packet.
 * @param .None.
 * @return None. 
 */
bool LIN_receivePacket(void);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This routine must be called send data packet.
 * @param .None.
 * @return None. 
 */
void LIN_sendPacket(void);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This routine must be called  Initializes and return data Packet.
 * @param dataPointer.
 * @return unsigned 8-bit integer. 
 */
uint8_t LIN_getPacket(uint8_t* data);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This routine must be called  Initializes and return state struct.
 * @param None.
 * @return lin_state_t struct. 
 */
lin_state_t LIN_handler(void);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This return the checksum value.
 * @param length:unsigned 8-bit integer , dataPointer.
 * @return unsigned 8-bit integer. 
 */
uint8_t LIN_getChecksum(uint8_t length, uint8_t* data);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This return the calculated parity value.
 * @param CMD:unsigned 8-bit integer.
 * @return unsigned 8-bit integer. 
 */
uint8_t LIN_calcParity(uint8_t CMD);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This start the timer.
 * @param timeout:unsigned 8-bit integer.
 * @return None. 
 */
void LIN_startTimer(uint8_t timeout);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This handle the timer.
 * @param  None.
 * @return None. 
 */
void LIN_timerHandler(void);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This set the timer.
 * @param  None.
 * @return None. 
 */
void LIN_setTimerHandler(void);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This stop the timer.
 * @param  None.
 * @return None. 
 */
void LIN_stopTimer(void);
/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This start the period.
 * @param  None.
 * @return None. 
 */

void LIN_startPeriod(void);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This stop the period.
 * @param  None.
 * @return None. 
 */
void LIN_stopPeriod(void);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This enable the receive.
 * @param  None.
 * @return None. 
 */
void LIN_enableRx(void);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This disable the receive.
 * @param  None.
 * @return None. 
 */
void LIN_disableRx(void);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This break the check.
 * @param  None.
 * @return bool. 
 */
void LIN_sendBreak(void);

/**
 * @ingroup linhost
 * @brief process the linhost module
 *        This send the periodically data transmit.
 * @param  None.
 * @return bool. 
 */
void LIN_sendPeriodicTx(void);


#endif	/* LIN_H */

