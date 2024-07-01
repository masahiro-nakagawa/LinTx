/**
 * LIN Host Interface Source File
 * 
 * @file lin_host.c
 * 
 * @ingroup  linhost
 * 
 * @brief This source file provides the interface between the user and the LIN drivers.
 *
 * @version LIN Host Driver Version 1.0.0
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

#include "../lin_host.h"
#include "../../uart/eusart1.h"
#include "../../timer/tmr2.h"

static void (*LIN_processData)(void);

static lin_packet_t LIN_packet;
static lin_rxpacket_t LIN_rxPacket;
bool LIN_txReady = false;
const lin_cmd_packet_t* schedule;
uint8_t scheduleLength;

static uint8_t LIN_timeout = 10;
static uint8_t LIN_period = 0;
static bool LIN_timerRunning = false;
static bool LIN_enablePeriodTx = false;
static volatile uint8_t LIN_timerCallBack = 0;
static volatile uint8_t LIN_periodCallBack = 0;


void LIN_init(uint8_t tableLength, const lin_cmd_packet_t* const table, void (*processData)(void)){
    schedule = table;
    scheduleLength = tableLength;
    LIN_processData = processData;
    LIN_stopTimer();
    LIN_setTimerHandler();

    LIN_startPeriod();
}

void LIN_queuePacket(uint8_t cmd, uint8_t* data){
    const lin_cmd_packet_t* tempSchedule = schedule;    //copy table pointer so we can modify it

    for(uint8_t i = 0; i < scheduleLength; i++){
        if(cmd == tempSchedule->cmd){
            break;
        }
        tempSchedule++;    //go to next entry
    }
    
    //clear previous data
    memset(LIN_packet.rawPacket, 0, sizeof(LIN_packet.rawPacket));
    
    //Add ID
    LIN_packet.PID = LIN_calcParity(tempSchedule->cmd);

    if(tempSchedule->type == TRANSMIT){
        //Build Packet - User defined data
        //add data
        if(tempSchedule->length > 0){
            LIN_packet.length = tempSchedule->length;
            memcpy(LIN_packet.data, data, tempSchedule->length);
        } else {
            LIN_packet.length = 1; //send dummy byte for checksum
            LIN_packet.data[0] = 0xAA;
        }

        //Add Checksum
        LIN_packet.checksum = LIN_getChecksum(LIN_packet.length, LIN_packet.PID, LIN_packet.data);

    } else { //Rx packet
        LIN_rxPacket.rxLength = tempSchedule->length; //data length for rx data processing
        LIN_rxPacket.cmd = tempSchedule->cmd; //command for rx data processing
        LIN_rxPacket.timeout = tempSchedule->timeout;
    }
    
    LIN_txReady = true;
}

lin_state_t LIN_handler(void){
    static lin_state_t LIN_state = LIN_IDLE;
    
    //State Machine
    switch(LIN_state){
        case LIN_IDLE:
            if(LIN_txReady == true){
                LIN_txReady = false;
                LIN_disableRx();   //disable EUSART rx
                //Send Transmission
                LIN_sendPacket();
                LIN_state = LIN_TX_IP;
            } else {
                //No Transmission to send
            }
            break;
        case LIN_TX_IP:
            //Transmission currently in progress.
            /*if(EUSART1_isTxInterrupt() == 0){ Maybe it is not required...*/
                if(EUSART1_IsTxDone() == 1){
                    //Packet transmitted
                    if(LIN_rxPacket.rxLength > 0){
                        //Need data returned?
                        LIN_startTimer(LIN_rxPacket.timeout);
                        LIN_enableRx();   //enable EUSART rx
                        LIN_state = LIN_RX_IP;
                    } else {
                        LIN_state = LIN_IDLE;
                    }
                }
            /*}*/
            break;
        case LIN_RX_IP:
            //Receiving Packet within window
            if(LIN_timerRunning == false){
                //Timeout
                LIN_state = LIN_IDLE;
                memset(LIN_rxPacket.rawPacket, 0, sizeof(LIN_rxPacket.rawPacket));  //clear receive data
            } else if(EUSART1_IsRxReady()){
                if(LIN_receivePacket() == true){
                    //All data received and verified
                    LIN_disableRx();   //disable EUSART rx
                    LIN_state = LIN_RX_RDY;
                }
            }
            break;
        case LIN_RX_RDY:
            //Received Transmission
            LIN_processData();
            LIN_state = LIN_IDLE;
            break;
    }
    return LIN_state;
}

bool LIN_receivePacket(void){
    static uint8_t rxIndex = 0;

    if(rxIndex < LIN_rxPacket.rxLength){
        //save data
        LIN_rxPacket.data[rxIndex++] = EUSART1_Read();
        NOP();
    } else {
        rxIndex = 0;
        //calculate checksum
        if(EUSART1_Read() == LIN_getChecksum(LIN_rxPacket.rxLength, LIN_packet.PID, LIN_rxPacket.data))
            return true;
            
    }
    //still receiving
    return false;
}

void LIN_sendPacket(void){
    //Build Packet - LIN required data
    //Add Break
    LIN_sendBreak();
    EUSART1_Write(0x00); //send dummy transmission
    //Add Preamble
    EUSART1_Write(0x55);
    //Add ID
    EUSART1_Write(LIN_packet.PID);

    if(LIN_rxPacket.rxLength == 0){ //not receiving data
        //Build Packet - User defined data
        //add data
        for(uint8_t i = 0; i < LIN_packet.length; i++){
            EUSART1_Write(LIN_packet.data[i]);
        }
        //Add Checksum
        EUSART1_Write(LIN_packet.checksum);
    }
}

uint8_t LIN_getPacket(uint8_t* data){
    uint8_t cmd = LIN_rxPacket.cmd & 0x3F;
    
    memcpy(data, LIN_rxPacket.data, sizeof(LIN_rxPacket.data));
    memset(LIN_rxPacket.rawPacket, 0, sizeof(LIN_rxPacket.rawPacket));  //clear receive data

    return cmd;
}

uint8_t LIN_calcParity(uint8_t CMD){
    lin_pid_t PID;
    PID.rawPID = CMD;

    //Workaround for compiler bug - CAE_MCU8-200:
//    PID.P0 = PID.ID0 ^ PID.ID1 ^ PID.ID2 ^ PID.ID4;
//    PID.P1 = ~(PID.ID1 ^ PID.ID3 ^ PID.ID4 ^ PID.ID5);
    PID.P0 = PID.ID0 ^ PID.ID1;
    PID.P0 = PID.P0 ^ PID.ID2;
    PID.P0 = PID.P0 ^ PID.ID4;
    PID.P1 = PID.ID1 ^ PID.ID3;
    PID.P1 = PID.P1 ^ PID.ID4;
    PID.P1 = PID.P1 ^ PID.ID5;
    PID.P1 = ~PID.P1;
    
    return PID.rawPID;
}

uint8_t LIN_getChecksum(uint8_t length, uint8_t pid, uint8_t* data){
    uint16_t checksum = pid;
    
    for (uint8_t i = 0; i < length; i++){
        checksum = checksum + *data++;
        if(checksum > 0xFF)
            checksum -= 0xFF;
    }
    checksum = ~checksum;
    
    return (uint8_t)checksum;
}

void LIN_startTimer(uint8_t timeout){
    LIN_timeout = timeout;
    Timer2_Write(0);
    Timer2_Start();
    LIN_timerRunning = true;
}

void LIN_timerHandler(void){

    if(LIN_timerRunning == true){
        if (++LIN_timerCallBack >= LIN_timeout){
            // ticker function call
            LIN_stopTimer();
        }
    }
    if(LIN_enablePeriodTx == true){
        if(++LIN_periodCallBack >= LIN_period){
            LIN_sendPeriodicTx();
        }
    }
        
}

void LIN_setTimerHandler(void){
    Timer2_OverflowCallbackRegister(LIN_timerHandler);
}

void LIN_stopTimer(void){
    // reset ticker counter
    LIN_timerCallBack = 0;
    LIN_timerRunning = false;
}

void LIN_startPeriod(void){
    LIN_enablePeriodTx = true;
}

void LIN_stopPeriod(void){
    // reset ticker counter
    LIN_periodCallBack = 0;
    LIN_enablePeriodTx = false;
}

void LIN_enableRx(void){
    EUSART1_ReceiveEnable();
    EUSART1_ReceiveInterruptEnable();
}

void LIN_disableRx(void){
    EUSART1_ReceiveDisable();
    EUSART1_ReceiveInterruptDisable();
}

void LIN_sendBreak(void){
    EUSART1_SendBreakControlEnable();
}

void LIN_sendPeriodicTx(void){
    static volatile uint8_t scheduleIndex = 0;
    const lin_cmd_packet_t* periodicTx;    //copy table pointer so we can modify it
    
    LIN_periodCallBack = 0;
    periodicTx = schedule + scheduleIndex;
    
    if(periodicTx->period > 0){
        LIN_queuePacket(periodicTx->cmd, periodicTx->data);
    }
    
    do{ //Go to next valid periodic command
        if(++scheduleIndex >= scheduleLength){
            scheduleIndex = 0;
        }
        periodicTx = schedule + scheduleIndex;
    } while(periodicTx->period == 0);
    
    LIN_period = periodicTx->period;
}