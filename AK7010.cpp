/**
 * @file AK7010.cpp
 * @author kaming (kamingwin@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-06
 *
 * @copyright Modify base on HKUST Robomaster. All Rights Reserved.
 *
 */

#include "AK7010.hpp"

#if USE_AK7010

#include "FreeRTOS.h"
#include "can.h"
#include "math.h"
#include "task.h"
#include "gpio.h"   // for testing purpose


#define AK7010_TIMEOUT pdMS_TO_TICKS(200)
#define AK7010_CAN hcan1

namespace AK7010
{
//constructor of the class
AK7010::AK7010()
    : rawEncoder(0),
      lastRawEncoder(0),
      position(0),         
      dps(0),
      actualCurrent(0),
      currentLimit(10000), 
      temperature(0),
      rotaryCnt(0),
      positionOffset(0),
      disconnectCnt(200),
      receiveCnt(0),
      connected(false),
      reverSign(1),
      errorCode(0)
{
}

AK7010 AK7010::motors[4];

uint16_t AK7010::getRawEncoder() const { return this->rawEncoder; }

float AK7010::getPosition() const { return this->position; }

void AK7010::setPosition(float setPosition)
{   // to be finished

    // uint32_t ulOriginalBASEPRI = 0;
    // if (__get_IPSR())
    //     ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
    // else
    //     taskENTER_CRITICAL();
    // // input in radian output in encoder degree form before the gear ratio
    // // (setPosition - position) / (2*M_PI) * 3600
    // this->positionOffset = (int16_t)((setPosition - this->position) /
    //                                  (2 * M_PI) * 360);
    // taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
}

int16_t AK7010::getRPM() const { return this->rpm; }

int16_t AK7010::getActualCurrent() const { return this->actualCurrent; }

int16_t AK7010::getOutputCurrent() const { return this->setCurrent; }

void AK7010::setOutputCurrent(int32_t current)
{
    if (current > this->currentLimit)
        current = this->currentLimit;
    else if (current < -this->currentLimit)
        current = -this->currentLimit;
    this->setCurrent = current;
}

void AK7010::setCurrentLimit(uint16_t current)
{
    // for current >2048 it will return execution halts
    configASSERT(current <= 50000);
    this->currentLimit = current;
}


void AK7010::setReverseSign(int16_t sign)
{
    configASSERT(sign == 1 || sign == -1);
    this->reverSign = sign;
}

uint8_t AK7010::getTemperature() const { return this->temperature; }

uint32_t AK7010::getReveiceCount() const { return this->receiveCnt; }

bool AK7010::isConnected() const { return this->connected; }

AK7010 &getMotor(uint32_t canid)
{
    canid -= 0x00;
    configASSERT(canid < 4);
    return AK7010::motors[canid];
}

void sendMotorGroup()
{
    uint32_t ulOriginalBASEPRI = 0;
    if (__get_IPSR())
        ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
    else
        taskENTER_CRITICAL();

    static uint8_t canTxData[4];
    static volatile uint32_t canMailBox[3];
    static volatile uint32_t canLastSend   = 0;
    static CAN_TxHeaderTypeDef canTxHeader = {
        0x0000,         // StdId
        0x0000,         // ExtId
        CAN_ID_EXT,     // IDE
        CAN_RTR_DATA,   // RTR
        4,              // DLC
        DISABLE         // Transmit Global Time
    };
    // add a reverse sign to the current
    for (uint32_t i = 0; i<4 ; i++)
    {
        //CAN_PACKET_SET_CURRENT = 0x0001 <<8 +1 = 0x0101
        canTxHeader.ExtId = (uint32_t)(0x0101+i);
        // canTxHeader.ExtId = (uint32_t)(0x0301+i); // speed control for testing

		canTxData[0] =  (AK7010::motors[i].setCurrent * AK7010::motors[i].reverSign) >> 24 & 0xFF; 
		canTxData[1] =  (AK7010::motors[i].setCurrent * AK7010::motors[i].reverSign) >> 16 & 0xFF;
		canTxData[2] =  (AK7010::motors[i].setCurrent * AK7010::motors[i].reverSign) >> 8 & 0xFF;
		canTxData[3] =  (AK7010::motors[i].setCurrent * AK7010::motors[i].reverSign) & 0xFF;
        AK7010::motors[i].sendFlag = true;
    

    if (HAL_CAN_GetTxMailboxesFreeLevel(&AK7010_CAN) == 0)
    {
        HAL_CAN_AbortTxRequest(&AK7010_CAN, canMailBox[(canLastSend + 1) % 3]);
    }
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    HAL_CAN_AddTxMessage(&AK7010_CAN,
                         &canTxHeader,
                         (uint8_t *)canTxData,
                         (uint32_t *)&canMailBox[canLastSend]);
    canLastSend = (canLastSend + 1) % 3;
    }

    if (__get_IPSR())
        taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
    else
        taskEXIT_CRITICAL();

}

static volatile uint32_t canFree;
static volatile uint16_t freeCnt;

void motorUpdate(void *)
{
    static TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        // to detect whether the CAN is unfunctional and restart it
        canFree = HAL_CAN_GetTxMailboxesFreeLevel(&AK7010_CAN);
        if (canFree == 0)
        {
            freeCnt++;
        }
        else
        {
            freeCnt = 0;
        }
        if (freeCnt > 100)
        {
            freeCnt = 0;
            HAL_CAN_AbortTxRequest(&AK7010_CAN, 0);
            HAL_CAN_AbortTxRequest(&AK7010_CAN, 1);
            HAL_CAN_AbortTxRequest(&AK7010_CAN, 2);
            HAL_CAN_Stop(&AK7010_CAN);
            vTaskDelay(pdMS_TO_TICKS(5));
            HAL_CAN_Start(&AK7010_CAN);
        }

        for (uint32_t motorNum = 0; motorNum < 4; motorNum++)
            if (AK7010::motors[motorNum].disconnectCnt++ > AK7010_TIMEOUT)
            {
                AK7010::motors[motorNum].connected     = false;
                AK7010::motors[motorNum].disconnectCnt = AK7010_TIMEOUT;
            }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

void AK7010::decodeFeedback(CAN_HandleTypeDef *)
{
    while (HAL_CAN_GetRxFifoFillLevel(&AK7010_CAN, CAN_RX_FIFO0))
    {
        CAN_RxHeaderTypeDef AK7010canRxHeader;
        AK7010canRxHeader.ExtId = 0;
        static uint8_t AK7010RxBuffer[8];
        HAL_CAN_GetRxMessage(&AK7010_CAN,
                             CAN_RX_FIFO0,
                             &AK7010canRxHeader,
                             (uint8_t *)&AK7010RxBuffer);
        if (AK7010canRxHeader.ExtId >= 0x2901 &&
            AK7010canRxHeader.ExtId <= 0x2904)
        {
            // why here dont take any __get_IPSR because of his low priority?
            uint32_t ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
            uint32_t id                = AK7010canRxHeader.ExtId - 0x2901;

            AK7010::motors[id].lastRawEncoder = AK7010::motors[id].rawEncoder;

            AK7010::motors[id].rawEncoder =
                AK7010RxBuffer[0] << 8 | AK7010RxBuffer[1];
            
            AK7010::motors[id].position =
                (float)(AK7010::motors[id].rawEncoder*0.1f +
                        AK7010::motors[id].positionOffset) * motors[id].reverSign;  // 16bit

            AK7010::motors[id].rpm = int16_t((AK7010RxBuffer[2] << 8 | AK7010RxBuffer[3])/20.0f * motors[id].reverSign);

            AK7010::motors[id].actualCurrent =
                (AK7010RxBuffer[4] << 8 | AK7010RxBuffer[5]) / 0.01f * motors[id].reverSign;

            AK7010::motors[id].temperature = AK7010RxBuffer[6];

            AK7010::motors[id].errorCode = AK7010RxBuffer[7];

            AK7010::motors[id].disconnectCnt = 0;
            AK7010::motors[id].receiveCnt++;
            AK7010::motors[id].connected = true;
            taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
        }
    }
}

static StackType_t uxMotorSendTaskStack[128];
static StaticTask_t xMotorSendTaskTCB;
static StackType_t uxMotorUpdateTaskStack[128];
static StaticTask_t xMotorUpdateTaskTCB;

void send(void *params)
{
    while (true)
    {
        sendMotorGroup();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void init()
{
    CAN_FilterTypeDef AK7010Filter = {
        0,                      // filterID HI
        0,                      // filterID LO
        0,                      // filterMask HI
        0,                      // filterMask LO
        CAN_FILTER_FIFO0,       // FIFO assignment
        0,                      // filterBank number
        CAN_FILTERMODE_IDMASK,  // filter mode
        CAN_FILTERSCALE_32BIT,  // filter size      // to be comfime
        CAN_FILTER_ENABLE,      // ENABLE or DISABLE
        0                       // Slave start bank
    };

    configASSERT(HAL_CAN_ConfigFilter(&AK7010_CAN, &AK7010Filter) == HAL_OK);
    configASSERT(HAL_CAN_ActivateNotification(
                     &AK7010_CAN, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK);
    configASSERT(HAL_CAN_RegisterCallback(&AK7010_CAN,
                                          HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
                                          AK7010::decodeFeedback) == HAL_OK);
    configASSERT(HAL_CAN_Start(&AK7010_CAN) == HAL_OK);
    xTaskCreateStatic(motorUpdate,
                      "MotorUpdate",
                      128,
                      NULL,
                      14,
                      uxMotorUpdateTaskStack,
                      &xMotorUpdateTaskTCB);
    xTaskCreateStatic(send,
                      "Motorsend",
                      128,
                      NULL,
                      14,
                      uxMotorSendTaskStack,
                      &xMotorSendTaskTCB);
}

} // namespace AK7010
#endif // USE_AK7010

