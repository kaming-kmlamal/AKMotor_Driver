/**
 * @file AK7010.hpp
 * @author kaming (kamingwin@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-06
 *
 * @copyright Modify base on HKUST Robomaster. All Rights Reserved.
 *
 */

#pragma once
#include "AppConfig.h"  // enable or disable the driver

#define USE_AK7010 1    // for testing purpose

#if USE_AK7010

#include "main.h"

namespace AK7010
{
class AK7010
{
    public:
    AK7010(const AK7010 &) = delete; // disable copy constructor
    AK7010 &operator=(const AK7010 &) = delete; // disable copy assignment

    /**
     * @brief Get the raw encoder value
     *
     * @return uint16_t
     */
    uint16_t getRawEncoder() const;

    /**
     * @brief Get the current position of the motor in radian
     * @note  You may need to multiply the reduction ratio of the motor to get
     * the actual position.
     *
     * @return float
     */
    virtual float getPosition() const;

    /**
     * @brief Set the Current Position object in radian
     * @note This just set the current reference position of the motor(offset), it does
     * not change the actual position of the motor.
     *
     * @param position
     */
    virtual void setPosition(float position);

    /**
     * @brief Get the current speed of the motor in revolutions per minute (rpm)
     *
     * @return int16_t
     */
    virtual int16_t getRPM() const;

    /**
     * @brief Get the actual output current(or voltage) of the motor
     *
     * @return int16_t
     */
    virtual int16_t getActualCurrent() const;

    /**
     * @brief Get the set output current(or voltage) of the motor
     *
     * @return int16_t
     */
    int16_t getOutputCurrent() const;

    /**
     * @brief Set the output current(or voltage) of the motor
     * @note  This function will limit the current(or voltage) according to the
     * current(or voltage) limit of the motor. Please call sendMotorGroup() to
     * send the command to the motor.
     *
     * @param current
     */
    virtual void setOutputCurrent(int32_t current);

    /**
     * @brief Set the Current(or voltage) Limit of the motor
     * @note  To avoid overflow,
     *          the maximum current limit for M3508 is 16384,
     *          and the maximum voltage limit for GM6020 is 30000.
     *
     * The default limit is 10000.
     *
     * @param current
     */
    void setCurrentLimit(uint16_t current);

    /**
     * @brief set the motor to record in reverse direction
     * 
     * @param sign
     */
    void setReverseSign(int16_t sign);

    /**
     * @brief Get the temperature of the motor
     *
     * @return uint8_t
     */
    uint8_t getTemperature() const;

    /**
     * @brief Get the Reveice Count of the motor, this can be used to estimate
     * the receive frequency of the motor
     *
     * @return uint32_t
     */
    virtual uint32_t getReveiceCount() const;

    /**
     * @brief Check if the motor is connected
     *
     * @return true
     * @return false
     */
    bool isConnected() const;

    /**
     * @brief The array of all the possible AK7010E
     */
    static AK7010 motors[4];

    /**
     * @attention   This function is used to decode the CAN message and update
     * the motor data, you should not use this function.
     */
    static void decodeFeedback(CAN_HandleTypeDef *);

    protected:
    /**
     * @attention   You should not call this constructor directly.
     *              Instead, call AK7010::getMotor() to get the motor instance
     * according to the motor CAN ID.
     */
    AK7010();

    volatile uint16_t rawEncoder;       // deg -32768 ~ 32767
    volatile uint16_t lastRawEncoder;   // deg -32768 ~ 32767
    volatile float position;            // rad  after the gear ratio
    volatile int16_t dps;               // deg per second after the gear ratio
    volatile float rpm;                 // revolutions per minute after the gear ratio
    volatile int16_t actualCurrent;     
    volatile int32_t setCurrent;
    volatile uint16_t currentLimit;

    volatile uint8_t temperature;

    volatile int32_t rotaryCnt;
    volatile int16_t positionOffset;    // deg before the gear ratio

    volatile uint32_t disconnectCnt;
    volatile uint32_t receiveCnt;
    volatile bool connected;
    volatile bool sendFlag = false;
    volatile uint8_t errorCode;
    // for reverse direction of the motor either 1 or -1
    volatile int16_t reverSign;

    friend AK7010 &getMotor(uint8_t id);
    friend void motorUpdate(void *);
    friend void sendMotorGroup();
};

/**
 * @brief Get the AK7010 object according to the CAN ID
 *
 * @param canid (eg. 0x205)
 * @return AK7010&
 */
AK7010 &getMotor(uint32_t canid);

/**
 * @brief   Send the command to the motor by group,
 *          call this function after you set the output current(or voltage) of
 * the motor.
 *
 * @param group     0 -> 0x200 , 1 -> 0x1ff, 2 -> 0x2ff
 */
void sendMotorGroup();

/**
 * @brief Initialize the AK7010 driver
 *          Call this function before using this AK7010 driver
 *
 * @note  If you do not want to use this AK7010 driver provided by us, do not
 * call this function.
 */
void init();

}   // namespace AK7010

#endif // USE_AK7010

































