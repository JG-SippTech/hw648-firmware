#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H

#include <Arduino.h>
#include "MotorController.h"
#include "SpeedRamp.h"

/**
 * @brief Robot motion coordinator for pipe crawler
 *
 * Manages coordinated motion of all three motors with smooth
 * acceleration/deceleration. Provides high-level motion commands.
 */
class RobotMotion {
public:
    /**
     * @brief Construct a new Robot Motion controller
     *
     * @param motor1 Pointer to Motor 1 controller
     * @param motor2 Pointer to Motor 2 controller
     * @param motor3 Pointer to Motor 3 controller
     * @param ramp Pointer to speed ramp controller
     */
    RobotMotion(MotorController* motor1, MotorController* motor2,
                MotorController* motor3, SpeedRamp* ramp);

    /**
     * @brief Initialize robot motion controller
     *
     * Call in setup() to initialize all motor controllers.
     */
    void begin();

    /**
     * @brief Move forward at specified speed
     *
     * All three wheels spin forward with smooth acceleration.
     *
     * @param speedPercent Speed as percentage (0-100)
     */
    void moveForward(float speedPercent);

    /**
     * @brief Move backward at specified speed
     *
     * All three wheels spin backward with smooth acceleration.
     *
     * @param speedPercent Speed as percentage (0-100)
     */
    void moveBackward(float speedPercent);

    /**
     * @brief Stop all motors
     *
     * Gracefully decelerates all motors to zero with smooth ramping.
     */
    void stop();

    /**
     * @brief Emergency stop
     *
     * Immediately stops all motors without ramping.
     * Use for safety-critical situations.
     */
    void emergencyStop();

    /**
     * @brief Set individual motor speed
     *
     * Allows manual control of a single motor for corrections.
     *
     * @param motorId Motor number (1, 2, or 3)
     * @param speedPercent Speed percentage (-100 to +100)
     *                     Positive = forward, negative = backward
     * @return true Command accepted
     * @return false Invalid motor ID
     */
    bool setMotor(int motorId, float speedPercent);

    /**
     * @brief Update motion control loop
     *
     * Updates ramp, calculates target velocities, and updates all motors.
     * Call this at regular intervals (e.g., 50 Hz).
     *
     * @param dt Time delta since last update (seconds)
     */
    void update(float dt);

    /**
     * @brief Check if robot is moving
     *
     * @return true If any motor has non-zero target velocity
     * @return false If robot is stopped
     */
    bool isMoving() const;

    /**
     * @brief Get current speed percentage
     *
     * @return float Current ramped speed (0-100)
     */
    float getCurrentSpeed() const { return m_currentSpeedPercent; }

    /**
     * @brief Get target speed percentage
     *
     * @return float Target speed (0-100)
     */
    float getTargetSpeed() const { return m_targetSpeedPercent; }

    /**
     * @brief Get motor controller by ID
     *
     * @param motorId Motor number (1, 2, or 3)
     * @return MotorController* Pointer to motor controller, or nullptr if invalid
     */
    MotorController* getMotor(int motorId);

    /**
     * @brief Reset all encoder positions to zero
     */
    void resetEncoders();

private:
    MotorController* m_motor1;
    MotorController* m_motor2;
    MotorController* m_motor3;
    SpeedRamp* m_ramp;

    float m_targetSpeedPercent;    // Target speed (0-100)
    float m_currentSpeedPercent;   // Current ramped speed (0-100)
    int m_direction;               // 1 = forward, -1 = backward, 0 = stopped
    bool m_individualControl;      // True if individual motor control is active

    float m_maxVelocityCPS;        // Maximum velocity in counts/sec at 100%

    /**
     * @brief Convert speed percentage to velocity in counts/sec
     *
     * @param speedPercent Speed percentage (0-100)
     * @return float Velocity in encoder counts per second
     */
    float speedToVelocity(float speedPercent);

    /**
     * @brief Clamp speed to valid range
     *
     * @param speed Speed value to clamp
     * @return float Clamped speed (0-100)
     */
    float clampSpeed(float speed);
};

#endif // ROBOT_MOTION_H
