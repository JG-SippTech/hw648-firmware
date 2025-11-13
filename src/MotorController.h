#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <Encoder.h>
#include "WEMOS_Motor.h"
#include "PIDController.h"

/**
 * @brief Motor controller with encoder feedback and PID velocity control
 *
 * Combines a DC motor, quadrature encoder, and PID controller into a
 * single closed-loop velocity control system.
 */
class MotorController {
public:
    /**
     * @brief Construct a new Motor Controller
     *
     * @param motor Pointer to WEMOS Motor object
     * @param encoder Pointer to Encoder object
     * @param pid Pointer to PID Controller object
     * @param fwdDir Motor direction constant for forward motion
     * @param backDir Motor direction constant for backward motion
     */
    MotorController(Motor* motor, Encoder* encoder, PIDController* pid,
                   uint8_t fwdDir, uint8_t backDir);

    /**
     * @brief Initialize the motor controller
     *
     * Resets encoder position and PID state. Call in setup().
     */
    void begin();

    /**
     * @brief Set target velocity with PID control
     *
     * @param targetVelocity Target velocity in encoder counts/second
     *                       Positive = forward, negative = backward, 0 = stop
     */
    void setVelocity(float targetVelocity);

    /**
     * @brief Update velocity control loop
     *
     * Measures current velocity, runs PID, and updates motor PWM.
     * Call this at regular intervals (e.g., 50 Hz).
     *
     * @param dt Time delta since last update (seconds)
     */
    void update(float dt);

    /**
     * @brief Stop the motor immediately
     *
     * Sets motor to STOP mode and resets PID state.
     */
    void stop();

    /**
     * @brief Emergency brake
     *
     * Activates motor short brake for rapid stopping.
     */
    void brake();

    /**
     * @brief Get current motor velocity
     *
     * @return float Current velocity in encoder counts/second
     */
    float getVelocity() const { return m_currentVelocity; }

    /**
     * @brief Get target velocity
     *
     * @return float Target velocity in encoder counts/second
     */
    float getTargetVelocity() const { return m_targetVelocity; }

    /**
     * @brief Get current encoder position
     *
     * @return long Encoder count since last reset
     */
    long getPosition() const;

    /**
     * @brief Reset encoder position to zero
     */
    void resetPosition();

    /**
     * @brief Get current PWM duty cycle being sent to motor
     *
     * @return float PWM value (0-100)
     */
    float getPWM() const { return m_currentPWM; }

    /**
     * @brief Get velocity error
     *
     * @return float Error between target and actual velocity (counts/sec)
     */
    float getError() const { return m_pid->getError(); }

private:
    Motor* m_motor;               // WEMOS Motor object
    Encoder* m_encoder;           // Quadrature encoder
    PIDController* m_pid;         // PID controller

    uint8_t m_fwdDir;            // Forward direction constant
    uint8_t m_backDir;           // Backward direction constant

    float m_targetVelocity;      // Target velocity (counts/sec)
    float m_currentVelocity;     // Measured velocity (counts/sec)
    float m_currentPWM;          // Current PWM duty cycle (0-100)

    long m_previousPosition;     // Last encoder reading
    unsigned long m_previousTime; // Last update timestamp (microseconds)

    /**
     * @brief Calculate current velocity from encoder
     *
     * @param dt Time delta since last calculation (seconds)
     */
    void calculateVelocity(float dt);

    /**
     * @brief Apply motor command based on PWM and direction
     *
     * @param pwm PWM duty cycle (0-100)
     * @param velocity Velocity sign determines direction
     */
    void applyMotorCommand(float pwm, float velocity);
};

#endif // MOTOR_CONTROLLER_H
