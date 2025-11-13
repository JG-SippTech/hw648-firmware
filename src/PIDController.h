#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

/**
 * @brief Generic PID Controller class for velocity control
 *
 * Implements a discrete-time PID controller with anti-windup protection.
 * Designed for motor velocity control using encoder feedback.
 */
class PIDController {
public:
    /**
     * @brief Construct a new PID Controller
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param outputMin Minimum output limit
     * @param outputMax Maximum output limit
     * @param integralMax Maximum integral accumulation (anti-windup)
     */
    PIDController(float kp, float ki, float kd,
                  float outputMin, float outputMax,
                  float integralMax);

    /**
     * @brief Compute PID output
     *
     * @param setpoint Desired value (target velocity)
     * @param measurement Current value (actual velocity)
     * @param dt Time delta since last computation (seconds)
     * @return float Control output (clamped to output limits)
     */
    float compute(float setpoint, float measurement, float dt);

    /**
     * @brief Reset the PID controller state
     *
     * Clears integral accumulation and previous error.
     * Call when starting motion or changing modes.
     */
    void reset();

    /**
     * @brief Update PID gains at runtime
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setGains(float kp, float ki, float kd);

    /**
     * @brief Get current error value
     * @return float Last computed error (setpoint - measurement)
     */
    float getError() const { return m_error; }

    /**
     * @brief Get integral accumulation
     * @return float Current integral term value
     */
    float getIntegral() const { return m_integral; }

    /**
     * @brief Get derivative term
     * @return float Last computed derivative value
     */
    float getDerivative() const { return m_derivative; }

    /**
     * @brief Get last computed output
     * @return float Last PID output value
     */
    float getOutput() const { return m_output; }

private:
    // PID gains
    float m_kp;
    float m_ki;
    float m_kd;

    // Output limits
    float m_outputMin;
    float m_outputMax;

    // Integral anti-windup limit
    float m_integralMax;

    // PID state
    float m_error;
    float m_previousError;
    float m_integral;
    float m_derivative;
    float m_output;

    /**
     * @brief Clamp a value between min and max
     *
     * @param value Value to clamp
     * @param min Minimum allowed value
     * @param max Maximum allowed value
     * @return float Clamped value
     */
    float clamp(float value, float min, float max);
};

#endif // PID_CONTROLLER_H
