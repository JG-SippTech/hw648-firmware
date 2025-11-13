#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd,
                             float outputMin, float outputMax,
                             float integralMax)
    : m_kp(kp)
    , m_ki(ki)
    , m_kd(kd)
    , m_outputMin(outputMin)
    , m_outputMax(outputMax)
    , m_integralMax(integralMax)
    , m_error(0.0f)
    , m_previousError(0.0f)
    , m_integral(0.0f)
    , m_derivative(0.0f)
    , m_output(0.0f)
{
}

float PIDController::compute(float setpoint, float measurement, float dt) {
    // Compute error
    m_error = setpoint - measurement;

    // Proportional term
    float pTerm = m_kp * m_error;

    // Integral term with anti-windup
    m_integral += m_error * dt;
    m_integral = clamp(m_integral, -m_integralMax, m_integralMax);
    float iTerm = m_ki * m_integral;

    // Derivative term
    if (dt > 0.0f) {
        m_derivative = (m_error - m_previousError) / dt;
    } else {
        m_derivative = 0.0f;
    }
    float dTerm = m_kd * m_derivative;

    // Compute output
    m_output = pTerm + iTerm + dTerm;

    // Clamp output to limits
    m_output = clamp(m_output, m_outputMin, m_outputMax);

    // Save error for next iteration
    m_previousError = m_error;

    return m_output;
}

void PIDController::reset() {
    m_error = 0.0f;
    m_previousError = 0.0f;
    m_integral = 0.0f;
    m_derivative = 0.0f;
    m_output = 0.0f;
}

void PIDController::setGains(float kp, float ki, float kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

float PIDController::clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
