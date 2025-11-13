#include "MotorController.h"

MotorController::MotorController(Motor* motor, Encoder* encoder, PIDController* pid,
                                 uint8_t fwdDir, uint8_t backDir)
    : m_motor(motor)
    , m_encoder(encoder)
    , m_pid(pid)
    , m_fwdDir(fwdDir)
    , m_backDir(backDir)
    , m_targetVelocity(0.0f)
    , m_currentVelocity(0.0f)
    , m_currentPWM(0.0f)
    , m_previousPosition(0)
    , m_previousTime(0)
{
}

void MotorController::begin() {
    // Reset encoder to zero
    m_encoder->write(0);
    m_previousPosition = 0;
    m_previousTime = micros();

    // Reset PID state
    m_pid->reset();

    // Ensure motor is stopped
    m_motor->setmotor(_STOP);

    m_targetVelocity = 0.0f;
    m_currentVelocity = 0.0f;
    m_currentPWM = 0.0f;
}

void MotorController::setVelocity(float targetVelocity) {
    m_targetVelocity = targetVelocity;
}

void MotorController::update(float dt) {
    // Calculate current velocity from encoder
    calculateVelocity(dt);

    // Run PID controller
    float pidOutput = m_pid->compute(m_targetVelocity, m_currentVelocity, dt);

    // Get absolute PWM value and apply
    m_currentPWM = fabs(pidOutput);

    // Apply motor command with direction
    applyMotorCommand(m_currentPWM, m_targetVelocity);
}

void MotorController::stop() {
    m_targetVelocity = 0.0f;
    m_currentPWM = 0.0f;
    m_motor->setmotor(_STOP);
    m_pid->reset();
}

void MotorController::brake() {
    m_targetVelocity = 0.0f;
    m_currentPWM = 0.0f;
    m_motor->setmotor(_SHORT_BRAKE);
    m_pid->reset();
}

long MotorController::getPosition() const {
    return m_encoder->read();
}

void MotorController::resetPosition() {
    m_encoder->write(0);
    m_previousPosition = 0;
}

void MotorController::calculateVelocity(float dt) {
    // Read current encoder position
    long currentPosition = m_encoder->read();

    // Calculate position change
    long positionDelta = currentPosition - m_previousPosition;

    // Calculate velocity (counts per second)
    if (dt > 0.0f) {
        m_currentVelocity = static_cast<float>(positionDelta) / dt;
    } else {
        m_currentVelocity = 0.0f;
    }

    // Save for next iteration
    m_previousPosition = currentPosition;
}

void MotorController::applyMotorCommand(float pwm, float velocity) {
    // If PWM is very small, just stop the motor
    if (pwm < 1.0f) {
        m_motor->setmotor(_STOP);
        return;
    }

    // Clamp PWM to valid range
    if (pwm > 100.0f) {
        pwm = 100.0f;
    }

    // Convert to integer for motor library
    int pwmInt = static_cast<int>(pwm);

    // Determine direction based on target velocity sign
    if (velocity > 0.0f) {
        // Forward
        m_motor->setmotor(m_fwdDir, pwmInt);
    } else if (velocity < 0.0f) {
        // Backward
        m_motor->setmotor(m_backDir, pwmInt);
    } else {
        // Zero velocity target - stop
        m_motor->setmotor(_STOP);
    }
}
