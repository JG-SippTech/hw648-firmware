#include "RobotMotion.h"
#include "config.h"

RobotMotion::RobotMotion(MotorController* motor1, MotorController* motor2,
                         MotorController* motor3, SpeedRamp* ramp)
    : m_motor1(motor1)
    , m_motor2(motor2)
    , m_motor3(motor3)
    , m_ramp(ramp)
    , m_targetSpeedPercent(0.0f)
    , m_currentSpeedPercent(0.0f)
    , m_direction(0)
    , m_individualControl(false)
    , m_maxVelocityCPS(MAX_VELOCITY_CPS)
{
}

void RobotMotion::begin() {
    // Initialize all motor controllers
    m_motor1->begin();
    m_motor2->begin();
    m_motor3->begin();

    // Reset speed ramp
    m_ramp->reset();

    m_targetSpeedPercent = 0.0f;
    m_currentSpeedPercent = 0.0f;
    m_direction = 0;
    m_individualControl = false;
}

void RobotMotion::moveForward(float speedPercent) {
    m_targetSpeedPercent = clampSpeed(speedPercent);
    m_direction = 1;  // Forward
    m_individualControl = false;

    // Set ramp target
    float targetVelocity = speedToVelocity(m_targetSpeedPercent);
    m_ramp->setTarget(targetVelocity);
}

void RobotMotion::moveBackward(float speedPercent) {
    m_targetSpeedPercent = clampSpeed(speedPercent);
    m_direction = -1;  // Backward
    m_individualControl = false;

    // Set ramp target
    float targetVelocity = speedToVelocity(m_targetSpeedPercent);
    m_ramp->setTarget(targetVelocity);
}

void RobotMotion::stop() {
    m_targetSpeedPercent = 0.0f;
    m_direction = 0;
    m_individualControl = false;

    // Ramp down to zero
    m_ramp->setTarget(0.0f);
}

void RobotMotion::emergencyStop() {
    // Immediate stop without ramping
    m_targetSpeedPercent = 0.0f;
    m_currentSpeedPercent = 0.0f;
    m_direction = 0;
    m_individualControl = false;

    // Reset ramp
    m_ramp->reset();

    // Stop all motors immediately
    m_motor1->stop();
    m_motor2->stop();
    m_motor3->stop();
}

bool RobotMotion::setMotor(int motorId, float speedPercent) {
    // Get motor controller
    MotorController* motor = getMotor(motorId);
    if (motor == nullptr) {
        return false;  // Invalid motor ID
    }

    // Enable individual control mode
    m_individualControl = true;

    // Clamp speed
    speedPercent = constrain(speedPercent, -100.0f, 100.0f);

    // Convert to velocity (sign determines direction)
    float targetVelocity = speedToVelocity(fabs(speedPercent));
    if (speedPercent < 0.0f) {
        targetVelocity = -targetVelocity;
    }

    // Set motor velocity directly (no ramping for individual control)
    motor->setVelocity(targetVelocity);

    return true;
}

void RobotMotion::update(float dt) {
    if (m_individualControl) {
        // Individual motor control - just update each motor's PID
        m_motor1->update(dt);
        m_motor2->update(dt);
        m_motor3->update(dt);
        return;
    }

    // Update speed ramp
    float rampedVelocity = m_ramp->update(dt);

    // Calculate current speed percentage from ramped velocity
    if (m_maxVelocityCPS > 0.0f) {
        m_currentSpeedPercent = (rampedVelocity / m_maxVelocityCPS) * 100.0f;
    } else {
        m_currentSpeedPercent = 0.0f;
    }

    // Apply direction
    float targetVelocity = rampedVelocity * m_direction;

    // Set all motors to same target velocity
    m_motor1->setVelocity(targetVelocity);
    m_motor2->setVelocity(targetVelocity);
    m_motor3->setVelocity(targetVelocity);

    // Update all motor PID controllers
    m_motor1->update(dt);
    m_motor2->update(dt);
    m_motor3->update(dt);
}

bool RobotMotion::isMoving() const {
    return (m_motor1->getTargetVelocity() != 0.0f) ||
           (m_motor2->getTargetVelocity() != 0.0f) ||
           (m_motor3->getTargetVelocity() != 0.0f);
}

MotorController* RobotMotion::getMotor(int motorId) {
    switch (motorId) {
        case 1: return m_motor1;
        case 2: return m_motor2;
        case 3: return m_motor3;
        default: return nullptr;
    }
}

void RobotMotion::resetEncoders() {
    m_motor1->resetPosition();
    m_motor2->resetPosition();
    m_motor3->resetPosition();
}

float RobotMotion::speedToVelocity(float speedPercent) {
    // Convert speed percentage (0-100) to velocity (counts/sec)
    return (speedPercent / 100.0f) * m_maxVelocityCPS;
}

float RobotMotion::clampSpeed(float speed) {
    if (speed < MIN_SPEED) return MIN_SPEED;
    if (speed > MAX_SPEED) return MAX_SPEED;
    return speed;
}
