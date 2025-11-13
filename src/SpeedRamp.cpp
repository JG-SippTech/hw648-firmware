#include "SpeedRamp.h"

SpeedRamp::SpeedRamp(float rampTimeMs, float maxVelocity)
    : m_rampTimeMs(rampTimeMs)
    , m_maxVelocity(maxVelocity)
    , m_currentVelocity(0.0f)
    , m_targetVelocity(0.0f)
    , m_rampRate(0.0f)
{
    calculateRampRate();
}

void SpeedRamp::setTarget(float targetVelocity) {
    m_targetVelocity = targetVelocity;
    calculateRampRate();
}

float SpeedRamp::update(float dt) {
    // If already at target, no need to ramp
    if (isAtTarget()) {
        return m_currentVelocity;
    }

    // Calculate velocity change for this time step
    float velocityChange = m_rampRate * dt;

    // Ramp toward target
    if (m_currentVelocity < m_targetVelocity) {
        // Ramping up
        m_currentVelocity += velocityChange;
        if (m_currentVelocity > m_targetVelocity) {
            m_currentVelocity = m_targetVelocity;  // Don't overshoot
        }
    } else {
        // Ramping down
        m_currentVelocity -= velocityChange;
        if (m_currentVelocity < m_targetVelocity) {
            m_currentVelocity = m_targetVelocity;  // Don't overshoot
        }
    }

    return m_currentVelocity;
}

bool SpeedRamp::isAtTarget() const {
    // Use small epsilon for floating point comparison
    const float epsilon = 0.1f;
    return fabs(m_currentVelocity - m_targetVelocity) < epsilon;
}

void SpeedRamp::reset() {
    m_currentVelocity = 0.0f;
    m_targetVelocity = 0.0f;
}

void SpeedRamp::setRampTime(float rampTimeMs) {
    m_rampTimeMs = rampTimeMs;
    calculateRampRate();
}

void SpeedRamp::calculateRampRate() {
    // Calculate rate of change (units per second)
    // This is how fast we can change velocity to reach target in rampTimeMs
    if (m_rampTimeMs > 0.0f) {
        // Convert milliseconds to seconds
        float rampTimeSec = m_rampTimeMs / 1000.0f;
        // Calculate max rate: full range change in ramp time
        m_rampRate = m_maxVelocity / rampTimeSec;
    } else {
        // Instant change if no ramp time
        m_rampRate = m_maxVelocity * 1000.0f;  // Very large rate
    }
}
