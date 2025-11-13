#ifndef SPEED_RAMP_H
#define SPEED_RAMP_H

#include <Arduino.h>

/**
 * @brief Speed ramping class for smooth acceleration/deceleration
 *
 * Generates smooth velocity transitions to prevent mechanical shock
 * and wheel slip. Uses linear ramping with configurable ramp time.
 */
class SpeedRamp {
public:
    /**
     * @brief Construct a new Speed Ramp
     *
     * @param rampTimeMs Time to ramp from 0 to max velocity (milliseconds)
     * @param maxVelocity Maximum velocity value
     */
    SpeedRamp(float rampTimeMs, float maxVelocity);

    /**
     * @brief Set a new target velocity with ramping
     *
     * @param targetVelocity Desired final velocity
     */
    void setTarget(float targetVelocity);

    /**
     * @brief Update the ramp and get current velocity
     *
     * Call this periodically (e.g., every control loop iteration)
     *
     * @param dt Time delta since last update (seconds)
     * @return float Current ramped velocity
     */
    float update(float dt);

    /**
     * @brief Get current ramped velocity without updating
     *
     * @return float Current velocity value
     */
    float getCurrentVelocity() const { return m_currentVelocity; }

    /**
     * @brief Get target velocity
     *
     * @return float Target velocity value
     */
    float getTargetVelocity() const { return m_targetVelocity; }

    /**
     * @brief Check if ramp has reached target
     *
     * @return true If current velocity equals target velocity
     * @return false If still ramping
     */
    bool isAtTarget() const;

    /**
     * @brief Reset to zero velocity instantly
     *
     * Sets both current and target velocity to zero without ramping.
     * Useful for emergency stops.
     */
    void reset();

    /**
     * @brief Set ramp time
     *
     * @param rampTimeMs New ramp time in milliseconds
     */
    void setRampTime(float rampTimeMs);

private:
    float m_rampTimeMs;         // Ramp duration (milliseconds)
    float m_maxVelocity;        // Maximum velocity
    float m_currentVelocity;    // Current ramped velocity
    float m_targetVelocity;     // Target velocity
    float m_rampRate;           // Velocity change rate (units/sec)

    /**
     * @brief Calculate ramp rate based on current settings
     */
    void calculateRampRate();
};

#endif // SPEED_RAMP_H
