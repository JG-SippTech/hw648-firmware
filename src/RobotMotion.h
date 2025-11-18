#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H

#include <Arduino.h>
#include "MotorController.h"
#include "SpeedRamp.h"

/**
 * @brief Crawler selection mode
 */
enum CrawlerSelection {
    SELECT_CRAWLER_1 = 1,    // Control only Crawler 1 (motors 1-3)
    SELECT_CRAWLER_2 = 2,    // Control only Crawler 2 (motors 4-6)
    SELECT_BOTH = 3          // Control both crawlers (default)
};

/**
 * @brief Dual crawler motion coordinator
 *
 * Manages coordinated motion of up to 6 motors across 2 independent
 * 3-wheeled crawlers. Supports auto-detection and graceful degradation.
 * Crawlers can operate independently or in synchronized motion.
 */
class RobotMotion {
public:
    /**
     * @brief Construct a new Robot Motion controller for dual crawlers
     *
     * @param motor1 Pointer to Motor 1 controller (Crawler 1)
     * @param motor2 Pointer to Motor 2 controller (Crawler 1)
     * @param motor3 Pointer to Motor 3 controller (Crawler 1)
     * @param motor4 Pointer to Motor 4 controller (Crawler 2, or nullptr if not present)
     * @param motor5 Pointer to Motor 5 controller (Crawler 2, or nullptr if not present)
     * @param motor6 Pointer to Motor 6 controller (Crawler 2, or nullptr if not present)
     * @param ramp Pointer to speed ramp controller
     */
    RobotMotion(MotorController* motor1, MotorController* motor2,
                MotorController* motor3, MotorController* motor4,
                MotorController* motor5, MotorController* motor6,
                SpeedRamp* ramp);

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
     * @param motorId Motor number (1-6)
     * @param speedPercent Speed percentage (-100 to +100)
     *                     Positive = forward, negative = backward
     * @return true Command accepted
     * @return false Invalid motor ID or motor not active
     */
    bool setMotor(int motorId, float speedPercent);

    /**
     * @brief Configure active crawlers
     *
     * Sets which crawlers are available for operation based on shield detection.
     *
     * @param crawler1 True if Crawler 1 (motors 1-3) is available
     * @param crawler2 True if Crawler 2 (motors 4-6) is available
     */
    void setActiveCrawlers(bool crawler1, bool crawler2);

    /**
     * @brief Select which crawler(s) to control
     *
     * Sets which crawler(s) respond to movement commands (FORWARD, BACKWARD, STOP).
     * Selection persists until changed. Individual motor commands (M1-M6) always work.
     *
     * @param selection Crawler selection mode (SELECT_CRAWLER_1, SELECT_CRAWLER_2, SELECT_BOTH)
     * @return true Selection accepted
     * @return false Invalid selection (crawler not available)
     */
    bool selectCrawler(CrawlerSelection selection);

    /**
     * @brief Get current crawler selection
     *
     * @return CrawlerSelection Currently selected crawler(s)
     */
    CrawlerSelection getCrawlerSelection() const { return m_crawlerSelection; }

    /**
     * @brief Get number of active crawlers
     *
     * @return int Number of crawlers available (0, 1, or 2)
     */
    int getActiveCrawlers() const { return m_activeCrawlers; }

    /**
     * @brief Check if Crawler 1 is active
     *
     * @return true Crawler 1 (motors 1-3) is operational
     */
    bool isCrawler1Active() const { return m_crawler1Active; }

    /**
     * @brief Check if Crawler 2 is active
     *
     * @return true Crawler 2 (motors 4-6) is operational
     */
    bool isCrawler2Active() const { return m_crawler2Active; }

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
     * @param motorId Motor number (1-6)
     * @return MotorController* Pointer to motor controller, or nullptr if invalid/inactive
     */
    MotorController* getMotor(int motorId);

    /**
     * @brief Reset all encoder positions to zero
     */
    void resetEncoders();

    /**
     * @brief Get maximum position error across all motors
     *
     * @return float Maximum deviation from average position (encoder counts)
     */
    float getMaxPositionError() const { return m_maxPositionError; }

    /**
     * @brief Get average encoder position across all motors
     *
     * @return float Average position in encoder counts
     */
    float getAveragePosition() const { return m_averagePosition; }

private:
    // Motor controllers (Crawler 1: motors 1-3, Crawler 2: motors 4-6)
    MotorController* m_motor1;
    MotorController* m_motor2;
    MotorController* m_motor3;
    MotorController* m_motor4;
    MotorController* m_motor5;
    MotorController* m_motor6;
    SpeedRamp* m_ramp;

    // Crawler status
    bool m_crawler1Active;         // Crawler 1 (motors 1-3) operational
    bool m_crawler2Active;         // Crawler 2 (motors 4-6) operational
    int m_activeCrawlers;          // Number of active crawlers (0, 1, or 2)
    CrawlerSelection m_crawlerSelection;  // Which crawler(s) to control

    // Motion state
    float m_targetSpeedPercent;    // Target speed (0-100)
    float m_currentSpeedPercent;   // Current ramped speed (0-100)
    int m_direction;               // 1 = forward, -1 = backward, 0 = stopped
    bool m_individualControl;      // True if individual motor control is active

    float m_maxVelocityCPS;        // Maximum velocity in counts/sec at 100%

    // Position synchronization state (per crawler)
    float m_averagePosition;       // Average position across all motors
    float m_maxPositionError;      // Maximum deviation from average

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

    /**
     * @brief Calculate position synchronization corrections per crawler
     *
     * Computes velocity corrections for each motor based on position error
     * to keep motors synchronized within each crawler independently.
     *
     * @param correction1 Output: velocity correction for motor 1 (counts/sec)
     * @param correction2 Output: velocity correction for motor 2 (counts/sec)
     * @param correction3 Output: velocity correction for motor 3 (counts/sec)
     * @param correction4 Output: velocity correction for motor 4 (counts/sec)
     * @param correction5 Output: velocity correction for motor 5 (counts/sec)
     * @param correction6 Output: velocity correction for motor 6 (counts/sec)
     */
    void calculatePositionCorrections(float& correction1, float& correction2, float& correction3,
                                     float& correction4, float& correction5, float& correction6);
};

#endif // ROBOT_MOTION_H
