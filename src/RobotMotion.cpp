#include "RobotMotion.h"
#include "config.h"

RobotMotion::RobotMotion(MotorController* motor1, MotorController* motor2,
                         MotorController* motor3, MotorController* motor4,
                         MotorController* motor5, MotorController* motor6,
                         SpeedRamp* ramp)
    : m_motor1(motor1)
    , m_motor2(motor2)
    , m_motor3(motor3)
    , m_motor4(motor4)
    , m_motor5(motor5)
    , m_motor6(motor6)
    , m_ramp(ramp)
    , m_crawler1Active(true)   // Assume crawler 1 always present
    , m_crawler2Active(false)  // Will be set by setActiveCrawlers()
    , m_activeCrawlers(1)      // Start with just crawler 1
    , m_crawlerSelection(SELECT_BOTH)  // Default: control both crawlers
    , m_targetSpeedPercent(0.0f)
    , m_currentSpeedPercent(0.0f)
    , m_direction(0)
    , m_individualControl(false)
    , m_maxVelocityCPS(MAX_VELOCITY_CPS)
    , m_averagePosition(0.0f)
    , m_maxPositionError(0.0f)
{
}

void RobotMotion::setActiveCrawlers(bool crawler1, bool crawler2) {
    m_crawler1Active = crawler1;
    m_crawler2Active = crawler2;
    m_activeCrawlers = (crawler1 ? 1 : 0) + (crawler2 ? 1 : 0);
}

bool RobotMotion::selectCrawler(CrawlerSelection selection) {
    // Validate selection against available crawlers
    if (selection == SELECT_CRAWLER_1 && !m_crawler1Active) {
        return false;  // Crawler 1 not available
    }
    if (selection == SELECT_CRAWLER_2 && !m_crawler2Active) {
        return false;  // Crawler 2 not available
    }
    if (selection == SELECT_BOTH && (!m_crawler1Active && !m_crawler2Active)) {
        return false;  // No crawlers available
    }

    m_crawlerSelection = selection;
    return true;
}

void RobotMotion::begin() {
    // Initialize Crawler 1 motors (always present)
    if (m_crawler1Active) {
        m_motor1->begin();
        m_motor2->begin();
        m_motor3->begin();
    }

    // Initialize Crawler 2 motors (if present)
    if (m_crawler2Active) {
        if (m_motor4) m_motor4->begin();
        if (m_motor5) m_motor5->begin();
        if (m_motor6) m_motor6->begin();
    }

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

    // Stop selected crawler(s) immediately
    // Emergency stop respects selection - only stops selected crawler(s)
    bool controlCrawler1 = m_crawler1Active &&
                          (m_crawlerSelection == SELECT_CRAWLER_1 || m_crawlerSelection == SELECT_BOTH);
    bool controlCrawler2 = m_crawler2Active &&
                          (m_crawlerSelection == SELECT_CRAWLER_2 || m_crawlerSelection == SELECT_BOTH);

    if (controlCrawler1) {
        m_motor1->stop();
        m_motor2->stop();
        m_motor3->stop();
    }

    if (controlCrawler2) {
        if (m_motor4) m_motor4->stop();
        if (m_motor5) m_motor5->stop();
        if (m_motor6) m_motor6->stop();
    }
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
        if (m_crawler1Active) {
            m_motor1->update(dt);
            m_motor2->update(dt);
            m_motor3->update(dt);
        }
        if (m_crawler2Active) {
            if (m_motor4) m_motor4->update(dt);
            if (m_motor5) m_motor5->update(dt);
            if (m_motor6) m_motor6->update(dt);
        }
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
    float baseVelocity = rampedVelocity * m_direction;

    // Calculate position synchronization corrections per crawler (only when moving)
    float correction1 = 0.0f, correction2 = 0.0f, correction3 = 0.0f;
    float correction4 = 0.0f, correction5 = 0.0f, correction6 = 0.0f;

    // Only apply position sync when robot is actually moving
    // This prevents jittering when stopped with position misalignment
    if (fabs(baseVelocity) > 10.0f) {  // Small threshold to avoid jitter
        calculatePositionCorrections(correction1, correction2, correction3,
                                    correction4, correction5, correction6);
    } else {
        // Robot is stopped - update position tracking but don't apply corrections
        if (m_crawler1Active) {
            float avgPos1 = (m_motor1->getPosition() + m_motor2->getPosition() + m_motor3->getPosition()) / 3.0f;
            float err1 = fabs(avgPos1 - m_motor1->getPosition());
            float err2 = fabs(avgPos1 - m_motor2->getPosition());
            float err3 = fabs(avgPos1 - m_motor3->getPosition());
            m_maxPositionError = max(err1, max(err2, err3));
        }
    }

    // Apply base velocity + position correction to each SELECTED motor
    // Corrections adjust individual motor speeds to maintain alignment within each crawler
    // Respect crawler selection - only control selected crawler(s)
    bool controlCrawler1 = m_crawler1Active &&
                          (m_crawlerSelection == SELECT_CRAWLER_1 || m_crawlerSelection == SELECT_BOTH);
    bool controlCrawler2 = m_crawler2Active &&
                          (m_crawlerSelection == SELECT_CRAWLER_2 || m_crawlerSelection == SELECT_BOTH);

    if (controlCrawler1) {
        m_motor1->setVelocity(baseVelocity + correction1);
        m_motor2->setVelocity(baseVelocity + correction2);
        m_motor3->setVelocity(baseVelocity + correction3);
    }

    if (controlCrawler2) {
        if (m_motor4) m_motor4->setVelocity(baseVelocity + correction4);
        if (m_motor5) m_motor5->setVelocity(baseVelocity + correction5);
        if (m_motor6) m_motor6->setVelocity(baseVelocity + correction6);
    }

    // Update all active motor PID controllers
    if (m_crawler1Active) {
        m_motor1->update(dt);
        m_motor2->update(dt);
        m_motor3->update(dt);
    }

    if (m_crawler2Active) {
        if (m_motor4) m_motor4->update(dt);
        if (m_motor5) m_motor5->update(dt);
        if (m_motor6) m_motor6->update(dt);
    }

    // Check for slip on all motors (only when moving)
    if (fabs(baseVelocity) > 10.0f) {
        checkForSlip();
    }
}

bool RobotMotion::isMoving() const {
    bool moving = false;

    if (m_crawler1Active) {
        moving |= (m_motor1->getTargetVelocity() != 0.0f) ||
                  (m_motor2->getTargetVelocity() != 0.0f) ||
                  (m_motor3->getTargetVelocity() != 0.0f);
    }

    if (m_crawler2Active) {
        if (m_motor4) moving |= (m_motor4->getTargetVelocity() != 0.0f);
        if (m_motor5) moving |= (m_motor5->getTargetVelocity() != 0.0f);
        if (m_motor6) moving |= (m_motor6->getTargetVelocity() != 0.0f);
    }

    return moving;
}

MotorController* RobotMotion::getMotor(int motorId) {
    switch (motorId) {
        case 1: return (m_crawler1Active) ? m_motor1 : nullptr;
        case 2: return (m_crawler1Active) ? m_motor2 : nullptr;
        case 3: return (m_crawler1Active) ? m_motor3 : nullptr;
        case 4: return (m_crawler2Active) ? m_motor4 : nullptr;
        case 5: return (m_crawler2Active) ? m_motor5 : nullptr;
        case 6: return (m_crawler2Active) ? m_motor6 : nullptr;
        default: return nullptr;
    }
}

void RobotMotion::resetEncoders() {
    if (m_crawler1Active) {
        m_motor1->resetPosition();
        m_motor2->resetPosition();
        m_motor3->resetPosition();
    }

    if (m_crawler2Active) {
        if (m_motor4) m_motor4->resetPosition();
        if (m_motor5) m_motor5->resetPosition();
        if (m_motor6) m_motor6->resetPosition();
    }
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

void RobotMotion::calculatePositionCorrections(float& correction1, float& correction2, float& correction3,
                                              float& correction4, float& correction5, float& correction6) {
    #if ENABLE_POSITION_SYNC

    // Initialize all corrections to zero
    correction1 = correction2 = correction3 = 0.0f;
    correction4 = correction5 = correction6 = 0.0f;

    float maxError = 0.0f;

    // Crawler 1 position synchronization (motors 1, 2, 3)
    if (m_crawler1Active) {
        long pos1 = m_motor1->getPosition();
        long pos2 = m_motor2->getPosition();
        long pos3 = m_motor3->getPosition();

        // Calculate average position for Crawler 1
        float avgPos1 = (pos1 + pos2 + pos3) / 3.0f;

        // Calculate position errors (how far each motor is from average)
        float error1 = avgPos1 - pos1;  // Positive if motor is behind
        float error2 = avgPos1 - pos2;
        float error3 = avgPos1 - pos3;

        // Track maximum error
        maxError = max(fabs(error1), max(fabs(error2), fabs(error3)));

        // Calculate velocity corrections using proportional control
        correction1 = error1 * POSITION_SYNC_KP;
        correction2 = error2 * POSITION_SYNC_KP;
        correction3 = error3 * POSITION_SYNC_KP;

        // Clamp corrections
        correction1 = constrain(correction1, -MAX_SYNC_CORRECTION, MAX_SYNC_CORRECTION);
        correction2 = constrain(correction2, -MAX_SYNC_CORRECTION, MAX_SYNC_CORRECTION);
        correction3 = constrain(correction3, -MAX_SYNC_CORRECTION, MAX_SYNC_CORRECTION);

        m_averagePosition = avgPos1;
    }

    // Crawler 2 position synchronization (motors 4, 5, 6)
    // This is INDEPENDENT from Crawler 1 - each crawler syncs internally
    if (m_crawler2Active && m_motor4 && m_motor5 && m_motor6) {
        long pos4 = m_motor4->getPosition();
        long pos5 = m_motor5->getPosition();
        long pos6 = m_motor6->getPosition();

        // Calculate average position for Crawler 2
        float avgPos2 = (pos4 + pos5 + pos6) / 3.0f;

        // Calculate position errors
        float error4 = avgPos2 - pos4;
        float error5 = avgPos2 - pos5;
        float error6 = avgPos2 - pos6;

        // Track maximum error across both crawlers
        float maxError2 = max(fabs(error4), max(fabs(error5), fabs(error6)));
        maxError = max(maxError, maxError2);

        // Calculate velocity corrections
        correction4 = error4 * POSITION_SYNC_KP;
        correction5 = error5 * POSITION_SYNC_KP;
        correction6 = error6 * POSITION_SYNC_KP;

        // Clamp corrections
        correction4 = constrain(correction4, -MAX_SYNC_CORRECTION, MAX_SYNC_CORRECTION);
        correction5 = constrain(correction5, -MAX_SYNC_CORRECTION, MAX_SYNC_CORRECTION);
        correction6 = constrain(correction6, -MAX_SYNC_CORRECTION, MAX_SYNC_CORRECTION);
    }

    m_maxPositionError = maxError;

    #else

    // Position sync disabled
    correction1 = correction2 = correction3 = 0.0f;
    correction4 = correction5 = correction6 = 0.0f;
    m_averagePosition = 0.0f;
    m_maxPositionError = 0.0f;

    #endif
}

bool RobotMotion::checkForSlip() {
    #if ENABLE_SLIP_DETECTION

    bool slipDetected = false;
    int motorId = 0;
    int maxSlipLevel = 0;

    // Check all active motors for slip
    MotorController* motors[] = {m_motor1, m_motor2, m_motor3, m_motor4, m_motor5, m_motor6};

    for (int i = 0; i < 6; i++) {
        MotorController* motor = motors[i];

        // Skip inactive motors
        if (motor == nullptr) continue;
        if (i >= 3 && !m_crawler2Active) continue;  // Motors 4-6 only if crawler 2 active

        // Detect slip on this motor
        int slipLevel = motor->detectSlip();

        if (slipLevel > 0) {
            slipDetected = true;
            if (slipLevel > maxSlipLevel) {
                maxSlipLevel = slipLevel;
                motorId = i + 1;  // Motor IDs are 1-indexed
            }
        }
    }

    // Take action if slip detected
    if (slipDetected && maxSlipLevel > 0) {
        #if SLIP_ACTION == SLIP_ACTION_WARN
        // Print warning
        Serial.print("[SLIP] Motor ");
        Serial.print(motorId);
        if (maxSlipLevel == 1) {
            Serial.println(": Warning - possible slip starting");
        } else {
            Serial.println(": CRITICAL - severe slip detected!");
        }

        #elif SLIP_ACTION == SLIP_ACTION_STOP
        // Emergency stop
        Serial.print("[SLIP] Motor ");
        Serial.print(motorId);
        Serial.println(" slipping - EMERGENCY STOP");
        emergencyStop();

        #elif SLIP_ACTION == SLIP_ACTION_REDUCE
        // Reduce speed
        if (maxSlipLevel >= 2) {  // Only on critical slip
            Serial.print("[SLIP] Motor ");
            Serial.print(motorId);
            Serial.println(" slipping - reducing speed");

            // Reduce target speed by 30%
            m_targetSpeedPercent *= 0.7f;
            float targetVelocity = speedToVelocity(m_targetSpeedPercent);
            m_ramp->setTarget(targetVelocity);

            // Reset slip detection so it doesn't keep triggering
            for (int i = 0; i < 6; i++) {
                if (motors[i]) motors[i]->resetSlipDetection();
            }
        }
        #endif
    }

    return slipDetected;

    #else
    // Slip detection disabled
    return false;
    #endif
}
