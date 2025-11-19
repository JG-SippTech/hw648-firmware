#include "CommandParser.h"
#include "config.h"
#include <string.h>
#include <ctype.h>

CommandParser::CommandParser(RobotMotion* robot, unsigned long baudRate)
    : m_robot(robot)
    , m_baudRate(baudRate)
    , m_cmdIndex(0)
    , m_lastCommandTime(0)
    , m_lastStatusTime(0)
    , m_nudgeActive(false)
    , m_nudgeMotorId(0)
    , m_nudgeTargetPosition(0)
    , m_nudgeStartTime(0)
{
    memset(m_cmdBuffer, 0, sizeof(m_cmdBuffer));
}

void CommandParser::begin() {
    Serial.begin(m_baudRate);
    while (!Serial && millis() < 3000) {
        // Wait for serial connection (max 3 seconds)
    }

    Serial.println("\n====================================");
    Serial.println("  DUAL CRAWLER ROBOT CONTROLLER");
    Serial.println("====================================");
    Serial.println("Type HELP for available commands\n");

    m_lastCommandTime = millis();
    m_lastStatusTime = millis();
}

void CommandParser::update() {
    // Check for incoming serial data
    while (Serial.available() > 0) {
        char c = Serial.read();

        // Handle newline/carriage return (command complete)
        if (c == '\n' || c == '\r') {
            if (m_cmdIndex > 0) {
                m_cmdBuffer[m_cmdIndex] = '\0';  // Null terminate
                parseCommand(m_cmdBuffer);
                m_cmdIndex = 0;  // Reset buffer
            }
        }
        // Handle backspace
        else if (c == '\b' || c == 127) {
            if (m_cmdIndex > 0) {
                m_cmdIndex--;
                Serial.print("\b \b");  // Erase character on terminal
            }
        }
        // Add character to buffer
        else if (m_cmdIndex < (int)sizeof(m_cmdBuffer) - 1) {
            m_cmdBuffer[m_cmdIndex++] = c;
            Serial.print(c);  // Echo character
        }
    }

    // Periodic status printing (if enabled)
    #if DEBUG_ENABLED
    if (millis() - m_lastStatusTime >= STATUS_REPORT_MS) {
        printStatus();
        m_lastStatusTime = millis();
    }
    #endif

    // Update NUDGE command execution
    #if ENABLE_NUDGE_COMMAND
    if (m_nudgeActive) {
        MotorController* motor = m_robot->getMotor(m_nudgeMotorId);
        if (motor) {
            long currentPos = motor->getPosition();
            long targetPos = m_nudgeTargetPosition;
            long error = targetPos - currentPos;

            // Check if target reached (within 10 counts tolerance)
            if (abs(error) < 10) {
                // Target reached - stop motor
                motor->setVelocity(0.0f);
                m_nudgeActive = false;
                Serial.print("NUDGE complete - Motor ");
                Serial.print(m_nudgeMotorId);
                Serial.print(" at position ");
                Serial.println(currentPos);
            }
            // Check for timeout
            else if (millis() - m_nudgeStartTime > NUDGE_TIMEOUT_MS) {
                // Timeout - stop motor
                motor->setVelocity(0.0f);
                m_nudgeActive = false;
                Serial.println("[NUDGE] Timeout - stopping motor");
            }
            // Continue nudging
            // (Velocity is already set, just let it run)
        } else {
            // Motor not available - cancel NUDGE
            m_nudgeActive = false;
        }
    }
    #endif

    // Check watchdog timeout
    if (ENABLE_WATCHDOG && isWatchdogTimeout()) {
        Serial.println("\n[WATCHDOG] Command timeout - stopping robot");
        m_robot->emergencyStop();
        m_nudgeActive = false;  // Cancel any active NUDGE
        resetWatchdog();
    }
}

void CommandParser::parseCommand(const char* command) {
    // Make a working copy
    char cmd[64];
    strncpy(cmd, command, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';

    // Trim whitespace and convert to uppercase
    trim(cmd);
    toUpperCase(cmd);

    #if DEBUG_COMMANDS
    Serial.print("\n[CMD] ");
    Serial.println(cmd);
    #endif

    // Reset watchdog on any command
    resetWatchdog();

    // Parse command
    if (strncmp(cmd, "FORWARD", 7) == 0) {
        float speed = parseFloat(cmd + 7);
        m_robot->moveForward(speed);
        Serial.print("OK - Moving forward at ");
        Serial.print(speed);
        Serial.println("%");
    }
    else if (strncmp(cmd, "BACKWARD", 8) == 0) {
        float speed = parseFloat(cmd + 8);
        m_robot->moveBackward(speed);
        Serial.print("OK - Moving backward at ");
        Serial.print(speed);
        Serial.println("%");
    }
    else if (strcmp(cmd, "STOP") == 0) {
        m_robot->stop();
        Serial.println("OK - Stopping");
    }
    else if (strcmp(cmd, "ESTOP") == 0) {
        m_robot->emergencyStop();
        Serial.println("OK - EMERGENCY STOP");
    }
    else if (cmd[0] == 'M' && (cmd[1] >= '1' && cmd[1] <= '6')) {
        // Individual motor control: M1 FWD 50, M2 BACK 30, etc.
        int motorId = cmd[1] - '0';
        char* dirStr = cmd + 3;

        // Find direction
        float speed = 0.0f;
        bool forward = true;

        if (strncmp(dirStr, "FWD", 3) == 0 || strncmp(dirStr, "FORWARD", 7) == 0) {
            forward = true;
            // Find speed value after direction
            char* speedStr = strchr(dirStr, ' ');
            if (speedStr != nullptr) {
                speed = parseFloat(speedStr);
            }
        }
        else if (strncmp(dirStr, "BACK", 4) == 0 || strncmp(dirStr, "BACKWARD", 8) == 0) {
            forward = false;
            char* speedStr = strchr(dirStr, ' ');
            if (speedStr != nullptr) {
                speed = parseFloat(speedStr);
            }
        }

        // Apply direction sign
        if (!forward) {
            speed = -speed;
        }

        if (m_robot->setMotor(motorId, speed)) {
            Serial.print("OK - Motor ");
            Serial.print(motorId);
            Serial.print(" set to ");
            Serial.print(speed);
            Serial.println("%");
        } else {
            Serial.println("ERROR - Invalid motor ID");
        }
    }
    else if (strcmp(cmd, "STATUS") == 0) {
        printStatus();
    }
    else if (strcmp(cmd, "RESET") == 0) {
        m_robot->resetEncoders();
        Serial.println("OK - Encoders reset");
    }
    #if ENABLE_NUDGE_COMMAND
    else if (strncmp(cmd, "NUDGE", 5) == 0) {
        // NUDGE command: NUDGE M<id> <+/- counts>
        // Example: NUDGE M2 +50, NUDGE M5 -30
        char* arg = cmd + 5;
        while (*arg == ' ') arg++;  // Skip whitespace

        // Parse motor ID
        if (*arg == 'M' && (arg[1] >= '1' && arg[1] <= '6')) {
            int motorId = arg[1] - '0';
            arg += 2;  // Skip 'M' and digit
            while (*arg == ' ') arg++;  // Skip whitespace

            // Parse offset (can be positive or negative)
            int offset = parseInt(arg);

            // Validate offset
            if (abs(offset) > NUDGE_MAX_DISTANCE) {
                Serial.print("ERROR - NUDGE distance limited to +/-");
                Serial.println(NUDGE_MAX_DISTANCE);
                return;
            }

            // Get motor controller
            MotorController* motor = m_robot->getMotor(motorId);
            if (!motor) {
                Serial.println("ERROR - Invalid motor ID");
                return;
            }

            // Cancel any previous NUDGE
            if (m_nudgeActive) {
                MotorController* prevMotor = m_robot->getMotor(m_nudgeMotorId);
                if (prevMotor) prevMotor->setVelocity(0.0f);
            }

            // Set target position
            long currentPos = motor->getPosition();
            m_nudgeTargetPosition = currentPos + offset;
            m_nudgeMotorId = motorId;
            m_nudgeStartTime = millis();
            m_nudgeActive = true;

            // Set motor velocity (low speed for precision)
            float nudgeSpeed = NUDGE_SPEED_PERCENT;
            if (offset < 0) {
                nudgeSpeed = -nudgeSpeed;  // Negative for backward
            }
            float nudgeVelocity = (nudgeSpeed / 100.0f) * MAX_VELOCITY_CPS;
            motor->setVelocity(nudgeVelocity);

            Serial.print("NUDGE started - Motor ");
            Serial.print(motorId);
            Serial.print(" moving ");
            Serial.print(offset);
            Serial.print(" counts (");
            Serial.print(currentPos);
            Serial.print(" → ");
            Serial.print(m_nudgeTargetPosition);
            Serial.println(")");
        } else {
            Serial.println("ERROR - Invalid NUDGE syntax. Use: NUDGE M<id> <+/- counts>");
        }
    }
    #endif
    else if (strncmp(cmd, "SELECT", 6) == 0) {
        // Parse crawler selection: SELECT 1, SELECT 2, SELECT BOTH
        char* arg = cmd + 6;
        while (*arg == ' ') arg++;  // Skip whitespace

        CrawlerSelection selection;
        bool valid = false;

        if (strcmp(arg, "1") == 0) {
            selection = SELECT_CRAWLER_1;
            valid = true;
        } else if (strcmp(arg, "2") == 0) {
            selection = SELECT_CRAWLER_2;
            valid = true;
        } else if (strcmp(arg, "BOTH") == 0 || strcmp(arg, "ALL") == 0) {
            selection = SELECT_BOTH;
            valid = true;
        }

        if (valid) {
            if (m_robot->selectCrawler(selection)) {
                Serial.print("OK - Selected ");
                if (selection == SELECT_CRAWLER_1) {
                    Serial.println("Crawler 1 (motors 1-3)");
                } else if (selection == SELECT_CRAWLER_2) {
                    Serial.println("Crawler 2 (motors 4-6)");
                } else {
                    Serial.println("BOTH crawlers");
                }
            } else {
                Serial.println("ERROR - Selected crawler not available");
            }
        } else {
            Serial.println("ERROR - Invalid selection. Use: SELECT 1, SELECT 2, or SELECT BOTH");
        }
    }
    else if (strcmp(cmd, "HELP") == 0) {
        printHelp();
    }
    else if (strlen(cmd) > 0) {
        Serial.print("ERROR - Unknown command: ");
        Serial.println(cmd);
        Serial.println("Type HELP for available commands");
    }
}

void CommandParser::printStatus() {
    Serial.println("\n--- DUAL CRAWLER STATUS ---");

    // Show current selection
    Serial.print("Selected: ");
    CrawlerSelection selection = m_robot->getCrawlerSelection();
    if (selection == SELECT_CRAWLER_1) {
        Serial.println("Crawler 1 only");
    } else if (selection == SELECT_CRAWLER_2) {
        Serial.println("Crawler 2 only");
    } else {
        Serial.println("BOTH crawlers");
    }

    Serial.print("Speed: ");
    Serial.print(m_robot->getCurrentSpeed(), 1);
    Serial.print("% (Target: ");
    Serial.print(m_robot->getTargetSpeed(), 1);
    Serial.println("%)");

    // Show Crawler 1 status
    if (m_robot->isCrawler1Active()) {
        Serial.println("\nCRAWLER 1 (Motors 1-3):");

        for (int i = 1; i <= 3; i++) {
            MotorController* motor = m_robot->getMotor(i);
            if (motor) {
                Serial.print("  Motor ");
                Serial.print(i);
                Serial.print(" - Pos: ");
                Serial.print(motor->getPosition());
                Serial.print("  Vel: ");
                Serial.print(motor->getVelocity(), 1);
                Serial.print(" c/s  PWM: ");
                Serial.print(motor->getPWM(), 1);
                Serial.println("%");
            }
        }
    }

    // Show Crawler 2 status
    if (m_robot->isCrawler2Active()) {
        Serial.println("\nCRAWLER 2 (Motors 4-6):");

        for (int i = 4; i <= 6; i++) {
            MotorController* motor = m_robot->getMotor(i);
            if (motor) {
                Serial.print("  Motor ");
                Serial.print(i);
                Serial.print(" - Pos: ");
                Serial.print(motor->getPosition());
                Serial.print("  Vel: ");
                Serial.print(motor->getVelocity(), 1);
                Serial.print(" c/s  PWM: ");
                Serial.print(motor->getPWM(), 1);
                Serial.println("%");
            }
        }
    }

    // Position synchronization status
    Serial.print("\nSync - Avg Pos: ");
    Serial.print(m_robot->getAveragePosition(), 0);
    Serial.print("  Max Error: ");
    Serial.print(m_robot->getMaxPositionError(), 0);
    Serial.println(" counts");

    // Active crawler count
    Serial.print("Active Crawlers: ");
    Serial.print(m_robot->getActiveCrawlers());
    Serial.println("/2");

    Serial.println("---------------------------\n");
}

void CommandParser::resetWatchdog() {
    m_lastCommandTime = millis();
}

bool CommandParser::isWatchdogTimeout() {
    return (millis() - m_lastCommandTime) > WATCHDOG_TIMEOUT_MS;
}

void CommandParser::printHelp() {
    Serial.println("\n===== DUAL CRAWLER COMMANDS =====");
    Serial.println("SELECT 1            - Control only Crawler 1 (motors 1-3)");
    Serial.println("SELECT 2            - Control only Crawler 2 (motors 4-6)");
    Serial.println("SELECT BOTH         - Control both crawlers (default)");
    Serial.println("FORWARD <speed>     - Move selected crawler(s) forward (0-100)");
    Serial.println("BACKWARD <speed>    - Move selected crawler(s) backward (0-100)");
    Serial.println("STOP                - Stop selected crawler(s) (with ramping)");
    Serial.println("ESTOP               - Emergency stop selected crawler(s)");
    Serial.println("M<id> <dir> <speed> - Control individual motor (id:1-6)");
    Serial.println("                      Crawler 1: Motors 1-3");
    Serial.println("                      Crawler 2: Motors 4-6");
    Serial.println("                      Example: M1 FWD 50, M5 BACK 30");
    #if ENABLE_NUDGE_COMMAND
    Serial.println("NUDGE M<id> <count> - Move motor by exact encoder counts");
    Serial.println("                      Example: NUDGE M2 +50, NUDGE M5 -30");
    #endif
    Serial.println("STATUS              - Print detailed crawler status");
    Serial.println("RESET               - Reset encoder positions");
    Serial.println("HELP                - Show this message");
    Serial.println("==================================\n");
}

float CommandParser::parseFloat(const char* str) {
    // Skip leading whitespace
    while (*str == ' ' || *str == '\t') {
        str++;
    }

    return atof(str);
}

int CommandParser::parseInt(const char* str) {
    // Skip leading whitespace
    while (*str == ' ' || *str == '\t') {
        str++;
    }

    return atoi(str);
}

void CommandParser::trim(char* str) {
    // Trim leading whitespace
    char* start = str;
    while (*start == ' ' || *start == '\t') {
        start++;
    }

    // Trim trailing whitespace
    char* end = start + strlen(start) - 1;
    while (end > start && (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')) {
        end--;
    }
    *(end + 1) = '\0';

    // Move trimmed string to beginning
    if (start != str) {
        memmove(str, start, strlen(start) + 1);
    }
}

void CommandParser::toUpperCase(char* str) {
    while (*str) {
        *str = toupper(*str);
        str++;
    }
}
