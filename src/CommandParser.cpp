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
{
    memset(m_cmdBuffer, 0, sizeof(m_cmdBuffer));
}

void CommandParser::begin() {
    Serial.begin(m_baudRate);
    while (!Serial && millis() < 3000) {
        // Wait for serial connection (max 3 seconds)
    }

    Serial.println("\n=================================");
    Serial.println("  PIPE CRAWLER ROBOT CONTROLLER");
    Serial.println("=================================");
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

    // Check watchdog timeout
    if (ENABLE_WATCHDOG && isWatchdogTimeout()) {
        Serial.println("\n[WATCHDOG] Command timeout - stopping robot");
        m_robot->emergencyStop();
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
    else if (cmd[0] == 'M' && (cmd[1] >= '1' && cmd[1] <= '3')) {
        // Individual motor control: M1 FWD 50
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
    Serial.println("\n--- ROBOT STATUS ---");

    // Get motor controllers
    MotorController* m1 = m_robot->getMotor(1);
    MotorController* m2 = m_robot->getMotor(2);
    MotorController* m3 = m_robot->getMotor(3);

    Serial.print("Speed: ");
    Serial.print(m_robot->getCurrentSpeed(), 1);
    Serial.print("% (target: ");
    Serial.print(m_robot->getTargetSpeed(), 1);
    Serial.println("%)");

    Serial.print("Motor 1 - Pos: ");
    Serial.print(m1->getPosition());
    Serial.print("  Vel: ");
    Serial.print(m1->getVelocity(), 1);
    Serial.print("  PWM: ");
    Serial.println(m1->getPWM(), 1);

    Serial.print("Motor 2 - Pos: ");
    Serial.print(m2->getPosition());
    Serial.print("  Vel: ");
    Serial.print(m2->getVelocity(), 1);
    Serial.print("  PWM: ");
    Serial.println(m2->getPWM(), 1);

    Serial.print("Motor 3 - Pos: ");
    Serial.print(m3->getPosition());
    Serial.print("  Vel: ");
    Serial.print(m3->getVelocity(), 1);
    Serial.print("  PWM: ");
    Serial.println(m3->getPWM(), 1);

    Serial.println("--------------------\n");
}

void CommandParser::resetWatchdog() {
    m_lastCommandTime = millis();
}

bool CommandParser::isWatchdogTimeout() {
    return (millis() - m_lastCommandTime) > WATCHDOG_TIMEOUT_MS;
}

void CommandParser::printHelp() {
    Serial.println("\n===== AVAILABLE COMMANDS =====");
    Serial.println("FORWARD <speed>     - Move forward (speed: 0-100)");
    Serial.println("BACKWARD <speed>    - Move backward (speed: 0-100)");
    Serial.println("STOP                - Stop all motors (with ramping)");
    Serial.println("ESTOP               - Emergency stop (immediate)");
    Serial.println("M<id> <dir> <speed> - Control motor (id:1-3, dir:FWD/BACK)");
    Serial.println("                      Example: M1 FWD 50");
    Serial.println("STATUS              - Print robot status");
    Serial.println("RESET               - Reset encoder positions");
    Serial.println("HELP                - Show this message");
    Serial.println("==============================\n");
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
