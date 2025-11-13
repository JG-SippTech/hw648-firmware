#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>
#include "RobotMotion.h"

/**
 * @brief Serial command parser for robot control
 *
 * Parses ASCII commands from serial port and executes motion commands.
 * Designed to be easily replaceable with ROS2 subscriber in the future.
 *
 * Supported commands:
 * - FORWARD <speed>     : Move forward at speed (0-100)
 * - BACKWARD <speed>    : Move backward at speed (0-100)
 * - STOP                : Stop all motors
 * - ESTOP               : Emergency stop
 * - M<id> <dir> <speed> : Control individual motor (id=1-3, dir=FWD/BACK)
 * - STATUS              : Print robot status
 * - RESET               : Reset encoders to zero
 * - HELP                : Show available commands
 */
class CommandParser {
public:
    /**
     * @brief Construct a new Command Parser
     *
     * @param robot Pointer to RobotMotion controller
     * @param baudRate Serial communication baud rate
     */
    CommandParser(RobotMotion* robot, unsigned long baudRate = 115200);

    /**
     * @brief Initialize command parser
     *
     * Opens serial port and prints welcome message.
     * Call in setup().
     */
    void begin();

    /**
     * @brief Process incoming serial commands
     *
     * Call this regularly in loop() to check for new commands.
     */
    void update();

    /**
     * @brief Print robot status to serial
     *
     * Displays encoder positions, velocities, and motor states.
     */
    void printStatus();

    /**
     * @brief Reset watchdog timer
     *
     * Call whenever a valid command is received.
     */
    void resetWatchdog();

    /**
     * @brief Check if watchdog has timed out
     *
     * @return true If timeout occurred (should trigger safety stop)
     * @return false If still within timeout window
     */
    bool isWatchdogTimeout();

private:
    RobotMotion* m_robot;
    unsigned long m_baudRate;

    char m_cmdBuffer[64];           // Command buffer
    int m_cmdIndex;                 // Current buffer position

    unsigned long m_lastCommandTime; // Timestamp of last command (ms)
    unsigned long m_lastStatusTime;  // Timestamp of last status print (ms)

    /**
     * @brief Parse and execute a complete command
     *
     * @param command Null-terminated command string
     */
    void parseCommand(const char* command);

    /**
     * @brief Print help message
     */
    void printHelp();

    /**
     * @brief Parse float value from string
     *
     * @param str String containing number
     * @return float Parsed value, or 0.0 if invalid
     */
    float parseFloat(const char* str);

    /**
     * @brief Parse integer value from string
     *
     * @param str String containing number
     * @return int Parsed value, or 0 if invalid
     */
    int parseInt(const char* str);

    /**
     * @brief Trim whitespace from string
     *
     * @param str String to trim (modified in place)
     */
    void trim(char* str);

    /**
     * @brief Convert string to uppercase
     *
     * @param str String to convert (modified in place)
     */
    void toUpperCase(char* str);
};

#endif // COMMAND_PARSER_H
