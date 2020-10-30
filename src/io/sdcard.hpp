#ifndef _SDCARD_HPP_INCLUDED
#define _SDCARD_HPP_INCLUDED

#include "api.h"
#include "okapi/api.hpp"
#include <fstream>
#include "profiles.hpp"

// Various config macros
#define IS_RED_SIDE(config) ((config.autonSide & LocalStorage::AutonMode::RED) >> 1)
#define IS_TOP_SIDE(config) (config.autonSide & LocalStorage::AutonMode::TOP)
#define CONFIG_VERSION "v2020.1" // UPDATE THIS VALUE ON BREAKING CHANGE TO CONFIG

class LocalStorage {
private:
    okapi::Logger::LogLevel logLevel = okapi::Logger::LogLevel::debug;
    bool openConfFile(const char*, const char* mode = "a");
    void removeFile(const char *);
    FILE *confFileHandle, *logFileHandle = NULL;
    
public:
    enum AutonMode {
        BLUE_BOTTOM = 0b00,
        BLUE_TOP = 0b01,
        RED_BOTTOM = 0b10,
        RED_TOP = 0b11,
        TOP = 0b01, // For masking bits
        RED = 0b10  // For masking bits
    };
    struct RobotConfigs {
        bool driverSkills; // Driver skills mode
        int auton; // Auton mode
        AutonMode autonSide; // Side
        bool debugging; // Debug mode
        bool loggingEnable;
        double startingY; // Starting Y value in inches
    };
    LocalStorage();

    // This destructor is not called when the program exits and this object should
    // last the life of the program so usually this will go unused. Implemented
    // for consistency.
    ~LocalStorage();

	/// Read configurations from the SD card into robotConfigs.
    void readConfigs();

	/// Write robotConfigs to the the SD card.
    void writeConfigs();

    /**
	 * Set logger verbosity level.
     * Just reused log levels from okapi since there's no need to reimplement my own.
     *
     * \param level
     *        Verbosity level of the logger. Anything higher than this value is not
     *        logged when passed to the function.
	 */
    void setLogLevel(okapi::Logger::LogLevel);

    /**
	 * Log a message to either the SD card or remote console.
     *
     * \param message
     *        The message to log as a cstring (char pointer).
     * \param level
     *        The level to log the message at. Lower levels are higher priority.
	 */
    void log(const char*, okapi::Logger::LogLevel level = okapi::Logger::LogLevel::debug);
};
#endif /* _SDCARD_HPP_INCLUDED */
