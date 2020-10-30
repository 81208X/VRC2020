#include "sdcard.hpp"
#include "io.hpp"

LocalStorage::LocalStorage() {
#ifndef DISABLE_LOGGING
    logFileHandle = fopen(DEBUG_PORT, "w");
    // Our custom debug device does not support COBS, disable it if active
	// pros::c::fdctl(fileno(debugConsole), SERCTL_DISABLE_COBS, NULL);
    pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
    // Enable blocking writes to serial (TODO: evaluate perf impact)
    // Disabled because it doesn't seem to be needed. If concurrency issues appear, file an issue and ping Sean.
    // pros::c::serctl(SERCTL_BLKWRITE, NULL);
    log("Initialized logger.");
#endif
}

// V5 doesn't seem to send a SIGTERM or anything to programs being exited so this destructor is never called in
// normal program flow. Included here anyways in case we want to reinit the localstorage object at some point in
// the future.
LocalStorage::~LocalStorage() {
#ifndef DISABLE_LOGGING
    log("Cleaning up logger.");
    fclose(logFileHandle);
#endif
}

// Yes this wrapper is a bit weird to have because it doesn't actually change much
// but since the V5 seems to frequently corrupt files on code exit I want to have this
// in order to try buffering tricks on the opened files later.
bool LocalStorage::openConfFile(const char *filename, const char *mode) {
    if(!pros::usd::is_installed()) // Config is located on SD card so make sure its inserted
        return false;
    confFileHandle = fopen(filename, mode);
    if(confFileHandle == NULL)
        return false;
    return true;
}


// Unused as of Aug 16th 2020. Consider removal?
void LocalStorage::removeFile(const char *filename) {
    std::remove(filename);
}

//overrides the current config
void LocalStorage::writeConfigs() {
    if(!openConfFile(CONFIGFILE, "w"))
        return;
    fprintf(confFileHandle,
            CONFIG_VERSION "\n"
            "Auton selection: %i\n"
            "Auton side (red/blue): %u\n"
            "Auton side (top/bottom): %u\n"
            "Driver skills mode: %u\n"
            "Debugging: %u\n"
            "Logging enable: %u",
            robotConfigs.auton,
            robotConfigs.autonSide&AutonMode::RED?1:0,
            robotConfigs.autonSide&AutonMode::TOP?1:0,
            robotConfigs.driverSkills,
            robotConfigs.debugging,
            robotConfigs.loggingEnable
            );
    fclose(confFileHandle);
}

void LocalStorage::log(const char *input, okapi::Logger::LogLevel msgLevel) {
    // TODO: better pretty print for log timestamp
    // Thinking output similar to dmesg
#ifdef DISABLE_LOGGING
    return;
#endif
    if(msgLevel <= logLevel && logFileHandle != NULL && robotConfigs.loggingEnable)
        fprintf(logFileHandle, "%06lu | %s\n", (long unsigned int)pros::millis(), input);
}

void LocalStorage::setLogLevel(okapi::Logger::LogLevel msgLevel) {
    logLevel = msgLevel;
}

void LocalStorage::readConfigs() {
    if(!openConfFile(CONFIGFILE, "r")) // Check for a valid config file being present
        writeConfigs();                // Handle is closed in write so no need to close it
    else {
        fclose(confFileHandle);        // Close handle just in case, shouldn't be needed
        std::string input, value;
        std::ifstream configFile(CONFIGFILE); // Open a stream of the config file
        std::size_t valuePos;
        // Get first line (config version header)
        getline(configFile, input);
        if(input != CONFIG_VERSION) { // If it doesn't match the one this code was built for
            writeConfigs();           // Write new config
            return;
        }
        while(getline(configFile, input)) {
            valuePos = input.find(": ");
            value = input.substr(valuePos + 2); // Get value to the right of colon
            input = input.substr(0, valuePos); // Get string to left of colon
            // Bunch of if statements for possible values
            if(input == "Auton selection")
                robotConfigs.auton = std::stoi(value);
            else if(input == "Auton side (red/blue)") {
                if(std::stoi(value)) robotConfigs.autonSide = (AutonMode)(robotConfigs.autonSide|AutonMode::RED);
                else robotConfigs.autonSide = (AutonMode)(robotConfigs.autonSide & 0b01);
            }
            else if(input == "Auton side (top/bottom)") {
                if(std::stoi(value)) robotConfigs.autonSide = (AutonMode)(robotConfigs.autonSide|AutonMode::TOP);
                else robotConfigs.autonSide = (AutonMode)(robotConfigs.autonSide & 0b10);
            }
            else if(input == "Driver skills mode")
                robotConfigs.driverSkills = (bool)std::stoi(value);
            else if(input == "Debugging")
                robotConfigs.debugging = (bool)std::stoi(value);
            else if(input == "Logging enable")
                robotConfigs.loggingEnable = (bool)std::stoi(value);
        }
        configFile.close(); // Close handle
    }
    // Force disable debug for comp
#ifdef FORCE_COMPETITION
    robotConfigs.debugging = false;
#endif
}
