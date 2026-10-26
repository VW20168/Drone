#include <iostream>
#include <string>
#include <cstring>
#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "neo6m.h"
#include "utils.h"

GPS_NEO6M::GPS_NEO6M(const std::string& serialPort, int baudRate) : serialPort_(serialPort), baudRate_(baudRate), fd_(-1) {
    verbose = false;
}

void GPS_NEO6M::setVerbose(bool b) {
    verbose = b;
}

bool GPS_NEO6M::initialize() {
    fd_ = open(serialPort_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        //std::cerr << "Error opening serial port: " << serialPort_ << std::endl;
        return false;
    }

    //https://stackoverflow.com/questions/17254923/raspberry-pi-uart-program-in-c-using-termios-receives-garbage-rx-and-tx-are-con
    termios serialConfig;
    tcgetattr(fd_, &serialConfig);
    cfsetispeed(&serialConfig, baudRate_);
    cfsetospeed(&serialConfig, baudRate_);

    serialConfig.c_iflag = 0;
    serialConfig.c_oflag = 0;
    serialConfig.c_lflag = 0;
    serialConfig.c_cflag = 0;
    serialConfig.c_cflag = B9600 | CS8 | CREAD;
    serialConfig.c_cc[VMIN] = 0;
    serialConfig.c_cc[VTIME]= 1; //VTIME: Timeout in deciseconds(1/10 second) for noncanonical read (TIME).

    if (tcsetattr(fd_, TCSANOW, &serialConfig) != 0) {
        //std::cerr << "Error setting serial port attributes." << std::endl;
        close(fd_);
        return false;
    }
    return true;
}

double GPS_NEO6M::convertToDegrees(double raw_value) {
    double decimal_value = raw_value/100.00;
    int degrees = int(decimal_value);
    double mm_mmmm = (decimal_value - degrees)/0.6;
    double position = degrees + mm_mmmm;
    return position;
}

bool GPS_NEO6M::readData(double *latitude, double *longitude) {
    if (fd_ < 0)
        return false;

    bool ret = false;
    char buffer[256];
    for (int i = 0; i < 20; ++i) {
        //$GPGGA,223018.00,,,,,0,00,99.99,,,,,,*6C
        ssize_t bytesRead = 0;        
        //read one line
        bool eol = false;
        char *ptr = buffer;
        ssize_t got = read(fd_, ptr, 1);
        while (got > 0) {
            if (bytesRead > 0 || *ptr == '$') {//filter out some random characters
                eol = (*ptr == '\n');
                bytesRead += got;
                ptr += got;
            }
            if (eol)
                break;
            else {
                got = read(fd_, ptr, 1);
            }
        }
        //parse the message
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';
            if (verbose)
                std::cout << buffer;
            char *ptr = strstr(buffer, "$GPGGA,");
            if (ptr == NULL)
                ptr = strstr(buffer, "$GNGGA,");            
            if (ptr) {
                std::vector<std::string> tokens = Utils::split(ptr, ",");
                if (tokens.size() >= 15 && !tokens[2].empty() && !tokens[4].empty()) {
                    try {
                        double _latitude = convertToDegrees(std::stod(tokens[2]));
                        double _longitude= convertToDegrees(std::stod(tokens[4]));
                        if (tokens[3] == "S")
                            _latitude = 0 - _latitude;
                        if (tokens[5] == "W")
                            _longitude = 0 - _longitude;

                        *latitude = _latitude;
                        *longitude= _longitude;

                        ret = true;
                    }
                    catch (const std::invalid_argument& e) {
                        std::cerr << "Error: " << e.what() << ",GPS: " << buffer << "$$$$" << std::endl;
                    }
               }
            }
        }
        else {
            break;
        }
    }
    return ret;
}

void GPS_NEO6M::closePort() {
    if(fd_ != -1) {
        close(fd_);
        fd_ = -1;
    }
}
/*
    ggaData.time = tokens[1];
    ggaData.latitude = std::stod(tokens[2]);
    ggaData.nsIndicator = tokens[3][0];
    ggaData.longitude = std::stod(tokens[4]);
    ggaData.ewIndicator = tokens[5][0];
    ggaData.fixQuality = std::stoi(tokens[6]);
    ggaData.satellites = std::stoi(tokens[7]);
    ggaData.hdop = std::stod(tokens[8]);
    ggaData.altitude = std::stod(tokens[9]);
    ggaData.altitudeUnits = tokens[10][0];
    ggaData.geoidSeparation = std::stod(tokens[11]);
    ggaData.geoidUnits = tokens[12][0];
    ggaData.ageOfData = tokens[13];
    ggaData.refStationId = tokens[14];
*/
