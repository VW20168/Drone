#ifndef __NEO6M_H__
#define __NEO6M_H__


class GPS_NEO6M {
public:
    GPS_NEO6M(const std::string& serialPort, int baudRate);

    bool initialize();
    bool readData(double *latitude, double *longitude);
    void closePort();
    void setVerbose(bool b);

private:
    std::string serialPort_;
    int baudRate_;
    int fd_;
    bool verbose;
    
    double convertToDegrees(double raw_value);
};


#endif
