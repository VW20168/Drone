#include <fstream>
#include <math.h>
#include "drone.h"
#include "imu.h"
#include "mpu9250.h"

IMU::IMU (Configuration *config) {
    pMPU = new MPU9250(BUS_MPU9250); //bus #1

    anglePitch = angleRoll = angleYaw = READING_UNKNOWN;
    gyroPitchInput = gyroYawInput = 0;
    accX = accY = accZ = 0;
    gyroPitch = gyroRoll = gyroYaw = 0;

    compassX = compassY = compassZ = READING_UNKNOWN;
    compassXInput = compassYInput = compassZInput = 0;

    load(); //load IMU calibration data
}

IMU::~IMU() {
    delete pMPU;
}

void IMU::start() {
    float dta1, dta2, dta3;
    if (read(LOOP_TIMER_MS, &dta1, &dta2, &dta3, NULL))
        anglePitch=angleRoll=angleYaw=0;

    if (readCompass(&dta1, &dta2, &dta3))
        compassX = compassY = compassZ = 0;

    compassXInput = compassX;
    compassYInput = compassY;
    compassZInput = compassX;
}

//prepare to take off, alreay passed the preflight check
void IMU::prepare2takeoff() {
    anglePitch=angleRoll=angleYaw=0;

    //Magnetometer presents
    if (compassX < READING_UNKNOWN || compassY < READING_UNKNOWN || compassZ < READING_UNKNOWN) {
        compassX = compassY = compassZ = 0;
        compassXInput = compassYInput = compassZInput = 0;
    }
}

bool IMU::read(long dtMs, float *dta1, float *dta2, float *dta3, float *dta4) {
    //Read from the sensor, update all the values
    int16_t accGyro[10];
    if (!pMPU->readAccGyro(accGyro))
        return false;

    accY = accGyro[1]; //switch x - y
    accX = accGyro[0];
    accZ = accGyro[2];

    gyroRoll = accGyro[3];
    gyroPitch= 0 - accGyro[4]; //Invert the direction of the axis.
    gyroYaw  = 0 - accGyro[5]; //Invert the direction of the axis.

    accX -= accXBase;  //Subtact the manual accelerometer roll calibration value.
    accY -= accYBase;  //Subtact the manual accelerometer pitch calibration value.

    gyroRoll -= gyroRollBase; //Subtact the manual gyro roll calibration value.
    gyroPitch-= gyroPitchBase; //Subtact the manual gyro pitch calibration value.
    gyroYaw  -= gyroYawBase;   //Subtact the manual gyro yaw calibration value.
    
    if (dta4) {
        *dta4 = accGyro[6] * 0.00006f + 21; // Convert to Celsius
    }

    //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
    //#self.gyro_roll_input = self.gyro_roll_input * 0.7 + self.gyro_roll * 0.00458015 #/ 65.5 * 0.3)   #Gyro pid input is deg/sec.
    //#self.gyro_pitch_input= self.gyro_pitch_input* 0.7 + self.gyro_pitch* 0.00458015 #Gyro pid input is deg/sec.
    //#self.gyro_yaw_input  = self.gyro_yaw_input  * 0.7 + self.gyro_yaw  * 0.00458015      #Gyro pid input is deg/sec.

    //Gyro angle calculations
    //0.0000611 = 1 / 250Hz / 65.5 if dt=4ms
    float c1 = dtMs / 65500.0f;
    float c2 = c1 * 0.01745328f;  //3.14159 / 180

    anglePitch += gyroPitch* c1;  //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    angleRoll  += gyroRoll * c1;  //Calculate the traveled roll angle and add this to the angle_roll variable.
    angleYaw   += gyroYaw  * c1;  //Calculate the traveled yaw angle and add this to the angle_yaw variable.

    if (angleYaw < -180)
        angleYaw += 360;
    else if (angleYaw > 180)
        angleYaw -= 360;

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The sin function is in radians and not degrees.
    float transfer = sin(gyroYaw * c2);
    anglePitch-= angleRoll  * transfer;   //If the IMU has yawed transfer the roll angle to the pitch angel.
    angleRoll += anglePitch * transfer;  //If the IMU has yawed transfer the pitch angle to the roll angel.

    //self.angle_yaw -= course_deviation(angle_yaw, actual_compass_heading) / 1200.0;  #Calculate the difference between the gyro and compass heading and make a small correction.
    //if (angle_yaw < 0) angle_yaw += 360;                                             #If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
    //else if (angle_yaw >= 360) angle_yaw -= 360;                                     #If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

    //Accelerometer angle calculations
    /*if self.filter_on:
        angle_roll_acc = angle_pitch_acc = 0
        acc_total_vector = math.sqrt((self.acc_x * self.acc_x) + (self.acc_y * self.acc_y) + (self.acc_z * self.acc_z))    #Calculate the total accelerometer vector.
        if abs(self.acc_y) < acc_total_vector:                                             #Prevent the asin function to produce a NaN.
            angle_pitch_acc = math.asin(self.acc_y / acc_total_vector) * 57.296       #Calculate the pitch angle.
        if abs(self.acc_x) < acc_total_vector:                                             #Prevent the asin function to produce a NaN.
            angle_roll_acc = math.asin(self.acc_x / acc_total_vector) * 57.296        #Calculate the roll angle.

        self.angle_pitch= self.angle_pitch * 0.9997 + angle_pitch_acc * 0.0003         #Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
        self.angle_roll = self.angle_roll  * 0.9997 + angle_roll_acc  * 0.0003         #Correct the drift of the gyro roll angle with the accelerometer roll angle.
    */
    *dta1 = anglePitch;
    *dta2 = angleRoll;
    *dta3 = angleYaw;
    return true;
}

bool IMU::readCompass(float *dta1, float *dta2, float *dta3) {
    float xIn, yIn, zIn;
    if (!pMPU->readMagnetometer(&xIn, &yIn, &zIn))
        return false;

    //refer to https://github.com/kevinmcaleer/mpu9250/blob/main/ak8963.py
    float x = xIn - compassOffsetX;
    x *= compassScaleX;

    float y = yIn - compassOffsetY;
    y *= compassScaleY;

    float z = zIn - compassOffsetZ;
    z *= compassScaleZ;

    compassX = x;
    compassY = y;
    compassZ = z;
    
    compassXInput = compassXInput * 0.85f + compassX * 0.15f;
    compassYInput = compassYInput * 0.85f + compassY * 0.15f;
    compassZInput = compassZInput * 0.85f + compassZ * 0.15f;

    *dta1 = compassXInput;
    *dta2 = compassYInput;
    *dta3 = compassZInput;
    
    return true;
}

//calibrate IMU
void IMU::calibrate() {    
    resetCalibrations(true, false);

    float accXCal =0, accYCal = 0, accZCal = 0;
    float gyroRollCal = 0, gyroPitchCal = 0, gyroYawCal = 0;
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    
    int total = 8000; float x, y, z;
    for (int i = 0; i < total; ++i) { 
        read(LOOP_TIMER_MS, &x, &y, &z, NULL);

        accXCal += accX;
        accYCal += accY;
        accZCal += accZ;

        gyroRollCal  += gyroRoll;
        gyroPitchCal += gyroPitch;
        gyroYawCal   += gyroYaw;

        Utils::delayMs(LOOP_TIMER_MS);
    }

    accXBase = accXCal / total;
    accYBase = accYCal / total;
    accZBase = accZCal / total;

    gyroRollBase = gyroRollCal / total;
    gyroPitchBase= gyroPitchCal/ total;
    gyroYawBase  = gyroYawCal  / total;

    saveCalibrations();
}

void IMU::calibrateCompass() {
    resetCalibrations(false, true);

    float compassCalValues[6];
    for (int i = 0; i < 6; ++i)
        compassCalValues[i]=0;
        
    int total = 1500; float x, y, z;
    for (int i = 0; i < total; ++i) {
        if (readCompass(&x, &y, &z)) {
            x = compassX;
            y = compassY;
            z = compassZ;
            if (i <= 0) {
                compassCalValues[0] = compassCalValues[1] = x;
                compassCalValues[2] = compassCalValues[3] = y;
                compassCalValues[4] = compassCalValues[5] = z;
            }
            else {
                if (x < compassCalValues[0])
                    compassCalValues[0] = x;
                if (x > compassCalValues[1])
                    compassCalValues[1] = x;
                if (y < compassCalValues[2])
                    compassCalValues[2] = y;
                if (y > compassCalValues[3])
                    compassCalValues[3] = y;
                if (z < compassCalValues[4])
                    compassCalValues[4] = z;
                if (z > compassCalValues[5])
                    compassCalValues[5] = z;
            }
        }
        Utils::delayMs(LOOP_TIMER_MS);
    }
    calculateCompassCalibration(compassCalValues[0], compassCalValues[1], compassCalValues[2], compassCalValues[3], compassCalValues[4], compassCalValues[5]);
    saveCalibrations();
}

void IMU::resetCalibrations(bool imu, bool compass) {
    if (imu) {
        accXBase = accYBase = accZBase = 0;
        gyroRollBase  = gyroPitchBase = gyroYawBase = 0;
    }
    if (compass) {
        compassScaleX = compassScaleY = compassScaleZ = 1;
        compassOffsetX = compassOffsetY = compassOffsetZ = 0;
    }
}

void IMU::saveCalibrations() {
    //save to the file
    ofstream file("imu.ini");
    if (file.is_open()) {
        char szBuf[256];
        sprintf(szBuf, "%f,%f,%f,%f,%f,%f\n", accXBase, accYBase, accZBase, gyroRollBase, gyroPitchBase, gyroYawBase);
        file.write(szBuf, strlen(szBuf));
        sprintf(szBuf, "%f,%f,%f,%f,%f,%f\n", compassOffsetX, compassOffsetY, compassOffsetZ, compassScaleX, compassScaleY, compassScaleZ);
        file.write(szBuf, strlen(szBuf));
        file.close();
        log_info(LOGTAG, "IMU calibration data saved");
    }
}

void IMU::load() {
    resetCalibrations(true, true);

    ifstream file("imu.ini");
    string line;
    if (file.is_open()) {
        if (getline(file, line)) {
            vector<string> options = Utils::split(line, ",");
            if (options.size() == 6) {
                accXBase = stof(options[0]);
                accYBase = stof(options[1]);
                accXBase = stof(options[2]);

                gyroRollBase = stof(options[3]);
                gyroPitchBase = stof(options[4]);
                gyroYawBase = stof(options[5]);

                getline(file, line);
                options = Utils::split(line, ",");
                if (options.size() == 6) {
                    compassOffsetX = stof(options[0]);
                    compassOffsetY = stof(options[1]);
                    compassOffsetZ = stof(options[2]);

                    compassScaleX = stof(options[3]);
                    compassScaleY = stof(options[4]);
                    compassScaleZ = stof(options[5]);
                }
                cout << "!!!IMU calibration data loaded" << endl;
            }
            else {
                cout << "Malformed IMU configuration file" << endl;
            }
        }
        file.close();
    }
}

//Calculate the calibration offset and scale values
void IMU::calculateCompassCalibration(float minx, float maxx, float miny, float maxy, float minz, float maxz) {
    // Hard iron correction
    compassOffsetX = (maxx + minx) / 2;
    compassOffsetY = (maxy + miny) / 2;
    compassOffsetZ = (maxz + minz) / 2;

    // Soft iron correction
    float avgDeltaX = (maxx - minx) / 2;
    float avgDeltaY = (maxy - miny) / 2;
    float avgDeltaZ = (maxz - minz) / 2;

    float avgDelta = (avgDeltaX + avgDeltaY + avgDeltaZ) / 3;

    compassScaleX = avgDelta / avgDeltaX;
    compassScaleY = avgDelta / avgDeltaY;
    compassScaleZ = avgDelta / avgDeltaZ;
}
