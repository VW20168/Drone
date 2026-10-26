class Constants:
    MODE_INDOOR     = 0 #debugging
    MODE_OUTDOOR    = 1
    MODE_TRAINING   = 2
        
    LOOP_TIMER_S    = 4/1000.0 #4ms, to seconds
    
    PID_MAX_IMU = 60
    PID_MAX_ALT = 100
    
    MAX_ALTITUDE    = 0.6 #in meters if outdoor
    READING_UNKNOWN = 720

    MAX_BATTERY_VOLTAGE  = 12.5
    MIN_BATTERY_VOLTAGE  = 10.8
    CRITICAL_BATTERY_VOLTAGE  = 10.5

    #Drone(Flight Controller) status
    FS_SHUTDOWN     = 0
    FS_POWERED_ON   = 1
    FS_CALIBRATING  = 2
    FS_CALIBRATED   = 3
    FS_STOPPING     = 4
    FS_STOPPED      = 5
    FS_STARTING     = 6

    FS_STARTED      = 10
    FS_PREFLIGHT_CHK= 11
    FS_TAKING_OFF   = 12
    FS_LANDING      = 13
    FS_HOME         = 14
    
    FS_HOVERING     = 20
    FS_TURN_LEFT    = 21
    FS_TURN_RIGHT   = 22
    FS_MOVE_FORWARD = 23
    FS_MOVE_BACKWARD= 24
    FS_MOVE_LEFT    = 25
    FS_MOVE_RIGHT   = 26
    FS_FLY_HIGH     = 27
    FS_FLY_LOW      = 28
    FS_EMERGENCY    = 50
    
def __init__(self):
    #when the battery voltage reaches the MIN_BATTERY_VOLTAGE, the compensation is 40
    self.IDEAL_BATTERY_FACTOR  = 40 / (self.MAX_BATTERY_VOLTAGE - self.MIN_BATTERY_VOLTAGE)
