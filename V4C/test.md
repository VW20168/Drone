```mermaid
classDiagram
    class FlightController {
        + pMotor1: Motor
        + pMotor2: Motor
        + pMotor3: Motor
        + pMotor4: Motor
        + pRedLed: PiPIN
        + pGreenLed: PiPIN
        + mState: int
        + mSpeed: int
        + mMotorCompensations: int[]
        + pMutex: mutex
        + pConfig: Configuration
        + pAutoPilot: AutoPilot
        + pingReceived: bool
        + lastStopTime: int
        + autoPilotActive: bool
        + turnOffAllLeds(): void
        + setLeds(bool redLed, bool greenLed): void
        + setMode(int mode): void
        + setSpeed(int speed): void
        + setMotorCompensation(int motor, int compensation): void
        + setYawAngle(int angle): void
        + setAltitude(int altitude): void
        + startAutoPilot(): void
        + stopAutoPilot(): void
        + control(): void
    }