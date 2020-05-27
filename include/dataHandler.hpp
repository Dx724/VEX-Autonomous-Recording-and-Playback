#define AUTON_FILE_PATH_PREFIX "/usd/"
#define AUTON_FILE_WRITE_PATH "/usd/testDataV.txt" // Should also update in autonomous.cpp
#define PARAMETER_FILE_PATH "/usd/parameters.txt" //Autonomous Flywheel RPM; Drive Flywheel RPM; Team Color ("blue" or "red")

#define FILE_HEADER "v::"

#define autonCount 5
extern int activeAuton; //Names in initialize.cpp, file names in autonomous.cpp

extern char prefixes[16];

extern const int INITIAL_STATE[16];

extern int flywheelSpeed_auton;
extern int flywheelSpeed_drive;
extern int flywheelSpeed_driveMiddle;
extern bool isBlue;
extern bool doClimb;

void runDrive(int, int, int, int, int, int, int, int,
              int, int, int, int, int, int, int, int);

void changeFlywheelTarget(double);

void auton_init();
void toggleAimAlign();
void forceAimAlign();
void toggleFlywheel();
void forceFlywheel(bool);
void activateDoubleShot();
void endDoubleShot();
bool isDoubleShotFinished();
void activateClimb();
bool isClimbActive();
void changeDoubleShotVelocities(double, double);
void toggleDriveDirection();
void forceFlywheelOverride(bool);
void startNewShot();
void unbrakeDrive();
void setDoubleShotLimit(int);
void resetClimbGyro();
void forceClimb(bool);

void setFlywheelSpeed(int);
