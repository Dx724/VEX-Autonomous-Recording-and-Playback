#include "main.h"
#include "dataHandler.hpp"

char prefixes[16] = {'a', 'b', 'c', 'd', 'e', 'f', 'g',
                      'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p'};
                      /* LeftVertical, LeftHorizontal,
                          RightVertical, RightHorizontal,
                          Up, Down, Left, Right,
                          X, B, Y, A
                          L1, L2, R1, R2 */

const int INITIAL_STATE[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int flywheelSpeed_auton = 180;
int flywheelSpeed_drive = 160;
int flywheelSpeed_driveMiddle = 140;

// Drive motors | Also in autonomous.cpp and competition_initialize()
auto lfMotor = 15_mtr;
auto rfMotor = 8_rmtr;
auto lbMotor = 19_mtr;
auto rbMotor = 6_rmtr;

const int DRIVE_THRESHOLD = 10;

// Lift
auto liftMotor = 12_mtr;

// Brake status
bool brakesEnabled = false;
int lastBrakeStatus = 0;
const int BRAKE_DRIVE_THRESHOLD = 15;

bool driveReversed = false;
int lastReverseStatus = 0;

// Vision Sensor
pros::Vision vision_sensor(10);

bool aimAlignActive = false;
int flagWidthMin = 30;//5;
int flagWidthMax = 130;//250;
bool lastAimAlignStatus = false;

const int VISION_OVERRIDE_THRESHOLD = 25;
const int VISION_ALIGN_VELOCITY = 25;
const int VISION_ALIGN_VELOCITY_MAX = 70;
const int VISION_ALIGN_AIMOFFSET = 14;
const int VISION_ALIGN_AIMTHRESHOLD = 5;
const double VISION_ALIGN_SCALAR = 150.0;

const int BLUE_SIG = 1; //1-indexed
const int RED_SIG = 2;
const int GREEN_SIG = 3;
const pros::vision_color_code_t CCODE_BLUE_FLAG = vision_sensor.create_color_code(BLUE_SIG, GREEN_SIG, 0, 0, 0);
const pros::vision_color_code_t CCODE_RED_FLAG = vision_sensor.create_color_code(RED_SIG, GREEN_SIG, 0, 0, 0);

void task_vision_aimAlign(void* parameters) {
  while (1) {
    if (aimAlignActive) {
        vision_object_s_t vObj_arr[10]; //TODO: Test
        int numObjects = vision_sensor.read_by_sig(0, (isBlue ? 1 : 0), 10, vObj_arr); //Blue signature is 0, but target opposite color
        vision_object_s_t targetObject;
        bool objFound = false;

        for (int i = 0; i < numObjects; i++) {
          if (vObj_arr[i].width >= flagWidthMin && vObj_arr[i].width <= flagWidthMax) {
            targetObject = vObj_arr[i];
            objFound = true;
            break;
          }
        }
        if (objFound) {
          int aimMetric = targetObject.left_coord + ((int) round(targetObject.width / 2.0)) - round(VISION_FOV_WIDTH / 2.0 + ((VISION_ALIGN_AIMOFFSET + targetObject.width * 0.3) * (isBlue ? -1 : 1)));
          pros::lcd::set_text(2, std::to_string(targetObject.width));
          pros::lcd::set_text(3, std::to_string(aimMetric));

          int alignVel = (int) round(VISION_ALIGN_VELOCITY * (0.7 + 0.3 * fabs(aimMetric / VISION_ALIGN_SCALAR)));
          if (alignVel > VISION_ALIGN_VELOCITY_MAX) alignVel = VISION_ALIGN_VELOCITY_MAX;

          if (abs(aimMetric) < VISION_ALIGN_AIMTHRESHOLD) {
            lfMotor = 0;
            lbMotor = 0;
            rfMotor = 0;
            rbMotor = 0;
            aimAlignActive = false;
          }
          else if (aimMetric > 0) { //To the right
            lfMotor.move_velocity(alignVel);
            lbMotor.move_velocity(alignVel);
            rfMotor.move_velocity(-alignVel);
            rbMotor.move_velocity(-alignVel);

            pros::lcd::set_text(5, "RIGHT");
            pros::lcd::set_text(6, std::to_string(alignVel).c_str());
          }
          else { //To the left
            lfMotor.move_velocity(-alignVel);
            lbMotor.move_velocity(-alignVel);
            rfMotor.move_velocity(alignVel);
            rbMotor.move_velocity(alignVel);

            pros::lcd::set_text(5, "LEFT");
            pros::lcd::set_text(6, std::to_string(alignVel).c_str());
          }
        }
        else {
          lfMotor = 0;
          lbMotor = 0;
          rfMotor = 0;
          rbMotor = 0;
        }
    }
    delay(20);
  }
}

// Game subsystem motors
auto intakeMotor = 1_rmtr;
int INTAKE_SPEED = 127;
int OUTTAKE_SPEED = -80; //Use intake motor for cap control
int OUTTAKE_DURATION = 50;
int lastOuttakeStatus = 0;
int lastOuttakeStatus2 = 0;

auto internalIntakeMotor = 4_rmtr;

bool doubleShotActive = false;
bool flywheelIntakeOverride = false;

bool outtakeActive = false;
bool doubleOuttake = false;
bool intakeActive = false;
void task_outtake(void* parameters) {
  while (1) {
    if (outtakeActive) {
      intakeMotor = OUTTAKE_SPEED;
      if (doubleOuttake) internalIntakeMotor = OUTTAKE_SPEED;
    }
    else { //Block this section during autonomous to relinquish control of intake to manual code
      if (!intakeActive && !flywheelIntakeOverride && !pros::competition::is_autonomous()) intakeMotor = 0;
    }
    delay(20);
  }
}

auto flywheelMotor2 = 2_rmtr;
auto flywheelMotor = 3_mtr;
int flywheelVelocity = 160;
int lastFlywheelStatus = 0;
bool flywheelActive = false;
bool shotFinished = false;

double currVoltage = 95;
double targetVelocity = (double) flywheelSpeed_drive;

long lastMillis = pros::millis();
double flywheelAccum = 0;
double prevError = 0;
bool velReached = false;
int numReaches = 0;
const int TARGET_REACHES = 2;
const double VELOCITY_REACH_THRESHOLD = 2.0;
const int VELOCITY_REACH_MILLIS = 500;
const double VELOCITY_SHOOT_THRESHOLD = 2.0;
int velReachTime = -1;
int loopDelay = 2;
bool shotReady = false;

bool pidActive = true;

double kP = 0.027;
double kI = 0.00001755;
double kD = 2.79;
double kD_shoot = 200.3;
double kStable = 0.98;
double kStableMin = 0.3;

bool isShooting = false;
bool ds_forceMove = false;

pros::ADIDigitalIn limitSwitch(3);
pros::ADIDigitalIn limitSwitch2(6);

void task_flywheelPID(void* parameters) {
	while (true) {
		if (!flywheelActive) {
      flywheelMotor = 0;
      flywheelMotor2 = 0;
    }
		else {
      if (flywheelIntakeOverride) {
        pros::lcd::print(1, (std::to_string(velReached) + " " + std::to_string(shotReady)).c_str());
  			if (!velReached) {
			  internalIntakeMotor.move_velocity(0);
			  intakeMotor.move_velocity(0);
  			}
  			else {
          if (!limitSwitch2.get_value()) {
			  internalIntakeMotor.move_velocity(200);
              intakeMotor.move_velocity(200);
          }
          else {
            if (ds_forceMove || (!shotFinished && fabs(flywheelMotor.get_actual_velocity() - targetVelocity) < VELOCITY_SHOOT_THRESHOLD)) {
              internalIntakeMotor.move_velocity(200);
              intakeMotor.move_velocity(200);
            }
            else {
              internalIntakeMotor.move_velocity(0);
              intakeMotor.move_velocity(0);
            }
          }
  			}
      }

			double readVelocity = flywheelMotor.get_actual_velocity();
			double velError = readVelocity - targetVelocity;
			flywheelAccum += velError * (pros::millis() - lastMillis) / 1000.0;
			double velDeriv = (velError - prevError) / (pros::millis() - lastMillis);
      double kStabMultiplier = powf(kStable, numReaches);
      if (kStabMultiplier < kStableMin) kStabMultiplier = kStableMin;
      pros::lcd::print(7, "%d, %l", numReaches, kStabMultiplier);
			if (pidActive) currVoltage -= ((isShooting ? 0.05 : 1.0) * (kP * (velError) + kI * (flywheelAccum)) + (isShooting ? kD_shoot : kD) * velDeriv * (isShooting ? 1.0 : 1.0)) * ((velReached && !isShooting) ? kStabMultiplier : 1.0);

      pros::lcd::print(1, std::to_string(currVoltage).c_str());

			if (fabs(velError) < VELOCITY_REACH_THRESHOLD && velReachTime == -1) velReachTime = pros::millis();
			else if (velReachTime != -1 && pros::millis() - velReachTime > VELOCITY_REACH_MILLIS) {
        numReaches += 1;
        velReachTime = pros::millis();
        if (numReaches >= TARGET_REACHES && !velReached) {
          velReached = true;
          flywheelAccum = 0;
        }
      }

			prevError = velError;

			lastMillis = pros::millis();
			if (currVoltage > 127) currVoltage = 127;
			else if (currVoltage < 0) currVoltage = -0;
			flywheelMotor = (int) round(currVoltage);
      flywheelMotor2 = (int) round(currVoltage);

      pros::lcd::print(3, "%s | %s", std::to_string(flywheelMotor.get_actual_velocity()).c_str(), std::to_string(targetVelocity).c_str());
		}
		pros::delay(loopDelay);
	}
}

void retargetVelocity() {
  velReached = false;
  velReachTime = -1;
  numReaches = 0;
}

void changeFlywheelTarget(double newTargetVel) {
  if (newTargetVel == targetVelocity) return;
	flywheelAccum = 0;
	prevError = 0;
  retargetVelocity();
	targetVelocity = newTargetVel;
}

bool maxReached = false;
bool firstShot = false;
bool secondShot = false;

bool lastLimitState = false;
bool lastLimitState2 = false;
bool doubleShotFinished;

bool newLimitState = false,
      newLimitState2 = false,
      newUntoggle = false,
      newUntoggle2 = false,
      newToggle1 = false,
      newToggle2 = false;

const int LIMIT_DEBOUNCE_TIME = 200;
int lastUT1 = 0;
int lastUT2 = 0;
int lastT1 = 0;
int lastT2 = 0;

void task_limitMonitor(void* parameters) {
  while (true) {
    newLimitState = limitSwitch.get_value();
    newLimitState2 = limitSwitch2.get_value();

    //Must be set to false after registered
    bool t_newUntoggle = (!newLimitState && lastLimitState);
    if (t_newUntoggle && pros::millis() - lastUT1 > LIMIT_DEBOUNCE_TIME) {
      lastUT1 = pros::millis();
    }
    else {
      t_newUntoggle = false;
    }
    newUntoggle = t_newUntoggle || newUntoggle;

    bool t_newUntoggle2 = (!newLimitState2 && lastLimitState2);
    if (t_newUntoggle2 && pros::millis() - lastUT2 > LIMIT_DEBOUNCE_TIME) {
      lastUT2 = pros::millis();
      pros::lcd::print(2, "New untoggle: %d", lastUT2);
    }
    else {
      t_newUntoggle2 = false;
    }
    newUntoggle2 = t_newUntoggle2 || newUntoggle2;

    bool t_newToggle1 = (newLimitState && !lastLimitState);
    if (t_newToggle1 && pros::millis() - lastT1 > LIMIT_DEBOUNCE_TIME) {
      lastT1 = pros::millis();
    }
    else {
      t_newToggle1 = false;
    }
    newToggle1 = t_newToggle1 || newToggle1;

    bool t_newToggle2 = (newLimitState2 && !lastLimitState2);
    if (t_newToggle2 && pros::millis() - lastT2 > LIMIT_DEBOUNCE_TIME) {
      lastT2 = pros::millis();
    }
    else {
      t_newToggle2 = false;
    }
    newToggle2 = t_newToggle2 || newToggle2;

    delay(20);
    lastLimitState = newLimitState;
    lastLimitState2 = newLimitState2;
  }
}

void task_isShootingUntoggle(void* parameters) {
  while (1) {
    if (isShooting) {
      if (newToggle2) {
        newToggle2 = false;
        isShooting = false;
        retargetVelocity();
      }
      else {
        delay(1000); //TODO: Update
        isShooting = false;
        retargetVelocity();
      }
    }
    delay(20);
  }
}

//Also set in init functions
int shotVelFirst = 189.0;
int shotVelSecond = 133.0;

const int DOUBLESHOT_TOGGLE_TIME = 0.0; // (Disabled) 2350.0; //TODO: Change this if shooting gets faster
int firstShotTime = 0;

int ds_maxMillis = 0;
int ds_startTime = 0;

void task_doubleShot(void* parameters) {
	while (true) {
		if (!doubleShotActive) {
			pros::delay(20);
			continue;
		}
		if (!maxReached) {
			if (flywheelMotor.get_actual_velocity() >= targetVelocity) {
        maxReached = true;
        ds_startTime = pros::millis();
      }
		}
		else if (!firstShot && velReached) {
			if (newUntoggle2) {
        newUntoggle2 = false;
        isShooting = true;
				firstShot = true;
        ds_startTime = pros::millis();
        firstShotTime = pros::millis();
        delay(250);
				changeFlywheelTarget(shotVelSecond);
        if (shotVelSecond < shotVelFirst) currVoltage *= 0.15; //Does this work?
			}
		}
		else if (firstShot && !secondShot && velReached) { //This may cause issues if the first shot lowers the RPM to under 130.
			if (newUntoggle2) {
        if ((pros::millis() - firstShotTime) > DOUBLESHOT_TOGGLE_TIME) {
          newUntoggle2 = false;
          isShooting = true;
  				secondShot = true;
          delay(250);
  				changeFlywheelTarget(shotVelFirst);
        }
        else {
          newUntoggle2 = false;
        }
			}
		}
		else if (secondShot) {
			maxReached = false;
			firstShot = false;
			secondShot = false;
      doubleShotFinished = true;
      doubleShotActive = false;
      flywheelIntakeOverride = false;
      ds_forceMove = false;
      pros::lcd::print(5, "Double Shot Finished At: %d", pros::millis());
		}

    if (maxReached) {
      if (pros::millis() - ds_startTime > ds_maxMillis && ds_maxMillis > 0) {
        ds_forceMove = true;
      }
      else {
        ds_forceMove = false;
      }
    }

		pros::delay(20);
		pros::lcd::print(4, "%i, %i, %i", maxReached, firstShot, secondShot);
	}
}

void activateDoubleShot() {
  ds_startTime = pros::millis();
  maxReached = false;
  firstShot = false;
  secondShot = false;
  doubleShotActive = true;
  doubleShotFinished = false;
  newUntoggle2 = false;
  newToggle2 = false;
  isShooting = false;
  ds_forceMove = false;
  changeFlywheelTarget(shotVelFirst);
}

void endDoubleShot() {
  doubleShotActive = false;
}

bool lastMStatus = false;
bool lastNStatus = false;


auto pivotMotor = 17_mtr;
auto flipMotor = 13_mtr;

// The pivoting arm which moves the cap flipper
static int pivotTargets[2] = {730, 2100}; //49% to 83%
static int pivotTarget = pivotTargets[1];
const int PIVOT_SPEED_UP = 127;
const int PIVOT_SPEED_DOWN = -40;
const int PIVOT_SPEED_HOLD = 10;
const int PIVOT_THRESHOLD = 150;
ADIPotentiometer pivotSensor(1);

void enforceInitialBrake() {
  lfMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
  rfMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
  lbMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
  rbMotor.set_brake_mode(MOTOR_BRAKE_HOLD);

  //Enforce braking by stopping motors
  lfMotor.move_velocity(0);
  rfMotor.move_velocity(0);
  lbMotor.move_velocity(0);
  rbMotor.move_velocity(0);
}

const int CLIMB_SPEED = 200;
const double CLIMB_TILT_THRESHOLD = 14.3;
const double CLIMB_ZERO_THRESHOLD = 1.3;//0.97;
const double CLIMB_CONTINUE_DURATION = 100;
bool lastClimbStatus = false;
ADIGyro climbGyro(8, 0.1);
bool climbActive = false;
bool climbPassed = false;
pros::Controller guiController(pros::E_CONTROLLER_MASTER); //TODO: Remove

void task_climb(void* parameters) {
  while (1) {
    if (climbActive) {
      double gyroValue = climbGyro.get_value();
      double absGyroVal = fabs(gyroValue);
      if (absGyroVal > CLIMB_TILT_THRESHOLD) {
        climbPassed = true;
      }
      if (climbPassed && absGyroVal < CLIMB_ZERO_THRESHOLD) {
        delay(CLIMB_CONTINUE_DURATION); //Allow robot to climb slightly farther
        climbActive = false;
        brakesEnabled = true; //Brake
        enforceInitialBrake();
      }
    }
    else {
      climbPassed = false;
    }
    delay(20);
  }
}

void task_controllerText(void* parameters) {
  while(1) {
    guiController.clear_line(2);
    delay(55); //Minimum refresh delay: 50ms
    guiController.print(2, 1, "%s", (brakesEnabled ? "B" : "  "));
    delay(55);
    guiController.print(2, 5, "%s", (driveReversed ? "LIFT    " : "INTAKE"));
    delay(55);
    guiController.print(2, 12, "%s", (aimAlignActive ? "V " : " "));
    delay(55);
  }
}

bool hasInitialized = false;

void setFlywheelSpeed(int newSpeed) { //This doesn't do anything
  flywheelVelocity = newSpeed;
}

bool autonHasInited = false;
void auton_init() {
  if (autonHasInited) return;

  liftMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
  internalIntakeMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);

  Task aimAlignTask(task_vision_aimAlign);
  Task flywheelTask(task_flywheelPID);
  Task doubleShotTask(task_doubleShot);
  Task climbTask(task_climb);
  Task limitTask(task_limitMonitor);
  Task isShootingTask(task_isShootingUntoggle);
  pros::lcd::print(1, "AUTON INIT");
  autonHasInited = true;
}

void toggleFlywheel() {
  flywheelActive ^= true;
}

void forceFlywheel(bool forceValue) {
  flywheelActive = forceValue;
}

void forceAimAlign() {
  aimAlignActive = true;
}

void toggleAimAlign() {
  aimAlignActive ^= true;
}

bool isDoubleShotFinished() {
  return doubleShotFinished;
}

void activateClimb() {
  climbGyro.reset();
  climbActive = true;
}

void forceClimb(bool forceVal) {
  climbActive = forceVal;
}

void resetClimbGyro() {
  climbGyro.reset();
}

bool isClimbActive() {
  return climbActive;
}

void changeDoubleShotVelocities(double vel1, double vel2) {
  shotVelFirst = vel1;
  shotVelSecond = vel2;
}

void toggleDriveDirection() {
  driveReversed ^= true;
}

void forceFlywheelOverride(bool val) {
  flywheelIntakeOverride = val;
}

void startNewShot() {
  shotFinished = false;
}

void setDoubleShotLimit(int maxMillis) {
  ds_maxMillis = maxMillis;
}

void unbrakeDrive() {
  lfMotor.set_brake_mode(MOTOR_BRAKE_COAST);
  rfMotor.set_brake_mode(MOTOR_BRAKE_COAST);
  lbMotor.set_brake_mode(MOTOR_BRAKE_COAST);
  rbMotor.set_brake_mode(MOTOR_BRAKE_COAST);
}

pros::Controller partnerController(E_CONTROLLER_PARTNER);

void runDrive(int aVal, int bVal, int cVal, int dVal, int eVal, int fVal,
              int gVal, int hVal, int iVal, int jVal, int kVal, int lVal,
            int mVal, int nVal, int oVal, int pVal) { //Note: aVal and cVal are updated in drive

              if (!hasInitialized) {
                pivotMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
                flipMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
                unbrakeDrive();
                flipMotor.set_encoder_units(MOTOR_ENCODER_DEGREES);
                lfMotor.set_encoder_units(MOTOR_ENCODER_ROTATIONS);
                rfMotor.set_encoder_units(MOTOR_ENCODER_ROTATIONS);
                lbMotor.set_encoder_units(MOTOR_ENCODER_ROTATIONS);
                rbMotor.set_encoder_units(MOTOR_ENCODER_ROTATIONS);
                Task outtakeTask(task_outtake);
                Task controllerTextTask(task_controllerText);
                auton_init(); //Just in case autonomous was not run

                changeDoubleShotVelocities(189.0, 131.5); //133
                changeFlywheelTarget(189.0);
                driveReversed = false;
                climbActive = false;
                forceFlywheel(true);
                setDoubleShotLimit(-1);

                //For Skills
                // MATCH isBlue = false;
                setDoubleShotLimit(2500);
                doubleOuttake = false;
                // MATCH outtakeActive = true;

                hasInitialized = true;
              }

              double driveMultiplier = 200.0 / 127.0; //Scale to velocity range
              if ((aVal > 0) != (cVal > 0)) { //If turning
                //driveMultiplier *= 0.72; //Decrease drive speed
                driveMultiplier *= 1.0;
              }

              if (driveReversed) driveMultiplier *= -1;

              if (!doubleShotActive) {
                if (!brakesEnabled || (abs(aVal) > BRAKE_DRIVE_THRESHOLD) || (abs(cVal) > BRAKE_DRIVE_THRESHOLD)) {
                  if (!aimAlignActive) {
                    if (abs(aVal) < DRIVE_THRESHOLD) aVal = 0;
                    if (abs(cVal) < DRIVE_THRESHOLD) cVal = 0;

                    // Drive control
                    lfMotor.move_velocity((driveReversed ? cVal : aVal) * driveMultiplier); //Left joystick vertical
                    rfMotor.move_velocity((driveReversed ? aVal : cVal) * driveMultiplier); //Right joystick vertical
                    lbMotor.move_velocity((driveReversed ? cVal : aVal) * driveMultiplier);
                    rbMotor.move_velocity((driveReversed ? aVal : cVal) * driveMultiplier);
                  }

                  if (fabs(round((aVal) * driveMultiplier)) > VISION_OVERRIDE_THRESHOLD || fabs(round((cVal) * driveMultiplier)) > VISION_OVERRIDE_THRESHOLD) {
                    aimAlignActive = false;
                  }
                }
                else { //Enforce braking
                  lfMotor.move_velocity(0);
                  rfMotor.move_velocity(0);
                  lbMotor.move_velocity(0);
                  rbMotor.move_velocity(0);
                }
              }
              else {
                /*
                if (fabs(round((aVal) * driveMultiplier)) > VISION_OVERRIDE_THRESHOLD || fabs(round((cVal) * driveMultiplier)) > VISION_OVERRIDE_THRESHOLD) {
                  doubleShotActive = false;
                }
                */
              }

              if (eVal && !lastClimbStatus) { //Up
                climbActive ^= true;
                if (climbActive) {
                  climbGyro.reset();
                }
              }
              lastClimbStatus = eVal;

              if (gVal && !lastAimAlignStatus) { //Left
                toggleAimAlign();
              }
              lastAimAlignStatus = gVal;

              if (fVal && !lastBrakeStatus) { //Down
                brakesEnabled ^= true;
                if (brakesEnabled) {
                  enforceInitialBrake();
                }
                else {
                  unbrakeDrive();
                }
              }
              lastBrakeStatus = fVal; //Down

              if (climbActive || iVal) {
                lfMotor = driveReversed ? -127 : 127;
                rfMotor = driveReversed ? -127 : 127;
                lbMotor = driveReversed ? -127 : 127;
                rbMotor = driveReversed ? -127 : 127;
              }

              if (jVal && !lastReverseStatus) { //B
                toggleDriveDirection();//driveReversed ^= true; //Reverse drive direction
              }
              lastReverseStatus = jVal;

              // Async monitoring
              //   Pivot
              if (mVal != lastMStatus && mVal && !nVal) { //L1
                changeFlywheelTarget(shotVelFirst);
                currVoltage = (currVoltage < 105 ? 105 : currVoltage);
                startNewShot();
                newUntoggle2 = false;
                newToggle2 = false;
              }
              else if (nVal != lastNStatus && nVal && !mVal) { //L2
                changeFlywheelTarget(shotVelSecond);
                currVoltage *= 0.15; //Does this work?
                startNewShot();
                newUntoggle2 = false;
                newToggle2 = false;
              }

              if (mVal && nVal && !doubleShotActive && (!lastMStatus || !lastNStatus) ) {
                activateDoubleShot();
                startNewShot();
                pros::lcd::print(5, "Double Shot Started Manually At: %d", pros::millis());
              }
              else if (!(mVal && nVal) && doubleShotActive) {
                doubleShotActive = false;
                pros::lcd::print(5, "Double Shot Stopped Manually At: %d", pros::millis());
              }

              if ((mVal || nVal) && !shotFinished) {
                flywheelIntakeOverride = true;
                if (!doubleShotActive && newUntoggle2) {
                  shotFinished = true;
                  newUntoggle2 = false;
                  isShooting = true;
                }
              }
              else {
                flywheelIntakeOverride = false;
              }
              lastMStatus = mVal;
              lastNStatus = nVal;

              int pivotValue = pivotSensor.get_value();

              if (abs(pivotValue - pivotTarget) > PIVOT_THRESHOLD) {
                pivotMotor = ((pivotValue - pivotTarget > 0) ? PIVOT_SPEED_DOWN : PIVOT_SPEED_UP);
              }
              else {
                pivotMotor = (pivotTarget == pivotTargets[1] ? PIVOT_SPEED_HOLD : 0);
              }

              if (pVal) { // pVal (R2)
                intakeMotor = INTAKE_SPEED;
                intakeActive = true;
                outtakeActive = false;
              }
              else {
                if (!outtakeActive && !flywheelIntakeOverride) intakeMotor = 0;
                intakeActive = false;
              }

              if (partnerController.get_digital(E_CONTROLLER_DIGITAL_UP)) { // oVal (R2)
                internalIntakeMotor = 127;
              }
              else {
                if (!outtakeActive && !flywheelIntakeOverride) {
                  internalIntakeMotor = 0;
                }
              }

              bool temp_outtakeToggle = partnerController.get_digital(E_CONTROLLER_DIGITAL_DOWN);
              if (temp_outtakeToggle && !lastOuttakeStatus) { // lVal
                doubleOuttake = false;
                outtakeActive ^= true;
              }
              lastOuttakeStatus = temp_outtakeToggle;

              bool temp_outtakeToggle2 = partnerController.get_digital(E_CONTROLLER_DIGITAL_RIGHT);
              if (temp_outtakeToggle2 && !lastOuttakeStatus2) { // lVal
                doubleOuttake = true;
                outtakeActive ^= true;
              }
              lastOuttakeStatus2 = temp_outtakeToggle2;

              if (hVal && !lastFlywheelStatus) { // Right
                toggleFlywheel();

              }
              lastFlywheelStatus = hVal;

              if (partnerController.get_digital(E_CONTROLLER_DIGITAL_L1)) { // iVal (X)
                liftMotor = 127;
              }
              else if (partnerController.get_digital(E_CONTROLLER_DIGITAL_L2)) { // kVal (Y)
                liftMotor = -100;
              }
              else {
                liftMotor = 0;
              }

}
