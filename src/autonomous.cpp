#include "main.h"
#include "dataHandler.hpp"

int getPrefixIndex(char p) {
   for (int pIdx = 0; pIdx < sizeof(prefixes); pIdx++) {
     if (prefixes[pIdx] == p) return pIdx;
   }
   return -1;
 }

std::string autonFileNames[autonCount] = {
	"blueFront.txt",
	"redFront.txt",
	"blueBack.txt",
	"redBack.txt",
	"testDataV.txt"
};

//MANUAL AUTON START
auto auton_lfMotor = 15_mtr;
auto auton_rfMotor = 8_rmtr;
auto auton_lbMotor = 19_mtr;
auto auton_rbMotor = 6_rmtr;
auto auton_intakeMotor = 1_rmtr;
auto auton_internalIntakeMotor = 4_mtr;
auto auton_flywheelMotor = 2_mtr;

ADIGyro auton_turnGyro(7, 0.1);

const double MOVE_TOLERANCE = 0.025;
const double TURN_TOLERANCE = 0.025;
const double IDLE_THRESHOLD = 5;
const double COAST_VELOCITY = 25.0;
const double TURN_SCALING = 1.00;

bool isBlue = false;

void drive_voltage(int lVoltage, int rVoltage) {
  auton_lfMotor = lVoltage;
  auton_lbMotor = lVoltage;
  auton_rfMotor = rVoltage;
  auton_rbMotor = rVoltage;
}

void profile_motor(Motor motor, double final, double initial = 0.0, double mVelocity = 200.0) {
  double profileVelocity;
  double movePercent = (motor.get_position() - initial) / final;
  if (movePercent < 0.34) {
    profileVelocity = 0.5 + 0.5 * pow(movePercent * 3.0, 1.0/4.0);
  }
  else {
    profileVelocity = 0.5 + 0.5 * pow((1.0 - movePercent) * 3.0/2.0, 1.0/3.0);
  }
  profileVelocity *= mVelocity;
  if (final < initial) profileVelocity *= -1;
  motor.move_velocity((int) round(profileVelocity));
}

const double DEFAULT_RIGHT_BIAS = 1.083;
double rightBias = DEFAULT_RIGHT_BIAS;
void setRightBias(double newBias) {
  rightBias = newBias;
}

const int BIAS_BUFFER = 3;
const float EXTRA_DECEL_DISTANCE = 2.0;

const int MOVE_PERCENT_MIN = 0.1 / 8.0;

void profile_drive(double final, double initial = 0.0, double mVelocity = 200.0, double profileRatio = 0.7) {
  double profileVelocity;
  double movePercent = (auton_lfMotor.get_position() - initial) / final;
  if (movePercent < 0.0) movePercent = 0.0;
  else if (movePercent > 1.0) movePercent = 1.0;
  if (movePercent < MOVE_PERCENT_MIN) movePercent = MOVE_PERCENT_MIN;

  profileRatio = 0.85;

  if (movePercent < 0.34) {
    profileVelocity = (1.0 - profileRatio) + profileRatio * pow(movePercent * 3.0, 1.0/4.0);
    profileVelocity *= 0.762 + (1 - 0.762) * (1 - (movePercent / 0.34));
  }
  else {
    profileVelocity = (1.0 - profileRatio) + profileRatio * pow((1.0 - movePercent) * 3.0/2.0, 1.0/3.0);
    profileVelocity *= 0.3 + 0.7 * (1 - movePercent); //0.7, 0.3
  }
  profileVelocity *= mVelocity;

  double dBias = rightBias;
  if (final < initial) {
    profileVelocity *= -1;
  }
  pros::lcd::print(7, std::to_string(profileVelocity).c_str());
  pros::lcd::print(5, std::to_string(movePercent).c_str());

  //Auto-Straight
  float leftRatio = (BIAS_BUFFER + fabs(auton_lfMotor.get_position())) / (BIAS_BUFFER + fabs(auton_rfMotor.get_position() * dBias)); //[flip] Negative because opposite directions
  pros::lcd::set_text(2, std::to_string(leftRatio).c_str());
  delay(55);
  if (leftRatio < 0) {
    auton_lfMotor.move_velocity((int) round(profileVelocity)); //Guess 3: check ratio?
    auton_lbMotor.move_velocity((int) round(profileVelocity));
    auton_rfMotor.move_velocity((int) round(profileVelocity));
    auton_rbMotor.move_velocity((int) round(profileVelocity));
  }
  else {
    if (leftRatio < 1.0) {
      auton_lfMotor.move_velocity((int) round(profileVelocity)); //Guess 2 (voltage?)
      auton_lbMotor.move_velocity((int) round(profileVelocity));
      auton_rfMotor.move_velocity((int) round(profileVelocity * leftRatio));
      auton_rbMotor.move_velocity((int) round(profileVelocity * leftRatio));
    }
    else {
      auton_lfMotor.move_velocity((int) round(profileVelocity * (1 / leftRatio))); //1.0?
      auton_lbMotor.move_velocity((int) round(profileVelocity * (1 / leftRatio)));
      auton_rfMotor.move_velocity((int) round(profileVelocity));
      auton_rbMotor.move_velocity((int) round(profileVelocity));
    }
  }
}

void profile_driveTurn(double degreeTarget, double mVelocity = 200) {
  double profileVelocity;
  double movePercent = auton_turnGyro.get_value() / degreeTarget;
  if (movePercent < 0.0) movePercent = 0.0;
  else if (movePercent > 1.0) movePercent = 1.0;
  if (movePercent < MOVE_PERCENT_MIN * 3) movePercent = MOVE_PERCENT_MIN * 3;
  if (movePercent < 0.34) {
    profileVelocity = 0.3 + 0.7 * pow(movePercent * 3.0, 1.0/4.0);
  }
  else {
    profileVelocity = 0.3 + 0.7 * pow((1.0 - movePercent) * 3.0/2.0, 1.0/3.0);
  }
  profileVelocity *= mVelocity;
  if (degreeTarget > 0) profileVelocity *= -1;
  pros::lcd::print(7, std::to_string(profileVelocity).c_str());
  pros::lcd::print(5, std::to_string(movePercent).c_str());
  pros::lcd::print(4, std::to_string(auton_turnGyro.get_value()).c_str());

  //Auto-Straight (Disabled)
  float leftRatio = -auton_lfMotor.get_position() / auton_rfMotor.get_position(); //[flip] Positive because opposite directions (motor and movement both)
  if (leftRatio < 0 || true) {
    auton_lfMotor.move_velocity((int) round(-profileVelocity));
    auton_lbMotor.move_velocity((int) round(-profileVelocity));
    auton_rfMotor.move_velocity((int) round(profileVelocity));
    auton_rbMotor.move_velocity((int) round(profileVelocity));
  }
  else {
    if (leftRatio < 1.0) {
      auton_lfMotor.move_velocity((int) round(-profileVelocity));
      auton_lbMotor.move_velocity((int) round(-profileVelocity));
      auton_rfMotor.move_velocity((int) round(profileVelocity * leftRatio));
      auton_rbMotor.move_velocity((int) round(profileVelocity * leftRatio));
    }
    else {
      auton_lfMotor.move_velocity((int) round(-profileVelocity * (1 / leftRatio)));
      auton_lbMotor.move_velocity((int) round(-profileVelocity * (1 / leftRatio)));
      auton_rfMotor.move_velocity((int) round(profileVelocity));
      auton_rbMotor.move_velocity((int) round(profileVelocity));
    }
  }
}

const double circumference = (13.0 + 1.0/8.0)/12.0;//4.0 * 3.14159265 / 12.0;

void motor_dist(Motor motor, double rotations, double mVelocity = 200.0, bool profile = true) {
  motor.tare_position();
  motor.move_velocity(rotations > 0 ? mVelocity : -mVelocity);//motor.move_absolute(rotations, mVelocity);
  while (fabs(motor.get_position() - rotations) > MOVE_TOLERANCE * rotations && (rotations > 0 ? motor.get_position() < rotations : motor.get_position() > rotations)) {
    if (profile) profile_motor(motor, rotations, 0.0, mVelocity);
    delay(5);
  }
  motor.move_velocity(0);
}

void motor_dist_async(Motor motor, double rotations, double mVelocity = 200.0) {
  motor.tare_position();
  motor.move_absolute(rotations, mVelocity);
}

void motor_feet(Motor motor, double feet, double mVelocity = 200.0, bool profile = true) {
  motor_dist(motor, feet / circumference, mVelocity, profile);
}

bool runFlag(int flag, double movePercent) {
  switch (flag) {
    case 0:
      return true;
    case 101:
      if (movePercent > 0.865) {
        motor_dist_async(auton_intakeMotor, 10.35, 200.0);
        return true;
      }
      return false;
    case 102:
      if (movePercent > 0.01) {
        auton_intakeMotor.move_velocity(-200.0);
        return true;
      }
      return false;
  }
  return false;
}

void drive_voltage (int voltage) {
  auton_lfMotor.move_voltage(voltage);
  auton_rfMotor.move_voltage(voltage);
  auton_lbMotor.move_voltage(voltage);
  auton_rbMotor.move_voltage(voltage);
}

void drive_voltage_asymmetrical(int voltageL, int voltageR) {
  auton_lfMotor.move_voltage(voltageL);
  auton_lbMotor.move_voltage(voltageL);
  auton_rfMotor.move_voltage(voltageR);
  auton_rbMotor.move_voltage(voltageR);
}

void minimizeBacklash(bool forwardLeft, bool forwardRight) {
  drive_voltage_asymmetrical(forwardLeft ? 500 : -500, forwardRight ? 500 : -500); //1700
  delay(250);
  drive_voltage(0);
}

void drive_distance(double lDistance, double rDistance, double mVelocity = 200.0, bool profile = true, int flag = 0, double profileRatio = 0.7) {
  minimizeBacklash(lDistance > 0, lDistance > 0);

  auton_lfMotor.tare_position();
  auton_lbMotor.tare_position();
  auton_rfMotor.tare_position();
  auton_rbMotor.tare_position();
  if (!profile) {
    auton_lfMotor.move_velocity(mVelocity * (lDistance > 0 ? 1 : -1));
    auton_lbMotor.move_velocity(mVelocity * (lDistance > 0 ? 1 : -1));
    auton_rfMotor.move_velocity(mVelocity * (lDistance > 0 ? 1 : -1));
    auton_rbMotor.move_velocity(mVelocity * (lDistance > 0 ? 1 : -1));
  }
  bool flagRun = false;
  double movePercent = auton_lfMotor.get_position() / lDistance;
  while (fabs(movePercent) < 1 - MOVE_TOLERANCE) {
    movePercent = auton_lfMotor.get_position() / lDistance;
    if (profile) profile_drive(lDistance, 0, mVelocity, profileRatio);
    if (!flagRun) flagRun = runFlag(flag, movePercent);
    delay(5);
  }
  if (profile) profile_drive(lDistance, 0, mVelocity, profileRatio);
  auton_lfMotor.move_velocity(0);
  auton_lbMotor.move_velocity(0);
  auton_rfMotor.move_velocity(0);
  auton_rbMotor.move_velocity(0);
  runFlag(flag, 1.0);
  pros::lcd::print(2, "DONE");
}

void drive_feet(double lFeet, double rFeet, double mVelocity = 200.0, bool profile = true, int flag = 0, double profileRatio = 0.7) {
  drive_distance(lFeet / circumference, rFeet / circumference, mVelocity, profile, flag, profileRatio);
}

bool auton_intakeActive = false;
void toggle_intake() {
  auton_intakeActive ^= true;
  if (auton_intakeActive) auton_intakeMotor = 127;
  else auton_intakeMotor = 0;
}

void drive_turn(double degrees, double mVelocity = 200) {
  degrees *= TURN_SCALING;
  auton_turnGyro.reset();
  if (true) {
    if (degrees > 0) {
      minimizeBacklash(false, true);
    }
    else {
      minimizeBacklash(true, false);
    }
  }
  double movePercent = auton_turnGyro.get_value() / degrees;
  while (movePercent < 1.0 - TURN_TOLERANCE) {
    movePercent = auton_turnGyro.get_value() / degrees;
    if (movePercent < 0.0) movePercent = 0.0;
    else if (movePercent > 1.0) movePercent = 1.0;
    if (true) profile_driveTurn(degrees, mVelocity);
    delay(5);
  }
  auton_lfMotor.move_velocity(0);
  auton_lbMotor.move_velocity(0);
  auton_rfMotor.move_velocity(0);
  auton_rbMotor.move_velocity(0);
}

void drive_velocity(double targetVelocity) {
  auton_lfMotor.move_velocity(targetVelocity);
  auton_rfMotor.move_velocity(targetVelocity);
  auton_lbMotor.move_velocity(targetVelocity);
  auton_rbMotor.move_velocity(targetVelocity);
}

ADIDigitalIn auton_limitSwitch1(3);
ADIDigitalIn auton_limitSwitch2(6);

void secureBall() {
  auton_intakeMotor.move_velocity(200);
  delay(1000);
  auton_intakeMotor.move_velocity(0);
  auton_internalIntakeMotor.move_velocity(100);
  while (auton_limitSwitch2.get_value() != true) {
    delay(50);
  }
  auton_internalIntakeMotor.move_velocity(0);
}

void flipCap() {
  auton_intakeMotor.move_velocity(-200);
  delay(2500);
  drive_feet(0.7, 0.7, 200, true, 100);
  delay(2500);
  auton_intakeMotor.move_velocity(0);
}

void wallAlign() {
  drive_voltage(-65, -65);
  delay(1000);
}

void frontAlign() {
  drive_voltage(65, 65);
  delay(1000);
}

bool shouldLimitDoubleShot = true;
const int DOUBLE_SHOT_MAX_DURATION = 7000;

void auton_doubleShoot() {
  forceFlywheelOverride(true);
  activateDoubleShot();
  startNewShot();
  int ds_startTime = pros::millis();
  while (!isDoubleShotFinished()) {
    delay(20);
    if (shouldLimitDoubleShot && pros::millis() - ds_startTime > DOUBLE_SHOT_MAX_DURATION) {
      endDoubleShot();
      break;
    }
  }
  forceFlywheelOverride(false);
}
//MANUAL AUTON END

void autonomous() {
  auton_turnGyro.reset();
  auton_lfMotor.set_encoder_units(MOTOR_ENCODER_ROTATIONS);
  auton_rfMotor.set_encoder_units(MOTOR_ENCODER_ROTATIONS);
  auton_lbMotor.set_encoder_units(MOTOR_ENCODER_ROTATIONS);
  auton_rbMotor.set_encoder_units(MOTOR_ENCODER_ROTATIONS);
  auton_intakeMotor.set_encoder_units(MOTOR_ENCODER_ROTATIONS);
  auton_internalIntakeMotor.set_encoder_units(MOTOR_ENCODER_ROTATIONS);


  auton_lfMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
  auton_rfMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
  auton_lbMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
  auton_rbMotor.set_brake_mode(MOTOR_BRAKE_HOLD);

  auton_lfMotor.set_reversed(false);
  auton_rfMotor.set_reversed(true);
  auton_lbMotor.set_reversed(false);
  auton_rbMotor.set_reversed(true);

  auton_init();

  //MATCH setDoubleShotLimit(10000);
  setDoubleShotLimit(2500);

  shouldLimitDoubleShot = doClimb;

  //Recorded Auton

  setFlywheelSpeed(flywheelSpeed_auton); //Change to: changeFlywheelTarget(double);

  FILE* autonFile = fopen((std::string(AUTON_FILE_PATH_PREFIX) + autonFileNames[activeAuton]).c_str(), "r");
  fseek(autonFile, 0, SEEK_END);
  int fileSize = ftell(autonFile); //Should technically be 1 larger, but acceptable because final character is a terminating character with a fallback
  rewind(autonFile);
  char autonChars[fileSize];
  fgets(autonChars, fileSize, autonFile);
  pros::lcd::set_text(3, std::to_string(sizeof(autonChars)));
  pros::lcd::set_text(1, std::to_string(fileSize));
  fclose(autonFile);

  int activeVals[16];
  memcpy(activeVals, INITIAL_STATE, 16);
  int lastTime = -1;

  for (int i = 0; i < strlen(FILE_HEADER); i++) {
	if (autonChars[i] != FILE_HEADER[i]) {
	  pros::lcd::set_text(5, "WRONG FILE TYPE");
	  return;
	}
  }

  for (int i = strlen(FILE_HEADER); i < sizeof(autonChars); i++) {
	//Start of new line
	std::string timeString = "";
	while (getPrefixIndex(autonChars[i]) == -1) {
	  timeString += autonChars[i];
	  i += 1;
	} //Exit: current char is a prefix
	pros::lcd::set_text(5, std::to_string(i));
	int newTime = std::stoi(timeString);
	if (lastTime != -1) pros::Task::delay(newTime - lastTime);
	lastTime = newTime;

	do {
	  int pIndex = getPrefixIndex(autonChars[i]); //Get prefix
	  if (pIndex != -1) { //Should always be true due to time-acquisition code
		std::string numStr = "";
		i += 1;
		int nIndex;
		do {
		  numStr += autonChars[i]; //Assume next character will be a number
		  i += 1;
		}
		while (autonChars[i] != ';');
		activeVals[pIndex] = std::stoi(numStr);
		i += 1; //Increment, for ";!" case and to reach next prefix
	  }
	  else {
		//End of file
		break;
		//i += 1; //This should only be reached at end of file, but also allows for error handling.
	  }
	}
	while (autonChars[i] != '!');
	runDrive(activeVals[0], activeVals[1], activeVals[2], activeVals[3],
			  activeVals[4], activeVals[5], activeVals[6], activeVals[7],
			  activeVals[8], activeVals[9], activeVals[10], activeVals[11],
			  activeVals[12], activeVals[13], activeVals[14], activeVals[15]);
  }

  memcpy(activeVals, INITIAL_STATE, 16); //TODO: Make the "16" changeable for additional parameters?
  runDrive(activeVals[0], activeVals[1], activeVals[2], activeVals[3],
			activeVals[4], activeVals[5], activeVals[6], activeVals[7],
			activeVals[8], activeVals[9], activeVals[10], activeVals[11],
			activeVals[12], activeVals[13], activeVals[14], activeVals[15]);
}
