#include "main.h"
#include "dataHandler.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

 bool gyroHasInitialized = false;

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "9932ETIN!");

	if (!gyroHasInitialized) {
		ADIGyro climbGyro_init(8, 0.1);
		ADIGyro turnGyro_init(7, 0.1);
		delay(1500);
	}

	/*
	FILE* parameterFile = fopen(PARAMETER_FILE_PATH, "r");
  fseek(parameterFile, 0, SEEK_END);
  int fileSize = ftell(parameterFile) + 1; //TODO: Clean up ("+ 1")
  rewind(parameterFile);
  char parameterChars[fileSize];
  fgets(parameterChars, fileSize, parameterFile);
  fclose(parameterFile);

	int paramNumber = 0;
	int i = 0;
	while (i < fileSize - 1) {
		std::string readStr = "";
		while (parameterChars[i] != ';' && i < fileSize - 1) {
			readStr += parameterChars[i];
			i += 1;
		}
		switch (paramNumber) {
			case 0:
				flywheelSpeed_auton = std::stoi(readStr);
				break;
			case 1:
				flywheelSpeed_drive = std::stoi(readStr);
				break;
			case 2:
				flywheelSpeed_driveMiddle = std::stoi(readStr);
			case 3:
				isBlue = (readStr == "blue");
				readStr += '!';
				break;
		}
		paramNumber += 1;
		i += 1; // For the ';'
	}*/
	//delay(1500); //Allow gyro calibration to complete
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */

int activeAuton = autonCount - 1; //Default: Test_Local
std::string autonNames[autonCount] = {
	"Blue Front",
	"Red Front",
	"Blue Back",
	"Red Back",
	"Test_Local"
};

void changeAuton(int deltaNum) {
	activeAuton += deltaNum;
	if (activeAuton >= autonCount) activeAuton -= autonCount;
	else if (activeAuton < 0) activeAuton += autonCount;
	pros::lcd::clear_line(3);
	pros::lcd::print(3, "Active Auton: %s", autonNames[activeAuton]);
}

void on_left_button() {
	changeAuton(-1);
}

bool doClimb = true;
void toggleClimb() {
	doClimb ^= true;
	pros::lcd::print(4, "Climbing: %s", doClimb ? "Yes" : "No");
}
void on_center_button() {
	toggleClimb();
}

void on_right_button() {
	changeAuton(1);
}

void competition_initialize() {
	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn2_cb(on_right_button);
	auto brakeMotor1 = 15_mtr;
	auto brakeMotor2 = 8_rmtr;
	auto brakeMotor3 = 19_mtr;
	auto brakeMotor4 = 6_rmtr;
	brakeMotor1.set_brake_mode(MOTOR_BRAKE_HOLD);
	brakeMotor2.set_brake_mode(MOTOR_BRAKE_HOLD);
	brakeMotor3.set_brake_mode(MOTOR_BRAKE_HOLD);
	brakeMotor4.set_brake_mode(MOTOR_BRAKE_HOLD);
	activeAuton = 0; //Set initial selection to first auton file
	changeAuton(0); //Update display

	ADIGyro climbGyro_init(8, 0.1);
	ADIGyro turnGyro_init(7, 0.1);
	delay(1500);
	gyroHasInitialized = true;

	toggleClimb();
	toggleClimb();
}
