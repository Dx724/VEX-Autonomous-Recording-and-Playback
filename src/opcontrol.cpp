#include "main.h"
#include "dataHandler.hpp"


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

std::string activeData = FILE_HEADER;

void on_center_button() {
	return;
	 FILE* tFile = fopen(AUTON_FILE_WRITE_PATH, "w");
	 fputs(activeData.c_str(), tFile);
	 fclose(tFile);
	 pros::lcd::set_text(5, "Data saved!");
	 pros::lcd::set_text(7, activeData.c_str());
}

void opcontrol() {
	setFlywheelSpeed(flywheelSpeed_drive);

	int lastVals[16], currVals[16];
	memcpy(lastVals, INITIAL_STATE, 16);
	memcpy(currVals, INITIAL_STATE, 16);
	bool changed[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr(1);
	pros::Motor right_mtr(2);

	unbrakeDrive();

	while (true) {
		currVals[0] = master.get_analog(ANALOG_LEFT_Y);
		currVals[1] = master.get_analog(ANALOG_LEFT_X);
		currVals[2] = master.get_analog(ANALOG_RIGHT_Y);
		currVals[3] = master.get_analog(ANALOG_RIGHT_X);
		currVals[4] = master.get_digital(DIGITAL_UP);
		currVals[5] = master.get_digital(DIGITAL_DOWN);
		currVals[6] = master.get_digital(DIGITAL_LEFT);
		currVals[7] = master.get_digital(DIGITAL_RIGHT);
		currVals[8] = master.get_digital(DIGITAL_X);
		currVals[9] = master.get_digital(DIGITAL_B);
		currVals[10] = master.get_digital(DIGITAL_Y);
		currVals[11] = master.get_digital(DIGITAL_A);
		currVals[12] = master.get_digital(DIGITAL_L1);
		currVals[13] = master.get_digital(DIGITAL_L2);
		currVals[14] = master.get_digital(DIGITAL_R1);
		currVals[15] = master.get_digital(DIGITAL_R2);

		bool valChanged = false;

		//Arcade Drive | Otherwise Tank Drive
		//currVals[0] = (master.get_analog(ANALOG_LEFT_Y) + currVals[3] > 127 ? 127 : (master.get_analog(ANALOG_LEFT_Y) + currVals[3] < -127 ? -127 : master.get_analog(ANALOG_LEFT_Y) + currVals[3]));
		//currVals[2] = (master.get_analog(ANALOG_LEFT_Y) - currVals[3] > 127 ? 127 : (master.get_analog(ANALOG_LEFT_Y) - currVals[3] < -127 ? -127 : master.get_analog(ANALOG_LEFT_Y) - currVals[3]));

		//Dual Arcade Drive
	/*	int avgVertical = round((master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_Y)) / 2.0);
		int avgHorizontal = round((master.get_analog(ANALOG_LEFT_X) + master.get_analog(ANALOG_RIGHT_X)) / 2.0);
		currVals[0] = avgVertical + avgHorizontal;
		currVals[2] = avgVertical - avgHorizontal;
		currVals[0] = currVals[0] > 127 ? 127 : (currVals[0] < -127 ? -127 : currVals[0]);
		currVals[2] = currVals[2] > 127 ? 127 : (currVals[2] < -127 ? -127 : currVals[2]);
			*/

		/* Data recording start */
		for (int cIdx = 0; cIdx < 16; cIdx++) {
			if (currVals[cIdx] != lastVals[cIdx]) {
				pros::lcd::set_text(4, "Data changed");
				pros::lcd::set_text(6, activeData.c_str());
				if (!valChanged) {
					activeData += std::to_string(pros::millis());
				}
				valChanged = true;
				activeData += prefixes[cIdx];
				activeData += std::to_string(currVals[cIdx]);
				activeData += ";";
			}
		}
		if (valChanged) {
			activeData += '!';
		}

		memcpy(lastVals, currVals, sizeof(currVals));

		/* Data recording end */

		runDrive(currVals[0], currVals[1], currVals[2], currVals[3],
							currVals[4], currVals[5], currVals[6], currVals[7],
							currVals[8], currVals[9], currVals[10], currVals[11],
							currVals[12], currVals[13], currVals[14], currVals[15]);

		pros::delay(20);
	}
}
