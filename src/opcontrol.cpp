#include "main.h"
#include "PID.h"

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
void opcontrol() {

	pros::Controller master (pros::E_CONTROLLER_MASTER);

	pros::Motor flywheel_mtr1 (1);
	pros::Motor flywheel_mtr2 (2);
	PID flywheelPID (0.2,0.1,0,-127,127,-127,127,0);

	int targetrpm = 3600;
  double lastpos = 0;
	int lasttime = pros::millis();
	int hundredcounter = 0;
	int motorpower = 120;
	int oldmotorpower = 120;
	double flywheel_vels[3];
	for(int i = 0; i < 3; i++) {
		flywheel_vels[i] = 0;
	}
	int flywheel_vels_pos = 0;

	while (true) {

		flywheel_mtr1.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
		double pos = flywheel_mtr2.get_position();
		int currtime = pros::millis();
		if(currtime-lasttime > 1) {
			flywheel_vels[flywheel_vels_pos] = (pos-lastpos)/((currtime-lasttime)/1000.0)/300.0*60.0*(80.0/12.0);
			flywheel_vels_pos = (flywheel_vels_pos+1)%5;
			double vels_sum = 0;
			for(int i = 0 ; i < 3; i++) {
				vels_sum += flywheel_vels[i];
			}
			double flywheel_velocity = vels_sum/3.0;
			lasttime = currtime;
			lastpos = pos;
			pros::lcd::set_text(0,"RPM: " + std::to_string(flywheel_velocity));
			flywheelPID.compute(3600, flywheel_velocity);
	  }
		flywheel_mtr1.move(static_cast<int>(-flywheelPID.getOutput()));
		flywheel_mtr2.move(static_cast<int>(flywheelPID.getOutput()));
		pros::lcd::set_text(1, "Output: " + std::to_string(flywheelPID.getOutput()));
		// if(currtime > hundredcounter){
		// 	std::cout << currtime << "\t" << flywheel_velocity << std::endl;
		// 	hundredcounter += 100;
		// }
		pros::delay(20);
	}
}
