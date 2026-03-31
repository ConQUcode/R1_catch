#include "chassis.h"
#include "shoot.h"
#include "arm.h"
#include "DJI_motor.h"
#include "bsp_dwt.h"
#include "remote.h"
#include "ins_task.h"
#include "Test.h"
#include "catch.h"

extern RC_ctrl_t *rc_cmd;

void all_init_Task(){
	
catch_init();
	
}
void all_cmd_Task(){

	  catch_all();
		DJIMotorControl();
}