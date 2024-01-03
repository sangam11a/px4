#include "phi_commander.h"
#include <poll.h>
#include <string.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_acceleration.h>
// #include <uORB/topics/>


int PhiCommanderModule::print_status(){
	PX4_INFO("Running PHiCommanderModule");
	return 0;
}

int PhiCommanderModule::custom_command(int argc, char *argv[]){
	if(!is_running()){
		print_usage("phicommander is not working");
		return 1;
	}

	if(!strcmp(argv[0],"start")){
		PX4_INFO("got start as a first arg");
		return 0;
	}
	elseif(!strcmp(argv[0],"accelerate")){
		PX4_INFO("Got acceleration data");
		return 0;

	}
	
	return print_usage("unknown command");
}


int PhiCommanderModule::task_spawn(int argc, char *argv[]){
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}


PhiCommanderModule *PhiCommanderModule::instantiate(int argc, char *argv[]){
	const char *myoptarg = nullptr;
	int ch;
	int myoptind = 1;
	int example_param = 0;
	bool example_flag = false;

	while((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg))!= EOF){
		switch (ch)
		{
		case "f": example_param = (int)strtol(myoptarg,nullptr,10);
			/* code */
			break;
		case "p": example_flag = true;
			/* code */
			break;

		default:
			PX4_WARN("unrecognized flag");
			break;
		}
	}
	if(error_flag){
		return nullptr;
	}
	PhiCommanderModule *instance = new PhiCommanderModule(example_param,example_flag);

	if(instance == nullptr){
		PX4_ERR("alloc failed");
	}
	return instance;
}

PhiCommanderModule::PhiCommanderModule(int example_param, bool example_flag)
:ModuleParams(nullptr){

}

void PhiCommanderModule::run(){
	PX4_INFO("Commander module runs ");
}

void PhiCommanderModule::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int PhiCommanderModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.
Phi Commander Module

This is a template for a phi commander module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ phi_commander start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int phi_commander_main(int argc, char *argv[])
{
	return PhiCommanderModule::main(argc, argv);
}

