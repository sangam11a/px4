#include "phi.h"
#include <poll.h>
#include <string.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_magnetometer.h>
// #include<uORB/topics/vehicle_.h>
// #include <uORB/topics/>
#include </home/sangam/PX4-Autopilot/src/examples/hello/Px4.c>
#include</home/sangam/PX4-Autopilot/src/examples/hello/RCFilter.c>

#include</home/sangam/PX4-Autopilot/src/examples/hello/EKF.c>
// typedef struct {

// 	float phi_r;
// 	float theta_r;

// 	float P[2][2];

// 	float Q[2];
// 	float R[3];

// } EKF;

// void RCFilter_Init(RCFilter *filt, float cutoffFreqHz, float sampleTimeS) {

// 	/* Compute equivalent 'RC' constant from cut-off frequency */
// 	float RC = 1.0f / (6.28318530718f * cutoffFreqHz);

// 	/* Pre-compute filter coefficients for first-order low-pass filter */
// 	filt->coeff[0] = sampleTimeS / (sampleTimeS + RC);
// 	filt->coeff[1] = RC / (sampleTimeS + RC);

// 	/* Clear output buffer */
// 	filt->out[0] = 0.0f;
// 	filt->out[1] = 0.0f;

// }

// float RCFilter_Update(RCFilter *filt, float inp) {

// 	/* Shift output samples */
// 	filt->out[1] = filt->out[0];

// 	/* Compute new output sample */
// 	filt->out[0] = filt->coeff[0] * inp + filt->coeff[1] * filt->out[1];

// 	/* Return filtered sample */
// 	return (filt->out[0]);

// }

// void IMU_ReadAccelerometerComplete(Px4 *imu) {



// 	/* Filter accelerometer data */
// 	RCFilter_Update(&lpfAcc[0], imu->acc_mps2[0]);
// 	RCFilter_Update(&lpfAcc[1], imu->acc_mps2[1]);
// 	RCFilter_Update(&lpfAcc[2], imu->acc_mps2[2]);

// 	//Filtered accelerometer measurement
// 	float ax_mps2 = lpfAcc[0].out[0];
// 	float ay_mps2 = lpfAcc[1].out[0];
// 	float az_mps2 = lpfAcc[2].out[0];
// 	PX4_INFO("%f %f %f",(double)ax_mps2,(double)ay_mps2,(double)az_mps2);
// 	PX4_INFO("%f %f %f",(double)lpfAcc[0].out[1],(double)lpfAcc[1].out[1],(double)lpfAcc[2].out[1]);
// 	// PX4_INFO("");
// 	// PX4_INFO("");


// 	/*Calculate roll (phi) and pitch(theta) angle estimates using filtered accelerometer readings*/
// 	phiHat_deg_ = atanf(ay_mps2 / az_mps2) * RAD_TO_DEG;
// 	thetaHat_deg_ = asinf(ax_mps2 / G_MPS2) * RAD_TO_DEG;
// 	PX4_INFO("PhiHat degree : %f",(double)phiHat_deg_);
// 	PX4_INFO("ThetaHat degree : %f",(double)thetaHat_deg_);

// }
// void IMU_ReadGyroscopeComplete(Px4 *imu) {
// 	float phiHat_rad_ = 0.0f;
// 	float thetaHat_rad_ = 0.0f;


// 	/* Pre-compute gyroscope conversion constant (raw to rad/s) */
// 	imu->gyrConversion = 0.01745329251f / 131.0f; /* Datasheet page 39 */

// 	// /* Form signed 16-bit integers */
// 	// int16_t gyrX = (int16_t) ((imu->gyrRxBuf[0] << 8) | imu->gyrRxBuf[1]);
// 	// int16_t gyrY = (int16_t) ((imu->gyrRxBuf[2] << 8) | imu->gyrRxBuf[3]);
// 	// int16_t gyrZ = (int16_t) ((imu->gyrRxBuf[4] << 8) | imu->gyrRxBuf[5]);

// 	// /* Convert to deg/s */
// 	// imu->gyr_rps[0] = imu->gyrConversion * gyrX;
// 	// imu->gyr_rps[1] = imu->gyrConversion * gyrY;
// 	// imu->gyr_rps[2] = imu->gyrConversion * gyrZ;

// 	/* Filter gyroscope data */
// 	RCFilter_Update(&lpfGyr[0], imu->gyr_rps[0]);
// 	RCFilter_Update(&lpfGyr[1], imu->gyr_rps[1]);
// 	RCFilter_Update(&lpfGyr[2], imu->gyr_rps[2]);

// 	//Filtered accelerometer measurement
// 	float p_rps = lpfGyr[0].out[0];
// 	float q_rps = lpfGyr[1].out[0];
// 	float r_rps = lpfGyr[2].out[0];

// 	//Transform body rates to Euler rates to get estimate of roll and pitch angles
// 	float phiDot_rps = p_rps
// 			+ tanf(thetaHat_rad_)
// 					* (sinf(phiHat_rad_) * q_rps + cosf(phiHat_rad_) * r_rps);
// 	float thetaDot_rps = cosf(phiHat_rad_) * q_rps - sinf(phiHat_rad_) * r_rps;

// 	//Integrate Euler rates to get estimate of roll and pitch angles
// 	phiHat_rad_ = (phiHat_rad_ + (1000 / 1000.0f) * phiDot_rps)
// 			* RAD_TO_DEG;
// 	thetaHat_rad_ = (thetaHat_rad_
// 			+ (1000 / 1000.0F) * thetaDot_rps) * RAD_TO_DEG;

// 	PX4_INFO("PhiHat rad : %f",(double)phiHat_deg_);
// 	PX4_INFO("ThetaHat rad : %f",(double)thetaHat_deg_);

// }


int HelloModule::print_status(){
	PX4_INFO("Running HelloModule");
	return 0;
}


int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
         PX4_INFO("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        PX4_INFO("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        PX4_INFO("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        PX4_INFO("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}

int uart_init(const char *uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        PX4_ERR("failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}


int HelloModule::custom_command(int argc, char *argv[]){
	PX4_INFO("custom command argc:%i argv:%s",argc,argv[0]);
	 char str[255];
	 int  j=0;
	  char* c;
    //UART open
    int uart = uart_init("/dev/ttyS3"); //FTDI module comm ttyS2

    PX4_INFO("uart first :%i ",uart);
    if(false == uart)return -1;
    if(false == set_uart_baudrate(uart,115200)){
            printf("[YCM]set_uart_baudrate is failed\n");
        //     return -1;
    }
    PX4_INFO("UART %i",uart);
//     else{
	// if(!is_running()){
	// 	print_usage("phicommander is not working");
	// 	PX4_INFO("phicommander is not working");
	// 	return 1;
	// }
	int get_accelerometer_data = orb_subscribe(ORB_ID(sensor_combined));
	int get_temperature_data = orb_subscribe(ORB_ID(sensor_accel));
	int get_magnetometer_data = orb_subscribe(ORB_ID(vehicle_magnetometer));


	px4_pollfd_struct_t fds[] = {
    { .fd = get_accelerometer_data,   .events = POLLIN },
    { .fd = get_temperature_data,   .events = POLLIN },
    { .fd = get_magnetometer_data,   .events = POLLIN },
    /* there could be more file descriptors here, in the form like:
     * { .fd = other_sub_fd,   .events = POLLIN },
     */
    };

       int poll_ret = px4_poll(fds, 3, 1000);

	struct sensor_combined_s raw;
	struct sensor_accel_s acceleration;
	struct vehicle_magnetometer_s mag;
	// PX4_INFO("%i",strcmp(argv[0],"temperature"));
	// PX4_INFO("%i",strcmp(argv[0],"temp"));
	// PX4_INFO("%i",strcmp(argv[0],"magnetometer"));
	if(strcmp(argv[0],"accelerometer")==0){
		PX4_INFO("");
		if(poll_ret > 0){
			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_combined), get_accelerometer_data, &raw);
				//  struct Px4 *imu;
				for (int n = 0; n < 3; n++) {

					RCFilter_Init(&lpfAcc[n], 5.0f, 0.01f);

					RCFilter_Init(&lpfGyr[n], 25.0f, 0.01f);

				}
				Px4 myImu;
				Px4 *imu = &myImu;
				imu->acc_mps2[0] = (float)raw.accelerometer_m_s2[0];
				imu->acc_mps2[1] = (float)raw.accelerometer_m_s2[1];
				imu->acc_mps2[2] = (float)raw.accelerometer_m_s2[2];

				imu->gyr_rps[0] = (float)raw.gyro_rad[0];
				imu->gyr_rps[1] = (float)raw.gyro_rad[1];
				imu->gyr_rps[2] = (float)raw.gyro_rad[2];

				EKF myEkf;
				EKF *ekf = &myEkf;
				// IMU_ReadAccelerometerComplete(imu);
				// IMU_ReadGyroscopeComplete(imu);
				// Comlementary_Filter(imu);
				EKF_AccelGyro(imu, ekf);

				for(int i=0;i<3;i++){
					switch(i){
						case 0:
						sprintf(str, "Ax:%8.4f Gx : %8.4f\n\t",(double)raw.accelerometer_m_s2[0],(double)raw.gyro_rad[0]);//Data from accelerometer:\n
						break;
						case 1:
						sprintf(str, "Ay:%8.4f Gy:%8.4f\n\t",(double)raw.accelerometer_m_s2[1],(double)raw.gyro_rad[1]);
						break;
						case 2:
						sprintf(str, "Az:%8.4f Gz:%8.4f\n\t",(double)raw.accelerometer_m_s2[2],(double)raw.gyro_rad[2]);
						break;
					}
					PX4_INFO("%s",str);
					j=0;
					//  char* c;
						// do {
						// 	c = &str[j];
						// 	j++;
						// 	write(uart, c, 1);
						// } while (*c != '\t');
				}
			}
                // close(uart);
		}
		if(argc>2){
			if (strcmp(argv[1],"n")==0 ||strcmp(argv[1],"-n")==0 || strcmp(argv[1],"- n")){
				{
					PX4_INFO("Got -n as second arg");
				}
			}
		}
		return 0;
	}

	else if(strcmp(argv[0],"gyro")==0){
		if(poll_ret > 0){
                   if (fds[0].revents & POLLIN) {
                        orb_copy(ORB_ID(sensor_combined), get_accelerometer_data, &raw);
                        for(int i=0;i<3;i++){
				switch(i){

					case 0:
					sprintf(str, "Data from Gyroscope  :\nGx:%8.4f\n\t",(double)raw.gyro_rad[0]);
					break;
					case 1:
					sprintf(str, "Gy:%8.4f\n\t",(double)raw.gyro_rad[1]);
					break;
					case 2:
					sprintf(str, "Gz:%8.4f\n\t",(double)raw.gyro_rad[2]);
					break;
				}

				j=0;

				do {
					c = &str[j];
					j++;
					write(uart, c, 1);
				} while (*c != '\t');

				PX4_INFO("%s",str);
			}
		   }
		}

		PX4_INFO("Got gyroscope data");
		return 0;

	}

	else if(strcmp(argv[0],"magnetometer")==0){
		PX4_INFO("poll ret :%i",poll_ret);
		if(poll_ret > 0){
                   if (fds[2].revents & POLLIN) {
                        orb_copy(ORB_ID(vehicle_magnetometer), get_magnetometer_data, &mag);

			PX4_INFO("xmag :%f",(double)mag.magnetometer_ga[0]);
                        for(int i=0;i<3;i++){
				switch(i){

					case 0:
					sprintf(str, "Data from magnetometer  :\nMagx:%8.4f\n\t",(double)mag.magnetometer_ga[0]);
					break;
					case 1:
					sprintf(str, "Magy:%8.4f\n\t",(double)mag.magnetometer_ga[1]);
					break;
					case 2:
					sprintf(str, "Magz:%8.4f\n\t",(double)mag.magnetometer_ga[2]);
					break;
				}

				j=0;

				do {
					c = &str[j];
					j++;
					write(uart, c, 1);
				} while (*c != '\t');

				PX4_INFO("magnetometer %s",str);
			}
		   }
		}
		else{
			PX4_INFO("Got some srror mag");
		}

		PX4_INFO("Got magnetometer data");
		return 0;

	}

      else if(strcmp(argv[0],"temperature")==0||strcmp(argv[0],"temp")==0){
		if(poll_ret > 0){
                   if (fds[1].revents & POLLIN) {
                        	orb_copy(ORB_ID(sensor_accel), get_temperature_data, &acceleration);

				sprintf(str, "System's Temperature : %8.4f C\n\t",(double)acceleration.temperature);
				j=0;
				do {
					c = &str[j];
					j++;
					write(uart, c, 1);
				} while (*c != '\t');
				PX4_INFO("%s",str);
			}
		}
	}


	else{
		// PX4_INFO(strcmp(argv[0],"temperature"));
		PX4_WARN("Unknown cmd 123");
	}
	 close(uart);

	return print_usage("unknown command");
}


int HelloModule::task_spawn(int argc, char *argv[]){
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


HelloModule *HelloModule::instantiate(int argc, char *argv[]){ //looks after -dash
	const char *myoptarg = nullptr;
	PX4_INFO("Instatiated");
	int ch;
	int myoptind = 1;
	int example_param = 0;
	bool example_flag = false;

	while((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg))!= EOF){
		switch (ch)
		{
		case 'f': example_param = (int)strtol(myoptarg,nullptr,10);
			/* code */
			PX4_INFO("gOT F AS AN ARG");
			break;
		case 'p': example_flag = true;
			/* code */
			break;
		case 'n':PX4_INFO("got n as arg");
		        break;

		default:
			PX4_WARN("unrecognized flag");
			break;
		}
	}
	// if(error_flag){
	// 	return nullptr;
	// }
	HelloModule *instance = new HelloModule(example_param,example_flag);

	if(instance == nullptr){
		PX4_ERR("alloc failed");
	}
	return instance;
}

HelloModule::HelloModule(int example_param, bool example_flag)
:ModuleParams(nullptr){

}

void HelloModule::run(){
	PX4_INFO("Commander module runs ");
	// Got   argc:%i arrrgv: %s",_argc,_argv[0]);

}

// void HelloModule::parameters_update(bool force)
// {
// 	// check for parameter updates
// 	if (_parameter_update_sub.updated() || force) {
// 		// clear update
// 		parameter_update_s update;
// 		_parameter_update_sub.copy(&update);

// 		// update parameters from storage
// 		updateParams();
// 	}
// }

int HelloModule::print_usage(const char *reason)
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
		$ hello start

		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hello", "template");
	// PRINT_MODULE_USAGE_COMMAND("start");
	// PRINT_MODULE_USAGE_COMMAND("accelerometer");
	PRINT_MODULE_USAGE_COMMAND_DESCR("accelerometer", "print accelerometer sensor data from satellite");
	PRINT_MODULE_USAGE_COMMAND_DESCR("gyro", "print gyro  data from satellite");
	PRINT_MODULE_USAGE_COMMAND_DESCR("temperature", "print temperature  data from satellite");
	// PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	// PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int hello_main(int argc, char *argv[])
{
	return HelloModule::main(argc, argv);
}

