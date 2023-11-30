
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>

// #include<uORB/topics/satelliteinfo.h>

// #include<ulog/log.h>
__EXPORT int px4_testing_main(int argc, char *argv[]);

int px4_testing_main(int argc, char *argv[])
{
// 	px4_logger_t *logger = px4_logger_create("first_log");
// 	px4_logger_write(logger,"Mock data");
// 	px4_logger_destroy(logger);

// 	PX4_INFO("Hello Sky!");
// 	int sensor_sub_fd= orb_subscribe(ORB_ID(sensor_combined));
// 	int sat_sub_fd = orb_subscribe(ORB_ID(satelliteinfo));
// 	orb_set_interval(sensor_sub_fd,200);
// 	struct satelliteinfo sat;
// 	memset(&sat,0,sizeof(sat));
	struct vehicle_attitude_s att;
	memset(&att,0,sizeof(att));
// 	// orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude));
// 	px4_pollfd_struct_t fds[] = {{
// 		.fd = sensor_sub_fd,
// 		.events = POLLIN
// 	},
// 	};
// 	int error_counter = 0;
// 	for (int i = 0; i<5 ; i++){
// 		int poll_ret = px4_poll(fds,1,1000);
// 		if(poll_ret == 0){
// 			PX4_ERR("Got no data within a second");
// 		}
// 		else if(poll_ret < 0){
// 			if(error_counter < 10 || error_counter % 50 == 0){
// 				PX4_ERR("Error return value from  poll () : %d",poll_ret);
// 			}
// 			error_counter++;

// 		}
// 		else{
// 			if (fds[0].revents & POLLIN){
// 				struct sensor_combined_s raw;
// 				orb_copy(ORB_ID(sensor_combined),sensor_sub_fd,&raw);
// 				PX4_INFO("Accelerometer : \t%8.4f\t%8.4f\t%8.4f",
// 				(double)raw.accelerometer_m_s2[0],
// 				(double) raw.accelerometer_m_s2[1],
// 				(double)raw.accelerometer_m_s2[2]
// 				);

// 			}
// 		}
// 	}
	// ulog_t *ulog =ulog_create("first","/tmp/first.txt");
	// ulog_write(ulog,"sjdlfjsldjflsdj");
	// ulog_destroy(ulog);
	return 0;
}
