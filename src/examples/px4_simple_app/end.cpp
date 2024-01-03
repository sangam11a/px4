#include <poll.h>
#include <string.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdbool.h>


#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_magnetometer.h>

#define UART_DEV_PATH "/dev/ttyS2"

int uart_init(char *uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        PX4_ERR("failed to open port: %s", uart_name);
        return -1;
    }

    struct termios uart_config;

    tcgetattr(serial_fd, &uart_config);

    cfsetispeed(&uart_config, B115200);
    cfsetospeed(&uart_config, B115200);

    uart_config.c_cflag &= ~PARENB;
    uart_config.c_cflag &= ~CSTOPB;
    uart_config.c_cflag &= ~CSIZE;
    uart_config.c_cflag |= CS8;

    tcsetattr(serial_fd, TCSANOW, &uart_config);

    return serial_fd;
}

int px4_simple_app_main(int argc, char *argv[])
{
    // UART open
    int uart = uart_init(UART_DEV_PATH);
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
	// struct sensor_accel_s acceleration;
	struct vehicle_magnetometer_s mag;
	PX4_INFO("%i",strcmp(argv[0],"temperature"));
	PX4_INFO("%i",strcmp(argv[0],"temp"));
	PX4_INFO("%i",strcmp(argv[0],"magnetometer"));

    PX4_INFO("UART file descriptor: %d", uart);

    if (uart < 0) {
        PX4_ERR("Failed to initialize UART");
        return -1;
    }

    char buff[255];
    ssize_t ret;
    char str[255];
    int j = 0;
    char* c;
    while (true)
    {
        // Read from UART
        ret = read(uart, buff, sizeof(buff));

        if (ret > 0) {
            // Null-terminate the received data
                        buff[ret] = '\0';

                        // Print received data
                        // PX4_INFO("Received data: %s", buff);

                        // Compare received data with commands
                        if (strcmp(buff, "phit1") == 0) {
                            PX4_INFO("Received command 1");
                            if(poll_ret > 0){
                            if (fds[0].revents & POLLIN) {
                                orb_copy(ORB_ID(sensor_combined), get_accelerometer_data, &raw);
                                for(int i=0;i<3;i++){
                                    switch(i){
                                        case 0:
                                        sprintf(str, "Data from accelerometer:\n Ax:%8.4f\n\t",(double)raw.accelerometer_m_s2[0]);
                                        break;
                                        case 1:
                                        sprintf(str, "Ay:%8.4f\n\t",(double)raw.accelerometer_m_s2[1]);
                                        break;
                                        case 2:
                                        sprintf(str, "Az:%8.4f\n\t",(double)raw.accelerometer_m_s2[2]);
                                        break;
                                    }

                                    j=0;
                                    //  char* c;
                                        do {
                                            c = &str[j];
                                            j++;
                                            write(uart, c, 1);
                                        } while (*c != '\t');
                                    }
                                }
                                // close(uart);
                            }
                     }
              }
            else if (strcmp(buff, "phit2") == 0) {
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

            }
             else if (strcmp(buff, "phit3") == 0) {
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
            }
            else{
                PX4_WARN("Unknown command");
                break;
            }
    }

        usleep(1000000); // Sleep for 100ms, adjust as needed
    // }

    // close(uart);

    return 0;
}
