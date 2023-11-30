// /****************************************************************************
//  *
//  *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions
//  * are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in
//  *    the documentation and/or other materials provided with the
//  *    distribution.
//  * 3. Neither the name PX4 nor the names of its contributors may be
//  *    used to endorse or promote products derived from this software
//  *    without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
//  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  *
//  ****************************************************************************/

// /**
//  * @file myserial.c
//  *  Homemade serial link
//  * @author Leroy Theophile - leroy.theophile@gmail.com
//  */
// // #include <px4_config.h>
// // #include <tasks.h>
// #include <px4_posix.h>
// #include <px4_log.h>

// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>

// #include <unistd.h>
// #include <stdio.h>
// #include <stdarg.h>
// #include <poll.h>
// #include <string.h>
// #include <math.h>

// #include <termios.h>
// #include <unistd.h>
// #include <stdbool.h>
// #include <errno.h>
// #include <drivers/drv_hrt.h>

// #include <uORB/uORB.h>
// #include <uORB/topics/sensor_combined.h>
// /**
//  * Get values from  sensors
//  *
//  */
// void getValue(double *data);
// /**
//  * Main loop of daemon
//  */
// int myserial_thread_main(int argc, char *argv[]);
// /**
//  * Print the correct usage.
//  */
// static void usage(const char *reason);
// /**
//  * ...
//  */
// int set_uart_baudrate(const int fd, unsigned int baud);
// /**
//  * ...
//  */
// static int uart_init(char * uart_name);


// __EXPORT int myserial_main(int argc, char *argv[]);

// /* static variables */
// static bool thread_should_exit = false;        /**< Daemon exit flag */
// static bool thread_running = false;        /**< Daemon status flag */
// static int deamon_task;                /**< Handle of deamon task / thread */

// int set_uart_baudrate(const int fd, unsigned int baud)
// {
//     int speed;

//     switch (baud) {
//         case 9600:   speed = B9600;   break;
//         case 19200:  speed = B19200;  break;
//         case 38400:  speed = B38400;  break;
//         case 57600:  speed = B57600;  break;
//         case 115200: speed = B115200; break;
//         default:
//             warnx("ERR: baudrate: %d\n", baud);
//             return -EINVAL;
//     }

//     struct termios uart_config;

//     int termios_state;

//     /* fill the struct for the new configuration */
//     tcgetattr(fd, &uart_config);
//     /* clear ONLCR flag (which appends a CR for every LF) */
//     uart_config.c_oflag &= ~ONLCR;
//     /* no parity, one stop bit */
//     uart_config.c_cflag &= ~(CSTOPB | PARENB);
//     /* set baud rate */
//     if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
//         warnx("ERR: %d (cfsetispeed)\n", termios_state);
//         return false;
//     }

//     if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
//         warnx("ERR: %d (cfsetospeed)\n", termios_state);
//         return false;
//     }

//     if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
//         warnx("ERR: %d (tcsetattr)\n", termios_state);
//         return false;
//     }

//     return true;
// }

// int uart_init(char * uart_name)
// {
//     int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

//     if (serial_fd < 0) {
//         err(1, "failed to open port: %s", uart_name);
//         return false;
//     }
//     return serial_fd;
// }

// void
// usage(const char *reason)
// {
//     if (reason) {
//         fprintf(stderr, "%s\n", reason);
//     }

//     fprintf(stderr, "usage: myserial {start|stop}\n\n");
//     exit(1);
// }

// int
// myserial_main(int argc, char *argv[])
// {

//     if (argc < 2) {
//         usage("missing command");
//     }

//     if (!strcmp(argv[1], "start")) {

//         if (thread_running) {
//             warnx("running");
//             /* this is not an error */
//             exit(0);
//         }

//         thread_should_exit = false;
//         deamon_task = px4_task_spawn_cmd("myserial",
//                         SCHED_DEFAULT,
//                         SCHED_PRIORITY_DEFAULT,
//                         3000,
//                         myserial_thread_main,
//                         (char * const *)&argv[0]);
//         thread_running = true;
//         exit(0);
//     }

//     if (!strcmp(argv[1], "stop")) {
//         thread_should_exit = true;
//         exit(0);
//     }

//     if (!strcmp(argv[1], "status")) {
//         if (thread_running) {
//             warnx("running");

//         } else {
//             warnx("not started");
//         }

//         exit(0);
//     }



// return 0;
// }

// int
// myserial_thread_main(int argc, char *argv[])
// {

//     char str[500]={0};
//     //UART open
//     int uart = uart_init("/dev/ttyS6");
//     if(false == uart)return -1;
//     if(false == set_uart_baudrate(uart,9600)){
//             printf("[YCM]set_uart_baudrate is failed\n");
//             return -1;
//     }
//     printf("[YCM]uart init is successful\n");
//      //QUEUE open -- does not work
//     //mqd_t mqret = mq_open("xQueue",O_RDONLY);

//     //UORB request

//     /* subscribe to sensor_combined topic */
//     int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
//     /* limit the update rate to 5 Hz */
//     orb_set_interval(sensor_sub_fd, 200);

//     px4_pollfd_struct_t fds[] = {
//     { .fd = sensor_sub_fd,   .events = POLLIN },
//     /* there could be more file descriptors here, in the form like:
//      * { .fd = other_sub_fd,   .events = POLLIN },
//      */
//     };
//     //main thread loop
//     while (!thread_should_exit) {

//         //Receive from message queue "xQueue"
//         // mq_receive(mqret, (void *) data, sizeof(data),NULL);

//         int poll_ret = px4_poll(fds, 1, 1000);
//         if(poll_ret > 0){
//             if (fds[0].revents & POLLIN) {
//                 /* obtained data for the first file descriptor */
//                 struct sensor_combined_s raw;
//                 /* copy sensors raw data into local buffer */
//                 orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);

//                 for(int i=0;i<6;i++){
//                     switch(i){
//                         case 0:
//                             sprintf(str, "Ax %8.4f",(double)raw.accelerometer_m_s2[0]);
//                             break;
//                         case 1:
//                             sprintf(str, "Ay %8.4f",(double)raw.accelerometer_m_s2[1]);
//                             break;
//                         case 2:
//                             sprintf(str, "Az %8.4f",(double)raw.accelerometer_m_s2[2]);
//                             break;
//                         case 3:
//                             sprintf(str, "Gx %8.4f",(double)raw.gyro_rad[0]);
//                             break;
//                         case 4:
//                             sprintf(str, "Gy %8.4f",(double)raw.gyro_rad[1]);
//                             break;
//                         case 5:
//                             sprintf(str, "Gz %8.4f",(double)raw.gyro_rad[2]);
//                             break;
//                     }
//                     PX4_INFO("%s",str);
//                     write(uart, str, 1);
//                 }
//             }
//         }
//     }
//     thread_running = false;
//     close(uart);
//     return 0;
// }
