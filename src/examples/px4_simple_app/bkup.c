/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file myserial.c
 *  Homemade serial link
 * @author Leroy Theophile - leroy.theophile@gmail.com
 */
// #include <px4_config.h>
// #include <px4_tasks.h>
// #include <px4_posix.h>
#include <px4_log.h>
#include <px4_platform_common/posix.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include<nuttx/irq.h>

#include <unistd.h>
#include <stdio.h>
#include <stdarg.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
// #include <stm32_irq.c>
// /home/sangam/PX4-Autopilot/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32f7/
//<platforms/nuttx/arch/risc-v/src/c906/c906_irq.c>
#include <uORB/topics/sensor_combined.h>
#define UART_IRQ 38
/**

 * Get values from  sensors
 *
 */
void getValue(double *data);
/**
 * Main loop of daemon
 */
int myserial_thread_main(int argc, char *argv[]);
/**
 * Print the correct usage.
 */
// static void usage(const char *reason);
/**
 * ...
 */
int set_uart_baudrate(const int fd, unsigned int baud);
/**
 * ...
 */
static int uart_init(char * uart_name);

// static void int_handler(int irq, void *context, void *arg){
//     printf("Interrupt occured on IRQ %d\n",irq);
//     PX4_INFO("INT occured on IRQ %d\n",irq);
// }




/*Interrupt based uart*/
// static int uart_interrupt_handler(int irq, FAR void *context, FAR void *arg)
// {
//   /* Handle UART interrupt */
//   PX4_INFO("Got the interrupt called with irq : %i",irq);
//   return OK;
// }

// /* Register the UART interrupt handler */
// void register_uart_interrupt(void)
// {
//   irq_attach(UART_IRQ, uart_interrupt_handler, NULL);
//   up_enable_irq(UART_IRQ);
// }





__EXPORT int myserial_main(int argc, char *argv[]);

/* static variables */
// static bool thread_should_exit = false;        /**< Daemon exit flag */
// static bool thread_running = false;        /**< Daemon status flag */
// static int deamon_task;                /**< Handle of deamon task / thread */

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

int uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        PX4_ERR("failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}


int px4_simple_app_main(int argc, char *argv[])
{

    char str[255];
    //UART open
    int uart = uart_init("/dev/ttyS2"); //FTDI module comm

    PX4_INFO("uart first :%i ",uart);
    if(false == uart)return -1;
    if(false == set_uart_baudrate(uart,115200)){
            printf("[YCM]set_uart_baudrate is failed\n");
        //     return -1;
    }
    PX4_INFO("UART %i",uart);
//     else{
// 	uart=115200;
// 	set_uart_baudrate(uart);
//     }
    printf("[YCM]uart init is successful\n");
     //QUEUE open -- does not work
    //mqd_t mqret = mq_open("xQueue",O_RDONLY);

    //UORB request

    /* subscribe to sensor_combined topic */
    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    /* limit the update rate to 5 Hz */
    orb_set_interval(sensor_sub_fd, 800);
	int ret;
    px4_pollfd_struct_t fds[] = {
    { .fd = sensor_sub_fd,   .events = POLLIN },
    /* there could be more file descriptors here, in the form like:
     * { .fd = other_sub_fd,   .events = POLLIN },
     */
    };
    int count=0;
    // int ret2;
    int j=0;
    char* buff;

    // register_uart_interrupt();
    while (count < 2) {
        count++;
        // perf_begin(buf);
        // ret2 = write(uart,"123\\r\n",sizeof(buf));
        // PX4_INFO("count %i ret2 %i\n",count,ret2);
        // read(uart,&buf,sizeof(buf));
        PX4_INFO("\nPlease send sth");
        ret = read(uart, &buff, sizeof(buff));
        PX4_INFO("ret is :%d ,Size of buff is : %d %i\n Data is : %s\n Data started to read :",ret,sizeof(buff),sizeof(buff),buff);
        do {
            PX4_INFO("%c", buff[j]);
            j++;
        } while (buff[j - 1] != '\0');


        // PX4_INFO("Received %s size : %d  (%i) :ret2 %i\n",buf,ret,sizeof(buf),sizeof(buf));

    }
    j=0;
    // irq_attach(38,int_handler,NULL);
    // up_enable_irq(38);

        //Receive f
    //main thread loop
    while (count < 10) {
        count++;
        //Receive from message queue "xQueue"
        // mq_receive(mqret, (void *) data, sizeof(data),NULL);

        int poll_ret = px4_poll(fds, 1, 1000);
        PX4_INFO("poll ret %i",poll_ret);
        if(poll_ret > 0){
            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct sensor_combined_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);

                for(int i=0;i<6;i++){
                    switch(i){
                        case 0:
                            sprintf(str, "Ax:%8.4f\n",(double)raw.accelerometer_m_s2[0]);
                            break;
                        case 1:
                            sprintf(str, "Ay:%8.4f\n",(double)raw.accelerometer_m_s2[1]);
                            break;
                        case 2:
                            sprintf(str, "Az:%8.4f\n",(double)raw.accelerometer_m_s2[2]);
                            break;
                        case 3:
                            sprintf(str, "Gx:%8.4f\n",(double)raw.gyro_rad[0]);
                            break;
                        case 4:
                            sprintf(str, "Gy:%8.4f\n",(double)raw.gyro_rad[1]);
                            break;
                        case 5:
                            sprintf(str, "Gz:%8.4f\n",(double)raw.gyro_rad[2]);
                            break;
                    }
                    PX4_INFO("String : %s",str);
                    char* c;
                    PX4_INFO("Size of str is %s\n",str);
                    // ret = write(uart, str, sizeof(str));
                    j=0;

                   do {
                        c = &str[j];
                        j++;
                        write(uart, c, 1);
                    } while (*c != '\r');
                }
		PX4_INFO("Return count ,ret : %i, %i",ret,count);
            }
        }
    }
//     thread_running = false;
    close(uart);
    return 0;
}
