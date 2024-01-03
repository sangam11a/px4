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
 *  Homemade serial link with interrupt-based reception
 * @author Leroy Theophile - leroy.theophile@gmail.com
 */
#include <px4_log.h>
#include <fcntl.h>
#include <termios.h>
#include <stdbool.h>
#include <unistd.h>
#include <signal.h>

#define UART_DEV_PATH "/dev/ttyS2"

volatile bool data_received = false;

void uart_interrupt_handler(int sig)
{
    // Signal handler for the UART interrupt
    data_received = true;
}

int uart_init(const char *uart_name)
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
int irq = stm32_get_irqnumber(STM32_IRQ_USART2);

    PX4_INFO("UART file descriptor: %d", uart);

    if (uart < 0) {
        PX4_ERR("Failed to initialize UART");
        return -1;
    }

    // Install the interrupt handler
    signal(SIGUSR1, uart_interrupt_handler);

    // Allow the process to receive SIGUSR1
    fcntl(uart, F_SETOWN, getpid());
    fcntl(uart, F_SETFL, FASYNC);

    char buff[255];

    while (true) {
        if (data_received) {
            // Read from UART
            ssize_t ret = read(uart, buff, sizeof(buff));

            if (ret > 0) {
                // Print received data
                PX4_INFO("Received data: %.*s", (int)ret, buff);
            } else if (ret < 0) {
                PX4_ERR("Error reading from UART");
                break;
            }

            data_received = false; // Reset the flag
        }

        usleep(100000); // Sleep for 100ms, adjust as needed
    }

    close(uart);

    return 0;
}
