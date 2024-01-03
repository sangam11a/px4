
//  // Include this for NuttX serial functions

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/fs/fs.h>
#include <nuttx/serial/serial.h>


#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdbool.h>
#include </home/sangam/PX4-Autopilot/boards/apn/phi-v1/nuttx-config/include/board.h>
#include <chip.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>
// #include <stm32_irq.h>

#define UART_DEV_PATH "/dev/ttyS2"

// static const struct uart_config_s g_uart_config =
// {
//     .baud     = 115200,
//     .parity   = 0,
//     .bits     = 8,
//     .stopbits = 1,
// };

static int uart_fd;

volatile bool data_received = false;

static void uart_interrupt_handler(int irq, void *context, FAR void *arg)
{
    // Signal handler for the UART interrupt
    data_received = true;
}


static int uart_init(char *uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);
    if (serial_fd < 0)
    {
        printf("failed to open port: %s\n", uart_name);
        return -1;
    }

    // Configure UART
    struct termios uart_config;

    tcgetattr(serial_fd, &uart_config);

    cfsetispeed(&uart_config, B115200);
    cfsetospeed(&uart_config, B115200);
//     uart_setformat(serial_fd, 8, 0, 1);  // 8 data bits, no parity, 1 stop bit
//     uart_setbaud(serial_fd, g_uart_config.baud);

    return serial_fd;
}

int main(int argc, FAR char *argv[])
{
    // UART open
    uart_fd = uart_init(UART_DEV_PATH);

    printf("UART file descriptor: %d\n", uart_fd);

    if (uart_fd < 0)
    {
        printf("Failed to initialize UART\n");
        return -1;
    }

    // Set up interrupt handlers
    int irq = 38;
    int ret3 = irq_attach(irq, uart_interrupt_handler, NULL);
// irq_attach(irq, uart_interrupt_handler, NULL);

    up_enable_irq(irq);

    // Allow the process to receive the interrupt signal
    fcntl(uart_fd, F_SETOWN, getpid());
    fcntl(uart_fd, F_SETFL, FASYNC);

    char buff[255];

    while (1)
    {
        if (data_received)
        {
            // Read from UART
            ssize_t ret = uart_read(uart_fd, buff, sizeof(buff));

            if (ret > 0)
            {
                // Print received data
                printf("Received data: %.*s\n", (int)ret, buff);
            }
            else if (ret < 0)
            {
                printf("Error reading from UART\n");
                break;
            }

            data_received = false; // Reset the flag
        }

        usleep(100000); // Sleep for 100ms, adjust as needed
    }

    close(uart_fd);

    return 0;
}

