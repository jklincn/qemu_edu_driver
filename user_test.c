#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define EDU_DEVICE "/dev/edu"

#define EDU_FACT_CALC 0x08
#define EDU_DMA_SRC_ADDRESS 0x80
#define EDU_DMA_DST_ADDRESS 0x88
#define EDU_DMA_COUNT 0x90
#define EDU_DMA_CMD 0x98

#define EDU_DMA_GET 0x1234

#define EDU_DMA_RAM2EDU 0x0
#define EDU_DMA_EDU2RAM 0x02

#define EDU_BUFFER_ADDRESS 0x40000

int fd;
char read_buffer[100];
const char *write_buffer =
    "This is a content to test QEMU EDU device DMA function.";

void signal_handler(int signum, siginfo_t *info, void *context) {
    printf("Received SIGUSR1, DMA transfer complete!\n");

    if (info->si_int == EDU_DMA_EDU2RAM) {
        int read_value = 0;
        if (pread(fd, &read_value, sizeof(read_value), EDU_DMA_GET) ==
            sizeof(read_value)) {
            if (strcmp(write_buffer, read_buffer) == 0) {
                printf("DMA test pass!\n");
            } else {
                printf("DMA test failed!\n");
            }
        } else {
            printf("Failed to read at EDU_DMA_GET\n");
        }
    }
}

int main() {
    int ret;
    uint32_t read_value, write_value;
    ssize_t bytes_write;
    uintptr_t buffer_address;
    fd = open(EDU_DEVICE, O_RDWR);
    if (fd < 0) {
        printf("Failed to open the device");
        return errno;
    }

    // ============== Test Factorial Computation =============

    write_value = 12;
    if ((bytes_write = pwrite(fd, &write_value, sizeof(write_value),
                              EDU_FACT_CALC)) == sizeof(write_value)) {
        printf("Write Value: %d, Write Size: %zd\n", write_value, bytes_write);
    } else {
        printf("Failed to write the device");
        close(fd);
        return errno;
    }

    // Get factorial result
    if (pread(fd, &read_value, sizeof(read_value), EDU_FACT_CALC) ==
        sizeof(read_value)) {
        printf("Result: %d\n", read_value);
    } else {
        printf("Failed to read the device");
        close(fd);
        return errno;
    }

    // Register the SIGUSR1 signal handler
    struct sigaction sa;
    sa.sa_sigaction = signal_handler;
    sa.sa_flags = SA_SIGINFO;
    sigemptyset(&sa.sa_mask);

    if (sigaction(SIGUSR1, &sa, NULL) == -1) {
        perror("Failed to register SIGUSR1 handler");
        close(fd);
        return errno;
    }

    // ============== Test DMA: RAM to EDU =============

    // Set source address
    buffer_address = (uintptr_t)write_buffer;
    if (pwrite(fd, &buffer_address, sizeof(buffer_address),
               EDU_DMA_SRC_ADDRESS) != sizeof(buffer_address)) {
        printf("Failed to set DMA source address");
        return errno;
    }

    // Set destination address
    write_value = EDU_BUFFER_ADDRESS;
    if (pwrite(fd, &write_value, sizeof(write_value), EDU_DMA_DST_ADDRESS) !=
        sizeof(write_value)) {
        printf("Failed to set DMA destination address");
        return errno;
    }

    // Set transfer count
    write_value = strlen(write_buffer) + 1;  // 1 means '\0'
    if (pwrite(fd, &write_value, sizeof(write_value), EDU_DMA_COUNT) !=
        sizeof(write_value)) {
        printf("Failed to set DMA count");
        return errno;
    }

    // Start DMA transfer
    write_value = EDU_DMA_RAM2EDU;
    if (pwrite(fd, &write_value, sizeof(write_value), EDU_DMA_CMD) !=
        sizeof(write_value)) {
        printf("Failed to start DMA transfer");
        return errno;
    }

    // Wait for signal
    printf("Wait for signal\n");
    pause();

    // ============== Test DMA: EDU to RAM =============

    // Set source address
    write_value = EDU_BUFFER_ADDRESS;
    if (pwrite(fd, &write_value, sizeof(write_value), EDU_DMA_SRC_ADDRESS) !=
        sizeof(write_value)) {
        printf("Failed to set DMA source address");
        return errno;
    }

    // Set destination address
    buffer_address = (uintptr_t)read_buffer;
    if (pwrite(fd, &buffer_address, sizeof(buffer_address),
               EDU_DMA_DST_ADDRESS) != sizeof(buffer_address)) {
        printf("Failed to set DMA destination address");
        return errno;
    }

    // There is no need to reset transfer count, as it is the same as before

    // Start DMA transfer
    write_value = EDU_DMA_EDU2RAM;
    if (pwrite(fd, &write_value, sizeof(write_value), EDU_DMA_CMD) !=
        sizeof(write_value)) {
        printf("Failed to start DMA transfer");
        return errno;
    }

    // Wait for signal
    printf("Wait for signal\n");
    pause();

    close(fd);

    return 0;
}