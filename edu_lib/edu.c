#include "edu.h"

int fd;

int edu_init(void) {
    fd = open(EDU_DEVICE, O_RDWR);
    if (fd < 0) {
        return -1;
    }
    return 0;
}

uint32_t edu_fact(uint32_t x) {
    uint32_t read_value, write_value;
    write_value = x;
    if (pwrite(fd, &write_value, sizeof(write_value), EDU_FACT_CALC) ==
        sizeof(write_value)) {
    } else {
        close(fd);
        return -1;
    }

    // Get factorial result
    if (pread(fd, &read_value, sizeof(read_value), EDU_FACT_CALC) ==
        sizeof(read_value)) {
    } else {
        close(fd);
        return -1;
    }

    return read_value;
}