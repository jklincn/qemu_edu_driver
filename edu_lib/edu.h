#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>

#define EDU_DEVICE "/dev/edu"

#define EDU_FACT_CALC 0x08

int edu_init(void);

uint32_t edu_fact(uint32_t x);