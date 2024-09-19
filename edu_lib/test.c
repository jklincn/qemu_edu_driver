#include <stdio.h>

#include "edu.h"

int main() {
    unsigned int x = 10, result = 0;
    if (edu_init() < 0) {
        printf("Failed to init edu device.\n");
        return -1;
    }
    if ((result = edu_fact(x)) < 0) {
        printf("Failed to factorial computation.\n");
        return -1;
    } else {
        printf("The factorial of %d is %d\n", x, result);
    }
}