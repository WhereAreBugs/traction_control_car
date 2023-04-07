//
// Created by 神奇bug在哪里 on 3/25/23.
//
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "stringProcess.h"
#include <stdio.h>
#pragma clang diagnostic push
#pragma ide diagnostic ignored "readability-non-const-parameter"
/*
 *  注意: 这个函数内存不安全
 *  如果传入的内存大小不够, 会导致内存溢出
 */
//int getSet(char* buffer, uint8_t count, float *pDouble)
//{
////    for (int i = 0; i < sizeof(float); ++i) {
////        *(char *)(pDouble+i) = buffer[i+count];
////    }
//
//
////    atof(buffer+count);
//    *pDouble = (float)strtod(buffer+count, NULL);
//    return 1;
//}
int getSet(char *str, uint8_t count, float *pDouble) {
    double result = 0.0; // the result
    str+=count;
    int sign = 1;   // 1 for positive, -1 for negative
    if (*str == '-') {
        sign = -1; // negative
        str++;
    } else if (*str == '+') {
        str++;
    }
    while (isdigit(*str)) {
        result = result * 10 + (*str - '0');
        str++;
    }
    if (*str == '.') {
        str++;
        double decimal = 1.0;
        while (isdigit(*str)) {
            decimal /= 10;
            result += decimal * (*str - '0');
            str++;
        }
    }
    if (*str == 'e' || *str == 'E') {
        str++;
        int exponentSign = 1;
        if (*str == '-') {
            exponentSign = -1;
            str++;
        } else if (*str == '+') {
            str++;
        }
        int exponent = 0;
        while (isdigit(*str)) {
            exponent = exponent * 10 + (*str - '0');
            str++;
        }
        double power = 1.0;
        for (int i = 0; i < exponent; i++) {
            power *= 10;
        }
        if (exponentSign == -1) {
            result /= power;
        } else {
            result *= power;
        }
    }
    *pDouble = (float) (sign * result);
    printf("SetResult: %f\r\n", *pDouble);
    return 1;
}

#pragma clang diagnostic pop