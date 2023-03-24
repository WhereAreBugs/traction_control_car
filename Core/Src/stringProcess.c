//
// Created by 神奇bug在哪里 on 3/25/23.
//
#include "stringProcess.h"
#pragma clang diagnostic push
#pragma ide diagnostic ignored "readability-non-const-parameter"
/*
 *  注意: 这个函数内存不安全
 *  如果传入的内存大小不够, 会导致内存溢出
 */
int getSet(char* buffer, uint8_t count, float *pDouble)
{
    for (int i = 0; i < sizeof(float); ++i) {
        *(char *)(pDouble+i) = buffer[i+count];
    }return 1;
}
#pragma clang diagnostic pop