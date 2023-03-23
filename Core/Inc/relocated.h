//
// Created by 徐鑫平 on 2022/11/18.
//

#ifndef HOMEWORK4_RELOCATED_H
#define HOMEWORK4_RELOCATED_H
#include "stm32f1xx_hal.h"
#include <sys/stat.h>
#include <stdio.h>

void RetargetInit(UART_HandleTypeDef *huart);

int _isatty(int fd);

int _write(int fd, char *ptr, int len);

int _close(int fd);

int _lseek(int fd, int ptr, int dir);

int _read(int fd, char *ptr, int len);

int _fstat(int fd, struct stat *st);
#endif //HOMEWORK4_RELOCATED_H
