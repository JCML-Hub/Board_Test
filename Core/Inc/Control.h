//
// Created by 神奇bug在哪里 on 7/21/23.
//

#ifndef BOARD_TEST_CONTROL_H
#define BOARD_TEST_CONTROL_H
#include "cvector.h"
uint16_t VL53L0X_GetValue(int ch);
void selectNextTarget();

void processMultiStep(_Bool isInline);

_Bool checkIfHasTreasure();

const int * getNearbyInfo();

void stop();

void moveToExit();

void findTargetWithMap();

void goStraight();

int getAngleOffset();

void enterTheMaze();
void setup();
void ManualMode();
int getLeftDistance() {
    return VL53L0X_GetValue(0);
}

void update();
int getRightDistance()
{
    return VL53L0X_GetValue(1);
}
int getCurrentPosition();
int getFrontDistance()
{
    return VL53L0X_GetValue(2);
};

void updateX_Y(uint16_t left, uint16_t right);

void moveToTarget();
_Bool checkEnter();
void dijkstra(int start, int end, int n, cvector_vector_type(int) targetPath);
#define K 0.001
#endif //BOARD_TEST_CONTROL_H
