//
// Created by 神奇bug在哪里 on 7/15/23.
//

#include <stdbool.h>
#include <malloc.h>
#include "status.h"
#include "logger.h"
#include "stdio.h"
#include "PID.h"
#include "Control.h"
#include "hstack.h"
#include "Motor.h"

///外部对象
extern float pitch, yaw, roll;
///速度控制，角度控制
int speed = 0;
float angle = 0;
extern int speedLeft, speedRight;
extern int distanceLeft, distanceRight, distanceFront;
extern   _Motor motor0;
/// 仅用于自动寻路模式的变量
int auto_speed = 100;
///内部对象
int map[26][26];
int *scan_result;
float current_distance_left = 0, current_distance_right = 0, current_distance_front = 0;
int pidTarget = 0;
float x_offset = 0;
float y_offset = 0;
int mapInfo[5][5]=      {1,6,11,16,21,
                         2,7,12,17,22,
                         3,8,13,18,23,
                         4,9,14,19,24,
                         5,10,15,20,25};
extern PID pid;
int *nearResult;
hstack_ptr_t path;

///向量
cvector_vector_type(uint8_t) visited;

///FindTargetWithMap函数内部使用的变量
cvector_vector_type(int) mapTargetPath  = NULL;
int stepCount  =  0;

///moveToExit函数内部使用的变量
cvector_vector_type(int) mapExitPath = NULL;

/**
 * @brief 用于processMultiStep函数中，多步行走的标志。
 * @attention 以下的变量不应该在任何其他地方使用，否则会导致小车的运动出现问题。
 */
int8_t currentDest = 0;
cvector_vector_type(int) moveTargetPath = NULL;
int8_t stepMultiCount = 0;

void getGlobalBaseInfo() {

}


void initMap() {
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            map[i][j] = 0;
        }
    }
}


void setup() {
    nearResult = (int *) malloc(sizeof(int) * 3);
    scan_result = (int *) malloc(sizeof(int) * 3);
    path = hstack_create(sizeof(uint8_t));
    initMap();
}

void update() {
    getGlobalBaseInfo();
    current_distance_left = getLeftDistance() / 10;
    current_distance_right = getRightDistance() / 10;
    current_distance_front = getFrontDistance() / 10;
    if (!(status & STATUS_RUNNING_MASK)) {
        // 系统等待启动
        speedLeft = 0, speedRight = 0;
        return;
    }  //启动判断

    updateX_Y(0, 0); //更新坐标系

    if (status & STATUS_TURNING_MASK) {
        // 系统正在转弯, 检查转弯是否完成。
        LOGI("Turning")
        float pidResult = PID_realize(&pid, pidTarget, yaw);
        speedLeft = pidResult;
        speedRight = -pidResult;
        if (pidResult < 5) //认为转弯已经完成
        {
            LOGI("Turn Finished")
            status &= ~STATUS_TURNING_MASK;
        }
        return;
    } //转弯判断
    if (status & STATUS_MANUAL_CONTROL_MASK) {
        // 手动模式启用
        LOGV("Manual Mode")
        ManualMode();
        return;
    }  //手动模式
    if (status & STATUS_RUNNING_MASK && !(status & STATUS_ENTERED)) {
        // 系统已启动，但未进入迷宫
        static bool initialized = false;
        if (!initialized) {
            enterTheMaze();
            initialized = true;
        } else {
            if (checkEnter()) {
                static int count = 0;
                if (count++ < 300) {
                    float pidResult = PID_realize(&pid, pidTarget, yaw);
                    speedLeft = auto_speed + pidResult;
                    speedRight = auto_speed - pidResult;
                    return;
                }
                status = status | STATUS_ENTERED;
                status = status | STATUS_ARRIVED_MASK;
                LOGI("Entered the maze")
                initialized = false;
                return;
            } else {
                pidTarget = 0;
                float pidResult = PID_realize(&pid, pidTarget, yaw);
                speedLeft = auto_speed + pidResult;
                speedRight = auto_speed - pidResult;
                return;
            }
        }
    } //进入判断
    if (status & STATUS_TURNING_MASK) {
        // 系统正在转弯, 检查转弯是否完成。
        LOGV("Turning")
        float pidResult = PID_realize(&pid, pidTarget, yaw);
        speedLeft = pidResult;
        speedRight = -pidResult;
        if (getAngleOffset() < 5) //认为转弯已经完成
        {
            status &= ~STATUS_TURNING_MASK;
        }
        return;
    } //转弯判断
    if (status & STATUS_MAP_BUILD_MASK) {
        LOGI("Building Map finished")
        ///  地图已完成建立
        if (status & STATUS_EXIT_WALK_OUT_MASK) {
            /// 到达出口，只差最后一步，走出迷宫
            float pidResult = PID_realize(&pid, pidTarget, yaw);
            speedLeft = auto_speed + pidResult;
            speedRight = auto_speed - pidResult;
            goStraight();
        }
        if (!(status & STATUS_ARRIVED_MASK)) {
            /// 当前系统有下一个点的目标，但是仍然没有到达下一个点
            moveToTarget();
            float pid_result = PID_realize(&pid, pidTarget, yaw);
            speedLeft = auto_speed + pid_result;
            speedRight = auto_speed - pid_result;
        }
        if ((status & STATUS_AUTO_FIND_TARGET_MASK) && !(status & STATUS_FIND_TREASURE_MASK)) {
            /// 自动寻找目标启用且系统仍未找到目标
            findTargetWithMap(); //类似于selectTarget函数
            return;
        } else if (status & STATUS_FIND_TREASURE_MASK) {
            /// 系统已找到目标,且地图已经建立
            moveToExit(); //一种新的selectTarget函数，有着指定的位置
            return;
        } else if (status & STATUS_ARRIVED_EXIT_MASK) {
            /// 系统已到达出口
            stop();
            status = status & (~STATUS_RUNNING_MASK);// 系统停止运行
            return;
        } else {
            LOGE("status error, status can't be recognized。Current at map build finished")
            return;
        }
    }
    /// 地图建立未完成
    if (status & STATUS_ARRIVED_MASK) {
        /// 系统到达了上次行动要求的目标点
        if (!(status & STATUS_SCAN_MASK)) {
            scan_result = getNearbyInfo();
            status = status | STATUS_SCAN_MASK;//标志系统扫描完成
            status = status & (~STATUS_TARGET_SELECTED_MASK);
            LOGI("Current position scan finished")
        }
        if (!(status & STATUS_FIND_TREASURE_MASK)) {
            /// 检查当前位置是否检测到目标数字
            if (checkIfHasTreasure()) {
                HAL_Delay(100);
            }
            LOGI("FIND ok")
        }
        /// 系统目标需要更多步才能完成
        if (status & STATUS_MULTI_STEP_MASK) {
            LOGI("Multi step")
            processMultiStep(false);
            return; //等待下一次循环
        }
        if (!(status & STATUS_TARGET_SELECTED_MASK)) {

            selectNextTarget();
            LOGI("Next target selected")
            status = status | STATUS_TARGET_SELECTED_MASK;
            status = status & (~STATUS_ARRIVED_MASK) & (~STATUS_SCAN_MASK) & (~STATUS_FIND_TREASURE_MASK);
            return;
        }
    } //系统到达目标点
    else {
        /// 系统尚未未到达上次行动要求的目标点
        LOGI("Moving to target")
        moveToTarget();
        float pidResult = PID_realize(&pid, pidTarget, yaw);
        speedLeft = auto_speed + pidResult;
        speedRight = auto_speed - pidResult;
        return;
    }
}


bool checkEnter() {
    if (distanceLeft < 10 || distanceRight < 10) {
        return true;
    } else {
        return false;
    }
}

void ManualMode() {
    pidTarget = angle;
    float pidResult = PID_realize(&pid, pidTarget, yaw);
    speedLeft = speed + pidResult;
    speedRight = speed - pidResult;
}

void enterTheMaze() {
    LOGI("Let's enter the maze")
    speedLeft = auto_speed;
    speedRight = auto_speed;
}

void goStraight() {
    LOGI("go straight")
    auto_speed = 100;
    status &= ~STATUS_ARRIVED_MASK;
    status &= ~STATUS_SCAN_MASK;
    status &= ~STATUS_FIND_TREASURE_MASK;

}
/// 剩下的情况应该是地图未建立



void stop() {
    LOGW("stop")
    auto_speed = 0;
    speedLeft = 0;
    speedRight = 0;
}



void turnLeft() {
    LOGI("turn left")
    if ((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET == 0)  //若当前方向为0，则向左转向后应该为3
    {
        status |= STATUS_DIRECTION(3);
    } else //否则就直接减1
    {
        status |= STATUS_DIRECTION(((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET) - 1);
    }
    status |= STATUS_TURNING_MASK;
    if (pidTarget < -90) {
        pidTarget = 180;
    } else
        pidTarget -= 90;
}


void turnRight() {
    LOGI("turn right")
    if (((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET == 3))  //若当前方向为3，则向右转向后应该为0
    {
        status |= STATUS_DIRECTION(0);
    } else //否则就直接加1
    {
        status |= STATUS_DIRECTION(((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET) + 1);
    }
    status |= STATUS_TURNING_MASK;
    if (pidTarget > 90) {
        pidTarget = -180;
    } else
        pidTarget += 90;
}


void turnBack() {
    LOGI("turn back")
    if (((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET == 0))  //若当前方向为0，则向后转向后应该为2
    {
        status |= STATUS_DIRECTION(2);
    } else if (((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET == 1)) {
        status |= STATUS_DIRECTION(3);
    } else //否则就直接减2
    {
        status |= STATUS_DIRECTION(((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET) - 2);
    }
    status |= STATUS_TURNING_MASK;
    if (pidTarget == 0) {
        pidTarget = 180;
    } else if (pidTarget == 90) {
        pidTarget = -90;
    } else if (pidTarget == -90) {
        pidTarget = 90;
    } else {
        pidTarget = 0;
    }

}


void start() {
    LOGW("start")
    status = status | STATUS_RUNNING_MASK;

}


/**
 * @brief 获取周围的可达性
 * @return result[3], 0:左侧可达，1:前方可达，2:右侧可达
 */
const int *getNearbyInfo() {
    nearResult[0] = (distanceLeft) > 40;
    nearResult[1] = (distanceFront) > 40;
    nearResult[2] = (distanceRight) > 40;
    return nearResult;
}


bool isFindTargetWithMapInitialed = false;

void findTargetWithMap() {
    LOGI("find target with map start")
    int currentPos = getCurrentPosition();
    int targetPos = (status &= STATUS_TARGET_SELECTED_MASK) >> STATUS_TARGET_OFFSET;

    if (!isFindTargetWithMapInitialed) {
        dijkstra(currentPos, targetPos, 26, mapTargetPath);
        isFindTargetWithMapInitialed = true;
    } else {
        if (stepCount< cvector_size(mapTargetPath)) {
            status = status & STATUS_TARGET(mapTargetPath[stepCount++]);
            status = status & (~STATUS_ARRIVED_MASK);
            status = status & (~STATUS_SCAN_MASK);
            status = status | STATUS_TARGET_SELECTED_MASK;
            return;
        }
    }
}

bool isMoveToExitInitialed = false;



void moveToExit() {
    LOGI("move to exit start")
    int currentPos = getCurrentPosition();
    int targetPos = 25;
    if (currentPos == 25) {
        status = status | STATUS_EXIT_WALK_OUT_MASK;
        return;
    }
    if (!isMoveToExitInitialed) {
        dijkstra(currentPos, targetPos, 26, mapExitPath);
        isMoveToExitInitialed = true;
    } else {
        status = status & STATUS_TARGET(mapExitPath[stepCount++]);
        status = status & (~STATUS_ARRIVED_MASK);
        status = status & (~STATUS_SCAN_MASK);
        status = status | STATUS_TARGET_SELECTED_MASK;
        return;
    }
}

bool checkIfHasTreasure() {
    LOGI("check if has treasure start")
    int currentPos = getCurrentPosition();
    if (currentPos == 15 || currentPos == 17) {
        return true;
    }
    return false;
}

bool notEsxited(int i);

void updateMap(uint8_t start, int theEnd, bool isReachable);

void selectNextTarget() {
    LOGV("select next target start")
    uint8_t currentPos = getCurrentPosition();
    int *currentPosInfo = scan_result; //优化IDE的上下文搜索
    LOGD("current position is %d", currentPos)
    switch ((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET) {
        case 0: //方向：将入口方向作为上方，该方向为入口的下方
        {
            if (currentPosInfo[0] == 1 && (currentPos - 5) > 0 && notEsxited(currentPos - 5) && currentPos != 25) {
                // 左侧可达，且左侧未到达过
                uint8_t data = currentPos -5;
                hstack_push(path, &data,sizeof(uint8_t), NULL);
                updateMap(currentPos, currentPos - 5, true);
                updateMap(currentPos - 5, currentPos, true);
            }
            if (currentPosInfo[1] == 1 && (currentPos + 1) > 0 && notEsxited(currentPos + 1)) {
                // 前方可达，且前方未到达过
                uint8_t data = currentPos + 1;
                hstack_push(path, &data,sizeof(uint8_t), NULL);
                updateMap(currentPos, currentPos + 1, true);
                updateMap(currentPos + 1, currentPos, true);
            }
            if (currentPosInfo[2] == 1 && (currentPos + 5) > 0 && notEsxited(currentPos + 5)) {
                // 右侧可达，且右侧未到达过
                uint8_t data = currentPos + 5;
                hstack_push(path, &data,sizeof(uint8_t), NULL);
                updateMap(currentPos, currentPos + 5, true);
                updateMap(currentPos + 5, currentPos, true);
            }
            break;
        }
        case 1: //方向：将入口方向作为上方，该方向为入口的右方
        {
            if (currentPosInfo[0] == 1 && currentPos != 1 && notEsxited(currentPos - 1)) {
                // 左侧可达，且左侧未到达过
                uint8_t data = currentPos - 1;
                hstack_push(path, &data,sizeof(uint8_t), NULL);
                updateMap(currentPos, currentPos - 1, true);
                updateMap(currentPos - 1, currentPos, true);
            }
            if (currentPosInfo[1] == 1 && (currentPos + 5) > 0 && notEsxited(currentPos + 5) && currentPos != 25) {
                // 前方可达，且前方未到达过
                uint8_t data = currentPos - 5;
                hstack_push(path, &data,sizeof(uint8_t), NULL);
                updateMap(currentPos, currentPos - 5, true);
                updateMap(currentPos - 5, currentPos, true);
            }
            if (currentPosInfo[2] == 1 && (currentPos + 1) > 0 && notEsxited(currentPos + 1)) {
                // 右侧可达，且右侧未到达过
                uint8_t data = currentPos + 1;
                hstack_push(path, &data,sizeof(uint8_t), NULL);
                updateMap(currentPos, currentPos + 1, true);
                updateMap(currentPos + 1, currentPos, true);
            }
            break;

        }
        case 2: //方向：将入口作为上方，该方向为上方
        {
            if (currentPosInfo[0] == 1 && (currentPos - 5) > 0 && notEsxited(currentPos - 5)) {
                // 左侧可达，且左侧未到达过
                uint8_t data = currentPos - 5;
                hstack_push(path, &data,sizeof(uint8_t), NULL);
                updateMap(currentPos, currentPos - 5, true);
                updateMap(currentPos - 5, currentPos, true);
            }
            if (currentPosInfo[1] == 1 && (currentPos - 1) > 0 && notEsxited(currentPos - 1)) {
                // 前方可达，且前方未到达过
                uint8_t data = currentPos - 1;
                hstack_push(path, &data,sizeof(uint8_t), NULL);
                updateMap(currentPos, currentPos - 1, true);
                updateMap(currentPos - 1, currentPos, true);
            }
            if (currentPosInfo[2] == 1 && (currentPos + 5) > 0 && notEsxited(currentPos + 5) && currentPos != 25) {
                // 右侧可达，且右侧未到达过
                uint8_t data = currentPos + 5;
                updateMap(currentPos, currentPos + 5, true);
                updateMap(currentPos + 5, currentPos, true);
            }
            break;
        }
        case 3: {
            if (currentPosInfo[0] == 1 && (currentPos + 1) > 0 && notEsxited(currentPos + 1)) {
                // 左侧可达，且左侧未到达过
                uint8_t data = currentPos + 1;
                hstack_push(path, &data,sizeof(uint8_t), NULL);
                updateMap(currentPos, currentPos + 1, true);
                updateMap(currentPos + 1, currentPos, true);
            }
            if (currentPosInfo[1] == 1 && (currentPos - 5) > 0 && notEsxited(currentPos - 5) && currentPos != 25) {
                // 前方可达，且前方未到达过
                uint8_t data = currentPos - 5;
                hstack_push(path, &data,sizeof(uint8_t), NULL);
                updateMap(currentPos, currentPos - 5, true);
                updateMap(currentPos - 5, currentPos, true);
            }
            if (currentPosInfo[2] == 1 && (currentPos - 1) > 0 && notEsxited(currentPos - 1) && currentPos != 1) {
                // 右侧可达，且右侧未到达过
                uint8_t data = currentPos - 1;
                hstack_push(path, &data,sizeof(uint8_t), NULL);
                updateMap(currentPos, currentPos - 1, true);
                updateMap(currentPos - 1, currentPos, true);
            }
            break;
        }
        default: {
            LOGE("error direction")
            break;
        }
    }
    cvector_push_back(visited,currentPos); //将当前位置加入已访问列表
    if (!hstack_empty(path)) {
        uint8_t target = *(uint8_t *)hstack_top(path);
        status &= ~STATUS_TARGET_OFFSET;
        status = status | STATUS_TARGET(target);
        processMultiStep(true);
        hstack_pop(path);
    } else {
        status |= STATUS_MAP_BUILD_MASK;
    }
    LOGD("select next target end, next point: %llu" ,(status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET)
    return;
}

void updateMap(uint8_t start, int theEnd, bool isReachable) {
    if (start>26||theEnd>26){
        LOGE("update map error!  overflowed!")
        return;
    }
    map[start][theEnd] = isReachable;
}

void moveToTarget() {
    if ((getCurrentPosition() == (status & STATUS_DIRECTION_MASK))) {
        //到达目标点
        LOGD("arrive target point")
        status &= ~STATUS_DIRECTION_MASK; //清除目标点
        status |= STATUS_ARRIVED_MASK;
        return;
    }
    // 未到达目标点
    //TODO: 处理这种情况

}


void ManualGo(uint8_t direction) {
    if (!(status & STATUS_MANUAL_CONTROL_MASK)) {
        LOGE("Manual control is not enabled!")
        return;
    }
    switch (direction) {
        case 0:
            goStraight();
            break;
        case 1:
            turnLeft();
            break;
        case 2:
            turnRight();
            break;
        case 3:
            turnBack();
            break;
        default:
        LOGE("direction error")
            break;
    }

}


int getCurrentPosition() {
    if (x_offset > 200)
    {
        x_offset = 200;
    }
    if (y_offset > 200)
    {
        y_offset = 200;
    }
    if (x_offset < 0)
    {
        x_offset = 0;
    }
    if (y_offset < 0)
    {
        y_offset = 0;
    }
    return mapInfo[(int)(x_offset + 10) / 40][(int)(y_offset + 10) / 40];
}



const int ** getMap() {
    cvector_vector_type(cvector_vector_type(int)) result = NULL;
    cvector_set_capacity(result, 25);
    for (int i = 0; i < 25; ++i) {
        cvector_set_capacity(result[i], 25);
    }
    for (int i = 0; i < 25; ++i) {
        for (int j = 0; j < 25; ++j) {
             result[i][j] = map[i][j];
        }
    }
    return result;
}

bool notEsxited(int i) {
    for (uint8_t * it = cvector_begin(visited); it < cvector_end(visited); it++) {
        if (*it == i) {
            return false;
        }
    }
    return true;

}

/***
 * 更新x，y坐标
 * @param left 左侧编码器的值
 * @param right 右侧编码器的值
 * @attention 该函数应在中断中调用，调用后需要清空编码器的值。
 *
 */
void updateX_Y(uint16_t left, uint16_t right) {
    if (status & STATUS_TURNING_MASK) {
        return;
    }
    //TODO: 参考系统当前倾角作适当校准
    if ((status & STATUS_DIRECTION_MASK >> STATUS_DIRECTION_OFFSET) == 0) {
          x_offset += ((left+right) / 2.0f ) * K;
    } else if ((status & STATUS_DIRECTION_MASK >> STATUS_DIRECTION_OFFSET) == 1) {
        LOGI("Currenrt direction: %d", (status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET)
        y_offset += ((left+right) / 2.0f ) * K;
    } else if ((status & STATUS_DIRECTION_MASK >> STATUS_DIRECTION_OFFSET) == 2) {
        x_offset -= ((left+right) / 2.0f ) * K;
    }
    else {
        LOGI("Currenrt direction: %d", (status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET)
        y_offset -=  ((left+right) / 2.0f ) * K;
    }

}


#define INF 0x3f3f3f3f

void dijkstra(int start, int end, int n, cvector_vector_type(int) targetPath) {
    //targetPath 会存放数据
    cvector_vector_type(int) dist = NULL;
    for (int i = 0; i < n; ++i) {
        cvector_push_back(dist, INF);
    }
    cvector_vector_type(int) prev = NULL;
    for (int i = 0; i < n; ++i) {
        cvector_push_back(prev, -1);
    }
    cvector_vector_type(bool) visit = NULL;
    for (int i = 0; i < n; ++i) {
        cvector_push_back(visit, false);
    }

    dist[start] = 0;
    for (int i = 0; i < n - 1; ++i) {
        int u = -1;
        for (int j = 0; j < n; ++j) {
            if (!visit[j] && (u == -1 || dist[j] < dist[u])) {
                u = j;
            }
        }
        if (u == -1) break;
        visit[u] = true;
        for (int v = 0; v < n; ++v) {
            if (map[u][v] && !visit[v]) {
                int new_dist = dist[u] + map[u][v];
                if (new_dist < dist[v]) {
                    dist[v] = new_dist;
                    prev[v] = u;
                }
            }
        }
    }
    if (dist[end] == INF) {
        LOGE("No available path")
        return;
    }
    for (int v = end; v != -1; v = prev[v]) {
        cvector_push_back(targetPath, v);
    }
    ///别忘了清空空间
    cvector_free(dist);
    cvector_free(prev);
    cvector_free(visit);
}

void processMultiStep(bool inlineCall) {
    int  currentPos = getCurrentPosition();
    if (inlineCall) {
        int targetPos = ((status & STATUS_TARGET_MASK) >> STATUS_TARGET_OFFSET); //从内联调用时，检查是否能单步完成
        switch ((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET) {
            case 0: //当前朝向为下方
            {
                LOGI("Current direction: down")
                LOGI("Target position:%llu" ,targetPos)
                if (targetPos == currentPos + 1) {
                    //目标点在前方
                    goStraight();
                } else if (targetPos == currentPos + 5) {
                    //目标点在右方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnRight();
                } else if (targetPos == currentPos - 5) {
                    //目标点在左方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnLeft();
                } else if (targetPos == currentPos - 1) {
                    //目标点在后方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnBack();
                } else {
                    status |= STATUS_MULTI_STEP_MASK;
                    currentDest = targetPos;
                }
                break;
            }
            case 1: //当前方向为右方
            {
                LOGI("Current direction: right")
                if (targetPos == currentPos + 5) {
                    //目标点在前方
                    goStraight();
                } else if (targetPos == currentPos + 1) {
                    //目标点在右方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnRight();
                } else if (targetPos == currentPos - 1) {
                    //目标点在左方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnLeft();
                } else if (targetPos == currentPos - 5) {
                    //目标点在后方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnBack();
                } else {
                    status |= STATUS_MULTI_STEP_MASK;
                    currentDest = targetPos;
                }
                break;
            }
            case 2: //当前方向为上方
            {
                LOGI("Current direction: up")
                if (targetPos == currentPos - 1) {
                    //目标点在前方
                    goStraight();
                } else if (targetPos == currentPos + 5) {
                    //目标点在右方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnRight();
                } else if (targetPos == currentPos - 5) {
                    //目标点在左方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnLeft();
                } else if (targetPos == currentPos + 1) {
                    //目标点在后方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnBack();
                } else {
                    status |= STATUS_MULTI_STEP_MASK;
                    currentDest = targetPos;
                }
                break;
            }
            case 3: //当前方向为左方
            {
                LOGI("Current direction: left")
                if (targetPos == currentPos - 5) {
                    //目标点在前方
                    goStraight();
                } else if (targetPos == currentPos - 1) {
                    //目标点在右方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnRight();
                } else if (targetPos == currentPos + 1) {
                    //目标点在左方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnLeft();
                } else if (targetPos == currentPos + 5) {
                    //目标点在后方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnBack();
                } else {
                    status |= STATUS_MULTI_STEP_MASK;
                    currentDest = targetPos;
                }
                break;
            }
        } //检查系统的目标能否一步到达，以及应该怎么走
        if (status & STATUS_MULTI_STEP_MASK) {
            dijkstra(currentPos, targetPos, 26, moveTargetPath); // 规划路径
            status &= ~STATUS_TARGET_SELECTED_MASK; //清空这个标志位
            status |= STATUS_TARGET(moveTargetPath[currentDest++]);
            status &= STATUS_ARRIVED_MASK;
            status &= STATUS_SCAN_MASK;
        }
    } else {
        if (status & STATUS_TURN_FIRST_MASK) {
            //转弯后的第一步
            goStraight();
            status &= ~STATUS_TURN_FIRST_MASK; // 清除标志位
        } else if (status & STATUS_MULTI_STEP_MASK) {
            //多步行走
            if (currentPos == currentDest) //如果当前已到达目的地
            {
                status &= ~STATUS_MULTI_STEP_MASK;
                status &= ~STATUS_TARGET_SELECTED_MASK;
            } else {
                status &= ~STATUS_TARGET_SELECTED_MASK;
                status |= STATUS_TARGET(moveTargetPath[currentDest++]);
            }
        }
    }
}

cvector_vector_type(int) getDistance() {
    cvector_vector_type(int) dist = NULL;
    cvector_push_back(dist,current_distance_left);
    cvector_push_back(dist,current_distance_front);
    cvector_push_back(dist,current_distance_right);
    return dist;
}

cvector_vector_type(float) getX_Y() {
    cvector_vector_type(float ) result = NULL;
    cvector_push_back(result, x_offset);
    cvector_push_back(result, y_offset);
    return result;
}

int getAngleOffset() {
    switch (status & STATUS_DIRECTION_MASK >> STATUS_DIRECTION_OFFSET) {
        case 0:
            return yaw - 0;
        case 1:
            return yaw - 90;
        case 2:
            return yaw - 180;
        case 3:
            return yaw - 270;
    }
}


#pragma clang diagnostic pop
