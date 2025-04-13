#include "at.h"

ATC_HandleTypeDef hatc1;

void AT_Task() {
    while (true) {
        ATC_Loop(&hatc1);
        osDelay(1);
    }
}

void set_torque(const char* args, char* response) {
    q = atof(args);
    strcpy(response, "+OK");
}

void set_velocity(const char* args, char* response) {
    // velocity = atof(args);
    exp_velocity = atof(args);
    strcpy(response, "+OK");
}

ATC_CmdTypeDef at_commands[] = {
    {"AT+SET-TQ=", set_torque},
    {"AT+SET-V=", set_velocity}
};

void AT_Init() {
    ATC_Init(&hatc1, &huart2, 256, "Slave");
    ATC_SetCommands(&hatc1, at_commands);
    const osThreadAttr_t atTask_attributes = {
        .name = "AT_Task",
        .priority = (osPriority_t) osPriorityHigh1,
        .stack_size = 256 * 4
    };
    osThreadNew(AT_Task, NULL, &atTask_attributes);
}