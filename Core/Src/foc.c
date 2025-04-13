#include "foc.h"

float exp_velocity;
float velocity;
float angle;
float q, d;
float sensorAngle = 0;

void hall_read_velocity();
void hall_read_angle();
void step();
void set_pwm(float u, float v, float w);
float angle_normalize(float original_angle);
float low_pass_filter(float x);

void foc_init() {
    angle = 0;
    velocity = 0;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    const osThreadAttr_t read_angleTask_attributes = {
        .name = "read_angle_task",
        .priority = (osPriority_t) osPriorityHigh1,
        .stack_size = 512 * 4
    };
    osThreadNew(hall_read_angle, NULL, &read_angleTask_attributes);
    const osThreadAttr_t stepTask_attributes = {
        .name = "stepTask",
        .priority = (osPriority_t) osPriorityHigh1,
        .stack_size = 256 * 4
    };
    osThreadNew(step, NULL, &stepTask_attributes);
    const osThreadAttr_t read_velocityTask_attributes = {
        .name = "read_v_task",
        .priority = (osPriority_t) osPriorityHigh1,
        .stack_size = 512 * 4
    };
    osThreadNew(hall_read_velocity, NULL, &read_velocityTask_attributes);
}

void set_motor(float para_q, float para_d, float para_velocity) {
    q = para_q;
    d = para_d;
    velocity = para_velocity;
}

void set_pwm(float u, float v, float w) {
    u = fminf(fmaxf(u, 0.0f), 1.0f);
    v = fminf(fmaxf(v, 0.0f), 1.0f);
    w = fminf(fmaxf(w, 0.0f), 1.0f);
    TIM1->CCR1 = floor(u * TIMER_COUNTER_PERIOD);
    TIM1->CCR2 = floor(v * TIMER_COUNTER_PERIOD);
    TIM1->CCR3 = floor(w * TIMER_COUNTER_PERIOD);
}

void set_output() {
    float alpha = d * cos(angle) - q * sin(angle);
    float beta = q * cos(angle) + d * sin(angle);
    float a = alpha;
    float b = -0.5 * alpha + sqrt(3)/2 * beta;
    float c = -0.5 * alpha - sqrt(3)/2 * beta;
    a = (a + 1) / 2;
    b = (b + 1) / 2;
    c = (c + 1) / 2;
    set_pwm(a, b, c);
}

float angle_normalize(float original_angle) {
    return fmod(original_angle, (2 * M_PI));
}

void step() {
    while (true) {
        angle = angle_normalize(angle + (DIR) * (velocity / 1000.0f) * POLE_PAIR_NUM);
        set_output();
        osDelay(1);
    }
}

int turn_count = 0;
float sensorVelocity = 0;

// 全局互斥锁声明（需在适当位置初始化，例如 main 函数）
osMutexId_t sensorMutex;

#define SAMPLE_TIME     0.01f    // 采样时间10ms（与osDelay(10)对应）
#define CUTOFF_FREQ     5.0f    // 截止频率15Hz（根据实际需求调整）
static float alpha = 0.0f;       // 滤波器系数
float filtered_velocity = 0.0f;  // 滤波后的速度值

void hall_read_velocity() {
    float pre_angle = 0;
    int pre_turn_count = 0;

    float rc = 1.0f / (2 * M_PI * CUTOFF_FREQ);
    alpha = SAMPLE_TIME / (SAMPLE_TIME + rc);

    while(true) {
        // 获取原始速度（已修复符号问题的版本）
        osMutexAcquire(sensorMutex, osWaitForever);
        int current_turn_count = turn_count;
        float current_sensor_angle = sensorAngle;
        osMutexRelease(sensorMutex);

        sensorVelocity = ((current_turn_count - pre_turn_count) * 2 * M_PI 
                           + (current_sensor_angle - pre_angle)) / 0.01f;

        // 应用低通滤波器
        filtered_velocity = alpha * sensorVelocity + (1 - alpha) * filtered_velocity;

        // 更新前值
        pre_turn_count = current_turn_count;
        pre_angle = current_sensor_angle;
        
        osDelay(10);
    }
}

void hall_read_angle() {
    uint8_t prev_hall_state = 0;
    const float angle_step = (2 * M_PI / 6) / POLE_PAIR_NUM;
    const uint8_t valid_states[] = {0b001, 0b011, 0b010, 0b110, 0b100, 0b101};

    while (true) {
        bool hall_1 = HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin);
        bool hall_2 = HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin);
        bool hall_3 = HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin);
        uint8_t curr_hall_state = (hall_1 << 2) | (hall_2 << 1) | hall_3;

        int curr_idx = -1;
        for (int i = 0; i < 6; i++) {
            if (valid_states[i] == curr_hall_state) {
                curr_idx = i;
                break;
            }
        }
        if (curr_idx == -1) continue;

        if (prev_hall_state == 0) {
            prev_hall_state = curr_hall_state;
            continue;
        }

        int prev_idx = -1;
        for (int i = 0; i < 6; i++) {
            if (valid_states[i] == prev_hall_state) {
                prev_idx = i;
                break;
            }
        }
        if (prev_idx == -1) continue;

        int direction = 0;
        if ((prev_idx + 1) % 6 == curr_idx) {
            direction = 1;
        } else if ((prev_idx - 1 + 6) % 6 == curr_idx) {
            direction = -1;
        } else {
            continue;
        }

        // 修改前获取互斥锁
        osMutexAcquire(sensorMutex, osWaitForever);
        sensorAngle += DIR * direction * angle_step;

        // 处理溢出并更新 turn_count
        if (sensorAngle >= 2 * M_PI) {
            sensorAngle -= 2 * M_PI;
            turn_count++;
        } else if (sensorAngle < 0) {
            sensorAngle += 2 * M_PI;
            turn_count--;
        }
        osMutexRelease(sensorMutex);

        prev_hall_state = curr_hall_state;
        osDelay(1);
    }
}

float pre_error = 0;
float integral = 0;
float pre_timestamp_pid = 0;

float PID(float Kp, float Ki, float Kd, float error) {
    float compensation = 0;
    uint16_t time_now = TIM6->CNT;
    uint16_t dt = 0;
    if (pre_timestamp_pid < time_now) {
        dt = (0xFFFF - pre_timestamp_pid) + time_now;
    } else {
        dt = time_now - pre_timestamp_pid;
    }

    compensation += Kp * error;

    integral += 0.5f * (dt / 1e6f) * (pre_error + error); // trapezoidal approximation
    compensation += Ki * integral;

    pre_error = error;

    pre_timestamp_pid = TIM6->CNT;
    return compensation;
}