#include <Arduino.h>
#include <CAN.h>

#include "PID.hpp"

volatile int16_t angle   = 0;
volatile int16_t speed   = 0;
volatile int16_t current = 0;
volatile uint8_t temp    = 0;

volatile int16_t prev_raw_angle       = 0;
volatile int32_t cumulative_raw_angle = 0;
volatile double  output_angle         = 0.0;

int target_angle = 5000.;

constexpr double CONTROL_CYCLE = 1000.;

Pid angle_pid(2., 0.0, 0.003);
Pid speed_pid(1.0, 0.0, 0.003);

void onReceive(int packetSize);

double calcError(int target, double current);

void handleSerialInput();

void setup() {
    Serial.begin(115200);

    CAN.setPins(4, 5); // RX, TX

    if (!CAN.begin(1000000)) {
        Serial.println("CAN init failed");
        while (1)
            ;
    }

    CAN.onReceive(onReceive);

    volatile uint32_t* pREG_IER = (volatile uint32_t*)0x3ff6b010;
    *pREG_IER &= ~(uint8_t)0x10;

    Serial.println("ESP32 PID Control Started");
    Serial.println("Command format: t<value>  (e.g., t2000)");
}

void loop() {

    static unsigned long prev_time = micros();
    unsigned long        now       = micros();

    handleSerialInput();

    if (now - prev_time >= CONTROL_CYCLE) {
        prev_time += CONTROL_CYCLE;

        double dt = CONTROL_CYCLE / 1000.0; // ms

        // 位置PIDから速度
        double error = calcError(target_angle, output_angle);
        angle_pid.setTarget(0.);
        angle_pid.update(-error, dt);
        double target_speed = angle_pid.getOutput();

        constexpr int16_t SPEED_LIMIT = 2000;

        target_speed = constrain(target_speed, -SPEED_LIMIT, SPEED_LIMIT);

        // 速度PIDから電流
        speed_pid.setTarget(target_speed);
        speed_pid.update(speed, dt);
        int16_t motor_current = (int16_t)speed_pid.getOutput();

        constexpr int16_t MOTOR_CURRENT_LIMIT = 5000;

        motor_current = constrain(motor_current, -MOTOR_CURRENT_LIMIT, MOTOR_CURRENT_LIMIT);

        // 目標近くで停止
        if (abs(calcError(target_angle, output_angle)) < 50) {
            motor_current = 0;
        }

        Serial.printf("angle: %.2f target: %d speed: %d target_speed: %.2f current: %d\n", output_angle, target_angle, speed,
                      target_speed, motor_current);

        CAN.beginPacket(0x200);
        for (int i = 0; i < 4; i++) {
            CAN.write(motor_current >> 8);
            CAN.write(motor_current & 0xFF);
        }
        CAN.endPacket();
    }
}

void handleSerialInput() {
    if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == 't' || cmd == 'T') {
            // target_angleを設定
            int16_t value = Serial.parseInt();
            target_angle  = value;
        } else if (cmd == 'g' || cmd == 'G') {
            // 現在値表示
            Serial.printf("Current angle: %.2f, target: %d\n", output_angle, target_angle);
        }
    }
}

void onReceive(int packetSize) {
    int id = CAN.packetId();
    if (id != 0x204) return;

    uint8_t data[8];
    for (int i = 0; i < 8; i++) {
        data[i] = CAN.read();
    }

    int16_t raw_angle = (data[0] << 8) | data[1];
    speed             = (data[2] << 8) | data[3];
    current           = (data[4] << 8) | data[5];
    temp              = data[6];

    int32_t delta = (int32_t)raw_angle - (int32_t)prev_raw_angle;
    if (delta > 4096) delta -= 8192;
    else if (delta < -4096)
        delta += 8192;
    cumulative_raw_angle += delta;
    prev_raw_angle = raw_angle;

    output_angle = (double)cumulative_raw_angle / 19.0;
}

double calcError(int target, double current) {
    double error = (double)target - current;

    return error;
}