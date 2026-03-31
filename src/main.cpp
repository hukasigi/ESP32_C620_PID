#include <Arduino.h>
#include <CAN.h>
#include <ctype.h>

#include "PID.hpp"

volatile int16_t angle   = 0;
volatile int16_t speed   = 0;
volatile int16_t current = 0;
volatile uint8_t temp    = 0;

volatile int16_t prev_raw_angle       = 0;
volatile int32_t cumulative_raw_angle = 0;
volatile double  output_angle         = 0.0;

int target_angle = 5000.;

constexpr double   CONTROL_CYCLE           = 1000.;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 100; // 10Hz表示

Pid angle_pid(2.05, 0.001, 0.03);
Pid speed_pid(1.2, 0.0005, 0.03);

constexpr int32_t ENCODER_PERIOD = 8192; // 1回転

constexpr double  NEAR_ERROR_BAND   = 80.0; // angle単位
constexpr int16_t NEAR_SPEED_LIMIT  = 500;  // 目標近傍の速度制限
constexpr double  POSITION_DEADBAND = 2.0;  // 到達判定
constexpr int16_t SPEED_DEADBAND    = 20;   // 停止判定

constexpr size_t INPUT_BUFFER_SIZE               = 32;
char             input_buffer[INPUT_BUFFER_SIZE] = {0};
size_t           input_length                    = 0;
size_t           last_prompt_len                 = 0; // 直前の表示長

void   onReceive(int packetSize);
double calcError(int target, double current);
void   handleSerialInput();
void   refreshPrompt();
void   processCommand(const char* input);
void   clearCurrentLine();
double normalizeAngle(double value);

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
    refreshPrompt();
}

void loop() {
    static unsigned long prev_time     = micros();
    static uint32_t      last_debug_ms = 0;
    unsigned long        now           = micros();

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

        // 目標近傍では速度指令を弱める（行き過ぎ抑制）
        if (fabs(error) < NEAR_ERROR_BAND) {
            target_speed = constrain(target_speed, -NEAR_SPEED_LIMIT, NEAR_SPEED_LIMIT);
        }

        // 速度PIDから電流
        speed_pid.setTarget(target_speed);
        speed_pid.update(speed, dt);
        int16_t motor_current = (int16_t)speed_pid.getOutput();

        constexpr int16_t MOTOR_CURRENT_LIMIT = 5000;

        motor_current = constrain(motor_current, -MOTOR_CURRENT_LIMIT, MOTOR_CURRENT_LIMIT);

        // 目標到達時は電流0（微振動防止）
        if (fabs(error) <= POSITION_DEADBAND && abs(speed) <= SPEED_DEADBAND) {
            motor_current = 0;
        }

        if ((millis() - last_debug_ms) >= DEBUG_PRINT_INTERVAL_MS) {
            last_debug_ms = millis();

            clearCurrentLine();
            double error      = calcError(target_angle, output_angle);
            double norm_angle = normalizeAngle(output_angle);

            Serial.printf("angle: %.2f (norm: %.2f) target: %d error: %.2f speed: %d target_speed: %.2f current: %d\n",
                          output_angle, norm_angle, target_angle, error, speed, target_speed, motor_current);
            refreshPrompt();
        }

        CAN.beginPacket(0x200);
        for (int i = 0; i < 4; i++) {
            CAN.write(motor_current >> 8);
            CAN.write(motor_current & 0xFF);
        }
        CAN.endPacket();
    }
}

// キーボード入力を、1文字ずつ処理してバッファに貯める
void handleSerialInput() {
    bool updated = false;

    // 受信済みの処理を残さず全部処理
    while (Serial.available()) {
        char c = (char)Serial.read();

        // enter処理
        if (c == '\r' || c == '\n') {
            Serial.println();
            // 何も入力してない空行ならコマンド処理しない
            if (input_length > 0) {
                // コマンドとして解釈して実行する
                processCommand(input_buffer);
                // 長さを0に戻す
                input_length = 0;
                // 先頭を\nにして空文字列にする
                input_buffer[0] = '\0';
            }
            updated = true;
        } else if (c == 0x08 || c == 0x7F) { // BS,DL処理
            if (input_length > 0) {
                // 先に長さを1減らして、\0をいれて文字列の終わりとする
                input_buffer[--input_length] = '\0';
                updated                      = true;
            }
        } else if (isprint((unsigned char)c)) { // 画面に表示できる文字ならば
            // 文字列の最後には\0を入れる必要があるので、最大サイズ-1しか受け付けない
            if (input_length < INPUT_BUFFER_SIZE - 1) {
                // 代入したあとにinput_lengthを1増やす
                input_buffer[input_length++] = c;
                // 増やした位置に\0入れて終わりとする
                input_buffer[input_length] = '\0';
                updated                    = true;
            }
        }
    }

    if (updated) refreshPrompt();
}

// ログを出す前に、今表示している入力行を、消しておく
void clearCurrentLine() {
    // 同じ行の先頭に戻る
    Serial.print('\r');
    // 空白で上書き
    Serial.print("                                                                                ");
    // もう一度先頭へ
    Serial.print('\r');
}

// 同じ行に書き直し続ける
void refreshPrompt() {
    Serial.print('\r');
    Serial.print("> ");
    Serial.print(input_buffer);

    size_t now_len = 2 + input_length; // "> " + 入力文字列
    // 短くなったときに残骸が残らないようにする
    if (last_prompt_len > now_len) {
        // 短くなったぶんだけ消す
        for (size_t i = 0; i < (last_prompt_len - now_len); i++) {
            Serial.print(' ');
        }
        // 書き直し
        Serial.print('\r');
        Serial.print("> ");
        Serial.print(input_buffer);
    }
    last_prompt_len = now_len;
}

void processCommand(const char* input) {
    char cmd = input[0];

    if (cmd == 't' || cmd == 'T') {
        int value = atoi(input + 1);

        // 0〜8191に正規化（8192は0と同じ）
        value = ((value % ENCODER_PERIOD) + ENCODER_PERIOD) % ENCODER_PERIOD;

        target_angle = value;
        Serial.printf("target set: %d\n", target_angle);
    }
}

void onReceive(int packetSize) {
    if (CAN.packetId() != 0x202) {
        while (CAN.available())
            CAN.read(); // 破棄
        return;
    }

    if (packetSize < 7) {
        while (CAN.available())
            CAN.read(); // 不正長は破棄
        return;
    }

    uint8_t data[8] = {0};
    for (int i = 0; i < 8 && CAN.available(); i++) {
        data[i] = (uint8_t)CAN.read();
    }

    int16_t raw_angle = (int16_t)((data[0] << 8) | data[1]);
    speed             = (int16_t)((data[2] << 8) | data[3]);
    current           = (int16_t)((data[4] << 8) | data[5]);
    temp              = data[6];

    int16_t delta = raw_angle - prev_raw_angle;
    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;

    cumulative_raw_angle += delta;
    prev_raw_angle = raw_angle;

    output_angle = cumulative_raw_angle / 19.0;
}

double normalizeAngle(double value) {
    double n = fmod(value, (double)ENCODER_PERIOD);
    if (n < 0) n += ENCODER_PERIOD;
    return n;
}

double calcError(int target, double currentValue) {
    double t = normalizeAngle((double)target);
    double c = normalizeAngle(currentValue);

    double e = t - c;
    if (e > ENCODER_PERIOD / 2.0) e -= ENCODER_PERIOD;
    if (e < -ENCODER_PERIOD / 2.0) e += ENCODER_PERIOD;

    return e;
}