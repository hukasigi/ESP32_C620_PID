#pragma once
#include <Arduino.h>

class AnglePID {
    private:
        double kp, ki, kd;
        double out_min, out_max;
        double range;
        double integral;
        double integral_max, integral_min;
        double control;
        double prev_error;

        long prev_now_pos;
        bool first;

    public:
        AnglePID(double kp, double ki, double kd, double out_min, double out_max, double range, double INTEGRAL_MAX,
                 double INTEGRAL_MIN)
            : kp(kp), ki(ki), kd(kd), out_min(out_min), out_max(out_max), range(range), integral(0.0),
              integral_max(INTEGRAL_MAX), integral_min(INTEGRAL_MIN), control(0.0), prev_now_pos(0), first(true) {}

        // dt[s]  msなら中で/1000して
        double update(long target_pos, long now_pos, double dt) {
            if (dt <= 0.0) return control;

            // 角度誤差（wrap込み）
            double error = (double)target_pos - (double)now_pos;
            error        = fmod(error + range / 2.0, range);
            if (error < 0) error += range;
            error -= range / 2.0;

            prev_error = error;

            // I
            integral += error * dt;
            integral = constrain(integral, integral_min, integral_max);

            // D（測定値微分）
            double derivative = 0.0;
            if (!first) {
                const double dMeas = ((double)now_pos - (double)prev_now_pos) / dt;
                derivative         = -dMeas;
            } else {
                first = false;
            }
            prev_now_pos = now_pos;

            control = kp * error + ki * integral + kd * derivative;
            control = constrain(control, out_min, out_max);
            return control;
        }

        double getError() { return prev_error; }
        void   reset() {
            integral     = 0.0;
            control      = 0.0;
            first        = true;
            prev_now_pos = 0;
            prev_error   = 0.;
        }
};