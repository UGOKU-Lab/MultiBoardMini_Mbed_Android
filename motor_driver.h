#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include "mbed.h"

class MotorDriver {
  DigitalOut _forward;
  DigitalOut _backward;

  PwmOut _pwm;

  float _clamp(float x, float min, float max) {
    if (x < min) {
      return min;
    } else if (x > max) {
      return max;
    }
    return x;
  }

public:
  MotorDriver(PinName forward, PinName backward, PinName pwm)
      : _forward(forward), _backward(backward), _pwm(pwm) {
    // Set PWM width.
    _pwm.period(1. / 20000.f);
  }

  void drive(float duty) {
    duty = _clamp(duty, -1.f, 1.f);
    float density = duty > 0.f ? duty : -duty;

    _pwm.write(density);

    if (duty > 0.f) {
      _forward.write(1);
      _backward.write(0);
    } else if (duty < 0.f) {
      _forward.write(0);
      _backward.write(1);
    } else {
      _forward.write(0);
      _backward.write(0);
    }
  }

  void brake() {
    _pwm.write(0);
    _forward.write(1);
    _backward.write(1);
  }
};

#endif