#ifndef ROBOCUP_H
#define ROBOCUP_H

void set_movement(float left, float right);
void set_servo_angle(float angle);
void set_display_text(const char* text);

float get_ultrasonic();
int get_grayscale(int no);
float get_accel_x();
float get_accel_y();
float get_accel_z();
float get_audio_level();
float get_time();
void delay(int msec);

void setup();
void user_loop();

constexpr int LEFT_GS = 0;
constexpr int CENTER_GS = 1;
constexpr int RIGHT_GS = 2;

#endif // ROBOCUP_H
