#include "utils.h"
#include "robocup.h"

constexpr int MAX_REPEAT = 5;


float set_servo_and_read_ultrasonic(float angle, float prev_distance) {
	set_servo_angle(angle);
	float distance;
	int repeats = 0;
	while(true) {
		delay(1);
		distance = get_ultrasonic();
		if (distance == prev_distance && repeats < MAX_REPEAT) {
			repeats++;
			continue;
		}
		break;
	}
	repeats = 0;
	while (distance != prev_distance) {
		if (repeats >= MAX_REPEAT) break;
		delay(1);
		prev_distance = distance;
		distance = get_ultrasonic();
		repeats++;
	}
	return distance;
}

