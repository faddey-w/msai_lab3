#include <algorithm>
#define _USE_MATH_DEFINES 
#include <math.h>

#include "navigation.h"
#include "robocup.h"


void _calc_mid(float angle_min, float angle_max, float &angle_mid, float &dist_mid) {
	angle_mid = (angle_max + angle_min) / 2;
	set_servo_angle(angle_mid);
	delay(50);
	dist_mid = get_ultrasonic();
}


bool _detect_has_corner(float angle_min, float angle_max, float &angle_mid,
	                    float dist_min, float dist_max, float &dist_mid) {
	_calc_mid(angle_min, angle_max, angle_mid, dist_mid);
	float dist_no_corn = 2 * dist_min * dist_max * std::cosf(
		M_PI*(angle_min- angle_max)/360
	) / (dist_min + dist_max);
	return 0.01 * dist_mid < std::abs(dist_mid - dist_no_corn);
}


float _detect_corner_recursively(float angle_min, float angle_max, float angle_mid,
	                             float dist_min, float dist_max, float dist_mid) {

	if (std::abs(angle_max - angle_min) < 0.1) {
		return angle_mid;
	}

	float next_angle_mid, next_dist_mid;
	if (_detect_has_corner(angle_min, angle_mid, next_angle_mid, 
						   dist_min, dist_mid, next_dist_mid)) {
		return _detect_corner_recursively(angle_min, angle_mid, next_angle_mid, 
										  dist_min, dist_mid, next_dist_mid);
	} else {
		_calc_mid(angle_mid, angle_max, next_angle_mid, next_dist_mid);
		return _detect_corner_recursively(angle_mid, angle_max, next_angle_mid, 
										  dist_mid, dist_max, next_dist_mid);
	}

}


bool navigation::detect_corner(float angle_min, float angle_max, float& result) {
	if (angle_max < angle_min) std::swap(angle_max, angle_min);

	set_servo_angle(angle_min);
	delay(50);
	float dist_min = get_ultrasonic();

	set_servo_angle(angle_max);
	delay(50);
	float dist_max = get_ultrasonic();

	float angle_mid, dist_mid;
	if (!_detect_has_corner(angle_min, angle_max, angle_mid,
							dist_min, dist_max, dist_mid)) {
		// maybe there are no corners
		// try confirm this
		float tmp;
		bool hc1 = _detect_has_corner(angle_min, angle_mid, tmp,
									  dist_min, dist_mid, tmp);
		bool hc2 = _detect_has_corner(angle_mid, angle_max, tmp,
									  dist_mid, dist_max, tmp);
		if (!hc1 && !hc2) {
			return false;
		}
	}

	// there is corner. find it recursively
	result = _detect_corner_recursively(angle_min, angle_max, angle_mid, 
										dist_min, dist_max, dist_mid);
	return true;
}

