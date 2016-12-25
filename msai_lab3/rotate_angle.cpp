#include <algorithm>
#define _USE_MATH_DEFINES 
#include <math.h>
#include <vector>
#include <iostream>

#include "navigation.h"
#include "robocup.h"
#include "vision.h"
#include "utils.h"


int _cross_corr_argmax(const std::vector<float> &arr1, const std::vector<float> &arr2) {
	int result; float max_value = -1;
	for (int j = 0; j < arr1.size(); j++) {
		float value = 0;
		for (int i = 0; i < std::min(j + 1, (int)arr2.size()); i++) {
			value += std::abs(arr1[i - j + arr1.size() - 1] - arr2[i]);
		}
		if (value > max_value) {
			max_value = value;
			result = j;
		}
	}
	for (int k = 0; k < arr2.size()-1; k++) {
		float value = 0;
		for (int i = 0; i < std::min(arr1.size(), arr2.size()-k+1); i++) {
			value += std::abs(arr1[i] - arr2[k+1]);
		}
		if (value > max_value) {
			max_value = value;
			result = k + arr1.size();
		}
	}
	return result;
}


void navigation::rotate_angle(float angle) {
	if (std::abs(angle) > 80) {
		rotate_angle(angle / 2);
		rotate_angle(angle / 2);
		return;
	}

	int swap_id = -1;
	vision::data_array main_image;
	vision::data_array running_image;

	const float rot_speed = 1;
	int direction;
	if (angle > 0) {
		direction = 1;
		vision::read_image(main_image, swap_id, -1, vision::image_size / 4);
	} else {
		direction = -1;
		vision::read_image(main_image, swap_id, -1, 0, 3 * vision::image_size / 4);
	}
	for (auto x : main_image) { std::cout << x << ", "; }
	std::cout << std::endl;
	int shift;
	do {
		set_movement(-direction*rot_speed, direction*rot_speed);
		delay(10);
		set_movement(0, 0);
		vision::read_image(running_image, swap_id, swap_id,
						   vision::image_size / 4, vision::image_size / 2);

		for (auto x : running_image) { std::cout << x << ", "; }
		std::cout << std::endl;
		int sh1 = _cross_corr_argmax(main_image, running_image);
		int sh2 = _cross_corr_argmax(running_image, main_image);
		std::cout << sh1 << ' ' << sh2 << std::endl;
		shift = (sh1 - sh2) / 2;
	} while (std::abs(angle) > std::abs(shift * vision::resolution));
}

