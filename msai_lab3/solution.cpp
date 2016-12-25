#include <iostream>
#include <algorithm>
#include <thread>
#include <string>

#include "robocup.h"
#include "timer.h"
#include "navigation.h"
#include "vision.h"
#include "utils.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define LAB3_TASK2


#ifdef LAB3_TASK1

void setup() {};


bool sensor_in_area(int sensor_id) {
	return get_grayscale(sensor_id) > 0.5f;
}


void align_to_border() {
	set_movement(0.1, -0.1);
	while (!(sensor_in_area(RIGHT_GS) && sensor_in_area(CENTER_GS)
			 && !sensor_in_area(LEFT_GS))) {
		delay(5);
	}
	set_movement(-0.1, 0.1);
	while (sensor_in_area(CENTER_GS)) {
		delay(5);
	}
	set_movement(0, 0);
}


void user_loop() {
	static bool aligned = false;

	if (!aligned) {
		align_to_border();
		aligned = true;
	}

	if (sensor_in_area(CENTER_GS)) {
		set_movement(0.45, 0.55);
	} else {
		set_movement(0.55, 0.45);
	}
	delay(1);
	set_movement(0.5, 0.5);
	delay(1);

	if (sensor_in_area(LEFT_GS) || !sensor_in_area(RIGHT_GS)) {
		set_movement(-0.5, -0.5);
		aligned = false;
		while (sensor_in_area(LEFT_GS) || !sensor_in_area(RIGHT_GS)) {
			delay(1);
		}
		set_movement(0, 0);
	}

}
#else // task 2


vision::data_array image; 
int swap_id = -1;

static float _to_rad(float degs) {
	return M_PI * degs / 180;
}
static float _to_deg(float rads) {
	return 180 * rads / M_PI;
}

void move(navigation::point target_move) {
	std::map<int, int> mapping;
	std::set<int> lost_segments;
	std::set<int> new_segments;

	set_movement(0, 0);
	vision::read_image(image, swap_id, swap_id);
	auto prev_segments = navigation::detect_segments(image);

	const int rotate_direction = target_move.angle > 0 ? 1 : -1;

	bool do_move = true;
	while (rotate_direction*target_move.angle > 0) {
		if (do_move) {
			set_movement(-rotate_direction/2.0, rotate_direction/2.0);
			delay(75);
			set_movement(0, 0);
		}
		vision::read_image(image, swap_id, swap_id);
		auto segments = navigation::detect_segments(image);
		navigation::match_segments(
			prev_segments, segments,
			[](const navigation::segdescr& s1, const navigation::segdescr &s2) 
			{
			return s1.center.distance(s2.center)
				+ std::abs(s1.length - s2.length)
					+ std::abs(s1.center.radius() - s2.center.radius());
			},
			mapping, lost_segments, new_segments
		);

		do_move = mapping.size() != 0;
		if (!do_move) continue;

		float avg_angle = 0;
		for (auto &item : mapping) {
			auto prev_p = prev_segments[item.first].center;
			auto this_p = segments[item.second].center;
			avg_angle += navigation::point(prev_p).angle - navigation::point(this_p).angle;
		}
		avg_angle /= mapping.size();

		target_move.angle -= 1.5*avg_angle;

		prev_segments = segments;
	}


	do_move = true;
	while (target_move.dist > 0) {
		if (do_move) {
			set_movement(1, 1);
			delay(50);
			set_movement(0, 0);
		}
		vision::read_image(image, swap_id, swap_id);
		auto segments = navigation::detect_segments(image);
		navigation::match_segments(
			prev_segments, segments,
			[](const navigation::segdescr& s1, const navigation::segdescr &s2) {
			return s1.center.distance(s2.center)
				+ std::abs(s1.length - s2.length)
				+ std::abs(s1.center.radius() - s2.center.radius());
		},
			mapping, lost_segments, new_segments
			);

		do_move = mapping.size() != 0;
		if (!do_move) continue;

		float avg_move = 0;
		for (auto &item : mapping) {
			auto prev_p = prev_segments[item.first].center;
			auto this_p = segments[item.second].center;
			avg_move += prev_p.distance(this_p);
		}
		avg_move /= mapping.size();

		target_move.dist -= avg_move;

		prev_segments = segments;
	}
}

static const int target_room = 6;


class DoorDetector {
	float last_distance;
public:
	float angle;
	const std::string name;

	float read() const {
		set_servo_angle(angle);
		delay(20);
		return get_ultrasonic();
	}

	DoorDetector(float angle, const std::string side_name) 
	: angle(angle), name(side_name) {
		last_distance = read();
	}

	int detect() {
		float last_dist = last_distance, dist = read();
		last_distance = dist;
		if (dist < last_dist*0.8) {
			return -1;
		}
		if (dist > 1.2*last_dist) {
			return 1;
		}
		return 0;
	}
};



void setup() {
	set_movement(0.25, 0.25);
	std::cout << "Searching for target room #" << target_room << std::endl;
};

void user_loop() {
	static int room_counter = 0;
	static float target_room_found_time = -1;
	static DoorDetector detectors[2] = { DoorDetector(89, "left"), DoorDetector(-89, "right") };
	static DoorDetector *target_dd;

	if (target_room_found_time < 0) {
		for (DoorDetector &dd : detectors) {
			int result = dd.detect();
			if (result == 1) {
				room_counter++;
				std::cout << "entering room #" << room_counter << " on "
					<< dd.name << " side" << std::endl;
				if (room_counter == target_room) {
					std::cout << "Target room found, looking for end of the door" << std::endl;
					target_room_found_time = get_time();
					target_dd = &dd;
				}
			} else if (result == -1) {
				std::cout << "leaving room on " << dd.name << " side" << std::endl;
			}
		}
	} else {
		
		if (target_dd->detect() == -1) {
			std::cout << "found end of target room, going back to door center" << std::endl;

			float target_ride_time = get_time() - target_room_found_time;
			set_movement(-0.25, -0.25);
			delay(target_ride_time * 500);
			set_movement(0, 0);

			std::cout << "turn to the door" << std::endl;
			int turn_direction = target_dd->name == "right" ? 1 : -1;
			const float rot_speed = 0.3;

			DoorDetector scan_dd(target_dd->angle, "");
			while (scan_dd.detect() != -1) {
				scan_dd.angle += 0.5*turn_direction;
			}
			const float half_door_angle = std::abs(target_dd->angle - scan_dd.angle);

			float half_door_turn_time = get_time();
			set_movement(turn_direction*rot_speed, -turn_direction*rot_speed);
			while (target_dd->detect() != -1) {}
			set_movement(0, 0);
			half_door_turn_time = (get_time() - half_door_turn_time) - 0.01;

			float time_for_90deg = half_door_turn_time * 90 / half_door_angle;

			set_movement(turn_direction*rot_speed, -turn_direction*rot_speed);
			delay(1000*(time_for_90deg - half_door_turn_time));
			set_movement(0, 0);

			std::cout << "done." << std::endl;

			while (true) { delay(1000); }
		}
	}













	/*static int direction = 1;
	static std::thread vis_thr(vision::vision_thread);*/

	/*move({ 25, 90 });*/



	/*vision::read_image(image, swap_id, swap_id);
	auto prev_segments = navigation::detect_segments(image);
	for (auto &sd : prev_segments) {
		std::cout << sd.start.pt.dist << ' ' << sd.start.pt.angle << "       "
			<< sd.end.pt.dist << ' ' << sd.end.pt.angle << std::endl;
	}
	std::cout << std::endl;

	set_movement(direction*0.3, -direction*0.3);
	delay(1000);
	set_movement(0, 0);

	vision::read_image(image, swap_id, swap_id);
	auto segments = navigation::detect_segments(image);
	for (auto &sd : segments) {
		std::cout << sd.start.pt.dist << ' ' << sd.start.pt.angle << "       "
			<< sd.end.pt.dist << ' ' << sd.end.pt.angle << std::endl;
	}
	std::cout << std::endl;

	set_movement(0.15+direction*0.05, 0.15-direction*0.05);
	delay(1000);

	vision::read_image(image, swap_id, swap_id);
	for (auto &sd : navigation::detect_segments(image)) {
		std::cout << sd.start.pt.dist << ' ' << sd.start.pt.angle << "       "
			<< sd.end.pt.dist << ' ' << sd.end.pt.angle << std::endl;
	}
	std::cout << std::endl;

	set_movement(0, 0);

	std::map<int, int> mapping;
	std::set<int> lost_segments;
	std::set<int> new_segments;
	navigation::coord movement; 
	float angle;
	navigation::match_segments(
		prev_segments, segments, 
		[](const navigation::segdescr& s1, const navigation::segdescr &s2) {
		return s1.center.distance(s2.center)
			+ std::abs(s1.length - s2.length)
			+ std::abs(s1.center.radius() - s2.center.radius());
		}, 
		mapping, lost_segments, new_segments
	);

	direction = -direction;*/

}
#endif // LAB3_TASK1