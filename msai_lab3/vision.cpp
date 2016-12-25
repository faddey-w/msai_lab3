#include "vision.h"

#include <mutex>
#include <iostream>
#include <condition_variable>

#include "robocup.h"
#include "utils.h"

namespace vision {

	static data_array image_r( image_size, 0.0f ), image_w( image_size, 0.0f );
	static int swap_id = -1;
	static std::mutex access_mutex, stop_flag_mutex;
	static std::condition_variable swap_cond, pause_cond;
	static bool stop_flag, pause_flag; 

	bool is_not_paused() { return !pause_flag; }

	void vision_thread() {
		float last_distance = 0;
		last_distance = set_servo_and_read_ultrasonic(0, last_distance);
		{
			std::unique_lock<std::mutex> lk(access_mutex);
			stop_flag = false;
		}
		while (true) {

			{
				std::unique_lock<std::mutex> lk(access_mutex);
				if (stop_flag) break;
				pause_cond.wait(lk, is_not_paused);
			}


			for (int i = 0; i < image_size; i++) {
				image_w[i] = last_distance = set_servo_and_read_ultrasonic(
					idx2angle(i), last_distance
				);
			}
			{
				std::unique_lock<std::mutex> lk(access_mutex);
				swap_id++; image_r.swap(image_w);
				swap_cond.notify_all();
			}



			{
				std::unique_lock<std::mutex> lk(access_mutex);
				if (stop_flag) break;
				pause_cond.wait(lk, is_not_paused);
			}

			for (int i = image_size-1; i >= 0; i--) {
				image_w[i] = last_distance = set_servo_and_read_ultrasonic(
					idx2angle(i), last_distance
				);
			}
			{
				std::unique_lock<std::mutex> lk(access_mutex);
				swap_id++; image_r.swap(image_w);
				swap_cond.notify_all();
			}
		}
	}

	void read_image(data_array &buffer, int &out_swap_id, int prev_swap_id, 
					int read_start, int n_read) {
		std::unique_lock<std::mutex> lk(access_mutex);
		if (prev_swap_id != -1 || swap_id == -1) {
			swap_cond.wait(lk, [prev_swap_id]() { return prev_swap_id != swap_id; });
		}
		if (n_read < 0) { n_read = image_r.size() - read_start; }
		buffer.assign(image_r.begin() + read_start, 
					  image_r.begin() + read_start + n_read);
		out_swap_id = swap_id;
	}

	void stop_thread() {
		std::unique_lock<std::mutex> lk(access_mutex);
		stop_flag = true;
	}

	void set_pause(bool pause) {
		std::unique_lock<std::mutex> lk(access_mutex);
		pause_flag = pause;
		pause_cond.notify_all();
	}

	void wait_for_swap(int &out_swap_id, int prev_swap_id) {
		std::unique_lock<std::mutex> lk(access_mutex); 
		if (prev_swap_id == -1) prev_swap_id = swap_id;
		if (prev_swap_id != -1 || swap_id == -1) {
			swap_cond.wait(lk, [prev_swap_id]() { return prev_swap_id != swap_id; });
		}
		out_swap_id = swap_id;
	}

	float idx2angle(int idx) {
		return rightmost + idx*resolution;
	}

}

