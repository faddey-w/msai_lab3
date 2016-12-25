#pragma once

#include <vector>


namespace vision {

	constexpr int image_size = 50;
	constexpr float leftmost = 89.9f;
	constexpr float rightmost = -89.9f;
	constexpr float resolution = (leftmost - rightmost) / (image_size-1);

	typedef std::vector<float> data_array;

	void vision_thread();

	void read_image(data_array &buffer, int &out_swap_id, int prev_swap_id=-1,
					int read_start=0, int n_read=-1);

	void stop_thread();

	float idx2angle(int idx);

	void set_pause(bool pause);

	void wait_for_swap(int &out_swap_id, int prev_swap_id=-1);

}

