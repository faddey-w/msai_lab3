#pragma once

#include <stdexcept>
#include "robocup.h"


namespace timer {


	class Timeout: public std::runtime_error {
	public:
		const float actual_time;
		const float declared_time;
		Timeout(float actual_time, float declared_time)
			: std::runtime_error("fixed block executes too long")
			, actual_time(actual_time), declared_time(declared_time) {}
	};


	class timer {
	protected:
		const float start_time;
		const float interval;
	public:
		timer(float interval)
			: start_time(get_time()), interval(interval) {};

		float time_elapsed() const { return 1000.0f * (get_time() - start_time); }
		float time_left() const { return interval - time_elapsed(); }

		void wait_elapse() const { delay((int)time_elapsed()); }

		bool timed_out() const { return time_left() <= 0; };

		void throw_if_elapsed() const {
			float time_left = this->time_left();
			if (time_left <= 0) {
				throw Timeout(interval - time_left, interval);
			}
		}

	};

	class block: public timer {

	public:
		block(float interval) : timer(interval) {};

		~block() {
			float time_left = this->time_left();
			if (time_left > 0) {
				delay(int(time_left));
			}
		}
	};
}
