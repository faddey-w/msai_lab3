#pragma once

#include <vector>
#include <map>
#include <set>
#include <functional>

#include "vision.h"


namespace navigation {

	struct coord;

	struct point {
		float dist;
		float angle;

		point(float dist, float angle) : dist(dist), angle(angle) {};
		point() : point(0, 0) {};
		point(const coord&);

	};

	struct coord {
		float x;
		float y;

		coord() : coord(0, 0) {};
		coord(float x, float y) : x(x), y(y) {};
		coord(const point&);

		coord operator+(const coord& other) const {
			return{ x + other.x, y + other.y };
		}

		coord operator-(const coord& other) const {
			return{ x - other.x, y - other.y };
		}

		coord& operator+=(const coord& other) {
			x += other.x; y += other.y;
			return *this;
		}

		float radius() const {
			return std::sqrtf(x*x + y*y);
		}

		float distance(const coord& c) const {
			float dx = x - c.x, dy = y - c.y;
			return sqrtf(dx*dx + dy*dy);
		}
	};

	coord operator*(float koeff, const coord& c);

	struct keypoint {
		point pt;
		bool is_r2l;
	};

	struct descriptor {
		static const int N_ATTRS = 5;
		union {
			struct {
				point pt;
				coord cr;
				float gradient;
			};
			float data[N_ATTRS];
		};
	};

	struct segdescr {
		descriptor start;
		descriptor end;

		coord center;
		float length;
		float orientation;

		segdescr(const descriptor &start, const descriptor &end);
	};

	point add(const point&, const point&);
	point sub(const point&, const point&);
	float dist(const point&, const point&);

	std::vector<keypoint> detect_keypoints(const vision::data_array&);
	std::vector<segdescr> detect_segments(const vision::data_array&);
	point match_keypoints(
		const std::vector<keypoint>& prev_keypoints,
		const std::vector<keypoint>& keypoints,
		std::map<int, int> &mapping,
		std::set<int> &lost_keypoints,
		std::set<int> &new_keypoints
	);
	void match_segments(
		const std::vector<segdescr>& prev_segments,
		const std::vector<segdescr>& segments,
		std::function<float(const segdescr&, const segdescr&)> get_distance,
		std::map<int, int> &mapping,
		std::set<int> &lost_segments,
		std::set<int> &new_segments
	);


	std::vector<point> to_points(const vision::data_array &image);
	std::vector<coord> to_coords(const std::vector<point> &points);
	std::vector<descriptor> build_descriptors(const vision::data_array &image);

	bool detect_corner(float min_angle, float max_angle, float& result);
	void rotate_angle(float angle);
}

