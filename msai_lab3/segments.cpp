#include "navigation.h"
#include <math.h>
#include <algorithm>
#include <list>


struct Segment {
	int x1, x2;

	float energy(const std::vector<navigation::descriptor>& image, int attr) const {
		int segsize = (x2 - x1);
		//if (segsize < 2) return 0;
		return size() / (0.1 + get_gradient_dispersion(image, attr));
	}

	float get_gradient_dispersion(
		const std::vector<navigation::descriptor>& image, int attr,
		const int* start=nullptr, const int *stop=nullptr
	) const {
		if (start == nullptr) start = &x1;
		if (stop == nullptr) stop = &x2;
		float sqr_grad_sum = 0, grad_sum = 0;
		for (int i = *start+1; i < *stop; ++i) {
			float grad = image[i].data[attr] - image[i - 1].data[attr];
			sqr_grad_sum += grad*grad;
			grad_sum += grad;
		}
		float avg_grad = grad_sum / size();
		return sqr_grad_sum / size() - avg_grad*avg_grad;
	}

	float get_avg_gradient(const std::vector<navigation::descriptor>& image, int attr) const {
		return (image[x2].data[attr] - image[x1].data[attr]) / size();
	}

	bool mutate(const std::vector<navigation::descriptor>& image, int attr) {
		float best = energy(image, 2) + energy(image, 3);
		bool found_better = false;
		std::pair<int, int> shift;
		std::vector<std::pair<int, int> > all_shifts{ 
			{0,1}, {0,-1}, {1,0}, {-1,0}, 
			{-2,-1}, {1,2}, {-3, -2}, {2, 3}
		};
		for (auto &sh : all_shifts) {
			if (x1 + sh.first >= image.size() || x1 + sh.first < 0
				|| x2 + sh.second >= image.size() || x2 + sh.second < 0) {
				continue;
			}

			x1 += sh.first; x2 += sh.second;
			float enrg = energy(image, 2) + energy(image, 3);
			if (enrg > best) {
				shift = sh;
				found_better = true;
			}
			x1 -= sh.first; x2 -= sh.second;
		}
		if (found_better) {
			x1 += shift.first; x2 += shift.second;
		}
		return found_better;
	}

	bool is_within(const Segment& other) const {
		return x1 >= other.x1 && x2 <= other.x2;
	}

	bool operator==(const Segment& other) const {
		return x1 == other.x1 && x2 == other.x2;
	}

	int size() const {
		return x2 - x1;
	}
};



namespace navigation {

	std::vector<segdescr> detect_segments(const vision::data_array& image) {
		std::vector<navigation::descriptor> descr_image = build_descriptors(image);
		std::vector<Segment> segments, buffer;
		const int USE_ATTR = 2;

		//generate initial segments
		for (int i = 0; i < image.size() - 3; i++) {
			segments.push_back({ i, i + 3 });
		}

		// fit
		for (auto& segm : segments) {
			bool need_mutate = true;
			while (need_mutate) {
				need_mutate = segm.mutate(descr_image, USE_ATTR);
			}
		}

		//remove duplicates is any
		buffer.swap(segments);
		for (auto& segm : buffer) {
			bool found = false;
			for (auto &prev_segm : segments) {
				if (segm == prev_segm) {
					found = true;
					break;
				}
			}
			if (!found) {
				segments.push_back(segm);
			}
		}
		buffer.clear();

		// remove those segments that are within others
		buffer.swap(segments);
		for (int i = 0; i < buffer.size(); i++) {
			auto &segm = buffer[i];
			bool found_enclosing = false;
			for (int j = 0; j < buffer.size(); j++) {
				if (i == j) continue;
				auto &other_segm = buffer[j];
				if (segm.is_within(other_segm)) {
					found_enclosing = true;
					break;
				}
			}
			if (!found_enclosing) {
				segments.push_back(segm);
			}
		}
		buffer.clear();

		// sort segments by angle
		std::sort(segments.begin(), segments.end(), [](Segment &s1, Segment&s2) {
			return s1.x1 < s2.x1;
		});

		// remove "dead" segments that may appear on junction points
		buffer.swap(segments);
		//segments.push_back(buffer[0]);
		for (int i = 0; i < buffer.size(); i++) {
			//auto &prev = buffer[i-1];
			auto &segm = buffer[i];
			//auto &next = buffer[i+1];
			if (segm.size() <= 2) { // & segm.x1 < prev.x2 && segm.x2 > next.x1) {
				continue;
			}
			segments.push_back(segm);
		}
		//segments.push_back(buffer[buffer.size()-1]);
		buffer.clear();

		std::vector<segdescr> result;
		for (auto &segm : segments) {
			result.push_back({
				descr_image[segm.x1],
				descr_image[segm.x2 - 1]
			});
		}

		return result;
	}


	coord optimal_transition(const segdescr&, const segdescr&);


	void match_segments(
		const std::vector<segdescr>& prev_segments,
		const std::vector<segdescr>& segments,
		std::function<float(const segdescr&, const segdescr&)> get_distance,
		std::map<int, int> &mapping,
		std::set<int> &lost_segments,
		std::set<int> &new_segments
	) {
		std::vector<std::vector<float> > distances{ 
			prev_segments.size(), std::vector<float>(segments.size(), 0) 
		};
		for (int i = 0; i < prev_segments.size(); ++i) {
			for (int j = 0; j < segments.size(); ++j) {
				distances[i][j] = get_distance(prev_segments[i], segments[j]);
			}
		}
		std::vector<float> min_distances_for_prev, min_distances_for_this;

		for (int i = 0; i < prev_segments.size(); ++i) {
			min_distances_for_prev.push_back(distances[i][0]);
			for (int j = 0; j < segments.size(); ++j) {
				if (min_distances_for_prev[i] > distances[i][j]) {
					min_distances_for_prev[i] = distances[i][j];
				}
			}
		}
		for (int j = 0; j < segments.size(); ++j) {
			min_distances_for_this.push_back(distances[0][j]);
			for (int i = 0; i < prev_segments.size(); ++i) {
				if (min_distances_for_this[j] > distances[i][j]) {
					min_distances_for_this[j] = distances[i][j];
				}
			}
		}

		std::vector<std::vector<float> > probabilities{
			prev_segments.size(), std::vector<float>(segments.size(), 0)
		};
		for (int i = 0; i < prev_segments.size(); ++i) {
			for (int j = 0; j < segments.size(); ++j) {
				float dist = distances[i][j];
				if (dist < 0.00001) {
					probabilities[i][j] = 1;
					continue;
				}
				probabilities[i][j] = min_distances_for_prev[i] * min_distances_for_this[j] / (dist*dist);
			}
		}

		mapping.clear();
		lost_segments.clear();
		new_segments.clear();
		std::set<int> mapped_new;
		for (int i = 0; i < prev_segments.size(); ++i) {
			float max_prob = probabilities[i][0];
			int idx = 0;
			for (int j = 1; j < segments.size(); ++j) {
				float prob = probabilities[i][j];
				if (prob > max_prob && mapped_new.count(j) == 0) {
					max_prob = prob;
					idx = j;
					mapped_new.insert(j);
				}
			}
			if (max_prob > 0.95) {
				mapping[i] = idx;
			}
		}
		for (int i = 0; i < prev_segments.size(); i++) {
			if (mapping.count(i) == 0) {
				lost_segments.insert(i);
			}
		}
		for (int j = 0; j < segments.size(); j++) {
			if (mapped_new.count(j) == 0) {
				new_segments.insert(j);
			}
		}

	}


	coord _solution1(float X1, float X2, float Y1, float Y2) {
		return{
			sqrtf(X1*X2),
			(X1*Y2 + X2*Y1 + (Y1 + Y2)*sqrtf(X1*X2)) / (X1 + X2 + 2 * sqrtf(X1*X2))
		};
	}

	coord _solution2(float X1, float X2, float Y1, float Y2) {
		float V = sqrtf((X1 + X2)*(X1*Y2 + X2*Y1) / (Y1 + Y2));
		return{ V, (X2*Y1 + X1*Y2 + (Y1 + Y2)*V) / (X1 + X2 + 2 * V) };
	}


	std::function<coord(float, float, float, float)> swapaxes(
		std::function<coord(float, float, float, float)> func
	) {
		return [func](float X1, float X2, float Y1, float Y2) {
			auto transition = func(Y1, Y2, X1, X2);
			return coord{ transition.y, transition.x };
		};
	}


	std::function<coord(float, float, float, float)> swapaxes2(
		std::function<coord(float, float, float, float)> func
	) {
		return [func](float X1, float X2, float Y1, float Y2) {
			return func(Y1, Y2, X1, X2);
		};
	}


	float segdistance(const segdescr& s1, const segdescr& s2, const coord& t) {
		return (s1.start.cr - s2.start.cr + t).radius() + (s1.end.cr - s2.end.cr + t).radius();
	}


	coord optimal_transition(const segdescr& s1, const segdescr& s2) {
		return s2.center - s1.center;
		/*coord result = { 0,0 };
		float opt_distance = segdistance(s1, s2, result);
		std::pair<float, float> shifts[] = { 
			{ 0.05, 0.0 }, { -0.05, 0.0 },
			{ 0.0, 0.05 },{ 0.0, -0.05 }, 
		};
		for (int counter = 0, was_shifted = true; was_shifted && counter < 1000; ++counter) {
			was_shifted = false;
			for (auto shift : shifts) {
				auto shifted = result + coord{shift.first, shift.second};
				float distance = segdistance(s1, s2, shifted);
				if (distance < opt_distance) {
					opt_distance = distance;
					result = shifted;
					was_shifted = true;
					break;
				}
			}
		}
		return result;*/
		/*float optimal_distance;
		const float X1 = s1.start.cr.x - s2.start.cr.x,
			X2 = s1.end.cr.x - s2.end.cr.x,
			Y1 = s1.start.cr.y - s2.start.cr.y,
			Y2 = s1.end.cr.y - s2.end.cr.y;
		auto sol1 = _solution1(X1, X2, Y1, Y2);
		auto sol2 = _solution2(X1, X2, Y1, Y2);
		auto sol1sa = swapaxes(_solution1)(X1, X2, Y1, Y2);
		auto sol2sa = swapaxes(_solution2)(X1, X2, Y1, Y2);
		auto sol1sa2 = swapaxes2(_solution1)(X1, X2, Y1, Y2);
		auto sol2sa2 = swapaxes2(_solution2)(X1, X2, Y1, Y2);
		return sol1;*/
	}

}
