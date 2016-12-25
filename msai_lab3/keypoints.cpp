#include "navigation.h"
#include "vision.h"

#include <iostream>
#include <vector>
#include <algorithm>

using vision::idx2angle;
using std::abs;

namespace navigation {
	std::vector<keypoint> detect_keypoints(const vision::data_array& image) {
		std::vector<float> grad;
		grad.reserve(image.size());
		for (int i = 1; i < image.size(); i++) {
			grad.push_back(image[i] - image[i - 1]);
		}
		std::vector<keypoint> keypoints;
		for (int i = 1; i < grad.size() - 1; ++i) {
			if (abs(grad[i - 1]) < abs(grad[i]) && abs(grad[i]) > abs(grad[i + 1])) {
				keypoints.push_back({
					{ 
						(idx2angle(i) + idx2angle(i-1))/2,
						(image[i] + image[i-1])/2 
					},
					grad[i] > 0
				});
			}
		}
		return keypoints;
	}


	std::vector<int> _k_means_most_dense_cluster(const std::vector<point>& points, int n_clusters) {
		std::vector<point> centroids;
		for (int i = 0; i < n_clusters; i++) {
			centroids.push_back(points[i]);
		}
		std::vector<int> assignments(points.size(), 0);

		int iters_cnt = 0;
		bool has_transitions = false;
		do {
			for (int i = 0; i < points.size(); i++) {
				float min_dist = 1000000000, best_label;
				for (int label = 0; label < n_clusters; label++) {
					float dst = dist(points[i], centroids[label]);
					dst = dst*dst*dst;
					if (dst < min_dist) {
						min_dist = dst;
						best_label = label;
					}
				}
				if (best_label != assignments[i]) {
					has_transitions = true;
					assignments[i] = best_label;
				}
			}

			centroids.assign(n_clusters, { 0, 0 });
			std::vector<int> cluster_sizes(n_clusters, 0);
			for (int i = 0; i < points.size(); i++) {
				centroids[assignments[i]] = add(centroids[assignments[i]], points[i]);
				cluster_sizes[assignments[i]] += 1;
			}
			for (int label = 0; label < n_clusters; label++) {
				if (cluster_sizes[label] == 0) continue;
				centroids[label] = {
					centroids[label].angle,
					centroids[label].dist / cluster_sizes[label]
				};
			}

			iters_cnt++;
		} while (has_transitions && iters_cnt < 100);

		float best_pairwise_distances = 10000000000; int best_label;
		for (int label = 0; label < n_clusters; ++label) {
			float pairwise_distances = 0;
			for (int i = 0; i < points.size(); i++) {
				if (assignments[i] != label) continue;
				for (int j = i + 1; j < points.size(); j++) {
					if (assignments[j] != label) continue;
					pairwise_distances += dist(points[i], centroids[label]);;
				}
			}
			if (pairwise_distances < best_pairwise_distances) {
				best_pairwise_distances = pairwise_distances;
				best_label = label;
			}
		}

		std::vector<int> result;
		for (int i = 0; i < points.size(); i++) {
			if (assignments[i] == best_label) {
				result.push_back(i);
			}
		}
		return result;
	}

	point match_keypoints(
		const std::vector<keypoint>& prev_keypoints,
		const std::vector<keypoint>& keypoints,
		std::map<int, int> &mapping,
		std::set<int> &lost_keypoints,
		std::set<int> &new_keypoints
	) {
		std::vector<point> transitions;
		for (auto& prev_kp : prev_keypoints) {
			for (auto& kp : keypoints) {
				transitions.push_back(sub(kp.pt, prev_kp.pt));
			}
		}
		/*for (auto &t : transitions) {
			std::cout << t.dist << ' ' << t.angle << std::endl;
		}*/
		const int n_clusters = 7;
		auto dense_cluster = _k_means_most_dense_cluster(transitions, n_clusters);

		for (int i : dense_cluster) {
			const keypoint& prev_kp = prev_keypoints[i / prev_keypoints.size()];
			const keypoint& kp = keypoints[i % prev_keypoints.size()];
			std::cout << prev_kp.pt.dist << ',' << prev_kp.pt.angle 
				<< " -> " << kp.pt.dist << ',' << kp.pt.angle << std::endl;
		}


		// TODO
		return{};
	}
}
