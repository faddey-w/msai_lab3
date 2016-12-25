#include "navigation.h"
#define _USE_MATH_DEFINES
#include <math.h>

static float _to_rad(float degs) {
	return M_PI * degs / 180;
}
static float _to_deg(float rads) {
	return 180 * rads / M_PI;
}

namespace navigation {

	coord::coord(const point& p)
		: x(p.dist*cosf(_to_rad(p.angle)))
		, y(p.dist*sinf(_to_rad(p.angle))) {};


	point::point(const coord& cr)
		: dist(sqrtf(cr.x*cr.x + cr.y*cr.y))
		, angle(_to_deg(acosf(cr.x / dist)) * (cr.y>0 ? 1 : -1)) {};

	std::vector<descriptor> build_descriptors(const vision::data_array &image) {
		std::vector<descriptor> result;
		auto points = to_points(image);
		auto coords = to_coords(points);
		for (int i = 0; i < image.size(); ++i) {
			result.push_back({ 
				points[i], coords[i], 0
			});
		}
		return result;
	}

	coord operator*(float koeff, const coord& c) {
		return{ koeff*c.x, koeff*c.y };
	}

	std::vector<point> to_points(const vision::data_array &image) {
		std::vector<point> result;
		for (int i = 0; i < image.size(); ++i) {
			result.push_back({ image[i], vision::idx2angle(i) });
		}
		return result;
	}

	std::vector<coord> to_coords(const std::vector<point> &points) {
		std::vector<coord> result;
		for (int i = 0; i < points.size(); ++i) {
			result.push_back(coord(points[i]));
		}
		return result;
	}

	segdescr::segdescr(const descriptor &start, const descriptor &end)
	: start(start), end(end) {
		center = {
			(start.cr.x + end.cr.x) / 2,
			(start.cr.y + end.cr.y) / 2
		};
		length = dist(start.pt, end.pt);
		orientation = acosf(
			(start.cr.x*center.x + start.cr.y*center.y)
			/ (start.cr.radius() * center.radius())
		);
	}

}
