#include "navigation.h"
#define _USE_MATH_DEFINES
#include <math.h>

using std::cosf;
using std::acosf;
using std::sinf;
using std::sqrtf;

static float _to_rad(float degs) {
	return M_PI * degs / 180;
}
static float _to_deg(float rads) {
	return 180 * rads / M_PI;
}


namespace navigation {

	point add(const point& p1, const point& p2) {
		float x = p1.dist*cosf(_to_rad(p1.angle)) + p2.dist*cosf(_to_rad(p2.angle));
		float y = p1.dist*sinf(_to_rad(p1.angle)) + p2.dist*sinf(_to_rad(p2.angle));
		float dist = sqrt(x*x + y*y);
		float angle;
		if (dist < 0.0001) {
			angle = 0;
		} else {
			angle = acosf(y / dist);
			if (x < 0) angle = -angle;
		}
		return { _to_deg(angle), dist };
	}

	point sub(const point& p1, const point& p2) {
		return add(p1, { p2.angle+180, p2.dist });
	}

	float dist(const point& p1, const point& p2) {
		float dx = p1.dist*cosf(_to_rad(p1.angle)) - p2.dist*cosf(_to_rad(p2.angle));
		float dy = p1.dist*sinf(_to_rad(p1.angle)) - p2.dist*sinf(_to_rad(p2.angle));
		return sqrt(dx*dx + dy*dy);
	}
}
