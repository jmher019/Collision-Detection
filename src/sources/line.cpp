/**
 *	@file		line.cpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/17/2018
 *	@version	1.0
 *
 *	@brief Line class definition
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for the Line class.
 *
 *	Common use cases for the Line class is for utility
 *	purposes and should not be used as a replacement data
 *	structure for managing vertices.
 */

#include <line.hpp>

using namespace collision;

/**
 * @brief Default constructor
 */
Line::Line(const vec3& a, const vec3& b):
	a(a),
	b(b) {}

/**
 * @brief Copy constructor
 */
Line::Line(const Line& l):
	a(l.a), b(l.b) {}

/**
 * @brief Move constructor
 */
Line::Line(Line&& l) {
	a = move(l.a);
	b = move(l.b);
}

/**
 * @brief gets the start point on the line
 *
 * @return the starting point on the line
 */
const vec3& Line::getPointA(void) const {
	return a;
}

/**
 * @brief sets the start point of the line
 *
 * @param a the point to be used as the starting point
 * @return void
 */
void Line::setPointA(const vec3& a) {
	this->a = a;
}

/**
 * @brief gets the end point on the line
 *
 * @return the ending point on the line
 */
const vec3& Line::getPointB(void) const {
	return b;
}

/**
 * @brief sets the end point of the line
 *
 * @param b the point to be used as the emding point
 * @return void
 */
void Line::setPointB(const vec3& b) {
	this->b = b;
}

/**
 * @brief treats the line as a segment with endpoints
 * a and b to find the closest point on the segment to another point
 *
 * @param p the point for which we need to find the closest point on the segment
 * @return the point on the segment closest the the input point
 */
vec3 Line::getClosestPtPointSegment(const vec3& p) const {
	const vec3 ab = b - a;
	// Project p onto ab, but deferring divide by Dot(ab, ab)
	const float t = dot(p - a, ab);
	if (t <= 0.0f) {
		// p projects outside the [a, b] interval, on the a side; clamp to a
		return a;
	}
	
	const float denom = dot(ab, ab); // Always nonnegative since denom = ||ab||^2
	if (t >= denom) {
		// p projects outside the [a, b] interval, on the b side; clamp to b
		return b;
	}

	return a + ab * t / denom;
}

/**
 * @brief treats the line as a segment with endpoints
 * a and b to find the closest point on the segment to another point
 *
 * @param p the point for which we need to find the closest point on the segment
 * @param a the starting point of the segment
 * @param b the ending point of the segment
 * @return the point on the segment closest the the input point
 */
vec3 Line::getClosestPtPointSegment(const vec3& p, const vec3& a, const vec3& b) {
	const vec3 ab = b - a;
	// Project p onto ab, but deferring divide by Dot(ab, ab)
	const float t = dot(p - a, ab);
	if (t <= 0.0f) {
		// p projects outside the [a, b] interval, on the a side; clamp to a
		return vec3(a);
	}

	const float denom = dot(ab, ab); // Always nonnegative since denom = ||ab||^2
	if (t >= denom) {
		// p projects outside the [a, b] interval, on the b side; clamp to b
		return vec3(b);
	}

	return a + ab * t / denom;
}

/**
 * @brief Computes the squared distance of a point and a line segment
 *
 * @param p the input point
 * @return the squared distance between the input point and the line segment
 */
float Line::getSquaredDistPointSegment(const vec3& p) const {
	const vec3 ab = b - a, ap = p - a, bp = p - b;
	const float e = dot(ap, ab);

	// Handle cases where c projects outside ab
	if (e <= 0.0f) return dot(ap, ap);

	const float f = dot(ab, ab);
	if (e >= f) return dot(bp, bp);

	// Handle cases where p projects onto ab
	return dot(ap, ap) - e * e / f;
}

/**
 * @brief Computes the squared distance of a point and a line segment
 *
 * @param p the input point
 * @param a the starting point of the segment
 * @param b the ending point of the segment
 * @return the squared distance between the input point and the line segment
 */
float Line::getSquaredDistPointSegment(const vec3& p, const vec3& a, const vec3& b) {
	const vec3 ab = b - a, ap = p - a, bp = p - b;
	const float e = dot(ap, ab);

	// Handle cases where c projects outside ab
	if (e <= 0.0f) return dot(ap, ap);

	const float f = dot(ab, ab);
	if (e >= f) return dot(bp, bp);

	// Handle cases where p projects onto ab
	return dot(ap, ap) - e * e / f;
}

/**
 * @brief Computes the closest point between two segments
 *
 * @param c1 the closest point on the first line segment
 * @param c2 the closest point on the second line segment
 * @param l2 the other line segment
 * @return the distance squared between c1 and c2
 */
float Line::getClosestPtSegmentSegment(vec3& c1, vec3& c2, const Line& l2) const {
	const vec3 d1 = b - a;
	const vec3 d2 = l2.b - l2.a;
	const vec3 r = a - l2.a;
	const float b = dot(d1, d1);
	const float e = dot(d2, d2);
	const float f = dot(d2, r);
	float s = 0.f, t = 0.f;

	// Check if either or both segments degenerate into points
	if (b <= 1e-8f && e <= 1e-8f) {
		// Both segments degenerate into points
		c1 = vec3(a);
		c2 = vec3(l2.a);
		return dot(c2 - c1, c2 - c1);
	}

	if (b <= 1e-8f) {
		// First segment degenerates into a point
		t = clamp(f / e, 0.f, 1.f);
	}
	else {
		const float c = dot(d1, r);
		if (e <= 1e-8f) {
			// second segment degenerates to a point
			s = clamp(-c / b, 0.f, 1.f);
		}
		else {
			// The general nondegenerate case starts here
			const float d = dot(d1, d2);
			const float denom = b * e - d * d;

			// If segments not parallel, compute closest point on L1 to L2 and
			// clamp to segment s1. Else pick arbitrary s (here 0)
			if (denom != 0.f) {
				s = clamp(d * f - c * e, 0.f, 1.f);
			}
			else {
				t = (d * s + f) / e;

				// If t in [0, 1] done. Else clamp t, recompute s for the new value of t
				if (t < 0.f) {
					t = 0.f;
					s = clamp(-c / b, 0.f, 1.f);
				}
				else if (t > 1.f) {
					t = 1.f;
					s = clamp((d - c) / b, 0.f, 1.f);
				}
			}
		}
	}

	c1 = a + d1 * s;
	c2 = l2.a + d2 * t;
	return dot(c2 - c1, c2 - c1);
}

/**
 * @brief Computes the closest point between two segments
 *
 * @param c1 the closest point on the first line segment
 * @param c2 the closest point on the second line segment
 * @param p2 the starting point of the second segment
 * @param q2 the end point of the second segment
 * @param l2 the other line segment
 * @return the distance squared between c1 and c2
 */
float Line::getClosestPtSegmentSegment(vec3& c1, vec3& c2, const vec3& p2, const vec3& q2) const {
	const vec3 d1 = b - a;
	const vec3 d2 = q2 - p2;
	const vec3 r = a - p2;
	const float b = dot(d1, d1);
	const float e = dot(d2, d2);
	const float f = dot(d2, r);
	float s = 0.f, t = 0.f;

	// Check if either or both segments degenerate into points
	if (b <= 1e-8f && e <= 1e-8f) {
		// Both segments degenerate into points
		c1 = vec3(a);
		c2 = vec3(p2);
		return dot(c2 - c1, c2 - c1);
	}

	if (b <= 1e-8f) {
		// First segment degenerates into a point
		t = clamp(f / e, 0.f, 1.f);
	}
	else {
		const float c = dot(d1, r);
		if (e <= 1e-8f) {
			// second segment degenerates to a point
			s = clamp(-c / b, 0.f, 1.f);
		}
		else {
			// The general nondegenerate case starts here
			const float d = dot(d1, d2);
			const float denom = b * e - d * d;

			// If segments not parallel, compute closest point on L1 to L2 and
			// clamp to segment s1. Else pick arbitrary s (here 0)
			if (denom != 0.f) {
				s = clamp(d * f - c * e, 0.f, 1.f);
			}
			else {
				t = (d * s + f) / e;

				// If t in [0, 1] done. Else clamp t, recompute s for the new value of t
				if (t < 0.f) {
					t = 0.f;
					s = clamp(-c / b, 0.f, 1.f);
				}
				else if (t > 1.f) {
					t = 1.f;
					s = clamp((d - c) / b, 0.f, 1.f);
				}
			}
		}
	}

	c1 = a + d1 * s;
	c2 = p2 + d2 * t;
	return dot(c2 - c1, c2 - c1);
}

/**
 * @brief Computes the closest point between two segments
 *
 * @param c1 the closest point on the first line segment
 * @param c2 the closest point on the second line segment
 * @param l1 the first line segment
 * @param l2 the other line segment
 * @return the distance squared between c1 and c2
 */
float Line::getClosestPtSegmentSegment(vec3& c1, vec3& c2, const Line& l1, const Line& l2) {
	const vec3 d1 = l1.b - l1.a;
	const vec3 d2 = l2.b - l2.a;
	const vec3 r = l1.a - l2.a;
	const float b = dot(d1, d1);
	const float e = dot(d2, d2);
	const float f = dot(d2, r);
	float s = 0.f, t = 0.f;

	// Check if either or both segments degenerate into points
	if (b <= 1e-8f && e <= 1e-8f) {
		// Both segments degenerate into points
		c1 = vec3(l1.a);
		c2 = vec3(l2.a);
		return dot(c2 - c1, c2 - c1);
	}

	if (b <= 1e-8f) {
		// First segment degenerates into a point
		t = clamp(f / e, 0.f, 1.f);
	}
	else {
		const float c = dot(d1, r);
		if (e <= 1e-8f) {
			// second segment degenerates to a point
			s = clamp(-c / b, 0.f, 1.f);
		}
		else {
			// The general nondegenerate case starts here
			const float d = dot(d1, d2);
			const float denom = b * e - d * d;

			// If segments not parallel, compute closest point on L1 to L2 and
			// clamp to segment s1. Else pick arbitrary s (here 0)
			if (denom != 0.f) {
				s = clamp(d * f - c * e, 0.f, 1.f);
			}
			else {
				t = (d * s + f) / e;

				// If t in [0, 1] done. Else clamp t, recompute s for the new value of t
				if (t < 0.f) {
					t = 0.f;
					s = clamp(-c / b, 0.f, 1.f);
				}
				else if (t > 1.f) {
					t = 1.f;
					s = clamp((d - c) / b, 0.f, 1.f);
				}
			}
		}
	}

	c1 = l1.a + d1 * s;
	c2 = l2.a + d2 * t;
	return dot(c2 - c1, c2 - c1);
}

/**
 * @brief Computes the closest point between two segments
 *
 * @param c1 the closest point on the first line segment
 * @param c2 the closest point on the second line segment
 * @param p1 the start point of the first line segment
 * @param q1 the end point of the first line segment
 * @param l2 the other line segment
 * @return the distance squared between c1 and c2
 */
float Line::getClosestPtSegmentSegment(vec3& c1, vec3& c2, const vec3& p1, const vec3& q1, const Line& l2) {
	const vec3 d1 = q1 - p1;
	const vec3 d2 = l2.b - l2.a;
	const vec3 r = p1 - l2.a;
	const float b = dot(d1, d1);
	const float e = dot(d2, d2);
	const float f = dot(d2, r);
	float s = 0.f, t = 0.f;

	// Check if either or both segments degenerate into points
	if (b <= 1e-8f && e <= 1e-8f) {
		// Both segments degenerate into points
		c1 = vec3(p1);
		c2 = vec3(l2.a);
		return dot(c2 - c1, c2 - c1);
	}

	if (b <= 1e-8f) {
		// First segment degenerates into a point
		t = clamp(f / e, 0.f, 1.f);
	}
	else {
		const float c = dot(d1, r);
		if (e <= 1e-8f) {
			// second segment degenerates to a point
			s = clamp(-c / b, 0.f, 1.f);
		}
		else {
			// The general nondegenerate case starts here
			const float d = dot(d1, d2);
			const float denom = b * e - d * d;

			// If segments not parallel, compute closest point on L1 to L2 and
			// clamp to segment s1. Else pick arbitrary s (here 0)
			if (denom != 0.f) {
				s = clamp(d * f - c * e, 0.f, 1.f);
			}
			else {
				t = (d * s + f) / e;

				// If t in [0, 1] done. Else clamp t, recompute s for the new value of t
				if (t < 0.f) {
					t = 0.f;
					s = clamp(-c / b, 0.f, 1.f);
				}
				else if (t > 1.f) {
					t = 1.f;
					s = clamp((d - c) / b, 0.f, 1.f);
				}
			}
		}
	}

	c1 = p1 + d1 * s;
	c2 = l2.a + d2 * t;
	return dot(c2 - c1, c2 - c1);
}

/**
 * @brief Computes the closest point between two segments
 *
 * @param c1 the closest point on the first line segment
 * @param c2 the closest point on the second line segment
 * @param l1 the first line segment
 * @param p2 the start point of the second line segment
 * @param q2 the end point of the second line segment
 * @return the distance squared between c1 and c2
 */
float Line::getClosestPtSegmentSegment(vec3& c1, vec3& c2, const Line& l1, const vec3& p2, const vec3& q2) {
	const vec3 d1 = l1.b - l1.a;
	const vec3 d2 = q2 - p2;
	const vec3 r = l1.a - p2;
	const float b = dot(d1, d1);
	const float e = dot(d2, d2);
	const float f = dot(d2, r);
	float s = 0.f, t = 0.f;

	// Check if either or both segments degenerate into points
	if (b <= 1e-8f && e <= 1e-8f) {
		// Both segments degenerate into points
		c1 = vec3(l1.a);
		c2 = vec3(p2);
		return dot(c2 - c1, c2 - c1);
	}

	if (b <= 1e-8f) {
		// First segment degenerates into a point
		t = clamp(f / e, 0.f, 1.f);
	}
	else {
		const float c = dot(d1, r);
		if (e <= 1e-8f) {
			// second segment degenerates to a point
			s = clamp(-c / b, 0.f, 1.f);
		}
		else {
			// The general nondegenerate case starts here
			const float d = dot(d1, d2);
			const float denom = b * e - d * d;

			// If segments not parallel, compute closest point on L1 to L2 and
			// clamp to segment s1. Else pick arbitrary s (here 0)
			if (denom != 0.f) {
				s = clamp(d * f - c * e, 0.f, 1.f);
			}
			else {
				t = (d * s + f) / e;

				// If t in [0, 1] done. Else clamp t, recompute s for the new value of t
				if (t < 0.f) {
					t = 0.f;
					s = clamp(-c / b, 0.f, 1.f);
				}
				else if (t > 1.f) {
					t = 1.f;
					s = clamp((d - c) / b, 0.f, 1.f);
				}
			}
		}
	}

	c1 = l1.a + d1 * s;
	c2 = p2 + d2 * t;
	return dot(c2 - c1, c2 - c1);
}

/**
 * @brief Computes the closest point between two segments
 *
 * @param c1 the closest point on the first line segment
 * @param c2 the closest point on the second line segment
 * @param p1 the start point of the first line segment
 * @param q1 the end point of the first line segment
 * @param p2 the start point of the second line segment
 * @param q2 the end point of the second line segment
 * @return the distance squared between c1 and c2
 */
float Line::getClosestPtSegmentSegment(vec3& c1, vec3& c2, const vec3& p1, const vec3& q1, const vec3& p2, const vec3& q2) {
	const vec3 d1 = q1 - p1;
	const vec3 d2 = q2 - p2;
	const vec3 r = p1 - p2;
	const float b = dot(d1, d1);
	const float e = dot(d2, d2);
	const float f = dot(d2, r);
	float s = 0.f, t = 0.f;

	// Check if either or both segments degenerate into points
	if (b <= 1e-8f && e <= 1e-8f) {
		// Both segments degenerate into points
		c1 = p1;
		c2 = p2;
		return dot(c2 - c1, c2 - c1);
	}

	if (b <= 1e-8f) {
		// First segment degenerates into a point
		t = clamp(f / e, 0.f, 1.f);
	}
	else {
		const float c = dot(d1, r);
		if (e <= 1e-8f) {
			// second segment degenerates to a point
			s = clamp(-c / b, 0.f, 1.f);
		}
		else {
			// The general nondegenerate case starts here
			const float d = dot(d1, d2);
			const float denom = b * e - d * d;

			// If segments not parallel, compute closest point on L1 to L2 and
			// clamp to segment s1. Else pick arbitrary s (here 0)
			if (denom != 0.f) {
				s = clamp(d * f - c * e, 0.f, 1.f);
			}
			else {
				t = (d * s + f) / e;

				// If t in [0, 1] done. Else clamp t, recompute s for the new value of t
				if (t < 0.f) {
					t = 0.f;
					s = clamp(-c / b, 0.f, 1.f);
				}
				else if (t > 1.f) {
					t = 1.f;
					s = clamp((d - c) / b, 0.f, 1.f);
				}
			}
		}
	}

	c1 = p1 + d1 * s;
	c2 = p2 + d2 * t;
	return dot(c2 - c1, c2 - c1);
}