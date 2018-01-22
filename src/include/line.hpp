#ifndef LINE_HPP
#define LINE_HPP

/**
 *	@file		line.hpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/17/2018
 *	@version	1.0
 *
 *	@brief Line class declaration
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for the Line class.
 *
 *	Common use cases for the Line class is for utility
 *	purposes and should not be used as a replacement data
 *	structure for managing vertices.
 */

#include <glm/glm.hpp>

using namespace glm;
using namespace std;

namespace collision {
	/**
	 * @brief Utility class for line operations
	 */
	class Line {
	private:
		vec3 a;
		vec3 b;

	public:
		// Default constructor
		Line(const vec3& a, const vec3& b);

		// Copy constructor
		Line(const Line& l);

		// Move constructor
		Line(Line&& l);

		// get the start point on the line
		const vec3& getPointA(void) const;

		// set the start point on the line
		void setPointA(const vec3& a);

		// get the end point on the line
		const vec3& getPointB(void) const;

		// set the end point on the line
		void setPointB(const vec3& b);

		// treat the line as a segment and find the closest point on it to another point
		vec3 getClosestPtPointSegment(const vec3& p) const;
		static vec3 getClosestPtPointSegment(const vec3& p, const vec3& a, const vec3& b);

		// computes the squared distance between a point and a line segment
		float getSquaredDistPointSegment(const vec3& p) const;
		static float getSquaredDistPointSegment(const vec3& p, const vec3& a, const vec3& b);

		// computes the closest point between two segments
		float getClosestPtSegmentSegment(vec3& c1, vec3& c2, const Line& l2) const;
		float getClosestPtSegmentSegment(vec3& c1, vec3& c2, const vec3& p2, const vec3& q2) const;
		static float getClosestPtSegmentSegment(vec3& c1, vec3& c2, const Line& l1, const Line& l2);
		static float getClosestPtSegmentSegment(vec3& c1, vec3& c2, const vec3& p1, const vec3& q1, const Line& l2);
		static float getClosestPtSegmentSegment(vec3& c1, vec3& c2, const Line& l1, const vec3& p2, const vec3& q2);
		static float getClosestPtSegmentSegment(vec3& c1, vec3& c2, const vec3& p1, const vec3& q1, const vec3& p2, const vec3& q2);
	};
}

#endif // !LINE_HPP
