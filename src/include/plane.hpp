#ifndef PLANE_HPP
#define PLANE_HPP

/**
 *	@file		plane.hpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/17/2018
 *	@version	1.0
 *
 *	@brief Plane class declaration
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for the Plane class.
 *
 *	Common use cases for the Plane class is for utility
 *	purposes and should not be used as a replacement data
 *	structure for managing faces.
 */

#include <glm/glm.hpp>

using namespace glm;
using namespace std;

namespace collision {
	/**
	 * @brief Utility class for plane operations
	 */
	class Plane {
	private:
		vec3 n;
		float d;

	public:
		// Constructor Option 1
		Plane(const vec3& p1, const vec3& p2, const vec3& p3);

		// Constructor Option 2
		Plane(const vec3& n, const float& d);

		// Copy constructor
		Plane(const Plane& P);

		// Move constructor
		Plane(Plane&& P);

		// Computes the closest point on the plane to an input point
		vec3 closestPtPointPlane(const vec3& p) const;
		static vec3 closestPtPointPlane(const vec3& p, const vec3& n, const float& d);

		// Checks if the point is outside of plane
		bool isPointOutsideOfPlane(const vec3& p) const;
		static bool isPointOutsideOfPlane(const vec3&p, const vec3& n, const float& d);

		// Checks if point p1 and p2 lie on opposite sides of the plane
		bool arePointsOnOppositeSides(const vec3& p1, const vec3& p2) const;
		static bool arePointsOnOppositeSides(const vec3& p1, const vec3& p2, const vec3& n, const float& d);
	};
}

#endif // !PLANE_HPP
