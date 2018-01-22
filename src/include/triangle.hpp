#ifndef TRIANGLE_HPP
#define TRIANGLE_HPP

/**
 *	@file		triangle.hpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/17/2018
 *	@version	1.0
 *
 *	@brief Triangle class declaration
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for the Triangle class.
 *
 *	Common use cases for the Triangle class is for utility
 *	purposes and should not be used as a replacement data
 *	structure for managing vertices.
 */

#include <vector>

#include <glm/glm.hpp>

#include <line.hpp>
#include <plane.hpp>

using namespace glm;
using namespace std;

namespace collision {
	/**
	 *	@brief Utility class for triangle operations
	 */
	class Triangle {
	private:
		vec3 v1; // shared pointer to the first vertex
		vec3 v2; // shared pointer to the second vertex
		vec3 v3; // shared pointer to the third vertex

	public:
		// Default constructor
		Triangle(const vec3& v1, const vec3& v2, const vec3& v3);

		// Copy constructor
		Triangle(const Triangle& t);

		// Move constructor
		Triangle(Triangle&& t);

		// getter for the first vertex
		const vec3& getVertex1(void) const;

		// setter for the first vertex
		void setVertex1(const vec3& v1);

		// getter for the second vertex
		const vec3& getVertex2(void) const;

		// setter for the second vertex
		void setVertex2(const vec3& v2);

		// getter for the third vertex
		const vec3& getVertex3(void) const;

		// setter for the third vertex
		void setVertex3(const vec3& v3);

		// calculates the triangle normal
		vec3 getNormal(void) const;
		static vec3 getNormal(const vec3& v1, const vec3& v2, const vec3& v3);

		// converts the 3d point to a barycentric coordinate (u, v, w)
		vec3 getBarycentricCoord(const vec3& p) const;
		static vec3 getBarycentricCoord(const vec3& p, const vec3& v1, const vec3& v2, const vec3& v3);

		// checks if the point is contained in the triangle
		bool isWithinTriangle(const vec3& p) const;
		static bool isWithinTriangle(const vec3& p, const vec3& v1, const vec3& v2, const vec3& v3);
	
		// computes the closest point on a triangle due to an input point
		vec3 getClosestPtPointTriangle(const vec3& p) const;
		static vec3 getClosestPtPointTriangle(const vec3& p, const vec3& v1, const vec3& v2, const vec3& v3);
	
		// checks if line segment intersects triangle
		bool isSegmentIntersecting(vec3& c, const Line& l) const;
		bool isSegmentIntersecting(vec3& c, const vec3& p, const vec3& q) const;
		static bool isSegmentIntersecting(vec3& c, const Line& l, const vec3& v1, const vec3& v2, const vec3& v3);
		static bool isSegmentIntersecting(vec3& c, const vec3& p, const vec3& q, const vec3& v1, const vec3& v2, const vec3& v3);


		// computes the closest points on a triangle and a line segment
		float getClosestPtSegmentTriangle(vec3& c1, vec3& c2, const Line& l) const;
		float getClosestPtSegmentTriangle(vec3& c1, vec3& c2, const vec3& p, const vec3& q) const;
		static float getClosestPtSegmentTriangle(vec3& c1, vec3& c2, const Line& l, const vec3& v1, const vec3& v2, const vec3& v3);
		static float getClosestPtSegmentTriangle(vec3& c1, vec3& c2, const vec3& p, const vec3& q, const vec3& v1, const vec3& v2, const vec3& v3);
	};
}

#endif // !TRIANGLE_HPP
