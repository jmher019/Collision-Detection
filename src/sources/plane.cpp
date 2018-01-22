/**
 *	@file		plane.cpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/17/2018
 *	@version	1.0
 *
 *	@brief Plane class definition
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for the Plane class.
 *
 *	Common use cases for the Plane class is for utility
 *	purposes and should not be used as a replacement data
 *	structure for managing faces.
 */

#include <plane.hpp>

using namespace collision;

/**
 * @brief in this constructor n = cross(p2 - p1, p3 - p1) and d = dot(n, p1)
 */
Plane::Plane(const vec3& p1, const vec3& p2, const vec3& p3) {

	n = cross(p2 - p1, p3 - p1);
	d = dot(n, p1);
}

/**
 * @brief directly supply normal and d inputs for plane
 */
Plane::Plane(const vec3& n, const float& d):
	n(n), d(d) {
}

/**
 * @brief Copy constructor
 */
Plane::Plane(const Plane& P):
	n(P.n),
	d(P.d) {}

/**
 * @brief Move constructor
 */
Plane::Plane(Plane&& P) {
	n = move(P.n);
	d = move(d);
}

/**
 * @brief Computes the closest point on the plane to an input point
 *
 * @param p the input point
 * @return the point on the plane closest to the input point
 */
vec3 Plane::closestPtPointPlane(const vec3& p) const {
	const float t = (dot(n, p) - d) / dot(n, n);
	return p - t * n;
}


/**
 * @brief Computes the closest point on the plane to an input point
 *
 * @param p the input point
 * @param n the plane normal
 * @param d this value is equivalent to the dot of a point on the plane and the normal
 * @return the point on the plane closest to the input point
 */
vec3 Plane::closestPtPointPlane(const vec3& p, const vec3& n, const float& d) {
	const float t = (dot(n, p) - d) / dot(n, n);
	return p - t * n;
}

/**
 * @brief Checks if a point lies outside a plane
 *
 * @param p the input point
 * @return a bool that determines if the point is outside the plane or not
 */
bool Plane::isPointOutsideOfPlane(const vec3& p) const {
	return fabs(dot(p, n) - d) < 1e-8f;
}

/**
 * @brief Checks if a point lies outside the plane or not
 *
 * @param p the input point
 * @param n the normal to the plane
 * @param d this value is equivalent to the dot of a point on the plane and the normal
 * @return a bool that determines if the point is outside the plane or not
 */
bool Plane::isPointOutsideOfPlane(const vec3& p, const vec3& n, const float& d) {
	return fabs(dot(p, n) - d) < 1e-8f;
}

/**
 * @brief Checks if the points p1 and p2 lie on opposite sides of the plane
 *
 * @param p1 the point to be checked
 * @param p2 the other point to be checked
 * @return a bool stating whether the points lie on opposited sides of the plane or not
 */
bool Plane::arePointsOnOppositeSides(const vec3& p1, const vec3& p2) const {
	const float signp1 = dot(p1, n);
	const float signp2 = dot(p2, n);

	return signp1 * signp2 < 0.f;
}

/**
 * @brief Checks if the points p1 and p2 lie on opposite sides of the plane
 *
 * @param p1 the point to be checked
 * @param p2 the other point to be checked
 * @param n the normal to the plane
 * @param d this value is equivalent to the dot of a point on the plane and the normal
 * @return a bool stating whether the points lie on opposited sides of the plane or not
 */
bool Plane::arePointsOnOppositeSides(const vec3& p1, const vec3& p2, const vec3& n, const float& d) {
	const float signp1 = dot(p1, n);
	const float signp2 = dot(p2, n);

	return signp1 * signp2 < 0.f;
}