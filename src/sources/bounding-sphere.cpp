#include <bounding-sphere.hpp>

/**
 *	@file		bounding-sphere.cpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/21/2018
 *	@version	1.0
 *
 *	@brief BoundingSphere derived class definition
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for bounding spheres.
 *
 *	The intended use case of this class is for simple rigid body collision
 */

using namespace collision;

/**
 * @brief The default constructor
 */
BoundingSphere::BoundingSphere(const float& radius) :
	radius(radius) {}

/**
 * @brief The copy constructor
 */
BoundingSphere::BoundingSphere(const BoundingSphere& s) :
	radius(s.radius) {
	transform = s.transform;
}

/**
 * @brief The move constructor
 */
BoundingSphere::BoundingSphere(BoundingSphere&& s) :
	radius(s.radius) {
	transform = move(s.transform);
}

/**
 * @brief gets the current radius of the sphere
 *
 * @return the radius of the sphere
 */
const float& BoundingSphere::getRadius(void) const {
	return radius;
}

/**
 * @brief handles updating transform matrix for this geometry
 *
 * @param transform will be used to update the geometry
 * @return void
 */
void BoundingSphere::update(const mat4& transform) {
	this->transform = transform * this->transform;

	// update radius
	vec4 temp = transform * vec4(radius, 0.f, 0.f, 0.f);
	radius = length(temp);
}

/**
 * @brief handles checking for intersection
 *
 * @param bv the bounding volume that will be checked against for intersection
 * @return bool that determines if the volumes are intersecting
 */
bool BoundingSphere::isIntersecting(BoundingVolume*& bv) const {
	// handle bounding sphere here
	if (const BoundingSphere* bSphere = dynamic_cast<BoundingSphere*>(bv)) {
		const vec4 offset = bSphere->getCenter() - getCenter();
		const float dist2 = dot(offset, offset);
		const float radiusSum = radius + bSphere->getRadius();

		return dist2 <= radiusSum * radiusSum;
	}

	// handle all others here
	BoundingVolume* self = (BoundingVolume*)this;
	return bv->isIntersecting(self);
}