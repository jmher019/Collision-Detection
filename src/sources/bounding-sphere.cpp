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
	velocity = s.velocity;
}

/**
 * @brief The move constructor
 */
BoundingSphere::BoundingSphere(BoundingSphere&& s) :
	radius(s.radius) {
	transform = move(s.transform);
	velocity = move(s.velocity);
}

/**
 * @brief The assignment operator
 *
 * @return the calling sphere
 */
BoundingSphere& BoundingSphere::operator=(const BoundingSphere& s) {
	transform = s.transform;
	velocity = s.velocity;
	radius = s.radius;

	return *this;
}

/**
 * @brief The move operator
 *
 * @return the calling sphere
 */
BoundingSphere& BoundingSphere::operator=(BoundingSphere&& s) {
	transform = move(s.transform);
	velocity = move(s.velocity);
	radius = s.radius;

	return *this;
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

/**
 * @brief handles checking if a bounding volume is enclosed by this sphere
 *
 * @param bv the bounding volume that will be checked if it is enclosed
 * @return bool that determines if the input volume is enclosed by the sphere
 */
bool BoundingSphere::enclosesGeometry(BoundingVolume*& bv) const {
	// handle bounding sphere here
	if (const BoundingSphere* bSphere = dynamic_cast<BoundingSphere*>(bv)) {
		const vec3 diff = vec3(getCenter()) - vec3(bSphere->getCenter());
		const float fullDist = length(diff) + bSphere->getRadius();

		return fullDist <= radius;
	}

	// handle all others here
	BoundingVolume* self = (BoundingVolume*)this;
	return bv->isEnclosed(self);
}

/**
 * @brief handles checking if the sphere is enclosed by the bounding volume
 *
 * @param bv the bounding volume that will be checked to see if it encloses the sphere
 * @return bool that determines if the input volume encloses the sphere
 */
bool BoundingSphere::isEnclosed(BoundingVolume*& bv) const {
	// handle bounding sphere here
	if (const BoundingSphere* bSphere = dynamic_cast<BoundingSphere*>(bv)) {
		const vec3 diff = vec3(getCenter()) - vec3(bSphere->getCenter());
		const float fullDist = length(diff) + radius;
		const float& bRadius = bSphere->getRadius();

		return fullDist <= bRadius;
	}

	// handle all others here
	BoundingVolume* self = (BoundingVolume*)this;
	return bv->enclosesGeometry(self);
}