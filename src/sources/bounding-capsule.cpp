#include <bounding-capsule.hpp>

/**
 *	@file		bounding-capsule.cpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/22/2018
 *	@version	1.0
 *
 *	@brief BoundingCapsule derived class definition
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for bounding capsules.
 *
 *	The intended use case of this class is for simple rigid body collision
 */

using namespace collision;

/**
 * @brief Default constructor
 */
BoundingCapsule::BoundingCapsule(const Line& l, const vec3& lineNormal, const float& radius):
	l(l), lineNormal(normalize(lineNormal)), radius(radius) {}

/**
 * @brief Copy constructor
 */
BoundingCapsule::BoundingCapsule(const BoundingCapsule& bc):
	l(bc.l), lineNormal(bc.lineNormal), radius(bc.radius) {}

/**
 * @brief Move constructor
 */
BoundingCapsule::BoundingCapsule(BoundingCapsule&& bc):
	radius(bc.radius), l(bc.l) {
	lineNormal = move(bc.lineNormal);
}

/**
 * @brief gets the line that represents the cylindrical portion
 *
 * @return the line representing the cylindircal portion
 */
const Line& BoundingCapsule::getLine(void) const {
	return l;
}

/**
 * @breif gets the normal to the line
 *
 * @return the normal to the line
 */
const vec3& BoundingCapsule::getLineNormal(void) const {
	return lineNormal;
}

/**
 * @brief gets the radius to the spheres at the ends
 *
 * @return the radius of the spheres at the ends
 */
const float& BoundingCapsule::getRadius(void) const {
	return radius;
}

/**
 * @brief handles updating transform matrix for this geometry
 *
 * @param transform will be used to update the geometry
 * @return void
 */
void BoundingCapsule::update(const mat4& transform) {
	const vec3 center = vec3(getCenter()) + vec3(transform[3]);
	
	const mat3 upperLeft = mat3(transform);
	const vec3 lineVector = 0.5f * (l.getPointB() - l.getPointA());

	const vec3 nextLineVector = upperLeft * lineVector;
	const vec3 nextNormal = upperLeft * (radius * lineNormal);

	// update everything
	this->transform = transform * this->transform;
	radius = length(nextNormal);
	lineNormal = normalize(nextNormal);
	l.setPointA(center - nextLineVector);
	l.setPointB(center + nextLineVector);
}

/**
 * @brief handles checking for intersection
 *
 * @param bv the bounding volume that will be checked against for intersection
 * @return bool that determines if the volumes are intersecting
 */
bool BoundingCapsule::isIntersecting(BoundingVolume*& bv) const {
	// handle bounding sphere here
	if (const BoundingSphere* bSphere = dynamic_cast<BoundingSphere*>(bv)) {
		const vec3 bCenter = vec3(bSphere->getCenter());
		const vec3 diff = l.getClosestPtPointSegment(bCenter) - bCenter;
		const float radiusSum = radius + bSphere->getRadius();
		return dot(diff, diff) <= radiusSum * radiusSum;
	}
	// handle axis aligned bounding box here
	else if (const AxisAlignedBoundingBox* bAABB = dynamic_cast<AxisAlignedBoundingBox*>(bv)) {
		vector<Triangle> triangles = move(bAABB->getTriangles());
		vec3 c1, c2;
		float dist2 = triangles[0].getClosestPtSegmentTriangle(c1, c2, l);

		for (size_t i = 1; i < triangles.size(); i++) {
			float currDist2 = triangles[i].getClosestPtSegmentTriangle(c1, c2, l);
			if (currDist2 < dist2) {
				dist2 = currDist2;
			}
		}

		return dist2 <= radius * radius;
	}
	// handle bounding capsule here
	else if (const BoundingCapsule* bCapsule = dynamic_cast<BoundingCapsule*>(bv)) {
		vec3 c1, c2;
		const float dist2 = l.getClosestPtSegmentSegment(c1, c2, bCapsule->getLine());
		const float radiusSum = radius + bCapsule->getRadius();
		return dist2 <= radiusSum * radiusSum;
	}

	// handle all others here
	BoundingVolume* self = (BoundingVolume*)this;
	return bv->isIntersecting(self);
}