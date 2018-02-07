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
	l(l), lineNormal(normalize(lineNormal)), radius(radius) {
	secondLineNormal = normalize(cross(l.getPointB() - l.getPointA(), this->lineNormal));
}

/**
 * @brief Copy constructor
 */
BoundingCapsule::BoundingCapsule(const BoundingCapsule& bc):
	l(bc.l), lineNormal(bc.lineNormal), secondLineNormal(bc.secondLineNormal), radius(bc.radius) {
	transform = bc.transform;
	velocity = bc.velocity;
}

/**
 * @brief Move constructor
 */
BoundingCapsule::BoundingCapsule(BoundingCapsule&& bc):
	radius(bc.radius), l(bc.l) {
	lineNormal = move(bc.lineNormal);
	secondLineNormal = move(bc.secondLineNormal);
	transform = move(bc.transform);
	velocity = move(bc.velocity);
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
 * @brief gets the normal to the line
 *
 * @return the normal to the line
 */
const vec3& BoundingCapsule::getLineNormal(void) const {
	return lineNormal;
}

/**
 * @brief gets the normal to the line and line normal
 *
 * return the normal to the line and line normal
 */
const vec3& BoundingCapsule::getSecondLineNormal(void) const {
	return secondLineNormal;
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
	this->transform = transform * this->transform;
	const vec3 center = vec3(getCenter());
	
	const mat3 upperLeft = mat3(transform);
	const vec3 lineVector = 0.5f * (l.getPointB() - l.getPointA());

	const vec3 nextLineVector = upperLeft * lineVector;
	const vec3 nextNormal = upperLeft * (radius * lineNormal);
	const vec3 nextSecondNormal = upperLeft * (radius * secondLineNormal);

	// update everything
	const float radius1 = length(nextNormal);
	const float radius2 = length(nextSecondNormal);

	if (radius1 > radius2) {
		radius = radius1;
	}
	else {
		radius = radius2;
	}

	lineNormal = normalize(nextNormal);
	secondLineNormal = normalize(nextSecondNormal);
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

/**
 * @brief handles checking if a bounding volume is enclosed by this capsule
 *
 * @param bv the bounding volume that will be checked if it is enclosed
 * @return bool that determines if the input volume is enclosed by the capsule
 */
bool BoundingCapsule::enclosesGeometry(BoundingVolume*& bv) const {
	// handle bounding sphere here
	if (const BoundingSphere* bSphere = dynamic_cast<BoundingSphere*>(bv)) {
		const vec3 bCenter = vec3(bSphere->getCenter());
		const vec3 diff = l.getClosestPtPointSegment(bCenter) - bCenter;
		const float fullDist = length(diff) + bSphere->getRadius();

		return fullDist <= radius;
	}
	// handle axis aligned bounding box
	else if (const AxisAlignedBoundingBox* bAABB = dynamic_cast<AxisAlignedBoundingBox*>(bv)) {
		const vec3 bCenter = vec3(bAABB->getCenter());
		const vec3 halfExtents = bAABB->getHalfExtents();

		// make sure that each point in the box is contiained in the capsule
		vector<vec3> pts;
		pts.push_back(bCenter - halfExtents);
		pts.push_back(bCenter + vec3(-halfExtents.x, -halfExtents.y, halfExtents.z));
		pts.push_back(bCenter + vec3(halfExtents.x, -halfExtents.y, -halfExtents.z));
		pts.push_back(bCenter + vec3(halfExtents.x, -halfExtents.y, halfExtents.z));
		pts.push_back(bCenter + halfExtents);
		pts.push_back(bCenter + vec3(halfExtents.x, halfExtents.y, -halfExtents.z));
		pts.push_back(bCenter + vec3(-halfExtents.x, halfExtents.y, halfExtents.z));
		pts.push_back(bCenter + vec3(-halfExtents.x, halfExtents.y, -halfExtents.z));

		for (size_t i = 0; i < pts.size(); i++) {
			const vec3 diff = l.getClosestPtPointSegment(pts[i]) - pts[i];
			if (dot(diff, diff) > radius * radius) {
				return false;
			}
		}

		return true;
	}
	// handle bounding capsule
	else if (const BoundingCapsule* bCapsule = dynamic_cast<BoundingCapsule*>(bv)) {
		const Line& bLine = bCapsule->getLine();
		const vec3& bPoint1 = bLine.getPointA();
		const vec3& bPoint2 = bLine.getPointB();
		const vec3 diff1 = l.getClosestPtPointSegment(bPoint1) - bPoint1;
		const vec3 diff2 = l.getClosestPtPointSegment(bPoint2) - bPoint2;
		const float fullDist1 = length(diff1) + bCapsule->getRadius();
		const float fullDist2 = length(diff2) + bCapsule->getRadius();

		return fullDist1 <= radius && fullDist2 <= radius;
	}

	// handle all others here
	BoundingVolume* self = (BoundingVolume*)this;
	return bv->isEnclosed(self);
}

/**
 * @brief handles checking if the sphere is enclosed by the bounding volume
 *
 * @param bv the bounding volume that will be checked to see if it encloses the capsule
 * @return bool that determines if the input volume encloses the capsule
 */
bool BoundingCapsule::isEnclosed(BoundingVolume*& bv) const {
	// handle bounding sphere here
	if (const BoundingSphere* bSphere = dynamic_cast<BoundingSphere*>(bv)) {
		const vec3& point1 = l.getPointA();
		const vec3& point2 = l.getPointB();
		const vec3 bCenter = vec3(bSphere->getCenter());
		const vec3 diff1 = l.getClosestPtPointSegment(point1) - bCenter;
		const vec3 diff2 = l.getClosestPtPointSegment(point2) - bCenter;
		const float fullDist1 = length(diff1) + radius;
		const float fullDist2 = length(diff2) + radius;

		return fullDist1 <= bSphere->getRadius() && fullDist2 <= bSphere->getRadius();
	}
	// handle axis aligned bounding box
	else if (const AxisAlignedBoundingBox* bAABB = dynamic_cast<AxisAlignedBoundingBox*>(bv)) {
		const vec3 bCenter = vec3(bAABB->getCenter());
		const vec3 bMin = bCenter - bAABB->getHalfExtents();
		const vec3 bMax = bCenter + bAABB->getHalfExtents();

		const vec3& center1 = l.getPointA();
		const vec3 min1 = center1 - vec3(radius, radius, radius);
		const vec3 max1 = center1 + vec3(radius, radius, radius);

		const vec3& center2 = l.getPointB();
		const vec3 min2 = center2 - vec3(radius, radius, radius);
		const vec3 max2 = center2 + vec3(radius, radius, radius);

		for (int i = 0; i < 3; i++) {
			if (bMin[i] > min1[i]) return false;
			else if (bMax[i] < max1[i]) return false;
		}

		for (int i = 0; i < 3; i++) {
			if (bMin[i] > min2[i]) return false;
			else if (bMax[i] < max2[i]) return false;
		}
		
		return true;
	}
	// handle bounding capsule
	else if (const BoundingCapsule* bCapsule = dynamic_cast<BoundingCapsule*>(bv)) {
		const Line& bLine = bCapsule->getLine();
		const vec3& point1 = l.getPointA();
		const vec3& point2 = l.getPointB();
		const vec3 diff1 = bLine.getClosestPtPointSegment(point1) - point1;
		const vec3 diff2 = bLine.getClosestPtPointSegment(point2) - point2;
		const float fullDist1 = length(diff1) + radius;
		const float fullDist2 = length(diff2) + radius;

		return fullDist1 <= bCapsule->getRadius() && fullDist2 <= bCapsule->getRadius();
	}

	// handle all others here
	BoundingVolume* self = (BoundingVolume*)this;
	return bv->enclosesGeometry(self);
}