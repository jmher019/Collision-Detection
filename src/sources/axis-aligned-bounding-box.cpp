/**
 *	@file		axis-aligned-bounding-box.cpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/21/2018
 *	@version	1.0
 *
 *	@brief AxisAlignedBoundingBox derived class definition
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for axis aligned bounding boxes.
 *
 *	The intended use case of this class is for simple rigid body collision
 */

#include <axis-aligned-bounding-box.hpp>

using namespace collision;

/**
 * @brief Default constructor
 */
AxisAlignedBoundingBox::AxisAlignedBoundingBox(const vec3& initialHalfExtents):
	initialHalfExtents(initialHalfExtents), halfExtents(initialHalfExtents) {}

/**
 * @brief Copy constructor
 */
AxisAlignedBoundingBox::AxisAlignedBoundingBox(const AxisAlignedBoundingBox& aabb): 
	initialHalfExtents(aabb.initialHalfExtents), halfExtents(aabb.halfExtents) {
	transform = aabb.transform;
	velocity = aabb.velocity;
}

/**
 * @bried Move constructor
 */
AxisAlignedBoundingBox::AxisAlignedBoundingBox(AxisAlignedBoundingBox&& aabb) {
	initialHalfExtents = move(aabb.initialHalfExtents);
	halfExtents = move(aabb.halfExtents);
	transform = move(aabb.transform);
	velocity = move(aabb.velocity);
}

/**
 * @brief gets the current half extents
 *
 * @return the current half extents
 */
const vec3& AxisAlignedBoundingBox::getHalfExtents(void) const {
	return halfExtents;
}

/**
 * @brief overwrites the initial and current half extents
 *
 * @param halfExtents a vector that contains half the length in each dimension
 * @return void
 */
void AxisAlignedBoundingBox::setHalfExtents(const vec3& halfExtents) {
	initialHalfExtents = halfExtents;
	this->halfExtents = halfExtents;
}

/**
 * @brief computes the squared distance between a point and the box
 *
 * @param p the input point
 * @return the squared distance between the box and the point
 */
float AxisAlignedBoundingBox::getSquaredDistancePtPointAABB(const vec3& p) const {
	float sqDist = 0.f;

	const vec3 min = vec3(getCenter()) - halfExtents;
	const vec3 max = vec3(getCenter()) + halfExtents;

	for (int i = 0; i < 3; i++) {
		// For each axis count any excess distance outside box extents
		const float& v = p[i];
		if (v < min[i]) sqDist += (min[i] - v) * (min[i] - v);
		if (v > max[i]) sqDist += (v - max[i]) * (v - max[i]);
	}
	return sqDist;
}

/**
 * @brief gets the triangles that make up the box
 *
 * @return a vector of triangles that make up the box
 */
vector<Triangle> AxisAlignedBoundingBox::getTriangles(void) const {
	vector<Triangle> triangles;

	const vec3 center = vec3(getCenter());

	// +x face
	// center + <he.x, +/-he.y, +/-he.z>
	triangles.push_back(Triangle(
		center + vec3(halfExtents.x, -halfExtents.y, halfExtents.z),
		center + vec3(halfExtents.x, -halfExtents.y, -halfExtents.z),
		center + vec3(halfExtents.x, halfExtents.y, halfExtents.z)
	));

	triangles.push_back(Triangle(
		center + vec3(halfExtents.x, -halfExtents.y, -halfExtents.z),
		center + vec3(halfExtents.x, halfExtents.y, -halfExtents.z),
		center + vec3(halfExtents.x, halfExtents.y, halfExtents.z)
	));

	// -x face
	// center + <-he.x, +/-he.y, +/-he.z>
	triangles.push_back(Triangle(
		center + vec3(-halfExtents.x, -halfExtents.y, halfExtents.z),
		center + vec3(-halfExtents.x, halfExtents.y, halfExtents.z),
		center + vec3(-halfExtents.x, -halfExtents.y, -halfExtents.z)
	));

	triangles.push_back(Triangle(
		center + vec3(-halfExtents.x, -halfExtents.y, -halfExtents.z),
		center + vec3(-halfExtents.x, halfExtents.y, halfExtents.z),
		center + vec3(-halfExtents.x, halfExtents.y, -halfExtents.z)
	));

	// +y face
	// center + <+/-he.x, he.y, +/-he.z>
	triangles.push_back(Triangle(
		center + vec3(-halfExtents.x, halfExtents.y, halfExtents.z),
		center + vec3(halfExtents.x, halfExtents.y, halfExtents.z),
		center + vec3(-halfExtents.x, halfExtents.y, -halfExtents.z)
	));

	triangles.push_back(Triangle(
		center + vec3(halfExtents.x, halfExtents.y, halfExtents.z),
		center + vec3(halfExtents.x, halfExtents.y, -halfExtents.z),
		center + vec3(-halfExtents.x, halfExtents.y, -halfExtents.z)
	));

	// -y face
	// center + <+/-he.x, -he.y, +/-he.z>
	triangles.push_back(Triangle(
		center + vec3(-halfExtents.x, -halfExtents.y, halfExtents.z),
		center + vec3(-halfExtents.x, -halfExtents.y, -halfExtents.z),
		center + vec3(halfExtents.x, -halfExtents.y, halfExtents.z)
	));

	triangles.push_back(Triangle(
		center + vec3(halfExtents.x, -halfExtents.y, halfExtents.z),
		center + vec3(-halfExtents.x, -halfExtents.y, -halfExtents.z),
		center + vec3(halfExtents.x, -halfExtents.y, -halfExtents.z)
	));

	// +z face
	// center + <+/-he.x, +/-he.y, he.z>
	triangles.push_back(Triangle(
		center + vec3(-halfExtents.x, -halfExtents.y, halfExtents.z),
		center + vec3(halfExtents.x, -halfExtents.y, halfExtents.z),
		center + vec3(-halfExtents.x, halfExtents.y, halfExtents.z)
	));

	triangles.push_back(Triangle(
		center + vec3(halfExtents.x, -halfExtents.y, halfExtents.z),
		center + vec3(halfExtents.x, halfExtents.y, halfExtents.z),
		center + vec3(-halfExtents.x, halfExtents.y, halfExtents.z)
	));

	// -z face
	// center + <+/-he.x, +/-he.y, -he.z>
	triangles.push_back(Triangle(
		center + vec3(-halfExtents.x, -halfExtents.y, -halfExtents.z),
		center + vec3(-halfExtents.x, halfExtents.y, -halfExtents.z),
		center + vec3(halfExtents.x, -halfExtents.y, -halfExtents.z)
	));

	triangles.push_back(Triangle(
		center + vec3(halfExtents.x, -halfExtents.y, -halfExtents.z),
		center + vec3(-halfExtents.x, halfExtents.y, -halfExtents.z),
		center + vec3(halfExtents.x, halfExtents.y, -halfExtents.z)
	));

	return triangles;
}

/**
 * @brief handles updating transform matrix for this geometry
 *
 * @param transform will be used to update the geometry
 * @return void
 */
void AxisAlignedBoundingBox::update(const mat4& transform) {
	const mat3& upperLeft = mat3(transform);
	for (int i = 0; i < 3; i++) {
		halfExtents[i] = 0.f;
		for (int j = 0; j < 3; j++) {
			halfExtents[i] += fabs(upperLeft[j][i] * initialHalfExtents[j]);
		}
	}

	this->transform = transform * this->transform;
}

/**
 * @brief handles checking for intersection
 *
 * @param bv the bounding volume that will be checked against for intersection
 * @return bool that determines if the volumes are intersecting
 */
bool AxisAlignedBoundingBox::isIntersecting(BoundingVolume*& bv) const {
	// handle bounding sphere here
	if (const BoundingSphere* bSphere = dynamic_cast<BoundingSphere*>(bv)) {
		const vec3 sphereCenter = vec3(bSphere->getCenter());
		const float& radius = bSphere->getRadius();
		const float sqDist = getSquaredDistancePtPointAABB(sphereCenter);
		return sqDist <= radius * radius;
	}
	// handle axis aligned bounding box here
	else if (const AxisAlignedBoundingBox* bAABB = dynamic_cast<AxisAlignedBoundingBox*>(bv)) {
		const vec4& center = getCenter();
		const vec4& bCenter = bAABB->getCenter();
		const vec3& bHalfExents = bAABB->getHalfExtents();

		for (int i = 0; i < 3; i++) {
			if (fabs(center[i] - bCenter[i]) > (halfExtents[i] + bHalfExents[i])) {
				return false;
			}
		}

		return true;
	}

	// handle all others here
	BoundingVolume* self = (BoundingVolume*)this;
	return bv->isIntersecting(self);
}

/**
 * @brief handles checking if a bounding volume is enclosed by this aabb
 *
 * @param bv the bounding volume that will be checked if it is enclosed
 * @return bool that determines if the input volume is enclosed by the aabb
 */
bool AxisAlignedBoundingBox::enclosesGeometry(BoundingVolume*& bv) const {
	// handle bounding sphere here
	if (const BoundingSphere* bSphere = dynamic_cast<BoundingSphere*>(bv)) {
		const float& bRadius = bSphere->getRadius();
		const vec3 bCenter = bSphere->getCenter();
		const vec3 bMin = bCenter - vec3(bRadius, bRadius, bRadius);
		const vec3 bMax = bCenter + vec3(bRadius, bRadius, bRadius);

		const vec3 center = vec3(getCenter());
		const vec3 min = center - halfExtents;
		const vec3 max = center + halfExtents;

		for (int i = 0; i < 3; i++) {
			if (bMin[i] < min[i]) return false;
			else if (bMax[i] > max[i]) return false;
		}
		return true;
	}
	// handle axis aligned bounding box here
	else if (const AxisAlignedBoundingBox* bAABB = dynamic_cast<AxisAlignedBoundingBox*>(bv)) {
		const vec3 bCenter = bAABB->getCenter();
		const vec3 bMin = bCenter - bAABB->getHalfExtents();
		const vec3 bMax = bCenter + bAABB->getHalfExtents();

		const vec3 center = vec3(getCenter());
		const vec3 min = center - halfExtents;
		const vec3 max = center + halfExtents;

		for (int i = 0; i < 3; i++) {
			if (bMin[i] < min[i]) return false;
			else if (bMax[i] > max[i]) return false;
		}
		return true;
	}

	// handle all others here
	BoundingVolume* self = (BoundingVolume*)this;
	return bv->isEnclosed(self);
}

/**
 * @brief handles checking if the aabb is enclosed by the bounding volume
 *
 * @param bv the bounding volume that will be checked to see if it encloses the aabb
 * @return bool that determines if the input volume encloses the aabb
 */
bool AxisAlignedBoundingBox::isEnclosed(BoundingVolume*& bv) const {
	// handle bounding sphere here
	if (const BoundingSphere* bSphere = dynamic_cast<BoundingSphere*>(bv)) {
		const float& bRadius = bSphere->getRadius();
		const vec3 bCenter = bSphere->getCenter();
		const vec3 bMin = bCenter - vec3(bRadius, bRadius, bRadius);
		const vec3 bMax = bCenter + vec3(bRadius, bRadius, bRadius);

		const vec3 center = vec3(getCenter());
		const vec3 min = center - halfExtents;
		const vec3 max = center + halfExtents;

		for (int i = 0; i < 3; i++) {
			if (bMin[i] > min[i]) return false;
			else if (bMax[i] < max[i]) return false;
		}
		return true;
	}
	// handle axis aligned bounding box here
	else if (const AxisAlignedBoundingBox* bAABB = dynamic_cast<AxisAlignedBoundingBox*>(bv)) {
		const vec3 bCenter = bAABB->getCenter();
		const vec3 bMin = bCenter - bAABB->getHalfExtents();
		const vec3 bMax = bCenter + bAABB->getHalfExtents();

		const vec3 center = vec3(getCenter());
		const vec3 min = center - halfExtents;
		const vec3 max = center + halfExtents;

		for (int i = 0; i < 3; i++) {
			if (bMin[i] > min[i]) return false;
			else if (bMax[i] < max[i]) return false;
		}
		return true;
	}

	// handle all others here
	BoundingVolume* self = (BoundingVolume*)this;
	return bv->enclosesGeometry(self);
}