/**
 *	@file		oriented-bounding-box.cpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/23/2018
 *	@version	1.0
 *
 *	@brief OrientedBoundingBox derived class definition
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for oriented bounding boxes.
 *
 *	The intended use case of this class is for simple rigid body collision
 */

#include <oriented-bounding-box.hpp>

using namespace collision;

/**
 * @brief Default constructor
 */
OrientedBoundingBox::OrientedBoundingBox(const vec3& halfExtents):
	halfExtents(halfExtents) {}

/**
 * @brief Copy constructor
 */
OrientedBoundingBox::OrientedBoundingBox(const OrientedBoundingBox& obb):
	halfExtents(obb.halfExtents) {}

/**
 * @brief Move constructor
 */
OrientedBoundingBox::OrientedBoundingBox(OrientedBoundingBox&& obb) {
	halfExtents = move(obb.halfExtents);
}

/**
 * @brief gets the half extents
 *
 * @return the half extents
 */
const vec3& OrientedBoundingBox::getHalfExtents(void) const {
	return halfExtents;
}

/**
 * @brief gets the transformed x-axis unit vector
 *
 * @return the transformed x-axis unit vector
 */
const vec3 OrientedBoundingBox::getTransformedXAxis(void) const {
	return normalize(vec3(transform[0]));
}

/**
 * @brief gets the transformed y-axis unit vector
 *
 * @return the transformed y-axis unit vector
 */
const vec3 OrientedBoundingBox::getTransformedYAxis(void) const {
	return normalize(vec3(transform[1]));
}

/**
 * @brief gets the transformed z-axis unit vector
 *
 * @return the transformed z-axis unit vector
 */
const vec3 OrientedBoundingBox::getTransformedZAxis(void) const {
	return normalize(vec3(transform[2]));
}

/**
 * @brief gets the triangles that make up the box
 *
 * @return a vector of triangles
 */
vector<Triangle> OrientedBoundingBox::getTriangles(void) const {
	vector<Triangle> triangles;

	const vec3 center = vec3(getCenter());
	const vec3 transformedX = halfExtents.x * getTransformedXAxis();
	const vec3 transformedY = halfExtents.y * getTransformedYAxis();
	const vec3 transformedZ = halfExtents.z * getTransformedZAxis();

	// +x transformed face
	triangles.push_back(Triangle(
		center + transformedX - transformedY + transformedZ,
		center + transformedX - transformedY - transformedZ,
		center + transformedX + transformedY + transformedZ
	));

	triangles.push_back(Triangle(
		center + transformedX - transformedY - transformedZ,
		center + transformedX + transformedY - transformedZ,
		center + transformedX + transformedY + transformedZ
	));

	// -x transformed face
	triangles.push_back(Triangle(
		center - transformedX - transformedY + transformedZ,
		center - transformedX - transformedY - transformedZ,
		center - transformedX + transformedY + transformedZ
	));

	triangles.push_back(Triangle(
		center - transformedX - transformedY - transformedZ,
		center - transformedX + transformedY - transformedZ,
		center - transformedX + transformedY + transformedZ
	));

	// +y transformed face
	triangles.push_back(Triangle(
		center - transformedX + transformedY + transformedZ,
		center + transformedX + transformedY + transformedZ,
		center - transformedX + transformedY - transformedZ
	));

	triangles.push_back(Triangle(
		center + transformedX + transformedY + transformedZ,
		center + transformedX + transformedY - transformedZ,
		center - transformedX + transformedY - transformedZ
	));

	// -y transformed face
	triangles.push_back(Triangle(
		center - transformedX - transformedY + transformedZ,
		center + transformedX - transformedY + transformedZ,
		center - transformedX - transformedY - transformedZ
	));

	triangles.push_back(Triangle(
		center + transformedX - transformedY + transformedZ,
		center + transformedX - transformedY - transformedZ,
		center - transformedX - transformedY - transformedZ
	));

	// +z transformed face
	triangles.push_back(Triangle(
		center - transformedX - transformedY + transformedZ,
		center + transformedX - transformedY + transformedZ,
		center - transformedX + transformedY + transformedZ
	));

	triangles.push_back(Triangle(
		center + transformedX - transformedY + transformedZ,
		center + transformedX + transformedY + transformedZ,
		center - transformedX + transformedY + transformedZ
	));

	// -z transformed face
	triangles.push_back(Triangle(
		center - transformedX - transformedY - transformedZ,
		center + transformedX - transformedY - transformedZ,
		center - transformedX + transformedY - transformedZ
	));

	triangles.push_back(Triangle(
		center + transformedX - transformedY - transformedZ,
		center + transformedX + transformedY - transformedZ,
		center - transformedX + transformedY - transformedZ
	));

	return triangles;
}

/**
 * @brief gets the closest point on the oriented bounding box given another point
 *
 * @param p the input point
 * @return the point on the oriented bounding box closest to the input point
 */
vec3 OrientedBoundingBox::getClosestPtPointOBB(const vec3& p) const {
	const vec3 center = vec3(getCenter());
	const vec3 d = p - center;
	// Start result at center of box; kame steps from there
	vec3 result = center;

	const vec3 xTransformed = getTransformedXAxis();
	const vec3 yTransformed = getTransformedXAxis();
	const vec3 zTransformed = getTransformedXAxis();

	// project d onto the transformed x-axis to get the distance
	// along the axis of d from the box center
	float distX = dot(d, xTransformed);
	if (distX > halfExtents.x) distX = halfExtents.x;
	else if (distX < -halfExtents.x) distX = halfExtents.x;

	result += distX * xTransformed;

	// project d onto the transformed y-axis to get the distance
	// along the axis of d from the box center
	float distY = dot(d, yTransformed);
	if (distY > halfExtents.y) distY = halfExtents.y;
	else if (distY < -halfExtents.y) distY = halfExtents.y;

	result += distY * yTransformed;

	// project d onto the transformed z-axis to get the distance
	// along the axis of d from the box center
	float distZ = dot(d, zTransformed);
	if (distZ > halfExtents.z) distZ = halfExtents.z;
	else if (distZ < -halfExtents.z) distZ = halfExtents.z;

	result += distZ * zTransformed;

	return result;
}

/**
 * @brief handles updating transform matrix for this geometry
 *
 * @param transform will be used to update the geometry
 * @return void
 */
void OrientedBoundingBox::update(const mat4& transform) {
	this->transform = transform * this->transform;
}

/**
 * @brief handles checking for intersection
 *
 * @param bv the bounding volume that will be checked against for intersection
 * @return bool that determines if the volumes are intersecting
 */
bool OrientedBoundingBox::isIntersecting(BoundingVolume*& bv) const {
	// handle bounding sphere here
	if (const BoundingSphere* bSphere = dynamic_cast<BoundingSphere*>(bv)) {
		const vec3 bCenter = vec3(bSphere->getCenter());
		const vec3 diff = getClosestPtPointOBB(bCenter) - bCenter;
		const float dist2 = dot(diff, diff);
		return dist2 <= bSphere->getRadius() * bSphere->getRadius();
	}
	// handle axis aligned bounding box here
	else if (const AxisAlignedBoundingBox* bAABB = dynamic_cast<AxisAlignedBoundingBox*>(bv)) {
		float ra, rb;
		vector<vec3> bAxis;
		bAxis.push_back(vec3(1.f, 0.f, 0.f));
		bAxis.push_back(vec3(0.f, 1.f, 0.f));
		bAxis.push_back(vec3(0.f, 0.f, 1.f));

		vector<vec3> axis;
		axis.push_back(getTransformedXAxis());
		axis.push_back(getTransformedYAxis());
		axis.push_back(getTransformedZAxis());

		mat3 R, absR;
		const vec3& bHalfExtents = bAABB->getHalfExtents();

		// compute rotation matrix expressing b in a's coordinate frame
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				R[j][i] = dot(axis[i], bAxis[j]);
			}
		}

		// compute translation vector t
		vec3 t = vec3(bAABB->getCenter()) - vec3(getCenter());
		// bring translation into a's coordinate frame
		t = vec3(dot(t, axis[0]), dot(t, axis[1]), dot(t, axis[2]));

		// Compute common subexpressions. Add in an epsilon term to counteract arithmetic errors
		// when two edges are parallel and their cross product is (near) null
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				absR[j][i] = fabs(R[j][i]) + 1e-8f;
			}
		}

		// Test axes L = A0, L = A1, L = A2
		for (int i = 0; i < 3; i++) {
			ra = halfExtents[i];
			rb = bHalfExtents[0] * absR[0][i] + bHalfExtents[1] * absR[1][i] + bHalfExtents[2] * absR[2][i];
			if (fabs(t[i]) > ra + rb) return false;
		}

		// Test axes L = B0, L = B1, L = B2
		for (int i = 0; i < 3; i++) {
			ra = halfExtents[0] * absR[i][0] + halfExtents[1] * absR[i][1] + halfExtents[2] * absR[i][2];
			rb = bHalfExtents[i];
			if (fabs(t[0] * R[i][0] + t[1] * R[i][1] + t[2] * R[i][2]) > ra + rb) return false;
		}

		// Test axis L = A0 x B0
		ra = halfExtents[1] * absR[0][2] + halfExtents[2] * absR[0][1];
		rb = bHalfExtents[1] * absR[2][0] + bHalfExtents[2] * absR[1][0];
		if (fabs(t[2] * R[0][1] - t[1] * R[0][2]) > ra + rb) return false;

		// Test axis L = A0 x B1
		ra = halfExtents[1] * absR[1][2] + halfExtents[2] * absR[1][1];
		rb = bHalfExtents[1] * absR[2][0] + bHalfExtents[2] * absR[0][0];
		if (fabs(t[2] * R[1][1] - t[1] * R[1][2]) > ra + rb) return false;

		// Test axis L = A0 X B2
		ra = halfExtents[1] * absR[2][2] + halfExtents[2] * absR[2][1];
		rb = bHalfExtents[0] * absR[1][0] + bHalfExtents[1] * absR[0][0];
		if (fabs(t[2] * R[2][1] - t[1] * R[2][2]) > ra + rb) return false;

		// Test axis L = A1 X B0
		ra = halfExtents[0] * absR[0][2] + halfExtents[2] * absR[0][0];
		rb = bHalfExtents[1] * absR[2][1] + bHalfExtents[2] * absR[1][1];
		if (fabs(t[0] * R[0][2] - t[2] * R[0][0]) > ra + rb) return false;

		// Test axis L = A1 X B1
		ra = halfExtents[0] * absR[1][2] + halfExtents[2] * absR[1][0];
		rb = bHalfExtents[0] * absR[2][1] + bHalfExtents[2] * absR[0][1];
		if (fabs(t[0] * R[1][2] - t[2] * R[1][0]) > ra + rb) return false;

		// Test axis L = A1 X B2
		ra = halfExtents[0] * absR[2][2] + halfExtents[2] * absR[2][0];
		rb = bHalfExtents[0] * absR[1][1] + bHalfExtents[1] * absR[0][1];
		if (fabs(t[0] * R[2][2] - t[2] * R[2][0]) > ra + rb) return false;

		// Test axis L = A2 X B0
		ra = halfExtents[0] * absR[0][1] + halfExtents[1] * absR[0][0];
		rb = bHalfExtents[1] * absR[2][2] + bHalfExtents[2] * absR[1][2];
		if (fabs(t[1] * R[0][0] - t[0] * R[0][1]) > ra + rb) return false;

		// Test axis L = A2 X B1
		ra = halfExtents[0] * absR[1][1] + halfExtents[1] * absR[1][0];
		rb = bHalfExtents[0] * absR[2][2] + bHalfExtents[2] * absR[0][2];
		if (fabs(t[1] * R[1][0] - t[0] * R[1][1]) > ra + rb) return false;

		// Test axis L = A2 X B2
		ra = halfExtents[0] * absR[2][1] + halfExtents[1] * absR[2][0];
		rb = bHalfExtents[0] * absR[1][2] + bHalfExtents[1] * absR[0][2];
		if (fabs(t[1] * R[2][0] - t[0] * R[2][1]) > ra + rb) return false;

		return true;
	}
	// handle bounding capsule here
	else if (const BoundingCapsule* bCapsule = dynamic_cast<BoundingCapsule*>(bv)) {
		vector<Triangle> triangles = move(getTriangles());
		vec3 c1, c2;
		float dist2 = triangles[0].getClosestPtSegmentTriangle(c1, c2, bCapsule->getLine());

		for (size_t i = 1; i < triangles.size(); i++) {
			vec3 c3, c4;
			float currDist2 = triangles[i].getClosestPtSegmentTriangle(c3, c4, bCapsule->getLine());
			if (currDist2 < dist2) {
				dist2 = currDist2;
				c1 = c3;
				c2 = c4;
			}
		}

		return dist2 <= bCapsule->getRadius() * bCapsule->getRadius();
	}
	// handle oriented bounding box here
	else if (const OrientedBoundingBox* bOBB = dynamic_cast<OrientedBoundingBox*>(bv)) {
		float ra, rb;
		vector<vec3> bAxis;
		bAxis.push_back(bOBB->getTransformedXAxis());
		bAxis.push_back(bOBB->getTransformedYAxis());
		bAxis.push_back(bOBB->getTransformedZAxis());

		vector<vec3> axis;
		axis.push_back(getTransformedXAxis());
		axis.push_back(getTransformedYAxis());
		axis.push_back(getTransformedZAxis());

		mat3 R, absR;
		const vec3& bHalfExtents = bOBB->getHalfExtents();

		// compute rotation matrix expressing b in a's coordinate frame
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				R[j][i] = dot(axis[i], bAxis[j]);
			}
		}
		
		// compute translation vector t
		vec3 t = vec3(bOBB->getCenter()) - vec3(getCenter());
		// bring translation into a's coordinate frame
		t = vec3(dot(t, axis[0]), dot(t, axis[1]), dot(t, axis[2]));

		// Compute common subexpressions. Add in an epsilon term to counteract arithmetic errors
		// when two edges are parallel and their cross product is (near) null
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				absR[j][i] = fabs(R[j][i]) + 1e-8f;
			}
		}

		// Test axes L = A0, L = A1, L = A2
		for (int i = 0; i < 3; i++) {
			ra = halfExtents[i];
			rb = bHalfExtents[0] * absR[0][i] + bHalfExtents[1] * absR[1][i] + bHalfExtents[2] * absR[2][i];
			if (fabs(t[i]) > ra + rb) return false;
		}

		// Test axes L = B0, L = B1, L = B2
		for (int i = 0; i < 3; i++) {
			ra = halfExtents[0] * absR[i][0] + halfExtents[1] * absR[i][1] + halfExtents[2] * absR[i][2];
			rb = bHalfExtents[i];
			if (fabs(t[0] * R[i][0] + t[1] * R[i][1] + t[2] * R[i][2]) > ra + rb) return false;
		}

		// Test axis L = A0 x B0
		ra = halfExtents[1] * absR[0][2] + halfExtents[2] * absR[0][1];
		rb = bHalfExtents[1] * absR[2][0] + bHalfExtents[2] * absR[1][0];
		if (fabs(t[2] * R[0][1] - t[1] * R[0][2]) > ra + rb) return false;

		// Test axis L = A0 x B1
		ra = halfExtents[1] * absR[1][2] + halfExtents[2] * absR[1][1];
		rb = bHalfExtents[1] * absR[2][0] + bHalfExtents[2] * absR[0][0];
		if (fabs(t[2] * R[1][1] - t[1] * R[1][2]) > ra + rb) return false;

		// Test axis L = A0 X B2
		ra = halfExtents[1] * absR[2][2] + halfExtents[2] * absR[2][1];
		rb = bHalfExtents[0] * absR[1][0] + bHalfExtents[1] * absR[0][0];
		if (fabs(t[2] * R[2][1] - t[1] * R[2][2]) > ra + rb) return false;

		// Test axis L = A1 X B0
		ra = halfExtents[0] * absR[0][2] + halfExtents[2] * absR[0][0];
		rb = bHalfExtents[1] * absR[2][1] + bHalfExtents[2] * absR[1][1];
		if (fabs(t[0] * R[0][2] - t[2] * R[0][0]) > ra + rb) return false;

		// Test axis L = A1 X B1
		ra = halfExtents[0] * absR[1][2] + halfExtents[2] * absR[1][0];
		rb = bHalfExtents[0] * absR[2][1] + bHalfExtents[2] * absR[0][1];
		if (fabs(t[0] * R[1][2] - t[2] * R[1][0]) > ra + rb) return false;

		// Test axis L = A1 X B2
		ra = halfExtents[0] * absR[2][2] + halfExtents[2] * absR[2][0];
		rb = bHalfExtents[0] * absR[1][1] + bHalfExtents[1] * absR[0][1];
		if (fabs(t[0] * R[2][2] - t[2] * R[2][0]) > ra + rb) return false;

		// Test axis L = A2 X B0
		ra = halfExtents[0] * absR[0][1] + halfExtents[1] * absR[0][0];
		rb = bHalfExtents[1] * absR[2][2] + bHalfExtents[2] * absR[1][2];
		if (fabs(t[1] * R[0][0] - t[0] * R[0][1]) > ra + rb) return false;

		// Test axis L = A2 X B1
		ra = halfExtents[0] * absR[1][1] + halfExtents[1] * absR[1][0];
		rb = bHalfExtents[0] * absR[2][2] + bHalfExtents[2] * absR[0][2];
		if (fabs(t[1] * R[1][0] - t[0] * R[1][1]) > ra + rb) return false;

		// Test axis L = A2 X B2
		ra = halfExtents[0] * absR[2][1] + halfExtents[1] * absR[2][0];
		rb = bHalfExtents[0] * absR[1][2] + bHalfExtents[1] * absR[0][2];
		if (fabs(t[1] * R[2][0] - t[0] * R[2][1]) > ra + rb) return false;

		return true;
	}

	// handle all others here
	BoundingVolume* self = (BoundingVolume*)this;
	return bv->isIntersecting(self);
}