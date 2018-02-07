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
	halfExtents(obb.halfExtents) {
	transform = obb.transform;
	velocity = obb.velocity;
}

/**
 * @brief Move constructor
 */
OrientedBoundingBox::OrientedBoundingBox(OrientedBoundingBox&& obb) {
	halfExtents = move(obb.halfExtents);
	transform = move(obb.transform);
	velocity = move(obb.velocity);
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
const vec3& OrientedBoundingBox::getXAxis(void) const {
	return xAxis;
}

/**
 * @brief gets the transformed y-axis unit vector
 *
 * @return the transformed y-axis unit vector
 */
const vec3& OrientedBoundingBox::getYAxis(void) const {
	return yAxis;
}

/**
 * @brief gets the transformed z-axis unit vector
 *
 * @return the transformed z-axis unit vector
 */
const vec3& OrientedBoundingBox::getZAxis(void) const {
	return zAxis;
}

/**
 * @brief gets the triangles that make up the box
 *
 * @return a vector of triangles
 */
vector<Triangle> OrientedBoundingBox::getTriangles(void) const {
	vector<Triangle> triangles;

	const vec3 center = vec3(getCenter());
	const vec3 transformedX = halfExtents.x * getXAxis();
	const vec3 transformedY = halfExtents.y * getYAxis();
	const vec3 transformedZ = halfExtents.z * getZAxis();

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
		center - transformedX + transformedY + transformedZ,
		center - transformedX - transformedY - transformedZ
	));

	triangles.push_back(Triangle(
		center - transformedX - transformedY - transformedZ,
		center - transformedX + transformedY + transformedZ,
		center - transformedX + transformedY - transformedZ
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
		center - transformedX - transformedY - transformedZ,
		center + transformedX - transformedY + transformedZ
	));

	triangles.push_back(Triangle(
		center + transformedX - transformedY + transformedZ,
		center - transformedX - transformedY - transformedZ,
		center + transformedX - transformedY - transformedZ
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
		center - transformedX + transformedY - transformedZ,
		center + transformedX - transformedY - transformedZ
	));

	triangles.push_back(Triangle(
		center + transformedX - transformedY - transformedZ,
		center - transformedX + transformedY - transformedZ,
		center + transformedX + transformedY - transformedZ
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

	const vec3 xTransformed = getXAxis();
	const vec3 yTransformed = getYAxis();
	const vec3 zTransformed = getZAxis();

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

	const mat3 upperLeft = mat3(transform);
	xAxis = upperLeft * xAxis;
	yAxis = upperLeft * yAxis;
	zAxis = upperLeft * zAxis;

	const float lengthX = length(xAxis);
	const float lengthY = length(yAxis);
	const float lengthZ = length(zAxis);

	halfExtents.x *= lengthX;
	halfExtents.y *= lengthY;
	halfExtents.z *= lengthZ;

	xAxis /= lengthX;
	yAxis /= lengthY;
	zAxis /= lengthZ;
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
		axis.push_back(getXAxis());
		axis.push_back(getYAxis());
		axis.push_back(getZAxis());

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
			float currDist2 = triangles[i].getClosestPtSegmentTriangle(c1, c2, bCapsule->getLine());
			if (currDist2 < dist2) {
				dist2 = currDist2;
			}
		}

		return dist2 <= bCapsule->getRadius() * bCapsule->getRadius();
	}
	// handle oriented bounding box here
	else if (const OrientedBoundingBox* bOBB = dynamic_cast<OrientedBoundingBox*>(bv)) {
		float ra, rb;
		vector<vec3> bAxis;
		bAxis.push_back(bOBB->getXAxis());
		bAxis.push_back(bOBB->getYAxis());
		bAxis.push_back(bOBB->getZAxis());

		vector<vec3> axis;
		axis.push_back(getXAxis());
		axis.push_back(getYAxis());
		axis.push_back(getZAxis());

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

/**
 * @brief handles checking if a bounding volume is enclosed by this oriented bounding box
 *
 * @param bv the bounding volume that will be checked if it is enclosed
 * @return bool that determines if the input volume is enclosed by the oriented bounding box
 */
bool OrientedBoundingBox::enclosesGeometry(BoundingVolume*& bv) const {
	// handle bounding sphere here
	if (const BoundingSphere* bSphere = dynamic_cast<BoundingSphere*>(bv)) {
		vector<vec3> axis;
		axis.push_back(getXAxis());
		axis.push_back(getYAxis());
		axis.push_back(getZAxis());

		vector<vec3> bAxis;
		bAxis.push_back(vec3(1.f, 0.f, 0.f));
		bAxis.push_back(vec3(0.f, 1.f, 0.f));
		bAxis.push_back(vec3(0.f, 0.f, 1.f));

		mat3 R;

		// compute rotation matrix expressing b in a's coordinate frame
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				R[j][i] = dot(axis[i], bAxis[j]);
			}
		}

		const float& bRadius = bSphere->getRadius();
		const vec3 bCenter = vec3(bSphere->getCenter());
		const vec3 bMin = bCenter - vec3(bRadius, bRadius, bRadius);
		const vec3 bMax = bCenter + vec3(bRadius, bRadius, bRadius);

		vector<vec3> testPoints;
		testPoints.push_back(bMin);
		testPoints.push_back(vec3(bMin.x, bMin.y, bMax.z));
		testPoints.push_back(vec3(bMax.x, bMin.y, bMin.z));
		testPoints.push_back(vec3(bMax.x, bMin.y, bMax.z));
		testPoints.push_back(vec3(bMax.x, bMax.y, bMax.z));
		testPoints.push_back(vec3(bMax.x, bMax.y, bMin.z));
		testPoints.push_back(vec3(bMin.x, bMax.y, bMax.z));
		testPoints.push_back(vec3(bMin.x, bMax.y, bMin.z));

		const vec3 center = vec3(getCenter());
		for (size_t i = 0; i < testPoints.size(); i++) {
			const vec3 pt = R * (testPoints[i] - center);

			if (pt.x > halfExtents.x || pt.x < -halfExtents.x ||
				pt.y > halfExtents.y || pt.y < -halfExtents.y ||
				pt.z > halfExtents.z || pt.z < -halfExtents.z) {
				return false;
			}
		}

		return true;
	}
	// handle Axis Aligned Bounding Box here
	else if (const AxisAlignedBoundingBox* bAABB = dynamic_cast<AxisAlignedBoundingBox*>(bv)) {
		vector<vec3> axis;
		axis.push_back(getXAxis());
		axis.push_back(getYAxis());
		axis.push_back(getZAxis());

		vector<vec3> bAxis;
		bAxis.push_back(vec3(1.f, 0.f, 0.f));
		bAxis.push_back(vec3(0.f, 1.f, 0.f));
		bAxis.push_back(vec3(0.f, 0.f, 1.f));

		mat3 R;

		// compute rotation matrix expressing b in a's coordinate frame
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				R[j][i] = dot(axis[i], bAxis[j]);
			}
		}

		const vec3 bCenter = vec3(bAABB->getCenter());
		const vec3 bMin = bCenter - bAABB->getHalfExtents();
		const vec3 bMax = bCenter + bAABB->getHalfExtents();

		vector<vec3> testPoints;
		testPoints.push_back(bMin);
		testPoints.push_back(vec3(bMin.x, bMin.y, bMax.z));
		testPoints.push_back(vec3(bMax.x, bMin.y, bMin.z));
		testPoints.push_back(vec3(bMax.x, bMin.y, bMax.z));
		testPoints.push_back(vec3(bMax.x, bMax.y, bMax.z));
		testPoints.push_back(vec3(bMax.x, bMax.y, bMin.z));
		testPoints.push_back(vec3(bMin.x, bMax.y, bMax.z));
		testPoints.push_back(vec3(bMin.x, bMax.y, bMin.z));

		const vec3 center = vec3(getCenter());
		for (size_t i = 0; i < testPoints.size(); i++) {
			const vec3 pt = R * (testPoints[i] - center);

			if (pt.x > halfExtents.x || pt.x < -halfExtents.x ||
				pt.y > halfExtents.y || pt.y < -halfExtents.y ||
				pt.z > halfExtents.z || pt.z < -halfExtents.z) {
				return false;
			}
		}

		return true;
	}
	// handle bounding capsule here
	else if (const BoundingCapsule* bCapsule = dynamic_cast<BoundingCapsule*>(bv)) {
		vector<vec3> axis;
		axis.push_back(getXAxis());
		axis.push_back(getYAxis());
		axis.push_back(getZAxis());

		vector<vec3> bAxis;
		bAxis.push_back(vec3(1.f, 0.f, 0.f));
		bAxis.push_back(vec3(0.f, 1.f, 0.f));
		bAxis.push_back(vec3(0.f, 0.f, 1.f));

		mat3 R;

		// compute rotation matrix expressing b in a's coordinate frame
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				R[j][i] = dot(axis[i], bAxis[j]);
			}
		}

		const float& bRadius = bCapsule->getRadius();
		const vec3 bPoint1 = bCapsule->getLine().getPointA();
		const vec3 bPoint2 = bCapsule->getLine().getPointB();
		const vec3 bMin1 = bPoint1 - vec3(bRadius, bRadius, bRadius);
		const vec3 bMax1 = bPoint1 + vec3(bRadius, bRadius, bRadius);
		const vec3 bMin2 = bPoint2 - vec3(bRadius, bRadius, bRadius);
		const vec3 bMax2 = bPoint2 + vec3(bRadius, bRadius, bRadius);

		vector<vec3> testPoints;
		testPoints.push_back(bMin1);
		testPoints.push_back(vec3(bMin1.x, bMin1.y, bMax1.z));
		testPoints.push_back(vec3(bMax1.x, bMin1.y, bMin1.z));
		testPoints.push_back(vec3(bMax1.x, bMin1.y, bMax1.z));
		testPoints.push_back(vec3(bMax1.x, bMax1.y, bMax1.z));
		testPoints.push_back(vec3(bMax1.x, bMax1.y, bMin1.z));
		testPoints.push_back(vec3(bMin1.x, bMax1.y, bMax1.z));
		testPoints.push_back(vec3(bMin1.x, bMax1.y, bMin1.z));

		testPoints.push_back(bMin2);
		testPoints.push_back(vec3(bMin2.x, bMin2.y, bMax2.z));
		testPoints.push_back(vec3(bMax2.x, bMin2.y, bMin2.z));
		testPoints.push_back(vec3(bMax2.x, bMin2.y, bMax2.z));
		testPoints.push_back(vec3(bMax2.x, bMax2.y, bMax2.z));
		testPoints.push_back(vec3(bMax2.x, bMax2.y, bMin2.z));
		testPoints.push_back(vec3(bMin2.x, bMax2.y, bMax2.z));
		testPoints.push_back(vec3(bMin2.x, bMax2.y, bMin2.z));

		const vec3 center = vec3(getCenter());
		for (size_t i = 0; i < testPoints.size(); i++) {
			const vec3 pt = R * (testPoints[i] - center);

			if (pt.x > halfExtents.x || pt.x < -halfExtents.x ||
				pt.y > halfExtents.y || pt.y < -halfExtents.y ||
				pt.z > halfExtents.z || pt.z < -halfExtents.z) {
				return false;
			}
		}

		return true;
	}
	// handle oriented bounding box here
	else if (const OrientedBoundingBox* bOBB = dynamic_cast<OrientedBoundingBox*>(bv)) {
		vector<vec3> axis;
		axis.push_back(getXAxis());
		axis.push_back(getYAxis());
		axis.push_back(getZAxis());

		vector<vec3> bAxis;
		bAxis.push_back(bOBB->getXAxis());
		bAxis.push_back(bOBB->getYAxis());
		bAxis.push_back(bOBB->getZAxis());

		mat3 R;

		// compute rotation matrix expressing b in a's coordinate frame
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				R[j][i] = dot(axis[i], bAxis[j]);
			}
		}

		const vec3 bCenter = vec3(bOBB->getCenter());
		const vec3& bHalfExtents = bOBB->getHalfExtents();
		const vec3 xLen = bAxis[0] * bHalfExtents.x;
		const vec3 yLen = bAxis[1] * bHalfExtents.y;
		const vec3 zLen = bAxis[2] * bHalfExtents.z;

		vector<vec3> testPoints;
		testPoints.push_back(bCenter - xLen - yLen - zLen);
		testPoints.push_back(bCenter - xLen - yLen + zLen);
		testPoints.push_back(bCenter + xLen - yLen - zLen);
		testPoints.push_back(bCenter + xLen - yLen + zLen);
		testPoints.push_back(bCenter + xLen + yLen + zLen);
		testPoints.push_back(bCenter + xLen + yLen - zLen);
		testPoints.push_back(bCenter - xLen + yLen + zLen);
		testPoints.push_back(bCenter - xLen + yLen - zLen);

		const vec3 center = vec3(getCenter());
		for (size_t i = 0; i < testPoints.size(); i++) {
			const vec3 pt = R * (testPoints[i] - center);

			if (pt.x > halfExtents.x || pt.x < -halfExtents.x ||
				pt.y > halfExtents.y || pt.y < -halfExtents.y ||
				pt.z > halfExtents.z || pt.z < -halfExtents.z) {
				return false;
			}
		}

		return true;
	}

	// handle all others here
	BoundingVolume* self = (BoundingVolume*)this;
	return bv->isEnclosed(self);
}

/**
 * @brief handles checking if the oriented bounding box is enclosed by the bounding volume
 *
 * @param bv the bounding volume that will be checked to see if it encloses the oriented bounding box
 * @return bool that determines if the input volume encloses the oriented bounding box
 */
bool OrientedBoundingBox::isEnclosed(BoundingVolume*& bv) const {
	// handle bounding sphere here
	if (const BoundingSphere* bSphere = dynamic_cast<BoundingSphere*>(bv)) {
		const vec3 center = vec3(getCenter());
		const vec3 xLen = getXAxis() * halfExtents.x;
		const vec3 yLen = getYAxis() * halfExtents.y;
		const vec3 zLen = getZAxis() * halfExtents.z;

		vector<vec3> testPoints;
		testPoints.push_back(center - xLen - yLen - zLen);
		testPoints.push_back(center - xLen - yLen + zLen);
		testPoints.push_back(center + xLen - yLen - zLen);
		testPoints.push_back(center + xLen - yLen + zLen);
		testPoints.push_back(center + xLen + yLen + zLen);
		testPoints.push_back(center + xLen + yLen - zLen);
		testPoints.push_back(center - xLen + yLen + zLen);
		testPoints.push_back(center - xLen + yLen - zLen);

		const vec3 bCenter = vec3(bSphere->getCenter());
		const float radius2 = bSphere->getRadius() * bSphere->getRadius();
		for (size_t i = 0; i < testPoints.size(); i++) {
			const vec3& diff = testPoints[i] - bCenter;

			if (dot(diff, diff) > radius2) {
				return false;
			}
		}

		return true;
	}
	// handle AxisAlignedBoundingBox
	else if (const AxisAlignedBoundingBox* bAABB = dynamic_cast<AxisAlignedBoundingBox*>(bv)) {
		const vec3 center = vec3(getCenter());
		const vec3 xLen = getXAxis() * halfExtents.x;
		const vec3 yLen = getYAxis() * halfExtents.y;
		const vec3 zLen = getZAxis() * halfExtents.z;

		vector<vec3> testPoints;
		testPoints.push_back(center - xLen - yLen - zLen);
		testPoints.push_back(center - xLen - yLen + zLen);
		testPoints.push_back(center + xLen - yLen - zLen);
		testPoints.push_back(center + xLen - yLen + zLen);
		testPoints.push_back(center + xLen + yLen + zLen);
		testPoints.push_back(center + xLen + yLen - zLen);
		testPoints.push_back(center - xLen + yLen + zLen);
		testPoints.push_back(center - xLen + yLen - zLen);

		const vec3 bCenter = vec3(bAABB->getCenter());
		const vec3& bHalfExtents = bAABB->getHalfExtents();
		const vec3 bMin = bCenter - bHalfExtents;
		const vec3 bMax = bCenter + bHalfExtents;

		for (size_t i = 0; i < testPoints.size(); i++) {
			const vec3& pt = testPoints[i];
			if (pt.x < bMin.x || pt.x > bMax.x ||
				pt.y < bMin.y || pt.y > bMax.y ||
				pt.z < bMin.z || pt.z > bMax.z) {
				return false;
			}
		}

		return true;
	}
	// handle bounding capsule
	else if (const BoundingCapsule* bCapsule = dynamic_cast<BoundingCapsule*>(bv)) {
		const vec3 center = vec3(getCenter());
		const vec3 xLen = getXAxis() * halfExtents.x;
		const vec3 yLen = getYAxis() * halfExtents.y;
		const vec3 zLen = getZAxis() * halfExtents.z;

		vector<vec3> testPoints;
		testPoints.push_back(center - xLen - yLen - zLen);
		testPoints.push_back(center - xLen - yLen + zLen);
		testPoints.push_back(center + xLen - yLen - zLen);
		testPoints.push_back(center + xLen - yLen + zLen);
		testPoints.push_back(center + xLen + yLen + zLen);
		testPoints.push_back(center + xLen + yLen - zLen);
		testPoints.push_back(center - xLen + yLen + zLen);
		testPoints.push_back(center - xLen + yLen - zLen);

		const Line bLine = bCapsule->getLine();
		const float radius2 = bCapsule->getRadius() * bCapsule->getRadius();
		for (size_t i = 0; i < testPoints.size(); i++) {
			const vec3& diff = bLine.getClosestPtPointSegment(testPoints[i]) - testPoints[i];

			if (dot(diff, diff) > radius2) {
				return false;
			}
		}

		return true;
	}
	// handle oriented bounding box here
	else if (const OrientedBoundingBox* bOBB = dynamic_cast<OrientedBoundingBox*>(bv)) {
		vector<vec3> axis;
		axis.push_back(getXAxis());
		axis.push_back(getYAxis());
		axis.push_back(getZAxis());

		vector<vec3> bAxis;
		bAxis.push_back(bOBB->getXAxis());
		bAxis.push_back(bOBB->getYAxis());
		bAxis.push_back(bOBB->getZAxis());

		mat3 R;

		// compute rotation matrix expressing b in a's coordinate frame
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				R[j][i] = dot(bAxis[j], axis[i]);
			}
		}

		const vec3 center = vec3(getCenter());
		const vec3 xLen = axis[0] * halfExtents.x;
		const vec3 yLen = axis[1] * halfExtents.y;
		const vec3 zLen = axis[2] * halfExtents.z;

		vector<vec3> testPoints;
		testPoints.push_back(center - xLen - yLen - zLen);
		testPoints.push_back(center - xLen - yLen + zLen);
		testPoints.push_back(center + xLen - yLen - zLen);
		testPoints.push_back(center + xLen - yLen + zLen);
		testPoints.push_back(center + xLen + yLen + zLen);
		testPoints.push_back(center + xLen + yLen - zLen);
		testPoints.push_back(center - xLen + yLen + zLen);
		testPoints.push_back(center - xLen + yLen - zLen);

		const vec3 bCenter = vec3(bOBB->getCenter());
		const vec3& bHalfExtents = bOBB->getHalfExtents();
		for (size_t i = 0; i < testPoints.size(); i++) {
			const vec3 pt = R * (testPoints[i] - bCenter);

			if (pt.x > bHalfExtents.x || pt.x < -bHalfExtents.x ||
				pt.y > bHalfExtents.y || pt.y < -bHalfExtents.y ||
				pt.z > bHalfExtents.z || pt.z < -bHalfExtents.z) {
				return false;
			}
		}

		return true;
	}

	// handle all others here
	BoundingVolume* self = (BoundingVolume*)this;
	return bv->enclosesGeometry(self);
}