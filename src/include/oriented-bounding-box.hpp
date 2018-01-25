#ifndef ORIENTED_BOUNDING_BOX_HPP
#define ORIENTED_BOUNDING_BOX_HPP

/**
 *	@file		oriented-bounding-box.hpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/23/2018
 *	@version	1.0
 *
 *	@brief OrientedBoundingBox derived class declaration
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for oriented bounding boxes.
 *
 *	The intended use case of this class is for simple rigid body collision
 */

#include <bounding-volume.hpp>
#include <bounding-capsule.hpp>

namespace collision {
	class OrientedBoundingBox : public BoundingVolume {
	private:
		vec3 halfExtents = vec3(0.f, 0.f, 0.f);

	public:
		// default constructor
		OrientedBoundingBox(const vec3& halfExtents);

		// copy constructor
		OrientedBoundingBox(const OrientedBoundingBox& obb);

		// move constructor
		OrientedBoundingBox(OrientedBoundingBox&& obb);

		// gets the half extent
		const vec3& getHalfExtents(void) const;

		// get transformed x-axis
		const vec3 getTransformedXAxis(void) const;

		// get transformed y-axis
		const vec3 getTransformedYAxis(void) const;

		// get transformed z-axis
		const vec3 getTransformedZAxis(void) const;

		// gets the triangles that make up the box
		vector<Triangle> getTriangles(void) const;

		// gets the closest point on the oriented bounding box given another point
		vec3 getClosestPtPointOBB(const vec3& p) const;

		// handles updating transform matrix for this geometry
		void update(const mat4& transform);

		// handles checking for intersection
		bool isIntersecting(BoundingVolume*& bv) const;
	};
}

#endif // !ORIENTED_BOUNDING_BOX_HPP
