#ifndef AXIS_ALIGNED_BOUNDING_BOX
#define AXIS_ALIGNED_BOUNDING_BOX

/**
 *	@file		axis-aligned-bounding-box.hpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/21/2018
 *	@version	1.0
 *
 *	@brief AxisAlignedBoundingBox derived class declaration
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for axis aligned bounding boxes.
 *
 *	The intended use case of this class is for simple rigid body collision
 */

#include <bounding-volume.hpp>
#include <bounding-sphere.hpp>

namespace collision {
	/**
	 * @brief class that handles rigid body collisions for bounding boxes
	 */
	class AxisAlignedBoundingBox : public BoundingVolume {
	private:
		vec3 initialHalfExtents = vec3(0.f, 0.f, 0.f); // half the size of the bounding box
		vec3 halfExtents = vec3(0.f, 0.f, 0.f); // half the size of the bounding box

	public:
		// default constructor
		AxisAlignedBoundingBox(const vec3& initialHalfExtents);

		// copy constructor
		AxisAlignedBoundingBox(const AxisAlignedBoundingBox& aabb);

		// move constructor
		AxisAlignedBoundingBox(AxisAlignedBoundingBox&& aabb);

		// gets the current half extents
		const vec3& getHalfExtents(void) const;

		// overwrites the initial and current half extents
		void setHalfExtents(const vec3& halfExtents);

		// computes the squared distance between a point and the box
		float getSquaredDistancePtPointAABB(const vec3& p) const;

		// handles updating transform matrix for this geometry
		void update(const mat4& transform);

		// handles checking for intersection
		bool isIntersecting(BoundingVolume*& bv) const;
	};
}

#endif // !AXIS_ALIGNED_BOUNDING_BOX
