#ifndef BOUNDING_CAPSULE_HPP
#define BOUNDING_CAPSULE_HPP

/**
 *	@file		bounding-capsule.hpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/22/2018
 *	@version	1.0
 *
 *	@brief BoundingCapsule derived class declaration
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for bounding capsules.
 *
 *	The intended use case of this class is for simple rigid body collision
 */

#include <line.hpp>
#include <axis-aligned-bounding-box.hpp>

namespace collision {
	class BoundingCapsule : public BoundingVolume {
	private:
		Line l; // the line representing the cylindrical part of the capsule
		vec3 lineNormal; // any vector normal to the line direction
		vec3 secondLineNormal; // this vector will be normal to the line and line normal
		float radius; // radius of the spheres at the ends

	public:
		// Default constructor
		BoundingCapsule(const Line& l, const vec3& lineNormal, const float& radius);

		// Copy constructor
		BoundingCapsule(const BoundingCapsule& bc);

		// Move constructor
		BoundingCapsule(BoundingCapsule&& bc);

		// get the line that represnts the cylindrical portion
		const Line& getLine(void) const;

		// get the normal to the line
		const vec3& getLineNormal(void) const;

		// get the normal to the line and line normal
		const vec3& getSecondLineNormal(void) const;

		// get the radius of the spheres at the end
		const float& getRadius(void) const;

		// handles updating transform matrix for this geometry
		void update(const mat4& transform);

		// handles checking for intersection
		bool isIntersecting(BoundingVolume*& bv) const;

		// handles checking if a bounding volume is enclosed by this sphere
		bool enclosesGeometry(BoundingVolume*& bv) const;

		// handles checking if the sphere is enclosed by the bounding volume
		bool isEnclosed(BoundingVolume*& bv) const;
	};
}

#endif // !BOUNDING_CAPSULE_HPP
