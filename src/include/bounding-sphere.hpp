#ifndef BOUNDING_SPHERE_HPP
#define BOUNDING_SPHERE_HPP

/**
 *	@file		bounding-sphere.hpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/21/2018
 *	@version	1.0
 *
 *	@brief BoundingSphere derived class declaration
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for bounding spheres.
 *
 *	The intended use case of this class is for simple rigid body collision
 */

#include <bounding-volume.hpp>

namespace collision {
	/**
	 * @brief class that handles rigid body collisions for bounding spheres
	 */
	class BoundingSphere : public BoundingVolume {
	private:
		float radius = 0.f; // the radius of the sphere

	public:
		// The default constructor
		BoundingSphere(const float& radius);

		// Copy constructor
		BoundingSphere(const BoundingSphere& s);

		// Move constructor
		BoundingSphere(BoundingSphere&& s);

		// get the radius of the sphere
		const float& getRadius(void) const;

		// handles updating transform matrix for this geometry
		void update(const mat4& transform);

		// handles checking for intersection
		bool isIntersecting(BoundingVolume*& bv) const;
	};
}

#endif // !BOUNDING_SPHERE_HPP
