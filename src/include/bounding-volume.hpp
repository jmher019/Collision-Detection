#ifndef BOUNDING_VOUME_HPP
#define BOUNDING_VOUME_HPP

/**
 *	@file		bounding-volume.hpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/21/2018
 *	@version	1.0
 *
 *	@brief BoundingVolume base class declaration
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API that all 3D bounding geometries will extend.
 *
 */

#include <glm/glm.hpp>

using namespace glm;
using namespace std;

namespace collision {
	/**
	 * @brief Base class for 3D bounding geometries
	 */
	class BoundingVolume {
	protected:
		mat4 transform = mat4(1.f);
		vec3 velocity = vec3(0.f, 0.f, 0.f);

	public:

		// gets the transform
		const mat4& getTransform(void) const {
			return transform;
		};

		// gets the center
		const vec4& getCenter(void) const {
			return transform[3];
		};

		// gets the velocity
		const vec3& getVelocity(void) const {
			return velocity;
		};

		// sets the velocity
		void setVelocity(const vec3& velocity) {
			this->velocity = velocity;
		};

		// function to implement that handles updating the transform matrix
		virtual void update(const mat4& transform) = 0;

		// function to implement that handles checking for intersection
		virtual bool isIntersecting(BoundingVolume*& other) const = 0;
	};
}

#endif // !BOUNDING_VOUME_HPP
