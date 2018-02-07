#ifndef BOUNDING_SPHERE_BINARY_TREE_HPP
#define BOUNDING_SPHERE_BINARY_TREE_HPP

/**
 *	@file		bounding-sphere-binary-tree.hpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/29/2018
 *	@version	1.0
 *
 *	@brief BoundingSphereBinaryTree class declaration
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for bounding sphere binary trees.
 */

#include <bounding-sphere.hpp>

namespace collision {
	class BoundingSphereNode {
	private:
		BoundingSphere s = BoundingSphere(0.f); // bounding sphere
		shared_ptr<BoundingVolume> enclosedPrimitive = shared_ptr<BoundingVolume>(nullptr); // pointer to enclosed primitive if any
		unsigned short left = 0; // left child array index
		unsigned short right = 0; // right child array index
	
	public:
		// Default constructor
		BoundingSphereNode(void);

		// Copy constructor
		BoundingSphereNode(const BoundingSphereNode& bsn);
		
		// Move constructor
		BoundingSphereNode(BoundingSphereNode&& bsn);

		// Assignment operator
		BoundingSphereNode& operator=(const BoundingSphereNode& bsn);

		// Move operator
		BoundingSphereNode& operator=(BoundingSphereNode&& bsn);

		// get the sphere
		const BoundingSphere& getSphere(void) const;
		
		// set the sphere of this node
		void setSphere(const BoundingSphere& bs);

		// get pointer to enclosed primitive bounding volume
		const shared_ptr<BoundingVolume> getEnclosedPrimitive(void) const;
		
		// set pointer to enclosed primitive bounding volume
		void setEnclosedPrimitive(const shared_ptr<BoundingVolume>& enclosedPrimitve);

		// get left child index
		const unsigned short& getLeft(void) const;
		
		// set left child index
		void setLeft(const unsigned short& left);

		// get right child index
		const unsigned short& getRight(void) const;
		
		// set right child index
		void setRight(const unsigned short& right);
	};

	class BoundingSphereBinaryTree {
	private:
		static const unsigned short MAX_TREE_SIZE = 65535; // the max number of nodes
		BoundingSphereNode nodes[MAX_TREE_SIZE]; // memory allocation for nodes

	public:
		// Default constructor
		BoundingSphereBinaryTree(void);

		// Copy constructor
		BoundingSphereBinaryTree(const BoundingSphereBinaryTree& bsbt);
		
		// Move constructor
		BoundingSphereBinaryTree(BoundingSphereBinaryTree&& bsbt);

		// handles inserting a node into the tree
		void insert(const BoundingSphereNode& bsn);

		// Returns the bounding volume that is being intersected
		const shared_ptr<BoundingVolume>& isIntersecting(BoundingVolume* bv) const;
	};
}

#endif // !BOUNDING_SPHERE_BINARY_TREE_HPP
