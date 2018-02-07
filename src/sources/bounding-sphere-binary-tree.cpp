#include <bounding-sphere-binary-tree.hpp>

/**
 *	@file		bounding-sphere-binary-tree.hpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/29/2018
 *	@version	1.0
 *
 *	@brief BoundingSphereBinaryTree class definition
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for bounding sphere binary trees.
 */

using namespace collision;

/**
 * @brief Default constructor
 */
BoundingSphereNode::BoundingSphereNode(void) {}

/**
 * @brief Copy constructor
 */
BoundingSphereNode::BoundingSphereNode(const BoundingSphereNode& bsn):
	s(bsn.s), left(bsn.left), right(bsn.right), enclosedPrimitive(bsn.enclosedPrimitive) {
}

/**
 * @brief Move constuctor
 */
BoundingSphereNode::BoundingSphereNode(BoundingSphereNode&& bsn):
	left(bsn.left), right(bsn.right) {
	s = move(bsn.s);
	enclosedPrimitive = move(bsn.enclosedPrimitive);
}

/**
 * @brief Assignment operator
 *
 * @param bsn node to copy
 * @return the calling object
 */
BoundingSphereNode& BoundingSphereNode::operator=(const BoundingSphereNode& bsn) {
	s = bsn.s;
	enclosedPrimitive = bsn.enclosedPrimitive;
	left = bsn.left;
	right = bsn.right;
	return *this;
}

/**
 * @brief Move operator
 *
 * @param bsn node to move
 * @return the calling object
 */
BoundingSphereNode& BoundingSphereNode::operator=(BoundingSphereNode&& bsn) {
	s = move(bsn.s);
	enclosedPrimitive = move(bsn.enclosedPrimitive);
	left = bsn.left;
	right = bsn.right;
	return *this;
}


/**
 * @brief gets the bounding sphere
 *
 * @return the bounding sphere
 */
const BoundingSphere& BoundingSphereNode::getSphere(void) const {
	return s;
}

/**
 * @brief sets the bounding sphere
 *
 * @param s the sphere to set in the node
 * @return void
 */
void BoundingSphereNode::setSphere(const BoundingSphere& s) {
	this->s = s;
}

/**
 * @brief gets the pointer to enclosed primitive bounding volume
 *
 * @return the pointer to enclosed primitive bounding volume
 */
const shared_ptr<BoundingVolume> BoundingSphereNode::getEnclosedPrimitive(void) const {
	return enclosedPrimitive;
}

/**
 * @brief sets the enclosed primitve for the node
 *
 * @param enclosedPrimitive the pointer to the primitive to enclose
 * @return void
 */
void BoundingSphereNode::setEnclosedPrimitive(const shared_ptr<BoundingVolume>& enclosedPrimitive) {
	this->enclosedPrimitive = move(enclosedPrimitive);
}

/**
 * @brief gets the left node index
 *
 * @return left node index
 */
const unsigned short& BoundingSphereNode::getLeft(void) const {
	return left;
}

/**
 * @brief sets the left index
 *
 * @param left the left node index
 * @return void
 */
void BoundingSphereNode::setLeft(const unsigned short& left) {
	this->left = left;
}

/**
 * @brief gets the right node index
 *
 * @return right node index
 */
const unsigned short& BoundingSphereNode::getRight(void) const {
	return right;
}

/**
 * @brief sets the right index
 *
 * @param right the right node index
 * @return void
 */
void BoundingSphereNode::setRight(const unsigned short& right) {
	this->right = right;
}

/**
 * @brief Default constructor
 */
BoundingSphereBinaryTree::BoundingSphereBinaryTree(void) {}

/**
 * @brief Copy constructor
 */
BoundingSphereBinaryTree::BoundingSphereBinaryTree(const BoundingSphereBinaryTree& bsbt) {
	for (int i = 0; i < MAX_TREE_SIZE; i++) {
		nodes[i] = bsbt.nodes[i];
	}
}

/**
 * @brief Move constructor
 */
BoundingSphereBinaryTree::BoundingSphereBinaryTree(BoundingSphereBinaryTree&& bsbt) {
	for (int i = 0; i < MAX_TREE_SIZE; i++) {
		nodes[i] = move(bsbt.nodes[i]);
	}
}

/**
 * @brief handles inserting a node into the tree
 *
 * @param bsn node to insert into the tree
 * @return void
 */
void BoundingSphereBinaryTree::insert(const BoundingSphereNode& bsn) {
	// TODO: handle insertion
}