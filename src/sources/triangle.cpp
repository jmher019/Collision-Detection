/**
 *	@file		triangle.cpp
 *	@author		Jonathan Hernandez (jmher019)
 *	@date		1/17/2018
 *	@version	1.0
 *
 *	@brief Triangle class definition
 *
 *	@section DESCRIPTION
 *
 *	This file is intended to be an API for the Triangle class.
 *
 *	Common use cases for the Triangle class is for utility
 *	purposes and should not be used as a replacement data
 *	structure for managing vertices.
 */

#include <triangle.hpp>

using namespace collision;

/**
 * @brief The Default constructor which wraps 3 vertices with shared pointers
 */
Triangle::Triangle(const vec3& v1, const vec3& v2, const vec3& v3):
	v1(v1), v2(v2), v3(v3) {}

/**
 * @brief The Copy constructor
 */
Triangle::Triangle(const Triangle& t):
	v1(t.v1), v2(t.v2), v3(t.v3) {}

/**
 * @brief The Move constructor
 */
Triangle::Triangle(Triangle&& t) {
	v1 = move(t.v1);
	v2 = move(t.v2);
	v3 = move(t.v3);
}

/**
 * @brief gets the first vertex
 *
 * @return the first vertex in the triangle
 */
const vec3& Triangle::getVertex1(void) const {
	return v1;
}

/**
  * @brief sets the first vertex
  *
  * @param a the first vertex for the triangle
  * @return void
  */
void Triangle::setVertex1(const vec3& v1) {
	this->v1 = v1;
}

/**
 * @brief gets the second vertex
 *
 * @return the second vertex in the triangle
 */
const vec3& Triangle::getVertex2(void) const {
	return v2;
}

/**
 * @brief sets the second vertex
 *
 * @param a the second vertex for the triangle
 * @return void
 */
void Triangle::setVertex2(const vec3& v2) {
	this->v2 = v2;
}

/**
 * @brief gets the third vertex
 *
 * @return the third vertex in the triangle
 */
const vec3& Triangle::getVertex3(void) const {
	return v3;
}

/**
 * @brief sets the third vertex
 *
 * @param a the third vertex for the triangle
 * @return void
 */
void Triangle::setVertex3(const vec3& v3) {
	this->v3 = v3;
}

/**
 * @brief calculates the triangle normal
 * 
 * @return the normal to the triangle
 */
vec3 Triangle::getNormal(void) const {
	return cross(v2 - v1, v3 - v2);
}

/**
 * @brief calculates the triangle normal
 *
 * @param v1 the first vertex in the triangle
 * @param v2 the second vertex in the triangle
 * @param v3 the third vertex in the triangle
 * @return the normal to the triangle
 */
vec3 Triangle::getNormal(const vec3& v1, const vec3& v2, const vec3& v3) {
	return cross(v2 - v1, v3 - v2);
}

/**
 * @brief converts a 3d point to a barycentric coordinate (u, v, w)
 *
 * @param p the point to be converted
 * @return the barycentic coordinate representation of the point
 */
vec3 Triangle::getBarycentricCoord(const vec3& p) const {
	const vec3 vec0 = v2 - v1, vec1 = v3 - v1, vec2 = p - v1;
	const float d00 = dot(vec0, vec0);
	const float d01 = dot(vec0, vec1);
	const float d11 = dot(vec1, vec1);
	const float d20 = dot(vec2, vec0);
	const float d21 = dot(vec2, vec1);
	const float denom = d00 * d11 - d01 * d01;

	const float v = (d11 * d20 - d01 * d21) / denom;
	const float w = (d00 * d21 - d01 * d20) / denom;
	return vec3(1.0f - v - w, v, w);
}

/**
 * @brief converts a 3d point to a barycentric coordinate (u, v, w)
 *
 * @param p the point to be converted
 * @param v1 the first vertex of the triangle
 * @param v2 the second vertex of the triangle
 * @param v3 the third vertex of the triangle
 * @return the barycentic coordinate representation of the point
 */
vec3 Triangle::getBarycentricCoord(const vec3& p, const vec3& v1, const vec3& v2, const vec3& v3) {
	const vec3 vec0 = v2 - v1, vec1 = v3 - v1, vec2 = p - v1;
	const float d00 = dot(vec0, vec0);
	const float d01 = dot(vec0, vec1);
	const float d11 = dot(vec1, vec1);
	const float d20 = dot(vec2, vec0);
	const float d21 = dot(vec2, vec1);
	const float denom = d00 * d11 - d01 * d01;

	const float v = (d11 * d20 - d01 * d21) / denom;
	const float w = (d00 * d21 - d01 * d20) / denom;
	return vec3(1.0f - v - w, v, w);
}

/**
 * @brief checks if a point is contained within the triangle
 *
 * @param p the point to check
 * @return a bool that states the result
 */
bool Triangle::isWithinTriangle(const vec3& p) const {
	const vec3 barycentricCoord = getBarycentricCoord(p);

	return barycentricCoord[1] >= 0.0f &&
		barycentricCoord[2] >= 0.0f &&
		(barycentricCoord[1] + barycentricCoord[2]) <= 1.0f;
}

/**
 * @brief checks if a point is contained within the triangle
 *
 * @param p the point to check
 * @param v1 the first vertex in the triangle
 * @param v2 the second vertex in the triangle
 * @param v3 the third vertex in the triangle
 * @return a bool that states the result
 */
bool Triangle::isWithinTriangle(const vec3& p, const vec3& v1, const vec3& v2, const vec3& v3) {
	const vec3 barycentricCoord = getBarycentricCoord(p, v1, v2, v3);

	return barycentricCoord[1] >= 0.0f &&
		barycentricCoord[2] >= 0.0f &&
		(barycentricCoord[1] + barycentricCoord[2]) <= 1.0f;
}

/**
 * @brief computes the closest point on a triangle due to an input point
 *
 * @param p the input point
 * @return the closest point on the triangle to the input point
 */
vec3 Triangle::getClosestPtPointTriangle(const vec3& p) const {
	// Check if P in vertex region outside A
	const vec3 ab = v2 - v1;
	const vec3 ac = v3 - v1;
	const vec3 ap = p - v1;
	const float d1 = dot(ab, ap);
	const float d2 = dot(ac, ap);
	if (d1 <= 0.0f && d2 <= 0.0f) return v1; // barycentric coordinates (1,0,0)

	// Check if P in vertex region outside B
	const vec3 bp = p - v2;
	const float d3 = dot(ab, bp);
	const float d4 = dot(ac, bp);
	if (d3 >= 0.0f && d4 <= d3) return v2; // barycentric coordinates (0,1,0)

	// Check if P in edge region of AB, if so return projection of P onto AB
	const float vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
		const float v = d1 / (d1 - d3);
		return v1 + v * ab; // barycentric coordinates (1-v,v,0)
	}
	// Check if P in vertex region outside C
	const vec3 cp = p - v3;
	const float d5 = dot(ab, cp);
	const float d6 = dot(ac, cp);
	if (d6 >= 0.0f && d5 <= d6) return v3; // barycentric coordinates (0,0,1)

	// Check if P in edge region of AC, if so return projection of P onto AC
	const float vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
		const float w = d2 / (d2 - d6);
		return v1 + w * ac; // barycentric coordinates (1-w,0,w)
	}
	// Check if P in edge region of BC, if so return projection of P onto BC
	const float va = d3 * d6 - d5 * d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
		const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return v2 + w * (v3 - v2); // barycentric coordinates (0,1-w,w)
	}
	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	const float denom = 1.0f / (va + vb + vc);
	const float v = vb * denom;
	const float w = vc * denom;
	return v1 + ab * v + ac * w; // = u*v1 + v*b + w*c, u = va * denom = 1.0f-v-w
}

/**
 * @brief computes the closest point on a triangle due to an input point
 *
 * @param p the input point
 * @param v1 the first vertex in the triangle
 * @param v2 the second vertex in the triangle
 * @param v3 the third vertex in the triangle
 * @return the closest point on the triangle to the input point
 */
vec3 Triangle::getClosestPtPointTriangle(const vec3& p, const vec3& v1, const vec3& v2, const vec3& v3) {
	// Check if P in vertex region outside A
	const vec3 ab = v2 - v1;
	const vec3 ac = v3 - v1;
	const vec3 ap = p - v1;
	const float d1 = dot(ab, ap);
	const float d2 = dot(ac, ap);
	if (d1 <= 0.0f && d2 <= 0.0f) return v1; // barycentric coordinates (1,0,0)
	
	// Check if P in vertex region outside B
	const vec3 bp = p - v2;
	const float d3 = dot(ab, bp);
	const float d4 = dot(ac, bp);
	if (d3 >= 0.0f && d4 <= d3) return v2; // barycentric coordinates (0,1,0)
	
	// Check if P in edge region of AB, if so return projection of P onto AB
	const float vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
		const float v = d1 / (d1 - d3);
		return v1 + v * ab; // barycentric coordinates (1-v,v,0)
	}
	// Check if P in vertex region outside C
	const vec3 cp = p - v3;
	const float d5 = dot(ab, cp);
	const float d6 = dot(ac, cp);
	if (d6 >= 0.0f && d5 <= d6) return v3; // barycentric coordinates (0,0,1)

	// Check if P in edge region of AC, if so return projection of P onto AC
	const float vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
		const float w = d2 / (d2 - d6);
		return v1 + w * ac; // barycentric coordinates (1-w,0,w)
	}
	// Check if P in edge region of BC, if so return projection of P onto BC
	const float va = d3 * d6 - d5 * d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
		const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return v2 + w * (v3 - v2); // barycentric coordinates (0,1-w,w)
	}
	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	const float denom = 1.0f / (va + vb + vc);
	const float v = vb * denom;
	const float w = vc * denom;
	return v1 + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = 1.0f-v-w
}

/**
 * @brief checks if line segment intersects triangle
 * @param c the point of intersection if any
 * @param l the segment
 * @return a bool which determines if the triangle and segment intersect
 */
bool Triangle::isSegmentIntersecting(vec3& c, const Line& l) const {
	const vec3 n = getNormal();
	const float d = dot(v1, n);
	
	const vec3 lVec = l.getPointB() - l.getPointA();
	const float len2 = dot(lVec, lVec);

	// if the line degenerates to a point
	if (len2 < 1e-8f) {
		c = l.getPointA();
		return isWithinTriangle(c);
	}

	const float t = dot(v1 - l.getPointA(), n) / dot(lVec, n);

	if (t < 0.f || t > 1.f) {
		return false;
	}

	c = t * lVec + l.getPointA();
	return isWithinTriangle(c);
}

/**
 * @brief checks if line segment intersects triangle
 * @param c the point of intersection if any
 * @param p the start point of the segment
 * @param q the end point of the segment
 * @return a bool which determines if the triangle and segment intersect
 */
bool Triangle::isSegmentIntersecting(vec3& c, const vec3& p, const vec3& q) const {
	const vec3 n = getNormal();
	const float d = dot(v1, n);

	const vec3 lVec = q - p;
	const float len2 = dot(lVec, lVec);

	// if the line degenerates to a point
	if (len2 < 1e-8f) {
		c = p;
		return isWithinTriangle(c);
	}

	const float t = dot(v2 - p, n) / dot(lVec, n);

	if (t < 0.f || t > 1.f) {
		return false;
	}

	c = t * lVec + p;
	return isWithinTriangle(c);
}

/**
 * @brief checks if line segment intersects triangle
 * @param c the point of intersection if any
 * @param l the line segment
 * @param v1 the first vertex of the triangle
 * @param v2 the first vertex of the triangle
 * @param v3 the first vertex of the triangle
 * @return a bool which determines if the triangle and segment intersect
 */
bool Triangle::isSegmentIntersecting(vec3& c, const Line& l, const vec3& v1, const vec3& v2, const vec3& v3) {
	const vec3 n = getNormal(v1, v2, v3);
	const float d = dot(v1, n);

	const vec3 lVec = l.getPointB() - l.getPointA();
	const float len2 = dot(lVec, lVec);

	// if the line degenerates to a point
	if (len2 < 1e-8f) {
		c = l.getPointA();
		return isWithinTriangle(c, v1, v2, v3);
	}

	const float t = dot(v1 - l.getPointA(), n) / dot(lVec, n);

	if (t < 0.f || t > 1.f) {
		return false;
	}

	c = t * lVec + l.getPointA();
	return isWithinTriangle(c, v1, v2, v3);
}

/**
 * @brief checks if line segment intersects triangle
 * @param c the point of intersection if any
 * @param p the first point in the segment
 * @param q the second point in the segment
 * @param v1 the first vertex of the triangle
 * @param v2 the first vertex of the triangle
 * @param v3 the first vertex of the triangle
 * @return a bool which determines if the triangle and segment intersect
 */
bool Triangle::isSegmentIntersecting(vec3& c, const vec3& p, const vec3& q, const vec3& v1, const vec3& v2, const vec3& v3) {
	const vec3 n = getNormal(v1, v2, v3);
	const float d = dot(v1, n);

	const vec3 lVec = q - p;
	const float len2 = dot(lVec, lVec);

	// if the line degenerates to a point
	if (len2 < 1e-8f) {
		c = p;
		return isWithinTriangle(c, v1, v2, v3);
	}

	const float t = dot(v1 - p, n) / dot(lVec, n);

	if (t < 0.f || t > 1.f) {
		return false;
	}

	c = t * lVec + p;
	return isWithinTriangle(c, v1, v2, v3);
}

/**
 * @brief computes the closest points on a triangle and line segment
 *
 * @param c1 the point on the line segment closest to the triangle
 * @param c2 the point on the triangle closest to the line segment
 * @param l the line segment
 * @return the distance squared between the closest points
 */
float Triangle::getClosestPtSegmentTriangle(vec3& c1, vec3& c2, const Line& l) const {
	if (isSegmentIntersecting(c1, l)) {
		c2 = c1;
		return 0.f;
	}

	vector<pair<vec3, vec3>> pairs;
	// closest point between l and v1 -> v2 segment
	pairs.push_back(make_pair(vec3(), vec3()));
	l.getClosestPtSegmentSegment(pairs.back().second, pairs.back().first, v1, v2);

	// closest point between l and v2 -> v3 segment
	pairs.push_back(make_pair(vec3(), vec3()));
	l.getClosestPtSegmentSegment(pairs.back().second, pairs.back().first, v2, v3);

	// closest point between l and v3 -> v1 segment
	pairs.push_back(make_pair(vec3(), vec3()));
	l.getClosestPtSegmentSegment(pairs.back().second, pairs.back().first, v3, v1);

	// closest point from the first vertex in l to the plane containing the triangle
	const vec3 n = getNormal();
	const float d = dot(v1 , n);
	const vec3 pc = Plane::closestPtPointPlane(l.getPointA(), n, d);
	// check if that point is within the triangle
	if (isWithinTriangle(pc)) {
		pairs.push_back(make_pair(l.getPointA(), pc));
	}

	// closest point from the second vertex in l to the plane containing the triangle
	const vec3 qc = Plane::closestPtPointPlane(l.getPointB(), n, d);
	// check if that point is within the triangle
	if (isWithinTriangle(qc)) {
		pairs.push_back(make_pair(l.getPointB(), qc));
	}

	pair<vec3, vec3>& result = pairs[0];
	vec3 diff = result.second - result.first;
	float dist2 = dot(diff, diff);

	for (size_t i = 1; i < pairs.size(); i++) {
		diff = pairs[i].second - pairs[i].first;
		float currDist2 = dot(diff, diff);
		if (dist2 > currDist2) {
			dist2 = currDist2;
			result = pairs[i];
		}
	}

	c1 = result.first;
	c2 = result.second;
	return dist2;
}

/**
 * @brief computes the closest points on a triangle and line segment
 *
 * @param c1 the point on the line segment closest to the triangle
 * @param c2 the point on the triangle closest to the line segment
 * @param p the start point of the segment
 * @param q the end point of the segment
 * @return the distance squared between the closest points
 */
float Triangle::getClosestPtSegmentTriangle(vec3& c1, vec3& c2, const vec3& p, const vec3& q) const {
	if (isSegmentIntersecting(c1, p, q)) {
		c2 = c1;
		return 0.f;
	}

	vector<pair<vec3, vec3>> pairs;
	// closest point between l and v1 -> v2 segment
	pairs.push_back(make_pair(vec3(), vec3()));
	Line::getClosestPtSegmentSegment(pairs.back().second, pairs.back().first, p, q, v1, v2);

	// closest point between l and v2 -> v3 segment
	pairs.push_back(make_pair(vec3(), vec3()));
	Line::getClosestPtSegmentSegment(pairs.back().second, pairs.back().first, p, q, v2, v3);

	// closest point between l and v3 -> v1 segment
	pairs.push_back(make_pair(vec3(), vec3()));
	Line::getClosestPtSegmentSegment(pairs.back().second, pairs.back().first, p, q, v3, v1);

	// closest point from the first vertex in l to the plane containing the triangle
	const vec3 n = getNormal();
	const float d = dot(v1, n);
	const vec3 pc = Plane::closestPtPointPlane(p, n, d);
	// check if that point is within the triangle
	if (isWithinTriangle(pc)) {
		pairs.push_back(make_pair(p, pc));
	}

	// closest point from the second vertex in l to the plane containing the triangle
	const vec3 qc = Plane::closestPtPointPlane(q, n, d);
	// check if that point is within the triangle
	if (isWithinTriangle(qc)) {
		pairs.push_back(make_pair(q, qc));
	}

	pair<vec3, vec3>& result = pairs[0];
	vec3 diff = result.second - result.first;
	float dist2 = dot(diff, diff);

	for (size_t i = 1; i < pairs.size(); i++) {
		diff = pairs[i].second - pairs[i].first;
		float currDist2 = dot(diff, diff);
		if (dist2 > currDist2) {
			dist2 = currDist2;
			result = pairs[i];
		}
	}

	c1 = result.first;
	c2 = result.second;
	return dist2;
}

/**
 * @brief computes the closest points on a triangle and line segment
 *
 * @param c1 the point on the line segment closest to the triangle
 * @param c2 the point on the triangle closest to the line segment
 * @param l the line segment
 * @param v1 the first vertex of the triangle
 * @param v2 the second vertex of the triangle
 * @param v3 the third vertex of the triangle
 * @return the distance squared between the closest points
 */
float Triangle::getClosestPtSegmentTriangle(vec3& c1, vec3& c2, const Line& l, const vec3& v1, const vec3& v2, const vec3& v3) {
	if (isSegmentIntersecting(c1, l, v1, v2, v3)) {
		c2 = c1;
		return 0.f;
	}

	vector<pair<vec3, vec3>> pairs;
	// closest point between l and v1 -> v2 segment
	pairs.push_back(make_pair(vec3(), vec3()));
	l.getClosestPtSegmentSegment(pairs.back().second, pairs.back().first, v1, v2);

	// closest point between l and v2 -> v3 segment
	pairs.push_back(make_pair(vec3(), vec3()));
	l.getClosestPtSegmentSegment(pairs.back().second, pairs.back().first, v2, v3);

	// closest point between l and v3 -> v1 segment
	pairs.push_back(make_pair(vec3(), vec3()));
	l.getClosestPtSegmentSegment(pairs.back().second, pairs.back().first, v3, v1);

	// closest point from the first vertex in l to the plane containing the triangle
	const vec3 n = getNormal(v1, v2, v3);
	const float d = dot(v1, n);
	const vec3 pc = Plane::closestPtPointPlane(l.getPointA(), n, d);
	// check if that point is within the triangle
	if (isWithinTriangle(pc, v1, v2, v3)) {
		pairs.push_back(make_pair(l.getPointA() , pc));
	}

	// closest point from the second vertex in l to the plane containing the triangle
	const vec3 qc = Plane::closestPtPointPlane(l.getPointB(), n, d);
	// check if that point is within the triangle
	if (isWithinTriangle(qc, v1, v2, v3)) {
		pairs.push_back(make_pair(l.getPointB(), qc));
	}

	pair<vec3, vec3>& result = pairs[0];
	vec3 diff = result.second - result.first;
	float dist2 = dot(diff, diff);

	for (size_t i = 1; i < pairs.size(); i++) {
		diff = pairs[i].second - pairs[i].first;
		float currDist2 = dot(diff, diff);
		if (dist2 > currDist2) {
			dist2 = currDist2;
			result = pairs[i];
		}
	}

	c1 = result.first;
	c2 = result.second;
	return dist2;
}

/**
* @brief computes the closest points on a triangle and line segment
*
* @param c1 the point on the line segment closest to the triangle
* @param c2 the point on the triangle closest to the line segment
* @param p the start point of the segment
* @param q the end point of the segment
* @param v1 the first vertex of the triangle
* @param v2 the second vertex of the triangle
* @param v3 the third vertex of the triangle
* @return the distance squared between the closest points
*/
float Triangle::getClosestPtSegmentTriangle(vec3& c1, vec3& c2, const vec3& p, const vec3& q, const vec3& v1, const vec3& v2, const vec3& v3) {
	if (isSegmentIntersecting(c1, p, q, v1, v2, v3)) {
		c2 = c1;
		return 0.f;
	}

	vector<pair<vec3, vec3>> pairs;
	// closest point between l and v1 -> v2 segment
	pairs.push_back(make_pair(vec3(), vec3()));
	Line::getClosestPtSegmentSegment(pairs.back().second, pairs.back().first, p, q, v1, v2);

	// closest point between l and v2 -> v3 segment
	pairs.push_back(make_pair(vec3(), vec3()));
	Line::getClosestPtSegmentSegment(pairs.back().second, pairs.back().first, p, q, v2, v3);

	// closest point between l and v3 -> v1 segment
	pairs.push_back(make_pair(vec3(), vec3()));
	Line::getClosestPtSegmentSegment(pairs.back().second, pairs.back().first, p, q, v3, v1);

	// closest point from the first vertex in l to the plane containing the triangle
	const vec3 n = getNormal(v1, v2, v3);
	const float d = dot(v1, n);
	const vec3 pc = Plane::closestPtPointPlane(p, n, d);
	// check if that point is within the triangle
	if (isWithinTriangle(pc, v1, v2, v3)) {
		pairs.push_back(make_pair(p, pc));
	}

	// closest point from the second vertex in l to the plane containing the triangle
	const vec3 qc = Plane::closestPtPointPlane(q, n, d);
	// check if that point is within the triangle
	if (isWithinTriangle(qc, v1, v2, v3)) {
		pairs.push_back(make_pair(q, qc));
	}

	pair<vec3, vec3>& result = pairs[0];
	vec3 diff = result.second - result.first;
	float dist2 = dot(diff, diff);

	for (size_t i = 1; i < pairs.size(); i++) {
		diff = pairs[i].second - pairs[i].first;
		float currDist2 = dot(diff, diff);
		if (dist2 > currDist2) {
			dist2 = currDist2;
			result = pairs[i];
		}
	}

	c1 = result.first;
	c2 = result.second;
	return dist2;
}