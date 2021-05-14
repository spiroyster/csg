/*
	MIT License

	Copyright (c) 2020 Cordell Barron

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

*/
#ifndef CSG_HPP
#define CSG_HPP

#include <algorithm>
#include <array>
#include <assert.h>
#include <cmath>
#include <sstream>
#include <limits>
#include <list>
#include <memory>
#include <vector>

namespace csg
{
	struct vertex { vertex() : x_(0), y_(0), z_(0) {} vertex(double x, double y, double z) : x_(x), y_(y), z_(z) {} double x_, y_, z_; };
	struct triangle { triangle(const vertex& a, const vertex& b, const vertex& c) : a_(a), b_(b), c_(c) {} vertex a_, b_, c_; };
	typedef std::vector<triangle> mesh;
	typedef std::pair<unsigned int, unsigned int> edge;
	typedef std::pair<vertex, vertex> segment;

	namespace impl
	{
		const static double pi = 3.14159265358979323846264338327950288;
		const static double superTriangleScaling = 2000.0;
		const static double defaultTolerance = 0.01;

		// vertex operations...
		static double magnitudeSquared(const vertex& v) { return (v.x_ * v.x_) + (v.y_ * v.y_) + (v.z_ * v.z_); }
		static double magnitude(const vertex& v) { return sqrt((v.x_ * v.x_) + (v.y_ * v.y_) + (v.z_ * v.z_)); }
		static vertex unitise(const vertex& v) { double oneOverMagnitude = 1.0 / magnitude(v); return vertex(v.x_ * oneOverMagnitude, v.y_ * oneOverMagnitude, v.z_ * oneOverMagnitude); }
		static vertex cross(const vertex& a, const vertex& b) { return vertex((a.y_ * b.z_) - (a.z_ * b.y_), (a.z_ * b.x_) - (a.x_ * b.z_), (a.x_ * b.y_) - (a.y_ * b.x_)); }
		static double dot(const vertex& a, const vertex& b) { return (a.x_ * b.x_) + (a.y_ * b.y_) + (a.z_ * b.z_); }
		static bool equals(const vertex& a, const vertex& b, double tolerance) { return (abs(a.x_ - b.x_) < tolerance && abs(a.y_ - b.y_) < tolerance && abs(a.z_ - b.z_) < tolerance); }
		static bool equals(const vertex& a, const vertex& b) { return (a.x_ == b.x_ && a.y_ == b.y_ && a.z_ == b.z_); }
		static vertex add(const vertex& a, const vertex& b) { return vertex(a.x_ + b.x_, a.y_ + b.y_, a.z_ + b.z_); }
		static vertex subtract(const vertex& a, const vertex& b) { return vertex(a.x_ - b.x_, a.y_ - b.y_, a.z_ - b.z_); }
		static vertex scale(const vertex& a, double s) { return vertex(a.x_ * s, a.y_ * s, a.z_ * s); }
		static vertex normal(const triangle& t) { return unitise(cross(subtract(t.b_, t.a_), subtract(t.c_, t.a_))); }
		static vertex rayPlaneIntersection(const vertex& rP, const vertex& rD, const vertex& pP, const vertex& pN, double dotrDpN) { double s = dot(subtract(pP, rP), pN) / dotrDpN; return vertex(rP.x_ + (rD.x_ * s), rP.y_ + (rD.y_ * s), rP.z_ + (rD.z_ * s)); }
		static double distanceFromPlane(const vertex& pP, const vertex& pN, const vertex& v, double tolerance = 0.001) { double d = dot(pN, (subtract(v, pP))); return abs(d) < tolerance ? 0 : d; }
		static vertex centroid(const vertex& a, const vertex& b, const vertex& c) { return vertex((a.x_ + b.x_ + c.x_) / 3, (a.y_ + b.y_ + c.y_) / 3, (a.z_ + b.z_ + c.z_) / 3); }
		static bool sameSide(const vertex& p1, const vertex& p2, const vertex& a, const vertex& b) { return dot(cross(subtract(b, a), subtract(p1, a)), cross(subtract(b, a), subtract(p2, a))) >= 0; }
		static bool pointWithinTriangle(const vertex& a, const vertex& b, const vertex& c, const vertex& p) { return sameSide(p, a, b, c) && sameSide(p, b, a, c) && sameSide(p, c, a, b); }
		static bool pointOnLineSegment(const vertex& i, const vertex& j, const vertex& p, double tolerance) { return abs(magnitude(subtract(p, i)) + magnitude(subtract(p, j)) - magnitude(subtract(i, j))) < tolerance; }
		static vertex midPoint(const vertex& i, const vertex& j) { return vertex((i.x_ + j.x_) * 0.5, (i.y_ + j.y_) * 0.5, (i.z_ + j.z_) * 0.5); }
		static double arcCosine(double val) { if (val <= -1.0) { return pi; } if (val >= 1.0) { return 0; } return acos(val); }
		static vertex rodrigues(const vertex& v, const vertex& axis, double angle)
		{
			double c = cos(angle);
			vertex vRot(v.x_ * c, v.y_ * c, v.z_ * c);
			vRot = add(scale(cross(axis, v), sin(angle)), vRot);
			return add(scale(axis, dot(axis, v) * (1 - c)), vRot);
		}
		static vertex closest(const triangle& t, const vertex& v)
		{
			double magA = magnitudeSquared(subtract(t.a_, v));
			double magB = magnitudeSquared(subtract(t.b_, v));
			double magC = magnitudeSquared(subtract(t.c_, v));

			if (magA < magB && magA < magC)
				return t.a_;
			else if (magB < magA && magB < magC)
				return t.b_;
			else
				return t.c_;
		}
		
		namespace ioop
		{
			enum ioop { inside, outside, onPlane };
		}
		
		namespace sanitise
		{
			bool isValid(const triangle& t, double tolerance) { return (!equals(t.a_, t.b_, tolerance) && !equals(t.a_, t.c_, tolerance) && !equals(t.b_, t.c_, tolerance) && !std::isnan(normal(t).x_)); }
		}

		namespace intersection
		{
			struct splitResult { std::vector<triangle> outside_; std::vector<triangle> inside_; vertex i_, j_; };

			// triangle/plane intersection...
			class trianglePlane
			{
			public:
				trianglePlane(const triangle* T, const vertex& p, const vertex& n, double tolerance)
					: normal_(n), tolerance_(tolerance), A_(*T), isClamped_(false)
				{
					directions_.x_ = distanceFromPlane(p, n, T->a_);
					directions_.y_ = distanceFromPlane(p, n, T->b_);
					directions_.z_ = distanceFromPlane(p, n, T->c_);

					// if a_ close to plane, clamp...
					if (abs(directions_.x_) < tolerance_)
					{
						A_.a_ = add(A_.a_, scale(n, -directions_.x_));
						directions_.x_ = 0;
						isClamped_ = true;
					}
					if (abs(directions_.y_) < tolerance_)
					{
						A_.b_ = add(A_.b_, scale(n, -directions_.y_));
						directions_.y_ = 0;
						isClamped_ = true;
					}
					if (abs(directions_.z_) < tolerance_)
					{
						A_.b_ = add(A_.b_, scale(n, -directions_.z_));
						directions_.z_ = 0;
						isClamped_ = true;
					}
				}

				bool entirelyOutside() const { return directions_.x_ > 0 && directions_.y_ > 0 && directions_.z_ > 0; }
				bool outside() const { return directions_.x_ >= 0 && directions_.y_ >= 0 && directions_.z_ >= 0; }
				bool entirelyInside() const { return directions_.x_ < 0 && directions_.y_ < 0 && directions_.z_ < 0; }
				bool inside() const { return directions_.x_ <= 0 && directions_.y_ <= 0 && directions_.z_ <= 0; }
				bool onPlane() const { return !directions_.x_ && !directions_.y_ && !directions_.z_; }
				bool edgeABOnPlane() const { return !directions_.x_ && !directions_.y_ && directions_.z_; }
				bool edgeBCOnPlane() const { return directions_.x_ && !directions_.y_ && !directions_.z_; }
				bool edgeCAOnPlane() const { return !directions_.x_ && directions_.y_ && !directions_.z_; }
				bool majorABminorC() const { return (directions_.x_ > 0 && directions_.y_ > 0 && directions_.z_ < 0) || (directions_.x_ < 0 && directions_.y_ < 0 && directions_.z_ > 0); }
				bool majorBCminorA() const { return (directions_.x_ < 0 && directions_.y_ > 0 && directions_.z_ > 0) || (directions_.x_ > 0 && directions_.y_ < 0 && directions_.z_ < 0); }
				bool majorCAminorB() const { return (directions_.x_ < 0 && directions_.y_ > 0 && directions_.z_ < 0) || (directions_.x_ > 0 && directions_.y_ < 0 && directions_.z_ > 0); }
				bool straddling() const
				{
					return (!directions_.x_ && ((directions_.y_ > 0 && directions_.z_ < 0) || (directions_.y_ < 0 && directions_.z_ > 0))) ||
						(!directions_.y_ && ((directions_.x_ > 0 && directions_.z_ < 0) || (directions_.x_ < 0 && directions_.z_ > 0))) ||
						(!directions_.z_ && ((directions_.y_ > 0 && directions_.x_ < 0) || (directions_.y_ < 0 && directions_.x_ > 0)));
				}
				double a() const { return directions_.x_; }
				double b() const { return directions_.y_; }
				double c() const { return directions_.z_; }
				const vertex& n() const { return normal_; }

				bool IsClamped() const { return isClamped_; }
				double Tolerance() const { return tolerance_; }
				const triangle& ClampedTriangle() const { return A_; }

				/*segment IntersectionEdge() const
				{

				}*/

			protected:
				vertex directions_;
				vertex normal_;
				triangle A_;

				bool isClamped_;		// true if we clamped the triangle A_ to the plane in any way.
				double tolerance_;
			};

			vertex segmentPlaneIntersection(const vertex& i, const vertex& j, const vertex& pP, const vertex& pN)
			{
				vertex ijUnitised = unitise(subtract(j, i));
				double dotrDpN = dot(ijUnitised, pN);
				if (!std::isnan(dotrDpN) && dotrDpN)
					return rayPlaneIntersection(i, ijUnitised, pP, pN, dotrDpN);
				else
					throw std::runtime_error("no edge plane intersection.");
			}

			void validateSplitTriangles(splitResult& result, const triangle& originalTriangle, double tolerance)
			{
				splitResult validTriangles;
				std::for_each(result.inside_.begin(), result.inside_.end(), [&validTriangles, &tolerance](triangle& t) { if (sanitise::isValid(t, tolerance)) { validTriangles.inside_.push_back(t); } });
				std::for_each(result.outside_.begin(), result.outside_.end(), [&validTriangles, &tolerance](triangle& t) { if (sanitise::isValid(t, tolerance)) { validTriangles.outside_.push_back(t); } });
				result.inside_.swap(validTriangles.inside_);
				result.outside_.swap(validTriangles.outside_);
			}

			splitResult splitTriangle(const trianglePlane& intersectionInfo, const triangle& t, const vertex& tN, const vertex& pP, const vertex& pN)
			{
				assert(!intersectionInfo.entirelyInside());
				assert(!intersectionInfo.entirelyOutside());
				assert(!intersectionInfo.onPlane());
				assert(!intersectionInfo.edgeABOnPlane());
				assert(!intersectionInfo.edgeBCOnPlane());
				assert(!intersectionInfo.edgeCAOnPlane());

				splitResult result;
				vertex& i = result.i_;
				vertex& j = result.j_;
				if (intersectionInfo.majorABminorC())
				{
					i = segmentPlaneIntersection(t.a_, t.c_, pP, pN);
					j = segmentPlaneIntersection(t.b_, t.c_, pP, pN);
					if (distanceFromPlane(pP, pN, t.a_) > 0)
					{
						result.outside_.push_back(triangle(t.a_, i, t.b_));
						result.outside_.push_back(triangle(i, j, t.b_));
						result.inside_.push_back(triangle(t.c_, j, i));
					}
					else
					{
						result.inside_.push_back(triangle(t.a_, i, t.b_));
						result.inside_.push_back(triangle(i, j, t.b_));
						result.outside_.push_back(triangle(t.c_, j, i));
					}
				}
				else if (intersectionInfo.majorBCminorA())
				{
					i = segmentPlaneIntersection(t.b_, t.a_, pP, pN);
					j = segmentPlaneIntersection(t.c_, t.a_, pP, pN);
					if (distanceFromPlane(pP, pN, t.b_) > 0)
					{
						result.outside_.push_back(triangle(t.b_, j, i));
						result.outside_.push_back(triangle(t.c_, j, t.b_));
						result.inside_.push_back(triangle(t.a_, i, j));
					}
					else
					{
						result.inside_.push_back(triangle(t.b_, j, i));
						result.inside_.push_back(triangle(t.c_, j, t.b_));
						result.outside_.push_back(triangle(t.a_, i, j));
					}
				}
				else if (intersectionInfo.majorCAminorB())
				{
					i = segmentPlaneIntersection(t.a_, t.b_, pP, pN);
					j = segmentPlaneIntersection(t.c_, t.b_, pP, pN);
					if (distanceFromPlane(pP, pN, t.a_) > 0)
					{
						result.outside_.push_back(triangle(t.a_, i, t.c_));
						result.outside_.push_back(triangle(i, j, t.c_));
						result.inside_.push_back(triangle(t.b_, j, i));
					}
					else
					{
						result.inside_.push_back(triangle(t.a_, i, t.c_));
						result.inside_.push_back(triangle(i, j, t.c_));
						result.outside_.push_back(triangle(t.b_, j, i));
					}
				}
				else if (intersectionInfo.straddling())
				{
					if (!intersectionInfo.a())
					{
						i = t.a_;
						j = segmentPlaneIntersection(t.c_, t.b_, pP, pN);
						result.inside_.push_back(triangle(i, t.b_, j));
						result.outside_.push_back(triangle(j, t.c_, i));
					}
					else if (!intersectionInfo.b())
					{
						i = t.b_;
						j = segmentPlaneIntersection(t.c_, t.a_, pP, pN);
						result.inside_.push_back(triangle(i, t.c_, j));
						result.outside_.push_back(triangle(j, t.a_, i));
					}
					else if (!intersectionInfo.c())
					{
						i = t.c_;
						j = segmentPlaneIntersection(t.a_, t.b_, pP, pN);
						result.inside_.push_back(triangle(i, t.a_, j));
						result.outside_.push_back(triangle(j, t.b_, i));
					}
				}
				else
					throw std::runtime_error("cannot split triangle.");

				// assert the correct winding...
				std::for_each(result.inside_.begin(), result.inside_.end(), [&tN, &intersectionInfo](triangle& t)
					{
						vertex nn = impl::normal(t);
						if (!equals(nn, tN, intersectionInfo.Tolerance()))
							impl::flip(t);
					});
				std::for_each(result.outside_.begin(), result.outside_.end(), [&tN, &intersectionInfo](triangle& t)
					{
						vertex nn = impl::normal(t);
						if (!equals(nn, tN, intersectionInfo.Tolerance()))
							impl::flip(t);
					});

				validateSplitTriangles(result, t, intersectionInfo.Tolerance());
				return result;
			}

			// triangle/triangle intersection...
			class triangleTriangle : public trianglePlane
			{
			public:
				triangleTriangle(const triangle& A, const triangle& B, const vertex& Bnormal, double tolerance)
					: trianglePlane(&A, B.a_, Bnormal, tolerance), B_(&B)
				{
				}

				bool intersects() const { return !(entirelyInside() || entirelyOutside()); }

				bool calculateIntersection(segment& result) const
				{
					if (edgeABOnPlane()) { return calculateIntersectionEdge(A_.a_, A_.b_, result); }
					if (edgeBCOnPlane()) { return calculateIntersectionEdge(A_.b_, A_.c_, result); }
					if (edgeCAOnPlane()) { return calculateIntersectionEdge(A_.c_, A_.a_, result); }

					if (outside() || inside())
						return false;

					splitResult split = splitTriangle(*this, A_, normal(A_), B_->a_, normal_);
					return calculateIntersectionEdge(split.i_, split.j_, result);
				}

				bool calculateCoplanarIntersection(std::pair<std::list<segment>, std::list<segment>>& result) const
				{
					segment intersectingSegment;
					if (calculateIntersectionEdge(A_.a_, A_.b_, intersectingSegment)) { result.second.push_back(intersectingSegment); }
					if (calculateIntersectionEdge(A_.b_, A_.c_, intersectingSegment)) { result.second.push_back(intersectingSegment); }
					if (calculateIntersectionEdge(A_.c_, A_.a_, intersectingSegment)) { result.second.push_back(intersectingSegment); }

					triangleTriangle ba(*B_, A_, normal(A_), tolerance_);
					if (ba.calculateIntersectionEdge(B_->a_, B_->b_, intersectingSegment)) { result.first.push_back(intersectingSegment); }
					if (ba.calculateIntersectionEdge(B_->b_, B_->c_, intersectingSegment)) { result.first.push_back(intersectingSegment); }
					if (ba.calculateIntersectionEdge(B_->c_, B_->a_, intersectingSegment)) { result.first.push_back(intersectingSegment); }

					return !result.first.empty() || !result.second.empty();
				}


			private:
				bool calculateIntersectionEdge(const vertex& i, const vertex& j, segment& result) const
				{
					bool iInB = pointWithinTriangle(B_->a_, B_->b_, B_->c_, i);
					bool jInB = pointWithinTriangle(B_->a_, B_->b_, B_->c_, j);

					// if both intersection points are within B, this is the segment...
					if (iInB && jInB)
						result = segment(i, j);
					else
					{
						std::list<vertex> triangleEdgeSegmentIntersections = calculateTriangleEdgesSegmentIntersection(*B_, i, j);

						assert(triangleEdgeSegmentIntersections.size() < 3);
						
						if (triangleEdgeSegmentIntersections.empty())
						{
							if (iInB)
								result = segment(i, i);
							else if (jInB)
								result = segment(j, j);
							return iInB || jInB;
						}
							
						// If only single intersection...
						else if (triangleEdgeSegmentIntersections.size() == 1)
						{
							// if either are in B, process this... otherwise neither in B and intersection should be euqal to a triangle vertex.
							if (iInB || jInB)
								result = segment(triangleEdgeSegmentIntersections.front(), iInB ? i : j);
							else
								return false;
						}
						else
							result = segment(triangleEdgeSegmentIntersections.front(), triangleEdgeSegmentIntersections.back());
					}
					return true;
				}
				std::list<vertex> calculateTriangleEdgesSegmentIntersection(const triangle& t, const vertex& i, const vertex& j) const
				{
					std::list<vertex> intersections;

					vertex ij = unitise(subtract(j, i));

					// ab edge...
					{
						vertex ab = subtract(t.b_, t.a_);
						vertex abNormal = cross(ab, normal_);
						double abDotrDpN = dot(ij, abNormal);
						if (!std::isnan(abDotrDpN) && abDotrDpN)
						{
							vertex ip = rayPlaneIntersection(i, ij, t.a_, abNormal, abDotrDpN);
							if (pointOnLineSegment(i, j, ip, tolerance_) && pointOnLineSegment(t.a_, t.b_, ip, tolerance_))
								intersections.push_back(ip);
						}

					}
					// bc edge...
					{
						vertex bc = subtract(t.c_, t.b_);
						vertex bcNormal = cross(bc, normal_);
						double bcDotrDpN = dot(ij, bcNormal);
						if (!std::isnan(bcDotrDpN) && bcDotrDpN)
						{
							vertex ip = rayPlaneIntersection(i, ij, t.b_, bcNormal, bcDotrDpN);
							if (pointOnLineSegment(i, j, ip, tolerance_) && pointOnLineSegment(t.b_, t.c_, ip, tolerance_) 
								&& std::find_if(intersections.begin(), intersections.end(), [&ip, this](vertex& iv) { return equals(iv, ip, Tolerance()); }) == intersections.end())		// check intersection point is not already present...
								intersections.push_back(ip);
						}
					}
					// ca edge...
					{
						vertex ca = subtract(t.a_, t.c_);
						vertex caNormal = cross(ca, normal_);
						double caDotrDpN = dot(ij, caNormal);
						if (!std::isnan(caDotrDpN) && caDotrDpN)
						{
							vertex ip = rayPlaneIntersection(i, ij, t.c_, caNormal, caDotrDpN);
							if (pointOnLineSegment(i, j, ip, tolerance_) && pointOnLineSegment(t.c_, t.a_, ip, tolerance_)
								&& std::find_if(intersections.begin(), intersections.end(), [&ip, this](vertex& iv) { return equals(iv, ip, Tolerance()); }) == intersections.end())		// check intersection point is not already present...
								intersections.push_back(ip);
						}
					}

					// Impossible to have 3! triangle must be really small so take the two triangle vertices closest to the edge points...
					if (intersections.size() > 2)
						return std::list<vertex>({ closest(t, i), closest(t, j) });

					return intersections;
				}

				const triangle* B_;
			};
		}

		class bsp
		{
		public:
			class node : public triangle
			{
			public:
				node(const triangle& t) : triangle(t.a_, t.b_, t.c_), normal_(normal(t))
				{
					if (std::isnan(normal_.x_) || std::isnan(normal_.y_) || std::isnan(normal_.z_))
						throw std::runtime_error("unable to calculate bsp node normal.");
				}
				vertex triangleIntersection(const triangle& t) const { return vertex(distanceFromPlane(p(), n(), t.a_), distanceFromPlane(p(), n(), t.b_), distanceFromPlane(p(), n(), t.c_)); }
				vertex calculateIntersection(const vertex& a, const vertex& b) { vertex rD = unitise(subtract(b, a)); return rayPlaneIntersection(a, rD, a_, normal_, dot(rD, normal_)); }
				const vertex& p() const { return a_; }
				const vertex& n() const { return normal_; }
				std::unique_ptr<node> inside_;
				std::unique_ptr<node> outside_;
			private:
				vertex normal_;
			};

			// assume all triangles are well-formed...
			bsp(const mesh& m, double tolerance) { construct(m, tolerance); }
			
			ioop::ioop ioop(const vertex& v, double onPlaneTolerance) const
			{
				unsigned int debugNum = 0;

				node* currentNode = root_.get();
				while (currentNode)
				{
					// first we check if on the plane... project point onto plane if distance to plane is < tol.
					double d = distanceFromPlane(currentNode->p(), currentNode->n(), v, onPlaneTolerance);

					if (d == 0)
					{
						if (pointWithinTriangle(currentNode->a_, currentNode->b_, currentNode->c_, v)) { return ioop::ioop::onPlane; }
						else if (currentNode->inside_.get()) { currentNode = currentNode->inside_.get(); }
						else { return ioop::ioop::outside; }
					}
					else if (d > 0)
					{
						if (currentNode->outside_) { currentNode = currentNode->outside_.get(); }
						else { return ioop::ioop::outside; }
					}
					else if (d < 0)
					{
						if (currentNode->inside_) { currentNode = currentNode->inside_.get(); }
						else { return ioop::ioop::inside; }
					}

					++debugNum;
				}
				throw std::runtime_error("cannot deduce if point is inside or outside.");
			}

		private:
			void construct(const mesh& m, double tolerance)
			{
				// create a working list of the triangles. We validate as we go along.
				std::list<triangle> workingTriangles(m.begin(), m.end());
				while (!workingTriangles.empty() && !sanitise::isValid(workingTriangles.front(), tolerance))
					workingTriangles.pop_front();

				if (workingTriangles.empty())
					return;

				root_.reset(new node(workingTriangles.front()));
				while (!workingTriangles.empty())
				{
					std::list<triangle>::iterator currentTriangle = workingTriangles.begin();
					if (!sanitise::isValid(*currentTriangle, tolerance))
					{
						workingTriangles.erase(currentTriangle);
						continue;
					}

					node* currentNode = root_.get();
					while (currentNode)
					{
						intersection::trianglePlane intersectionInfo(&(*currentTriangle), currentNode->p(), currentNode->n(), tolerance);

						const triangle* workingTriangle = intersectionInfo.IsClamped() ? &intersectionInfo.ClampedTriangle() : &(*currentTriangle);
						
						if (intersectionInfo.onPlane())
							currentNode = followInsideNode(currentNode, *workingTriangle);
						else if (intersectionInfo.outside())
							currentNode = followOutsideNode(currentNode, *workingTriangle);
						else if (intersectionInfo.inside())
							currentNode = followInsideNode(currentNode, *workingTriangle);
						else
						{
							intersection::splitResult splitTriangles = intersection::splitTriangle(intersectionInfo, *workingTriangle, normal(*workingTriangle), currentNode->p(), currentNode->n());
							workingTriangles.insert(workingTriangles.begin(), splitTriangles.inside_.begin(), splitTriangles.inside_.end());
							workingTriangles.insert(workingTriangles.begin(), splitTriangles.outside_.begin(), splitTriangles.outside_.end());
							currentNode = 0;
						}
						if (!currentNode)
							workingTriangles.erase(currentTriangle);
					}
				}
			}
			node* followOutsideNode(node* nde, const triangle& tri)
			{
				if (nde->outside_)
					return nde->outside_.get();
				nde->outside_.reset(new node(tri));
				return 0;
			}
			node* followInsideNode(node* nde, const triangle& tri)
			{
				if (nde->inside_)
					return nde->inside_.get();
				nde->inside_.reset(new node(tri));
				return 0;
			}
			std::unique_ptr<node> root_;
		};

		struct mergeIntersectResult { std::list<triangle> inside_, outside_, onPlane_; };

		namespace tessellate
		{
			struct vector2 { vector2() : x_(0), y_(0) {} vector2(double x, double y) : x_(x), y_(y) {} double x_, y_; };
			struct edge { unsigned int i_, j_; };
			struct triangle { unsigned int a_, b_, c_; };

			class projection
			{
			public:
				projection(const vertex& normal, const vertex& origin) : origin_(origin), angle_(0), axis_(0, 0, 1.0)
				{
					if (!equals(normal, axis_) && !equals(vertex(-normal.x_, -normal.y_, -normal.z_), axis_))
					{
						axis_ = unitise(cross(normal, vertex(0, 0, 1.0)));
						angle_ = arcCosine(dot(normal, vertex(0, 0, 1.0)));
					}
				}

				tessellate::vector2 get2D(const vertex& v) { vertex rotated = rodrigues(subtract(v, origin_), axis_, angle_); return vector2{ rotated.x_, rotated.y_ }; }
				vertex get3D(const double& x, const double& y) { return add(origin_, rodrigues(vertex(x, y, 0), axis_, -angle_)); }
				vertex get3D(const tessellate::vector2& v) { return add(origin_, rodrigues(vertex(v.x_, v.y_, 0), axis_, -angle_)); }
			private:
				vertex axis_, origin_;
				double angle_;
			};

			const static double pi = 3.14159265358979323846;

			class delaunay
			{
				struct workingVertex
				{
					workingVertex(const vector2* v, unsigned int index)
						: v_(v), index_(index)
					{
					}

					double dist(const vector2 &v) const
					{
						const double dx = v_->x_ - v.x_;
						const double dy = v_->y_ - v.y_;
						return dx * dx + dy * dy;
					}

					const vector2* v_;
					unsigned int index_;
				};

				static bool orEquals(unsigned int a, unsigned int b, unsigned int c, unsigned int equals)
				{
					return (a == equals || b == equals || c == equals);
				}

				struct workingTriangle
				{
					workingTriangle(const workingVertex* a, const workingVertex* b, const workingVertex* c)
						: a_(a), b_(b), c_(c), valid_(true) {}

					bool containsVertex(unsigned int index)
					{
						return orEquals(a_->index_, b_->index_, c_->index_, index);
					}

					const workingVertex* a_;
					const workingVertex* b_;
					const workingVertex* c_;
					bool valid_;

					bool isWithinCircumcircle(const workingVertex& v) const
					{
						const double ab = a_->v_->x_ * a_->v_->x_ + a_->v_->y_ * a_->v_->y_;
						const double cd = b_->v_->x_ * b_->v_->x_ + b_->v_->y_ * b_->v_->y_;
						const double ef = c_->v_->x_ * c_->v_->x_ + c_->v_->y_ * c_->v_->y_;

						const double ax = a_->v_->x_;
						const double ay = a_->v_->y_;
						const double bx = b_->v_->x_;
						const double by = b_->v_->y_;
						const double cx = c_->v_->x_;
						const double cy = c_->v_->y_;

						const double circum_x = (ab * (cy - by) + cd * (ay - cy) + ef * (by - ay)) / (ax * (cy - by) + bx * (ay - cy) + cx * (by - ay));
						const double circum_y = (ab * (cx - bx) + cd * (ax - cx) + ef * (bx - ax)) / (ay * (cx - bx) + by * (ax - cx) + cy * (bx - ax));

						vector2 circum(circum_x * 0.5, circum_y * 0.5);
						return  v.dist(circum) <= a_->dist(circum);
					}
				};

				struct workingEdge
				{
					workingEdge(const workingVertex* i, const workingVertex* j)
						: i_(i), j_(j), valid_(true)
					{
					}

					bool operator==(const workingEdge& rhs)
					{
						return (rhs.i_ == i_ && rhs.j_ == j_) || (rhs.i_ == j_ && rhs.j_ == i_);
					}

					const workingVertex* i_;
					const workingVertex* j_;
					bool valid_;
				};

				double angleDifference(const vector2& a, const vector2& b)
				{
					double angle = atan2(b.y_, b.x_) - atan2(a.y_, a.x_);

					if (angle > pi)
						angle -= 2 * pi;
					else if (angle <= -pi)
						angle += 2 * pi;

					return angle;
				}

				bool segmentSegmentIntersection(const vector2& a, const vector2& b, const vector2& c, const vector2& d)
				{
					double s1_x, s1_y, s2_x, s2_y;
					s1_x = b.x_ - a.x_;     s1_y = b.y_ - a.y_;
					s2_x = d.x_ - c.x_;     s2_y = d.y_ - c.y_;

					double sDenom = (-s2_x * s1_y + s1_x * s2_y);
					double tDenom = (-s2_x * s1_y + s1_x * s2_y);

					if (!sDenom || !tDenom)
						return false;

					double s = (-s1_y * (a.x_ - c.x_) + s1_x * (a.y_ - c.y_)) / sDenom;
					double t = (s2_x * (a.y_ - c.y_) - s2_y * (a.x_ - c.x_)) / tDenom;

					return (s >= 0 && s <= 1 && t >= 0 && t <= 1);
				}


			public:
				delaunay(const std::vector<vector2>& points)
					: vertices_(points)
				{
	#ifdef NDEBUG
					std::vector<vector2> verticesCopy = points;
					convenience::validate(verticesCopy);
					assert(verticesCopy.size() == points.size());
	#endif

					std::vector<unsigned int> allVertexIndexes(points.size(), 0);
					for (unsigned int i = 0; i < points.size(); ++i)
						allVertexIndexes[i] = i;

					tessellate(allVertexIndexes);

					// Remove our duplicate edges...
					removeDuplicateEdges();
				}

				void addConstraint(const edge& constraint)
				{
					// assert the requested constraint indexes are valid...
					assert(constraint.i_ < vertices_.size() && constraint.j_ < vertices_.size());

					// Check if this edge is already present, if so do nothing...
					if (std::find_if(edges_.begin(), edges_.end(), [&constraint](const edge& ee) { return (constraint.i_ == ee.i_ && constraint.j_ == ee.j_) || (constraint.i_ == ee.j_ && constraint.j_ == ee.i_); }) != edges_.end())
						return;

					std::vector<unsigned int> leftSide, rightSide;

					// find out which edges this constraint straddles...
					for (auto e = edges_.begin(); e != edges_.end();)
					{
						// Don't check edges which share a constraint vertex...
						if (constraint.i_ != e->i_ && constraint.i_ != e->j_ && constraint.j_ != e->i_ && constraint.j_ != e->j_)
						{
							const vector2& ci = vertices_[constraint.i_];
							const vector2& cj = vertices_[constraint.j_];
							const vector2& ei = vertices_[e->i_];
							const vector2& ej = vertices_[e->j_];

							double iAngle = angleDifference(vector2(ei.x_ - ci.x_, ei.y_ - ci.y_), vector2(cj.x_ - ci.x_, cj.y_ - ci.y_));
							double jAngle = angleDifference(vector2(ej.x_ - ci.x_, ej.y_ - ci.y_), vector2(cj.x_ - ci.x_, cj.y_ - ci.y_));

							if ((iAngle < 0 && jAngle > 0) || (iAngle > 0 && jAngle < 0))
							{
								// constraint straddles an edge, so check for edge intersection...
								if (segmentSegmentIntersection(ei, ej, ci, cj))
								{
									leftSide.push_back(iAngle > 0 ? e->i_ : e->j_);
									rightSide.push_back(iAngle < 0 ? e->i_ : e->j_);

									// Remove any triangles using the edge, and the edge itself...
									triangles_.erase(std::remove_if(triangles_.begin(), triangles_.end(), [&e](triangle& t) { return orEquals(t.a_, t.b_, t.c_, e->i_) && orEquals(t.a_, t.b_, t.c_, e->j_); }), triangles_.end());
									e = edges_.erase(e);

									continue;
								}
							}
						}

						++e;
					}

					// add the constraint edge to both sides, and tessellate subsets...
					leftSide.push_back(constraint.i_);
					leftSide.push_back(constraint.j_);
					rightSide.push_back(constraint.i_);
					rightSide.push_back(constraint.j_);

					// remove duplicate indexes...
					std::sort(leftSide.begin(), leftSide.end());
					leftSide.erase(std::unique(leftSide.begin(), leftSide.end()), leftSide.end());
					std::sort(rightSide.begin(), rightSide.end());
					rightSide.erase(std::unique(rightSide.begin(), rightSide.end()), rightSide.end());

					// retessellate both the right and left side
					tessellate(leftSide);
					tessellate(rightSide);

					// Remove our duplicate edges...
					removeDuplicateEdges();
				}

				const std::list<triangle>& getTriangles() const { return triangles_; }
				const std::vector<vector2>& getVertices() const { return vertices_; }
				const std::list<edge>& getEdges() const { return edges_; }

			private:
				void tessellate(const std::vector<unsigned int>& vertexIndexes)
				{
	#ifdef NDEBUG
					assert(!vertexIndexes.empty());
					assert(!vertices_.empty());
					for (unsigned int vi = 0; vi < vertexIndexes.size(); ++vi)
						assert(vertexIndexes[vi] < vertices_.size());
					std::vector<unsigned int> vertexIndexesCopy = vertexIndexes;
					std::sort(vertexIndexesCopy.begin(), vertexIndexesCopy.end(), [](const unsigned int& a, const unsigned int& b) { return a < b; });
					assert(std::unique(vertexIndexesCopy.begin(), vertexIndexesCopy.end()) == vertexIndexesCopy.end())
	#endif

						// create our sublist that we tessellate to....
						std::vector<workingVertex> workingVertices(vertexIndexes.size() + 3, workingVertex(0, 0));
					for (unsigned int i = 0; i < vertexIndexes.size(); ++i)
						workingVertices[i] = workingVertex(&vertices_[vertexIndexes[i]], vertexIndexes[i]);

					// determine the super triangle...
					double minX = workingVertices[0].v_->x_;
					double minY = workingVertices[0].v_->y_;
					double maxX = minX;
					double maxY = minY;

					for (std::size_t i = 0; i < vertexIndexes.size(); ++i)
					{
						if (workingVertices[i].v_->x_ < minX) minX = workingVertices[i].v_->x_;
						if (workingVertices[i].v_->y_ < minY) minY = workingVertices[i].v_->y_;
						if (workingVertices[i].v_->x_ > maxX) maxX = workingVertices[i].v_->x_;
						if (workingVertices[i].v_->y_ > maxY) maxY = workingVertices[i].v_->y_;
					}

					double dx = maxX - minX;
					double dy = maxY - minY;
					double deltaMax = (std::max)(dx, dy);
					double midx = (minX + maxX) * 0.5;
					double midy = (minY + maxY) * 0.5;

					// Add our three super triangle vertices...
					unsigned int p1Index = static_cast<unsigned int>(vertices_.size()), p2Index = static_cast<unsigned int>(vertices_.size()) + 1, p3Index = static_cast<unsigned int>(vertices_.size()) + 2;

					vector2 p1(midx - superTriangleScaling * deltaMax, midy - deltaMax);
					vector2 p2(midx, midy + superTriangleScaling * deltaMax);
					vector2 p3(midx + superTriangleScaling * deltaMax, midy - deltaMax);

					workingVertices[vertexIndexes.size()] = workingVertex(&p1, p1Index);
					workingVertices[vertexIndexes.size() + 1] = workingVertex(&p2, p2Index);
					workingVertices[vertexIndexes.size() + 2] = workingVertex(&p3, p3Index);

					// Create a list of triangles, and add the supertriangle in it
					std::list<workingTriangle> workingTriangles;
					workingTriangles.push_back(workingTriangle(&workingVertices[vertexIndexes.size()], &workingVertices[vertexIndexes.size() + 1], &workingVertices[vertexIndexes.size() + 2]));

					std::for_each(workingVertices.begin(), workingVertices.end(),
						[&workingTriangles](workingVertex& v)
						{
							std::vector<workingEdge> polygon;

							std::for_each(workingTriangles.begin(), workingTriangles.end(), [&v, &polygon](workingTriangle& t)
								{
									if (t.isWithinCircumcircle(v))
									{
										t.valid_ = false;
										polygon.push_back(workingEdge(t.a_, t.b_));
										polygon.push_back(workingEdge(t.b_, t.c_));
										polygon.push_back(workingEdge(t.c_, t.a_));
									}
								});

							workingTriangles.erase(std::remove_if(workingTriangles.begin(), workingTriangles.end(), [](workingTriangle& t) { return !t.valid_; }), workingTriangles.end());

							for (auto e1 = polygon.begin(); e1 != polygon.end(); ++e1)
							{
								for (auto e2 = e1 + 1; e2 != polygon.end(); ++e2)
								{
									if (*e1 == *e2)
									{
										e1->valid_ = false;
										e2->valid_ = false;
									}
								}
							}

							polygon.erase(std::remove_if(polygon.begin(), polygon.end(), [](workingEdge& e) { return !e.valid_; }), polygon.end());

							std::for_each(polygon.begin(), polygon.end(), [&workingTriangles, &v](workingEdge& e) { workingTriangles.push_back(workingTriangle(e.i_, e.j_, &v)); });

						});

					// Remove the original super triangle...
					workingTriangles.erase(std::remove_if(workingTriangles.begin(), workingTriangles.end(),
						[&p1Index, &p2Index, &p3Index](workingTriangle& t)
						{
							return (t.containsVertex(p1Index) ||
								t.containsVertex(p2Index) ||
								t.containsVertex(p3Index));
						}), workingTriangles.end());

					// Create our edges (required for constraint functionality) and triangles...
					for (auto tItr = workingTriangles.begin(); tItr != workingTriangles.end(); ++tItr)
					{
						triangles_.push_back(triangle{ tItr->a_->index_, tItr->b_->index_, tItr->c_->index_ });
						edges_.push_back(edge{ tItr->a_->index_, tItr->b_->index_ });
						edges_.push_back(edge{ tItr->b_->index_, tItr->c_->index_ });
						edges_.push_back(edge{ tItr->c_->index_, tItr->a_->index_ });
					}
				}

				void removeDuplicateEdges()
				{
					std::for_each(edges_.begin(), edges_.end(),
						[](edge& e)
						{
							if (e.j_ < e.i_)
							{
								unsigned int j = e.j_;
								e.j_ = e.i_;
								e.i_ = j;
							}
						});
					edges_.sort([](edge& e, edge& ee)
						{
							return e.i_ == ee.i_ ? e.j_ < ee.j_ : e.i_ < ee.i_;
						});
					edges_.unique([](edge& e, edge& ee)
						{
							return (e.i_ == ee.i_ && e.j_ == ee.j_) || (e.i_ == ee.j_ && e.j_ == ee.i_);
						});
				}

				std::list<triangle> triangles_;
				std::list<edge> edges_;
				std::vector<vector2> vertices_;
			};

			bool equalsDelaunayVertex(const tessellate::vector2& a, const tessellate::vector2& b, double tolerance)
			{
				return abs(a.x_ - b.x_) < tolerance && abs(a.y_ - b.y_) < tolerance;
			}

			unsigned int addDelaunayVertex(std::vector<tessellate::vector2>& vertices, const tessellate::vector2& v, double tolerance)
			{
				for (unsigned int dv = 0; dv < vertices.size(); ++dv)
					if (equalsDelaunayVertex(vertices[dv], v, tolerance))
						return dv;

				vertices.push_back(v);
				return static_cast<unsigned int>(vertices.size() - 1);
			}

		}

		typedef std::pair<mergeIntersectResult, mergeIntersectResult> mergeResult;

		enum OnPlaneIncludePolicy
		{
			Always,
			Never,
			NormalsEqual,
			NormalsNotEqual
		};

		class workingCSGTriangle
		{
		public:
			workingCSGTriangle() : triangle_(0), isClamped_(false), isValid_(false) {}
			workingCSGTriangle(const csg::triangle* t, double tolerance) : triangle_(t), normal_(impl::normal(*t)), isClamped_(false), isValid_(impl::sanitise::isValid(*t, tolerance)) {}

			bool IsValid() const { return isValid_; }
			void SetClamped(bool clamped) { isClamped_ = clamped; }

			std::string debugIntersectionOutputName_;
			std::string debugTessellationOutputName_;
			std::string debugTessellationPointsOutputName_;

			void Retessellate(double tolerance)
			{
				if (intersections_.empty())
				{
					subTriangles_.push_back(*triangle_);
					return;
				}
					
				tessellate::projection projectionTo2D(normal_, triangle_->a_);

				std::vector<tessellate::vector2> vertices(3);
				std::vector<tessellate::edge> constraints;

				vertices[0] = projectionTo2D.get2D(triangle_->a_);
				vertices[1] = projectionTo2D.get2D(triangle_->b_);
				vertices[2] = projectionTo2D.get2D(triangle_->c_);
				for (std::list<segment>::iterator itr = intersections_.begin(); itr != intersections_.end(); ++itr)
					constraints.push_back(tessellate::edge{ addDelaunayVertex(vertices, projectionTo2D.get2D(itr->first), tolerance), addDelaunayVertex(vertices, projectionTo2D.get2D(itr->second), tolerance) });

				tessellate::delaunay d(vertices);
				for (unsigned int c = 0; c < constraints.size(); ++c)
				{
					if (constraints[c].i_ != constraints[c].j_)
						d.addConstraint(constraints[c]);
				}

				
				const std::list<tessellate::triangle>& triangles = d.getTriangles();
				std::for_each(triangles.begin(), triangles.end(), [this, &d, &projectionTo2D](const tessellate::triangle& t)
					{
						subTriangles_.push_back(csg::triangle(projectionTo2D.get3D(d.getVertices()[t.a_]), projectionTo2D.get3D(d.getVertices()[t.b_]), projectionTo2D.get3D(d.getVertices()[t.c_])));
					});

				// Assume CCW, ensure new tessellation is winding the correct way...
				for (unsigned int t = 0; t < subTriangles_.size(); ++t)
				{
					if (magnitude(subtract(normal(subTriangles_[t]), normal_)) > 0.5)
						flip(subTriangles_[t]);
				}

			}

			const vertex& TriangleNormal() const { return normal_; }
			void AddCoplanarTriangleIntersection(unsigned int triangleIndex) { coplanarIntersectingTriangles_.push_back(triangleIndex); }
			void AddIntersection(segment& intersectionEdge) { intersections_.push_back(intersectionEdge); }
			void AddIntersections(std::list<segment>& intersectionEdges) { intersections_.insert(intersections_.end(), intersectionEdges.begin(), intersectionEdges.end()); }

			void AddToResult(const bsp& bsp, mergeIntersectResult& result, const std::vector<workingCSGTriangle>& othertriangles, const OnPlaneIncludePolicy& onPlaneIncludePolicy, double onPlaneTolerance) const
			{
				if (!coplanarIntersectingTriangles_.empty())
				{
					// check if this triangle is within colpanar... if so check policy 
					for (unsigned int st = 0; st < subTriangles_.size(); ++st)
					{
						vertex tCentre = centroid(subTriangles_[st].a_, subTriangles_[st].b_, subTriangles_[st].c_);
						bool triangleAdded = false;
						for (auto ct = coplanarIntersectingTriangles_.begin(); ct != coplanarIntersectingTriangles_.end() && !triangleAdded; ++ct)
						{
							const workingCSGTriangle& coplanarTriangle = othertriangles[*ct];

							if (impl::pointWithinTriangle(coplanarTriangle.triangle_->a_, coplanarTriangle.triangle_->b_, coplanarTriangle.triangle_->c_, tCentre))
							{
								if (onPlaneIncludePolicy == OnPlaneIncludePolicy::Always)
								{
									#pragma omp critical
									{
										result.onPlane_.push_back(subTriangles_[st]);
									}
								}
								else if (onPlaneIncludePolicy != OnPlaneIncludePolicy::Never)
								{
									bool normalsAreEqual = impl::equals(normal_, coplanarTriangle.normal_, onPlaneTolerance);
									if ((normalsAreEqual && onPlaneIncludePolicy == OnPlaneIncludePolicy::NormalsEqual) || (!normalsAreEqual && onPlaneIncludePolicy == OnPlaneIncludePolicy::NormalsNotEqual))
									{
										#pragma omp critical
										{
											result.onPlane_.push_back(subTriangles_[st]);
										}
									}
								}
								triangleAdded = true;
							}
						}

						if (!triangleAdded)
							AddToResultNonCoplanar(bsp, result, subTriangles_[st], onPlaneTolerance);
					}
				}
				else
				{
					for (unsigned int st = 0; st < subTriangles_.size(); ++st)
						AddToResultNonCoplanar(bsp, result, subTriangles_[st], onPlaneTolerance);
				}
			}

		private:

			void AddToResultNonCoplanar(const bsp& bsp, mergeIntersectResult& result, const csg::triangle& tri, double onPlaneTolerance) const
			{
				// clamp back to correct plane?

				const ioop::ioop& ioopResult = bsp.ioop(centroid(tri.a_, tri.b_, tri.c_), onPlaneTolerance);

				#pragma omp critical
				{
					if (ioopResult == ioop::ioop::inside)
						result.inside_.push_back(tri);
					else if (ioopResult == ioop::ioop::outside)
						result.outside_.push_back(tri);
				}
			}

			const csg::triangle* triangle_;			// the original triangle...
			vertex normal_;							// the normal of the original triangle...
			bool isClamped_;						// true if any part of this triangle is clamped...
			bool isValid_;
			std::list<segment> intersections_;		// the intersections of this triangle...
			mesh subTriangles_;						// sub triangles
			std::list<unsigned int> coplanarIntersectingTriangles_;
		};

		void mergeAndIntersect(const mesh& A, const OnPlaneIncludePolicy& AOnPlanePolicy, const mesh& B, const OnPlaneIncludePolicy& BOnPlanePolicy, mergeResult& result, double tolerance)
		{
			std::vector<workingCSGTriangle> aTriangles(A.size(), workingCSGTriangle());
			std::vector<workingCSGTriangle> bTriangles(B.size(), workingCSGTriangle());

			// Populate...
			for (unsigned int a = 0; a < A.size(); ++a) { aTriangles[a] = workingCSGTriangle(&A[a], tolerance); }
			for (unsigned int b = 0; b < B.size(); ++b) { bTriangles[b] = workingCSGTriangle(&B[b], tolerance); }

			// Intersect...
			#pragma omp for
			for (int a = 0; a < static_cast<int>(aTriangles.size()); ++a)
			{
				if (!aTriangles[a].IsValid())
					continue;

				for (int b = 0; b < static_cast<int>(bTriangles.size()); ++b)
				{
					if (!bTriangles[b].IsValid())
						continue;

					intersection::triangleTriangle tt(B[b], A[a], aTriangles[a].TriangleNormal(), tolerance);
					bTriangles[b].SetClamped(tt.IsClamped());

					if (!tt.intersects())
						continue;
					else if (tt.onPlane())
					{
						std::pair<std::list<segment>, std::list<segment>> coplanarResult;
						if (tt.calculateCoplanarIntersection(coplanarResult))
						{
							#pragma omp critical
							{
								aTriangles[a].AddCoplanarTriangleIntersection(b);
								aTriangles[a].AddIntersections(coplanarResult.second);
								bTriangles[b].AddCoplanarTriangleIntersection(a);
								bTriangles[b].AddIntersections(coplanarResult.first);
							}
						}
					}
					else
					{
						segment intersectionEdge;
						if (tt.calculateIntersection(intersectionEdge))
						{
							#pragma omp critical
							{
								aTriangles[a].AddIntersection(intersectionEdge);
								bTriangles[b].AddIntersection(intersectionEdge);
							}
						}
					}
				}
			}

			// tessellate required triangles...
			for (int a = 0; a < static_cast<int>(aTriangles.size()); ++a)
				aTriangles[a].Retessellate(tolerance);	

			for (int b = 0; b < static_cast<int>(bTriangles.size()); ++b)
				bTriangles[b].Retessellate(tolerance);
			
			// deduce inside/outside
			bsp bspB(B, tolerance);
			for (int a = 0; a < static_cast<int>(aTriangles.size()); ++a)
				aTriangles[a].AddToResult(bspB, result.first, bTriangles, AOnPlanePolicy, tolerance);

			bsp bspA(A, tolerance);
			for (int b = 0; b < static_cast<int>(bTriangles.size()); ++b)
				bTriangles[b].AddToResult(bspA, result.second, aTriangles, BOnPlanePolicy, tolerance);

		}

	}

	

	std::shared_ptr<mesh> Difference(const mesh& a, const mesh& b, double tolerance = 0.01)
	{
		impl::mergeResult abMerge;
		impl::mergeAndIntersect(a, impl::OnPlaneIncludePolicy::NormalsNotEqual, b, impl::OnPlaneIncludePolicy::Never, abMerge, tolerance);

		std::shared_ptr<mesh> result(new mesh());
		result->insert(result->end(), abMerge.first.outside_.begin(), abMerge.first.outside_.end());
		std::for_each(abMerge.second.inside_.begin(), abMerge.second.inside_.end(), [](triangle& t) { impl::flip(t); });
		result->insert(result->end(), abMerge.second.inside_.begin(), abMerge.second.inside_.end());
		result->insert(result->end(), abMerge.first.onPlane_.begin(), abMerge.first.onPlane_.end());

		return result;
	}

	std::shared_ptr<mesh> Union(const mesh& a, const mesh& b, double tolerance = 0.01)
	{
		impl::mergeResult abMerge;
		impl::mergeAndIntersect(a, impl::OnPlaneIncludePolicy::NormalsEqual, b, impl::OnPlaneIncludePolicy::Never, abMerge, tolerance);

		std::shared_ptr<mesh> result(new mesh());
		result->insert(result->end(), abMerge.first.outside_.begin(), abMerge.first.outside_.end());
		result->insert(result->end(), abMerge.second.outside_.begin(), abMerge.second.outside_.end());

		return result;
	}

	std::shared_ptr<mesh> Intersection(const mesh& a, const mesh& b, double tolerance = 0.01)
	{
		impl::mergeResult abMerge;
		impl::mergeAndIntersect(a, impl::OnPlaneIncludePolicy::Never, b, impl::OnPlaneIncludePolicy::Never, abMerge, tolerance);

		std::shared_ptr<mesh> result(new mesh());
		result->insert(result->end(), abMerge.first.inside_.begin(), abMerge.first.inside_.end());
		result->insert(result->end(), abMerge.second.inside_.begin(), abMerge.second.inside_.end());

		return result;
	}

	namespace convenience
	{
		std::string obj(const mesh& m)
		{
			std::ostringstream oss;
			std::vector<std::array<unsigned int, 3>> faces;
			oss << "# obj file. https://github.com/spiroyster/csg\n";
			oss << "\n# points...\n"; 
			unsigned int n = 1;
			for (unsigned int tri = 0; tri < m.size(); ++tri, n += 3)
			{
				oss << "v " << m[tri].a_.x_ << " " << m[tri].a_.y_ << " " << m[tri].a_.z_ << '\n';
				oss << "v " << m[tri].b_.x_ << " " << m[tri].b_.y_ << " " << m[tri].b_.z_ << '\n';
				oss << "v " << m[tri].c_.x_ << " " << m[tri].c_.y_ << " " << m[tri].c_.z_ << '\n';
				faces.push_back({ { n, n + 1, n + 2 } });
			}
			oss << "\n# faces...\n";
			std::for_each(faces.begin(), faces.end(), [&oss](const std::array<unsigned int, 3>& f) { oss << "f " << std::get<0>(f) << " " << std::get<1>(f) << " " << std::get<2>(f) << "\n"; });
			return oss.str();
		}
	}
	
	//std::shared_ptr<mesh> HalfSpace(const mesh& m, const vertex& pP, const vertex& pN, double tolerance = 0.01)
	//{
	//	struct hsTriangle
	//	{
	//		hsTriangle() : triangle_(0) {}

	//		hsTriangle(const triangle* tri) : triangle_(tri) {}
	//		
	//		const triangle* triangle_;
	//		std::vector<triangle> splitTriangles_;
	//		vertex normal_;
	//	};

	//	std::vector<hsTriangle> hsTriangles(m.size(), hsTriangle());
	//	for (unsigned int t = 0; t < hsTriangles.size(); ++t) { hsTriangles[t] = hsTriangle(&m[t]); }

	//	#pragma omp for
	//	for (int t = 0; t < static_cast<int>(hsTriangles.size()); ++t)
	//	{
	//		if (!hsTriangles[t].IsValid())
	//			continue;

	//		impl::intersection::trianglePlane tp(&m[t], pP, pN, tolerance);

	//		if (tp.outside() || tp.inside() || tp.onPlane())
	//			continue;
	//		else
	//		{
	//			// Split the triangle, retain the split intersection points to cap the mesh.
	//			impl::intersection::splitResult split = impl::intersection::splitTriangle(tp, m[t], workingTriangles[t].TriangleNormal(), pP, pN);

	//			


	//			/*segment intersectionEdge;
	//			if (tp.calculateIntersection(intersectionEdge))
	//				workingTriangles[t].AddIntersection(intersectionEdge);*/
	//		}
	//	}

	//}

}

#endif // CSG_HPP