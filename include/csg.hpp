/*
	MIT License

	Copyright (c) 2019 Cordell Barron

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

#include <memory>
#include <vector>
#include <list>
#include <algorithm>
#include <limits>
#include <assert.h>

namespace csg
{
	struct vertex { vertex() : x_(0), y_(0), z_(0) {} vertex(double x, double y, double z) : x_(x), y_(y), z_(z) {} double x_, y_, z_; };
	struct triangle { triangle(const vertex& a, const vertex& b, const vertex& c) : a_(a), b_(b), c_(c) {} vertex a_, b_, c_; };
	typedef std::vector<triangle> mesh;
	typedef std::pair<unsigned int, unsigned int> edge;
	typedef std::pair<vertex, vertex> segment;

	static double defaultTolerance = 0.001;

	namespace impl
	{
		const static double pi = 3.14159265358979323846264338327950288;

		// vertex operations...
		static double magnitude(const vertex& v) { return sqrt((v.x_ * v.x_) + (v.y_ * v.y_) + (v.z_ * v.z_)); }
		static vertex unitise(const vertex& v) { double oneOverMagnitude = 1.0 / magnitude(v); return vertex(v.x_ * oneOverMagnitude, v.y_ * oneOverMagnitude, v.z_ * oneOverMagnitude); }
		static vertex cross(const vertex& a, const vertex& b) { return vertex((a.y_ * b.z_) - (a.z_ * b.y_), (a.z_ * b.x_) - (a.x_ * b.z_), (a.x_ * b.y_) - (a.y_ * b.x_)); }
		static double dot(const vertex& a, const vertex& b) { return (a.x_ * b.x_) + (a.y_ * b.y_) + (a.z_ * b.z_); }
		static bool equals(const vertex& a, const vertex& b, double tolerance) { return (abs(a.x_ - b.x_) < tolerance && abs(a.y_ - b.y_) < tolerance && abs(a.z_ - b.z_) < tolerance); }
		//static bool equals(const vertex& a, const vertex& b, double tolerance) { return abs(magnitude(subtract(b, a))) < tolerance; }
		static bool equals(const vertex& a, const vertex& b) { return (a.x_ == b.x_ && a.y_ == b.y_ && a.z_ == b.z_); }
		static vertex add(const vertex& a, const vertex& b) { return vertex(a.x_ + b.x_, a.y_ + b.y_, a.z_ + b.z_); }
		static vertex subtract(const vertex& a, const vertex& b) { return vertex(a.x_ - b.x_, a.y_ - b.y_, a.z_ - b.z_); }
		static vertex scale(const vertex& a, double s) { return vertex(a.x_ * s, a.y_ * s, a.z_ * s); }
		static vertex normal(const triangle& t) { return unitise(cross(subtract(t.b_, t.a_), subtract(t.c_, t.a_))); }
		static double directionFromPlane(const vertex& p, const vertex& n, const vertex& v) { return !equals(v, p) ? dot(unitise(subtract(v, p)), n) : 0; }
		static vertex rayPlaneIntersection(const vertex& rP, const vertex& rD, const vertex& pP, const vertex& pN, double dotrDpN) { double s = dot(subtract(pP, rP), pN) / dotrDpN; return vertex(rP.x_ + (rD.x_ * s), rP.y_ + (rD.y_ * s), rP.z_ + (rD.z_ * s)); }
		static vertex centroid(const vertex& a, const vertex& b, const vertex& c) { return vertex((a.x_ + b.x_ + c.x_) / 3, (a.y_ + b.y_ + c.y_) / 3, (a.z_ + b.z_ + c.z_) / 3); }
		static bool sameSide(const vertex& p1, const vertex& p2, const vertex& a, const vertex& b) { return dot(cross(subtract(b, a), subtract(p1, a)), cross(subtract(b, a), subtract(p2, a))) >= 0; }
		static bool pointWithinTriangle(const vertex& a, const vertex& b, const vertex& c, const vertex& p) { return sameSide(p, a, b, c) && sameSide(p, b, a, c) && sameSide(p, c, a, b); }
		static bool pointOnLineSegment(const vertex& i, const vertex& j, const vertex& p, double tolerance = defaultTolerance) { return abs(magnitude(subtract(p, i)) + magnitude(subtract(p, j)) - magnitude(subtract(i, j))) < tolerance; }
		static vertex midPoint(const vertex& i, const vertex& j) { return vertex((i.x_ + j.x_) * 0.5, (i.y_ + j.y_) * 0.5, (i.z_ + j.z_) * 0.5); }
		static double arcCosine(double val) { if (val <= -1.0) { return pi; } if (val >= 1.0) { return 0; } return acos(val); }
		static vertex rodrigues(const vertex& v, const vertex& axis, double angle)
		{
			double c = cos(angle);
			vertex vRot(v.x_ * c, v.y_ * c, v.z_ * c);
			vRot = add(scale(cross(axis, v), sin(angle)), vRot);
			return add(scale(axis, dot(axis, v) * (1 - c)), vRot);
		}
		static void flip(triangle& t) { vertex tmp = t.c_; t.c_ = t.a_; t.a_ = tmp; }

		enum ioop { inside, outside, onPlane };

		namespace sanitise
		{
			bool isValid(const triangle& t, double tolerance) { return (!equals(t.a_, t.b_, tolerance) && !equals(t.a_, t.c_, tolerance) || !equals(t.b_, t.c_, tolerance)); }

			// we cannot work with bad triangles...
			/*void triangles(mesh& m, double tolerance)
			{
				std::list<triangle> triangles(m.begin(), m.end());
				std::list<std::list<triangle>::iterator> badTriangles;

				for (std::list<triangle>::iterator tItr = triangles.begin(); tItr != triangles.end(); ++tItr)
					if (equals(tItr->a_, tItr->b_, tolerance) || equals(tItr->a_, tItr->b_, tolerance) || equals(tItr->a_, tItr->b_, tolerance))
						badTriangles.push_back(tItr);

				std::for_each(badTriangles.begin(), badTriangles.end(), [&triangles](std::list<triangle>::iterator& badTri) { triangles.erase(badTri); });
				m = std::vector<triangle>(triangles.begin(), triangles.end());
			}*/
		}

		namespace intersection
		{
			struct splitResult { std::vector<triangle> outside_; std::vector<triangle> inside_; vertex i_, j_; };

			// triangle/plane intersection...
			class trianglePlane
			{
			public:
				trianglePlane(const triangle* T, const vertex& p, const vertex& n)
					: directions_(vertex(directionFromPlane(p, n, T->a_), directionFromPlane(p, n, T->b_), directionFromPlane(p, n, T->c_))), normal_(n)
				{
					directions_.x_ = abs(directions_.x_) < defaultTolerance ? 0 : directions_.x_;
					directions_.y_ = abs(directions_.y_) < defaultTolerance ? 0 : directions_.y_;
					directions_.z_ = abs(directions_.z_) < defaultTolerance ? 0 : directions_.z_;
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
				bool straddling() const { return (!directions_.x_ && directions_.y_ && directions_.z_) || (directions_.x_ && !directions_.y_ && directions_.z_) || (directions_.x_ && directions_.y_ && !directions_.z_); }
				double a() const { return directions_.x_; }
				double b() const { return directions_.y_; }
				double c() const { return directions_.z_; }
				const vertex& n() const { return normal_; }
			protected:
				vertex directions_;
				vertex normal_;
			};

			vertex segmentPlaneIntersection(const vertex& i, const vertex& j, const vertex& pP, const vertex& pN)
			{
				vertex ijUnitised = unitise(subtract(j, i));
				double dotrDpN = dot(ijUnitised, pN);
				if (!std::isnan(dotrDpN) && dotrDpN)
					return rayPlaneIntersection(i, ijUnitised, pP, pN, dotrDpN);
				else
					throw std::exception("no edge plane intersection.");
			}

			void validateSplitTriangles(splitResult& result, const triangle& originalTriangle, double tolerance)
			{
				splitResult validTriangles;
				std::for_each(result.inside_.begin(), result.inside_.end(), [&validTriangles, &tolerance](triangle& t) { if (sanitise::isValid(t, tolerance)) { validTriangles.inside_.push_back(t); } });
				std::for_each(result.outside_.begin(), result.outside_.end(), [&validTriangles, &tolerance](triangle& t) { if (sanitise::isValid(t, tolerance)) { validTriangles.outside_.push_back(t); } });
				result.inside_.swap(validTriangles.inside_);
				result.outside_.swap(validTriangles.outside_);
			}

			splitResult splitTriangle(const trianglePlane& intersectionInfo, const triangle& t, const vertex& tN, const vertex& pP, const vertex& pN, double tolerance)
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
					if (directionFromPlane(pP, pN, t.a_) > 0)
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
					if (directionFromPlane(pP, pN, t.b_) > 0)
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
					if (directionFromPlane(pP, pN, t.a_) > 0)
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
					throw std::exception("cannot split triangle.");

				validateSplitTriangles(result, t, tolerance);
				return result;
			}

			// triangle/triangle intersection...
			class triangleTriangle : public trianglePlane
			{
			public:
				triangleTriangle(const triangle& A, const triangle& B)
					: trianglePlane(&A, B.a_, normal(B)), A_(&A), B_(&B)
				{
				}

				bool intersects() const { return !(entirelyInside() || entirelyOutside()); }

				bool calculateIntersection(segment& result, double tolerance) const
				{
					if (edgeABOnPlane()) { return calculateIntersectionEdge(A_->a_, A_->b_, result); }
					if (edgeBCOnPlane()) { return calculateIntersectionEdge(A_->b_, A_->c_, result); }
					if (edgeCAOnPlane()) { return calculateIntersectionEdge(A_->c_, A_->a_, result); }

					splitResult split = splitTriangle(*this, *A_, normal(*A_), B_->a_, normal_, tolerance);
					return calculateIntersectionEdge(split.i_, split.j_, result);
				}

				bool calculateCoplanarIntersection(std::pair<std::list<segment>, std::list<segment>>& result, double tolerance) const
				{
					// Use 2D boolean operations...


					return false;



				}


			private:
				bool calculateIntersectionEdge(const vertex& i, const vertex& j, segment& result) const
				{
					bool iInB = pointWithinTriangle(B_->a_, B_->b_, B_->c_, i);
					bool jInB = pointWithinTriangle(B_->a_, B_->b_, B_->c_, j);

					// if both intersection points are within B, this is the segment...
					if (iInB && jInB)
						result = segment(i, j);
					// Otherwise...
					else
					{
						std::list<vertex> triangleEdgeSegmentIntersections = calculateTriangleEdgesSegmentIntersection(*B_, i, j);

						// If neither intersection points are within B...
						if (!iInB && !jInB)
						{
							// no points or a single point intersection edge is considered outside B entirely...
							if (triangleEdgeSegmentIntersections.size() <= 1)
								return false;
							if (triangleEdgeSegmentIntersections.size() > 2)
								throw std::exception("there should be exactly 2 intersections between segment and triangle edges in this case.");
							result = segment(triangleEdgeSegmentIntersections.front(), triangleEdgeSegmentIntersections.back());
						}
						else
						{
							if (triangleEdgeSegmentIntersections.size() != 1)
								throw std::exception("there should be only intersection between segment and triangle edges in this case.");
							result = segment(triangleEdgeSegmentIntersections.front(), iInB ? i : j);
						}
					}
					return !equals(result.first, result.second);
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
							if (pointOnLineSegment(i, j, ip) && pointOnLineSegment(t.a_, t.b_, ip))
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
							if (pointOnLineSegment(i, j, ip) && pointOnLineSegment(t.b_, t.c_, ip))
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
							if (pointOnLineSegment(i, j, ip) && pointOnLineSegment(t.c_, t.a_, ip))
								intersections.push_back(ip);
						}
					}
					return intersections;
				}

				const triangle* A_;
				const triangle* B_;
			};
		}

		class bsp
		{
		public:
			class node : public triangle
			{
			public:
				node(const triangle& t) : triangle(t.a_, t.b_, t.c_), normal_(normal(t)) { if (std::isnan(normal_.x_) || std::isnan(normal_.y_) || std::isnan(normal_.z_)) { throw std::exception("unable to calculate bsp node normal."); } }
				vertex triangleIntersection(const triangle& t) const { return vertex(directionFromPlane(p(), n(), t.a_), directionFromPlane(p(), n(), t.b_), directionFromPlane(p(), n(), t.c_)); }
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
			bsp(const mesh& m) { construct(m, defaultTolerance); }

			ioop ioop(const vertex& v) const
			{
				node* currentNode = root_.get();
				while (currentNode)
				{
					double direction = directionFromPlane(currentNode->p(), currentNode->n(), v);
					if (direction > 0)
					{
						if (currentNode->outside_) { currentNode = currentNode->outside_.get(); }
						else { return ioop::outside; }
					}
					else if (direction < 0)
					{
						if (currentNode->inside_) { currentNode = currentNode->inside_.get(); }
						else { return ioop::inside; }
					}
					else
					{
						if (pointWithinTriangle(currentNode->a_, currentNode->b_, currentNode->c_, v)) { return ioop::onPlane; }
						else if (currentNode->inside_.get()) { currentNode = currentNode->inside_.get(); }
						else { return ioop::outside; }
					}
				}
				throw std::exception("cannot deduce if point is inside or outside.");
			}

		private:
			void construct(const mesh& m, double tolerance)
			{
				// create a working list of the triangles. We validate as we go along.
				std::list<triangle> workingTriangles(m.begin(), m.end());
				while (!sanitise::isValid(workingTriangles.front(), tolerance))
					workingTriangles.pop_front();
				root_.reset(new node(workingTriangles.front()));
				while (!workingTriangles.empty())
				{
					std::list<triangle>::iterator currentTriangle = workingTriangles.begin();
					while (!sanitise::isValid(workingTriangles.front(), tolerance))
						workingTriangles.pop_front();
					node* currentNode = root_.get();
					while (currentNode)
					{
						intersection::trianglePlane intersectionInfo(&(*currentTriangle), currentNode->p(), currentNode->n());
						if (intersectionInfo.outside())
							currentNode = followOutsideNode(currentNode, *currentTriangle);
						else if (intersectionInfo.inside() || intersectionInfo.onPlane())
							currentNode = followInsideNode(currentNode, *currentTriangle);
						else
						{
							intersection::splitResult splitTriangles = intersection::splitTriangle(intersectionInfo, *currentTriangle, normal(*currentTriangle), currentNode->p(), currentNode->n(), tolerance);
							workingTriangles.insert(workingTriangles.begin(), splitTriangles.inside_.begin(), splitTriangles.inside_.end());
							workingTriangles.insert(workingTriangles.begin(), splitTriangles.outside_.begin(), splitTriangles.outside_.end());
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
					if (!equals(normal, axis_) && !equals(vertex(normal.x_, normal.y_, normal.z_), axis_))
					{
						axis_ = cross(normal, vertex(0, 0, 1.0));
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

					vector2 p1(midx - 20.0 * deltaMax, midy - deltaMax);
					vector2 p2(midx, midy + 20.0 * deltaMax);
					vector2 p3(midx + 20.0 * deltaMax, midy - deltaMax);

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

			unsigned int addDelaunayVertex(std::vector<tessellate::vector2>& vertices, const tessellate::vector2& v)
			{
				for (unsigned int dv = 0; dv < vertices.size(); ++dv)
					if (vertices[dv].x_ == v.x_ && vertices[dv].y_ == v.y_)
						return dv;

				vertices.push_back(v);
				return static_cast<unsigned int>(vertices.size() - 1);
			}

			static void retessellate(const csg::triangle& original, std::list<segment>& intersections, mesh& result)
			{
				for (std::list<segment>::iterator itr = intersections.begin(); itr != intersections.end();)
				{
					// check if the mid point of the segment is on the triangle edge...
					vertex segmentMidPoint = midPoint(itr->first, itr->second);
					if (pointOnLineSegment(original.a_, original.b_, segmentMidPoint) ||
						pointOnLineSegment(original.b_, original.c_, segmentMidPoint) ||
						pointOnLineSegment(original.c_, original.a_, segmentMidPoint))
						itr = intersections.erase(itr);
					else
						++itr;
				}
				vertex n = normal(original);
				projection projection(n, original.a_);

				std::vector<tessellate::vector2> vertices(3);
				std::vector<tessellate::edge> constraints;

				vertices[0] = projection.get2D(original.a_);
				vertices[1] = projection.get2D(original.b_);
				vertices[2] = projection.get2D(original.c_);
				for (std::list<segment>::iterator itr = intersections.begin(); itr != intersections.end(); ++itr)
					constraints.push_back(tessellate::edge{ addDelaunayVertex(vertices, projection.get2D(itr->first)), addDelaunayVertex(vertices, projection.get2D(itr->second)) });
				
				tessellate::delaunay d(vertices);
				for (unsigned int c = 0; c < constraints.size(); ++c)
					d.addConstraint(constraints[c]);

				const std::list<tessellate::triangle>& triangles = d.getTriangles();
				std::for_each(triangles.begin(), triangles.end(), [&result, &d, &projection](const tessellate::triangle& t) 
					{
						result.push_back(csg::triangle(projection.get3D(d.getVertices()[t.a_]), projection.get3D(d.getVertices()[t.b_]), projection.get3D(d.getVertices()[t.c_])));
					});

				// Assume CCW, ensure new tessellation is winding the correct way...
				vertex normal = cross(subtract(original.c_, original.a_), subtract(original.b_, original.a_));
				std::for_each(result.begin(), result.end(), [&normal](csg::triangle& t) 
					{
						vertex n = cross(subtract(t.c_, t.a_), subtract(t.b_, t.a_));
						if (magnitude(subtract(n, normal)) > 0.5)
							flip(t);
					});

			}
		}

		typedef std::pair<mergeIntersectResult, mergeIntersectResult> mergeResult;

		struct triangleInfo
		{
			//ioop ioop_;
			std::list<segment> intersections_;
			mesh subTriangles_;
		};

		void addResultTriangle(mergeIntersectResult& mergeResult, const csg::triangle& triangle, const bsp& bsp)
		{
			ioop ioop = bsp.ioop(centroid(triangle.a_, triangle.b_, triangle.c_));
			switch (ioop)
			{
			case ioop::inside:
				mergeResult.inside_.push_back(triangle);
				break;
			case ioop::onPlane:
				mergeResult.onPlane_.push_back(triangle);
				break;
			case ioop::outside:
				mergeResult.outside_.push_back(triangle);
				break;
			}
		}

		void mergeAndIntersect(const mesh& A, const mesh& B, mergeResult& result, double tolerance)
		{
			// construct arrays of our new triangleTessellator for A and B...
			std::vector<triangleInfo> aTriangles(A.size(), triangleInfo());
			std::vector<triangleInfo> bTriangles(B.size(), triangleInfo());

			// check triangle intersections against each other...
			#pragma omp parallel for
			for (int a = 0; a < A.size(); ++a)
			{
				for (int b = 0; b < B.size(); ++b)
				{
					if (a == b)
						continue;

					intersection::triangleTriangle tt(A[a], B[b]);

					if (!tt.intersects())
						continue;

					if (tt.onPlane())
					{
						std::pair<std::list<segment>, std::list<segment>> coplanarResult;
						if (tt.calculateCoplanarIntersection(coplanarResult, tolerance))
						{
							#pragma omp critical
							{
								aTriangles[a].intersections_.insert(aTriangles[a].intersections_.end(), coplanarResult.first.begin(), coplanarResult.first.end());
								bTriangles[b].intersections_.insert(bTriangles[b].intersections_.end(), coplanarResult.second.begin(), coplanarResult.second.end());
							}
						}
					}
					else
					{
						segment intersectionEdge;
						if (tt.calculateIntersection(intersectionEdge, tolerance))
						{
							#pragma omp critical
							{
								aTriangles[a].intersections_.push_back(intersectionEdge);
								bTriangles[b].intersections_.push_back(intersectionEdge);
							}
						}
					}
				}
			}

			// retessellate the triangles that have intersections...
			#pragma omp parallel for
			for (int a = 0; a < aTriangles.size(); ++a)
			{
				if (!aTriangles[a].intersections_.empty())
					tessellate::retessellate(A[a], aTriangles[a].intersections_, aTriangles[a].subTriangles_);
			}
			#pragma omp parallel for
			for (int b = 0; b < bTriangles.size(); ++b)
			{
				if (!bTriangles[b].intersections_.empty())
					tessellate::retessellate(B[b], bTriangles[b].intersections_, bTriangles[b].subTriangles_);
			}

			// put the triangles into the correct result...
			bsp bspB(B, tolerance);
			#pragma omp parallel for
			for (int a = 0; a < aTriangles.size(); ++a)
			{
				if (aTriangles[a].subTriangles_.empty())
					addResultTriangle(result.first, A[a], bspB);
				else
				{
					for (unsigned int st = 0; st < aTriangles[a].subTriangles_.size(); ++st)
						addResultTriangle(result.first, aTriangles[a].subTriangles_[st], bspB);
				}
			}

			bsp bspA(A, tolerance);
			#pragma omp parallel for
			for (int b = 0; b < bTriangles.size(); ++b)
			{
				if (bTriangles[b].subTriangles_.empty())
					addResultTriangle(result.second, B[b], bspA);
				else
				{
					for (unsigned int st = 0; st < bTriangles[b].subTriangles_.size(); ++st)
						addResultTriangle(result.second, bTriangles[b].subTriangles_[st], bspA);
				}
			}
		}
	}

	std::shared_ptr<mesh> Difference(const mesh& a, const mesh& b, double tolerance = defaultTolerance)
	{
		impl::mergeResult abMerge;
		impl::mergeAndIntersect(a, b, abMerge, tolerance);

		std::shared_ptr<mesh> result(new mesh());
		result->insert(result->end(), abMerge.first.outside_.begin(), abMerge.first.outside_.end());
		std::for_each(abMerge.second.inside_.begin(), abMerge.second.inside_.end(), [](triangle& t) { impl::flip(t); });
		result->insert(result->end(), abMerge.second.inside_.begin(), abMerge.second.inside_.end());
		result->insert(result->end(), abMerge.first.onPlane_.begin(), abMerge.first.onPlane_.end());

		return result;
	}

	std::shared_ptr<mesh> Union(const mesh& a, const mesh& b, double tolerance = defaultTolerance)
	{
		impl::mergeResult abMerge;
		impl::mergeAndIntersect(a, b, abMerge, tolerance);

		std::shared_ptr<mesh> result(new mesh());
		result->insert(result->end(), abMerge.first.outside_.begin(), abMerge.first.outside_.end());
		result->insert(result->end(), abMerge.second.outside_.begin(), abMerge.second.outside_.end());

		return result;
	}

	std::shared_ptr<mesh> Intersection(const mesh& a, const mesh& b, double tolerance = defaultTolerance)
	{
		impl::mergeResult abMerge;
		impl::mergeAndIntersect(a, b, abMerge, tolerance);

		std::shared_ptr<mesh> result(new mesh());
		result->insert(result->end(), abMerge.first.inside_.begin(), abMerge.first.inside_.end());
		result->insert(result->end(), abMerge.second.inside_.begin(), abMerge.second.inside_.end());

		return result;
	}


























	//	// delaunay...
	//	namespace Delaunay
	//	{
	//		const static double pi = 3.14159265358979323846;

	//		static bool almost_equal(const double& x, const double& y, int ulp = 2)
	//		{
	//			// the machine epsilon has to be scaled to the magnitude of the values used
	//			// and multiplied by the desired precision in ULPs (units in the last place)
	//			return std::abs(x - y) <= std::numeric_limits<double>::epsilon() * std::abs(x + y) * static_cast<double>(ulp)
	//				// unless the result is subnormal
	//				|| std::abs(x - y) < (std::numeric_limits<double>::min)();
	//		}

	//		static double half(const double x)
	//		{
	//			return 0.5 * x;
	//		}

	//		struct Vector2
	//		{
	//			Vector2() = default;
	//			Vector2(const Vector2 &v) = default;
	//			Vector2(Vector2&&) = default;
	//			Vector2(const double vx, const double vy)
	//				: x(vx), y(vy)
	//			{
	//			}

	//			double dist2(const Vector2 &v) const
	//			{
	//				const double dx = x - v.x;
	//				const double dy = y - v.y;
	//				return dx * dx + dy * dy;
	//			}

	//			double dist(const Vector2 &v) const
	//			{
	//				return hypot(x - v.x, y - v.y);
	//			}

	//			double norm2() const
	//			{
	//				return x * x + y * y;
	//			}

	//			Vector2 &operator=(const Vector2& v) = default;

	//			Vector2 &operator=(Vector2&&) = default;

	//			bool operator ==(const Vector2 &v) const
	//			{
	//				return (x == v.x) && (y == v.y);
	//			}

	//			double x;
	//			double y;
	//		};

	//		static bool almost_equal(const Vector2 &v1, const Vector2 &v2, int ulp = 2)
	//		{
	//			return almost_equal(v1.x, v2.x, ulp) && almost_equal(v1.y, v2.y, ulp);
	//		}

	//		struct Edge
	//		{
	//			Edge() = default;
	//			Edge(const Edge&) = default;
	//			Edge(Edge&&) = default;

	//			Edge &operator=(const Edge&) = default;
	//			Edge &operator=(Edge&&) = default;

	//			Edge(const Vector2 &v1, const Vector2 &v2)
	//				: v(&v1), w(&v2)
	//			{
	//			}

	//			Edge(Vector2* v1, Vector2* v2)
	//				: v(v1), w(v2)
	//			{
	//			}

	//			bool operator==(const Edge &e) const
	//			{
	//				return (*(this->v) == *e.v && *(this->w) == *e.w) ||
	//					(*(this->v) == *e.w && *(this->w) == *e.v);
	//			}

	//			const Vector2 *v;
	//			const Vector2 *w;
	//			bool isBad = false;
	//		};

	//		static bool almost_equal(const Edge &e1, const Edge &e2)
	//		{
	//			return	(almost_equal(*e1.v, *e2.v) && almost_equal(*e1.w, *e2.w)) ||
	//				(almost_equal(*e1.v, *e2.w) && almost_equal(*e1.w, *e2.v));
	//		}

	//		struct Triangle
	//		{
	//			Triangle() = default;
	//			Triangle(const Triangle&) = default;
	//			Triangle(Triangle&&) = default;
	//			Triangle(const Vector2 &v1, const Vector2 &v2, const Vector2 &v3)
	//				: a(&v1), b(&v2), c(&v3), isBad(false)
	//			{}

	//			bool containsVertex(const Vector2 &v) const
	//			{
	//				return almost_equal(*a, v) || almost_equal(*b, v) || almost_equal(*c, v);
	//			}

	//			bool constainsEdge(const Edge& e) const
	//			{
	//				return containsVertex(*e.v) && containsVertex(*e.w);
	//			}

	//			bool circumCircleContains(const Vector2 &v) const
	//			{
	//				const double ab = a->norm2();
	//				const double cd = b->norm2();
	//				const double ef = c->norm2();

	//				const double ax = a->x;
	//				const double ay = a->y;
	//				const double bx = b->x;
	//				const double by = b->y;
	//				const double cx = c->x;
	//				const double cy = c->y;

	//				const double circum_x = (ab * (cy - by) + cd * (ay - cy) + ef * (by - ay)) / (ax * (cy - by) + bx * (ay - cy) + cx * (by - ay));
	//				const double circum_y = (ab * (cx - bx) + cd * (ax - cx) + ef * (bx - ax)) / (ay * (cx - bx) + by * (ax - cx) + cy * (bx - ax));

	//				const Vector2 circum(half(circum_x), half(circum_y));
	//				const double circum_radius = a->dist2(circum);
	//				const double dist = v.dist2(circum);
	//				return dist <= circum_radius;
	//			}

	//			Triangle &operator=(const Triangle&) = default;
	//			Triangle &operator=(Triangle&&) = default;

	//			bool operator ==(const Triangle &t) const
	//			{
	//				return	(*this->a == *t.a || *this->a == *t.b || *this->a == *t.c) &&
	//					(*this->b == *t.a || *this->b == *t.b || *this->b == *t.c) &&
	//					(*this->c == *t.a || *this->c == *t.b || *this->c == *t.c);
	//			}

	//			const Vector2 *a;
	//			const Vector2 *b;
	//			const Vector2 *c;
	//			bool isBad = false;
	//		};

	//		static bool almost_equal(const Triangle &t1, const Triangle &t2)
	//		{
	//			return	(almost_equal(*t1.a, *t2.a) || almost_equal(*t1.a, *t2.b) || almost_equal(*t1.a, *t2.c)) &&
	//				(almost_equal(*t1.b, *t2.a) || almost_equal(*t1.b, *t2.b) || almost_equal(*t1.b, *t2.c)) &&
	//				(almost_equal(*t1.c, *t2.a) || almost_equal(*t1.c, *t2.b) || almost_equal(*t1.c, *t2.c));
	//		}

	//		static double angleDifference(const Vector2& a, const Vector2& b)
	//		{
	//			double angle = atan2(b.y, b.x) - atan2(a.y, a.x);

	//			if (angle > pi)
	//				angle -= 2 * pi;
	//			else if (angle <= -pi)
	//				angle += 2 * pi;

	//			return angle;
	//		}

	//		static std::optional<Delaunay::Vector2> segmentSegmentIntersection(const Vector2& a, const Vector2& b, const Vector2& i, const Vector2& j)
	//		{
	//			double s1_x, s1_y, s2_x, s2_y;
	//			s1_x = b.x - a.x;     s1_y = b.y - a.y;
	//			s2_x = j.x - i.x;     s2_y = j.y - i.y;

	//			double sDenom = (-s2_x * s1_y + s1_x * s2_y);
	//			double tDenom = (-s2_x * s1_y + s1_x * s2_y);

	//			if (!sDenom || !tDenom)
	//				return std::optional<Vector2>();

	//			double s = (-s1_y * (a.x - i.x) + s1_x * (a.y - i.y)) / sDenom;
	//			double t = (s2_x * (a.y - i.y) - s2_y * (a.x - i.x)) / tDenom;

	//			if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
	//				return std::optional<Vector2>(Vector2(a.x + (t * s1_x), a.y + (t * s1_y)));

	//			return std::optional<Vector2>();
	//		}

	//		class Tessellator
	//		{
	//		public:
	//			const std::list<Triangle>& triangulate(const std::vector<Vector2> &vertices)
	//			{
	//				// Store the vertices locally
	//				vertices_ = vertices;

	//				// Determinate the super triangle
	//				double minX = vertices_[0].x;
	//				double minY = vertices_[0].y;
	//				double maxX = minX;
	//				double maxY = minY;

	//				for (std::size_t i = 0; i < vertices_.size(); ++i)
	//				{
	//					if (vertices_[i].x < minX) minX = vertices_[i].x;
	//					if (vertices_[i].y < minY) minY = vertices_[i].y;
	//					if (vertices_[i].x > maxX) maxX = vertices_[i].x;
	//					if (vertices_[i].y > maxY) maxY = vertices_[i].y;
	//				}

	//				const double dx = maxX - minX;
	//				const double dy = maxY - minY;
	//				const double deltaMax = (std::max)(dx, dy);
	//				const double midx = half(minX + maxX);
	//				const double midy = half(minY + maxY);

	//				const Delaunay::Vector2 p1(midx - 20 * deltaMax, midy - deltaMax);
	//				const Delaunay::Vector2 p2(midx, midy + 20 * deltaMax);
	//				const Delaunay::Vector2 p3(midx + 20 * deltaMax, midy - deltaMax);

	//				// Create a list of triangles, and add the supertriangle in it
	//				triangles_.push_back(Triangle(p1, p2, p3));

	//				for (auto p = begin(vertices_); p != end(vertices_); p++)
	//				{
	//					std::vector<Edge> polygon;

	//					for (auto & t : triangles_)
	//					{
	//						if (t.circumCircleContains(*p))
	//						{
	//							t.isBad = true;
	//							polygon.push_back(Edge{ *t.a, *t.b });
	//							polygon.push_back(Edge{ *t.b, *t.c });
	//							polygon.push_back(Edge{ *t.c, *t.a });
	//						}
	//					}

	//					triangles_.erase(std::remove_if(begin(triangles_), end(triangles_), [](Triangle&t) {
	//						return t.isBad;
	//						}), end(triangles_));

	//					for (auto e1 = begin(polygon); e1 != end(polygon); ++e1)
	//					{
	//						for (auto e2 = e1 + 1; e2 != end(polygon); ++e2)
	//						{
	//							if (almost_equal(*e1, *e2))
	//							{
	//								e1->isBad = true;
	//								e2->isBad = true;
	//							}
	//						}
	//					}

	//					polygon.erase(std::remove_if(begin(polygon), end(polygon), [](Edge&e) {
	//						return e.isBad;
	//						}), end(polygon));

	//					for (const auto e : polygon)
	//						triangles_.push_back(Triangle(*e.v, *e.w, *p));
	//				}

	//				triangles_.erase(std::remove_if(begin(triangles_), end(triangles_), [p1, p2, p3](Triangle&t) {
	//					return t.containsVertex(p1) || t.containsVertex(p2) || t.containsVertex(p3);
	//					}), end(triangles_));

	//				for (const auto t : triangles_)
	//				{
	//					edges_.push_back(Edge{ *t.a, *t.b });
	//					edges_.push_back(Edge{ *t.b, *t.c });
	//					edges_.push_back(Edge{ *t.c, *t.a });
	//				}

	//				return triangles_;
	//			}

	//			const std::list<Delaunay::Triangle>& getTriangles() const { return triangles_; }
	//			const std::list<Delaunay::Edge>& getEdges() const { return edges_; }
	//			const std::vector<Delaunay::Vector2>& getVertices() const { return vertices_; }

	//			void addConstraint(const std::pair<unsigned int, unsigned int>& e)
	//			{
	//				// check if we already have this edge...
	//				Vector2* e1 = &vertices_[e.first];
	//				Vector2* e2 = &vertices_[e.second];

	//				if (std::find_if(edges_.begin(), edges_.end(), [&e1, &e2](const Edge& edge) { return ((e1 == edge.v && e2 == edge.w) || (e1 == edge.w && e2 == edge.v)); }) == edges_.end())
	//				{
	//					Vector2 constraintEdge(e2->x - e1->x, e2->y - e1->y);

	//					// find out which edges are intersected, and for the two vertices of the intersected edge, find out which side of the constraint they are on...
	//					std::list<Vector2> leftSide, rightSide;
	//					for (std::list<Edge>::iterator ee = edges_.begin(); ee != edges_.end();)
	//					{
	//						if (*ee->v == *e1 || *ee->w == *e1 || *ee->v == *e2 || *ee->w == *e2)
	//						{
	//							++ee;
	//							continue;
	//						}


	//						double vAngle = angleDifference(Vector2(ee->v->x - e1->x, ee->v->y - e1->y), constraintEdge);
	//						double wAngle = angleDifference(Vector2(ee->w->x - e2->x, ee->w->y - e2->y), constraintEdge);

	//						if ((vAngle < 0 && wAngle > 0) || (vAngle > 0 && wAngle < 0))
	//						{
	//							// do the addtional check to see if this constrain edge is intersected...
	//							if (segmentSegmentIntersection(*e1, *e2, *ee->v, *ee->w))
	//							{
	//								if (vAngle > 0)
	//									leftSide.push_back(*ee->v);
	//								else
	//									rightSide.push_back(*ee->v);

	//								if (wAngle > 0)
	//									leftSide.push_back(*ee->w);
	//								else
	//									rightSide.push_back(*ee->w);

	//								// and any triangles that use this edge...
	//								triangles_.erase(std::remove_if(begin(triangles_), end(triangles_), [&ee](Triangle& tri) { return tri.constainsEdge(*ee); }), end(triangles_));

	//								// remove the edge from our list...
	//								ee = edges_.erase(ee);

	//								// next edge...
	//								continue;
	//							}
	//						}

	//						++ee;
	//					}

	//					// add the constaint edge to both sides and remove the duplicates of both sides.
	//					leftSide.push_back(*e1);
	//					leftSide.push_back(*e2);
	//					rightSide.push_back(*e1);
	//					rightSide.push_back(*e2);

	//					// tessellate each subset...
	//					Tessellator leftTessellation, rightTessellation;

	//					// remove duplicates and tessellation the left side...
	//					leftSide.sort([](const Vector2& a, const Vector2& b) { if (a.x == b.x) { return a.y < b.y; } else { return a.x < b.x; } });
	//					leftSide.erase(std::unique(leftSide.begin(), leftSide.end()), leftSide.end());
	//					leftTessellation.triangulate(std::vector<Vector2>(leftSide.begin(), leftSide.end()));

	//					// remove duplicates and tessellation the right side...
	//					rightSide.sort([](const Vector2& a, const Vector2& b) { if (a.x == b.x) { return a.y < b.y; } else { return a.x < b.x; } });
	//					rightSide.erase(std::unique(rightSide.begin(), rightSide.end()), rightSide.end());
	//					rightTessellation.triangulate(std::vector<Vector2>(rightSide.begin(), rightSide.end()));

	//					// add these two subsets to the final results...
	//					append(leftTessellation);
	//					append(rightTessellation);
	//				}
	//			}

	//		private:
	//			Vector2* addVertex(const Vector2& v) { vertices_.push_back(v); return &vertices_.back(); }

	//			// destructive copy...
	//			void append(Tessellator& tessellation)
	//			{
	//				// create new container...
	//				std::vector<Vector2> newVertices;

	//				// copy the vertices across...
	//				newVertices.insert(newVertices.end(), vertices_.begin(), vertices_.end());
	//				newVertices.insert(newVertices.end(), tessellation.vertices_.begin(), tessellation.vertices_.end());

	//				// iterate through the original vertices of this tessellation and set the x value to the index of it.
	//				for (unsigned int v = 0; v < vertices_.size(); ++v)
	//					vertices_[v].x = static_cast<double>(v);

	//				// iterate through the original vertices of the tessellation to append and set the x value to the index of it + offset of original this tessellation vertices.
	//				unsigned int offset = static_cast<unsigned int>(vertices_.size());
	//				for (unsigned int v = 0; v < static_cast<unsigned int>(tessellation.vertices_.size()); ++v)
	//					tessellation.vertices_[v].x = static_cast<double>(v + offset);

	//				// set all the correct edge triangle pointers. new offset of new container is supplied by 

	//				// now copy the edges and triangles across to the new container, set the pointer to the new index supplied by x coord.
	//				triangles_.insert(triangles_.end(), tessellation.triangles_.begin(), tessellation.triangles_.end());
	//				std::for_each(triangles_.begin(), triangles_.end(),
	//					[&newVertices](Triangle& t)
	//					{
	//						t.a = &newVertices[static_cast<unsigned int>(t.a->x)];
	//						t.b = &newVertices[static_cast<unsigned int>(t.b->x)];
	//						t.c = &newVertices[static_cast<unsigned int>(t.c->x)];
	//					});

	//				// do the same for the edges...
	//				edges_.insert(edges_.end(), tessellation.edges_.begin(), tessellation.edges_.end());
	//				std::for_each(edges_.begin(), edges_.end(),
	//					[&newVertices](Edge& e)
	//					{
	//						e.v = &newVertices[static_cast<unsigned int>(e.v->x)];
	//						e.w = &newVertices[static_cast<unsigned int>(e.w->x)];
	//					});

	//				// swap vertices...
	//				vertices_.swap(newVertices);
	//			}

	//			std::list<Delaunay::Triangle> triangles_;
	//			std::list<Delaunay::Edge> edges_;
	//			std::vector<Delaunay::Vector2> vertices_;
	//		};

	//	}	// namespace Delaunay

	//	//namespace intersection
	//	//{
	//	//	struct mergeIntersectResult { std::list<triangle> inside_, outside_, onPlane_; };

	//	//	class retessellatedTriangle
	//	//	{
	//	//	public:
	//	//		retessellatedTriangle() : triangle_(0) {}

	//	//		retessellatedTriangle(const triangle* tri, const vertex& normal) : triangle_(tri), normal_(normal) {}

	//	//		bool needsFinalising() const {
	//	//			//return intersections_.size() == 1;
	//	//			return !intersections_.empty();
	//	//		}

	//	//		void finalise(mergeIntersectResult& result, bsp& bsp)
	//	//		{
	//	//			// santise the edges, remove the duplicate points...
	//	//			for (std::list<segment>::iterator itr = intersections_.begin(); itr != intersections_.end();)
	//	//			{
	//	//				// check if the mid point of the segment is on the triangle edge...
	//	//				vertex segmentMidPoint = midPoint(itr->first, itr->second);
	//	//				if (pointOnLineSegment(triangle_->a_, triangle_->b_, segmentMidPoint) ||
	//	//					pointOnLineSegment(triangle_->b_, triangle_->c_, segmentMidPoint) ||
	//	//					pointOnLineSegment(triangle_->c_, triangle_->a_, segmentMidPoint))
	//	//					itr = intersections_.erase(itr);
	//	//				else
	//	//					++itr;
	//	//			}

	//	//			Projection projection(normal_, triangle_->a_);

	//	//			std::vector<Delaunay::Vector2> vertices;
	//	//			std::vector<std::pair<unsigned int, unsigned int>> constraints;
	//	//			vertices.push_back(projection.get2D(triangle_->a_));
	//	//			vertices.push_back(projection.get2D(triangle_->b_));
	//	//			vertices.push_back(projection.get2D(triangle_->c_));
	//	//			for (std::list<segment>::iterator itr = intersections_.begin(); itr != intersections_.end(); ++itr)
	//	//			{
	//	//				constraints.push_back(std::pair<unsigned int, unsigned int>(
	//	//					static_cast<unsigned int>(addDelaunayVertex2D(vertices, projection.get2D(itr->first))),
	//	//					static_cast<unsigned int>(addDelaunayVertex2D(vertices, projection.get2D(itr->second)))));
	//	//			}
	//	//				
	//	//			// Tesssellate...
	//	//			Delaunay::Tessellator tessellator;
	//	//			tessellator.triangulate(vertices);

	//	//			// Add the constriants...
	//	//			for (unsigned int c = 0; c < constraints.size(); ++c)
	//	//				tessellator.addConstraint(constraints[c]);

	//	//			// populate our result...
	//	//			const std::list<Delaunay::Triangle> triangles = tessellator.getTriangles();

	//	//			#pragma omp critical
	//	//			{
	//	//				for (std::list<Delaunay::Triangle>::const_iterator tItr = triangles.begin(); tItr != triangles.end(); ++tItr)
	//	//				{
	//	//					triangle tri(projection.get3D(*tItr->a), projection.get3D(*tItr->b), projection.get3D(*tItr->c));
	//	//					if (!equals(normal(tri), normal_))
	//	//						flip(tri);
	//	//					result.addTriangle(tri, bsp);
	//	//				}
	//	//			}
	//	//			

	//	//		}

	//	//		const vertex& getNormal() const { return normal_; }

	//	//		void addIntersection(const segment& intersectionEdge)
	//	//		{
	//	//			intersections_.push_back(intersectionEdge);
	//	//		}

	//	//		void addIntersection(const std::list<segment>& intersectionEdges)
	//	//		{
	//	//			intersections_.insert(intersections_.end(), intersectionEdges.begin(), intersectionEdges.end());
	//	//		}

	//	//		const triangle* getTriangle() const { return triangle_; }

	//	//	private:
	//	//		int addDelaunayVertex2D(std::vector<Delaunay::Vector2>& vertices, const Delaunay::Vector2& v)
	//	//		{
	//	//			for (int i = 0; i < vertices.size(); ++i)
	//	//			{
	//	//				if (vertices[i] == v)
	//	//					return i;
	//	//			}
	//	//			vertices.push_back(v);
	//	//			return static_cast<int>(vertices.size() - 1);
	//	//		}

	//	//		class Projection
	//	//		{
	//	//		public:
	//	//			Projection(const vertex& normal, const vertex& origin)
	//	//				:	origin_(origin), angle_(0)
	//	//			{
	//	//				axis_ = vertex(0, 0, 1.0);
	//	//				if (!equals(normal, axis_)&&!equals(vertex(normal.x_, normal.y_, normal.z_), axis_))
	//	//				{
	//	//					axis_ = cross(normal, vertex(0, 0, 1.0));
	//	//					angle_ = arcCosine(dot(normal, vertex(0, 0, 1.0)));
	//	//				}
	//	//			}

	//	//			Delaunay::Vector2 get2D(const vertex& v)
	//	//			{
	//	//				vertex rotated = rodrigues(subtract(v, origin_), axis_, angle_);
	//	//				assert(abs(rotated.z_) < 0.001);
	//	//				return Delaunay::Vector2(rotated.x_, rotated.y_);
	//	//			}

	//	//			vertex get3D(const Delaunay::Vector2& v)
	//	//			{
	//	//				return add(origin_, rodrigues(vertex(v.x, v.y, 0), axis_, -angle_));
	//	//			}

	//	//		private:
	//	//			vertex axis_, origin_;
	//	//			double angle_;
	//	//		};

	//	//		const triangle* triangle_;
	//	//		vertex normal_;
	//	//		std::list<segment> intersections_;
	//	//	};


	//		


	//		//class triangleTriangle
	//		//{
	//		//public:
	//		//	triangleTriangle()
	//		//		: A_(0), B_(0)
	//		//	{
	//		//	}

	//		//	triangleTriangle(const triangle* A, const triangle* B, const vertex& normal)
	//		//		: A_(A), B_(B), Bnormal_(normal)
	//		//	{
	//		//		directions_ = calculateDirections(*A, *B);
	//		//	}

	//		//	bool notIntersecting() const 
	//		//	{
	//		//		return
	//		//		// if either there is only a single vertex of A on the B plane... 
	//		//			(((!directions_.x_ && directions_.y_ && directions_.z_) || (directions_.x_ && !directions_.y_ && directions_.z_) || (directions_.x_ && directions_.y_ && !directions_.z_))
	//		//		// or all directions are positive or all directions negative...
	//		//			|| (directions_.x_ > 0 && directions_.y_ > 0 && directions_.z_ > 0) || (directions_.x_ < 0 && directions_.y_ < 0 && directions_.z_ < 0));
	//		//	}

	//		//	bool onPlane() const { return (!directions_.x_ && !directions_.y_ && !directions_.z_); }

	//		//	bool calculateIntersection(segment& result) const
	//		//	{
	//		//		// what type of intersection is this? is it a single edge on the plane, or a clean two edges intersecting the plane...

	//		//		// Triangle A edge ab on plane...
	//		//		if (!directions_.x_ && !directions_.y_ && directions_.z_)
	//		//		{
	//		//			if (!calculateIntersectionEdge(A_->a_, A_->b_, result))
	//		//				return false;
	//		//		}
	//		//		// Triangle A edge bc on plane...
	//		//		else if (directions_.x_ && !directions_.y_ && !directions_.z_)
	//		//		{
	//		//			if (!calculateIntersectionEdge(A_->b_, A_->c_, result))
	//		//				return false;
	//		//		}
	//		//		// Triangle A edge ca on plane...
	//		//		else if (!directions_.x_ && directions_.y_ && !directions_.z_)
	//		//		{
	//		//			if (!calculateIntersectionEdge(A_->a_, A_->c_, result))
	//		//				return false;
	//		//		}
	//		//		// Or there is a clean two edges of A intersecting B plane...
	//		//		else
	//		//		{
	//		//			if ((directions_.x_ > 0 && directions_.y_ > 0 && directions_.z_ < 0) || (directions_.x_ < 0 && directions_.y_ < 0 && directions_.z_ > 0))
	//		//			{
	//		//				if (!calculateTwoEdgeIntersections(A_->a_, A_->b_, A_->c_, Bnormal_, B_->a_, result))
	//		//					return false;
	//		//			}
	//		//			else if ((directions_.x_ > 0 && directions_.y_ < 0 && directions_.z_ > 0) || (directions_.x_ < 0 && directions_.y_ > 0 && directions_.z_ < 0))
	//		//			{
	//		//				if (!calculateTwoEdgeIntersections(A_->a_, A_->c_, A_->b_, Bnormal_, B_->a_, result))
	//		//					return false;
	//		//			}
	//		//			else if ((directions_.x_ < 0 && directions_.y_ > 0 && directions_.z_ > 0) || (directions_.x_ > 0 && directions_.y_ < 0 && directions_.z_ < 0))
	//		//			{
	//		//				if (!calculateTwoEdgeIntersections(A_->b_, A_->c_, A_->a_, Bnormal_, B_->a_, result))
	//		//					return false;
	//		//			}
	//		//			else
	//		//				throw std::exception("intersection is not triangleTriangle intersection.");
	//		//		}

	//		//		return true;
	//		//	}


	//		//	void calculateCoplanarIntersection(std::pair<std::list<segment>, std::list<segment>>& result) const
	//		//	{
	//		//		// calculate the intersection of Aab, and B


	//		//		
	//		//		
	//		//	}


	//		//private:
	//		//	vertex calculateDirections(const triangle& A, const triangle& B)
	//		//	{
	//		//		double d1 = directionFromPlane(B.a_, Bnormal_, A.a_);
	//		//		double d2 = directionFromPlane(B.a_, Bnormal_, A.b_);
	//		//		double d3 = directionFromPlane(B.a_, Bnormal_, A.c_);
	//		//		return vertex(d1, d2, d3);
	//		//	}

	//		//	bool segmentIsValid(const segment& s) const
	//		//	{
	//		//		return !equals(s.first, s.second);
	//		//	}

	//		//	bool calculateTwoEdgeIntersections(const vertex& major1, const vertex& major2, const vertex& minor, const vertex& pP, const vertex& pN, segment& result) const
	//		//	{
	//		//		// Calculate the intersection points...
	//		//		vertex i, j;

	//		//		{
	//		//			vertex rD = unitise(subtract(minor, major1));
	//		//			double dotrDpN = dot(rD, Bnormal_);
	//		//			if (std::isnan(dotrDpN) || !dotrDpN)
	//		//				throw std::exception("no intersection.");
	//		//			i = rayPlaneIntersection(major1, rD, B_->a_, Bnormal_, dotrDpN);
	//		//		}
	//		//		{
	//		//			vertex rD = unitise(subtract(minor, major2));
	//		//			double dotrDpN = dot(rD, Bnormal_);
	//		//			if (std::isnan(dotrDpN) || !dotrDpN)
	//		//				throw std::exception("no intersection.");
	//		//			j = rayPlaneIntersection(major2, rD, B_->a_, Bnormal_, dotrDpN);
	//		//		}

	//		//		// We have two intersections... we then need to create an intersection edge on the B plane...
	//		//		return calculateIntersectionEdge(i, j, result);
	//		//	}

	//		//	bool calculateIntersectionEdge(const vertex& i, const vertex& j, segment& result) const
	//		//	{
	//		//		bool iInB = pointWithinTriangle(B_->a_, B_->b_, B_->c_, i);
	//		//		bool jInB = pointWithinTriangle(B_->a_, B_->b_, B_->c_, j);

	//		//		// if both intersection points are within B, this is the segment...
	//		//		if (iInB && jInB)
	//		//			result = segment(i, j);
	//		//		// Otherwise...
	//		//		else
	//		//		{
	//		//			std::list<vertex> triangleEdgeSegmentIntersections = calculateTriangleEdgesSegmentIntersection(*B_, i, j);

	//		//			// If neither intersection points are within B...
	//		//			if (!iInB && !jInB)
	//		//			{
	//		//				// no points or a single point intersection edge is considered outside B entirely...
	//		//				if (triangleEdgeSegmentIntersections.size() <= 1)
	//		//					return false;
	//		//				if (triangleEdgeSegmentIntersections.size() > 2)
	//		//					throw std::exception("there should be exactly 2 intersections between segment and triangle edges in this case.");
	//		//				result = segment(triangleEdgeSegmentIntersections.front(), triangleEdgeSegmentIntersections.back());
	//		//			}
	//		//			else
	//		//			{
	//		//				if (triangleEdgeSegmentIntersections.size() != 1)
	//		//					throw std::exception("there should be only intersection between segment and triangle edges in this case.");
	//		//				result = segment(triangleEdgeSegmentIntersections.front(), iInB ? i : j);
	//		//			}
	//		//		}

	//		//		return segmentIsValid(result);
	//		//	}

	//		//	std::list<vertex> calculateTriangleEdgesSegmentIntersection(const triangle& t, const vertex& i, const vertex& j) const
	//		//	{
	//		//		std::list<vertex> intersections;

	//		//		vertex ij = unitise(subtract(j, i));

	//		//		// ab edge...
	//		//		{
	//		//			vertex ab = subtract(t.b_, t.a_);
	//		//			vertex abNormal = cross(ab, Bnormal_);
	//		//			double abDotrDpN = dot(ij, abNormal);
	//		//			if (!std::isnan(abDotrDpN) && abDotrDpN)
	//		//			{
	//		//				vertex ip = rayPlaneIntersection(i, ij, t.a_, abNormal, abDotrDpN);
	//		//				if (pointOnLineSegment(i, j, ip) && pointOnLineSegment(t.a_, t.b_, ip))
	//		//					intersections.push_back(ip);
	//		//			}
	//		//				
	//		//		} 
	//		//		// bc edge...
	//		//		{
	//		//			vertex bc = subtract(t.c_, t.b_);
	//		//			vertex bcNormal = cross(bc, Bnormal_);
	//		//			double bcDotrDpN = dot(ij, bcNormal);
	//		//			if (!std::isnan(bcDotrDpN) && bcDotrDpN)
	//		//			{
	//		//				vertex ip = rayPlaneIntersection(i, ij, t.b_, bcNormal, bcDotrDpN);
	//		//				if (pointOnLineSegment(i, j, ip) && pointOnLineSegment(t.b_, t.c_, ip))
	//		//					intersections.push_back(ip);
	//		//			}
	//		//		}
	//		//		// ca edge...
	//		//		{
	//		//			vertex ca = subtract(t.a_, t.c_);
	//		//			vertex caNormal = cross(ca, Bnormal_);
	//		//			double caDotrDpN = dot(ij, caNormal);
	//		//			if (!std::isnan(caDotrDpN) && caDotrDpN)
	//		//			{
	//		//				vertex ip = rayPlaneIntersection(i, ij, t.c_, caNormal, caDotrDpN);
	//		//				if (pointOnLineSegment(i, j, ip) && pointOnLineSegment(t.c_, t.a_, ip))
	//		//					intersections.push_back(ip);
	//		//			}
	//		//		}
	//		//	
	//		//		return intersections;
	//		//	}


	//		//	vertex directions_;
	//		//	vertex Bnormal_;

	//		//	const triangle* A_;
	//		//	const triangle* B_;
	//		//};

	//		


	//	//}	// namespace intersection
	//	

	//	// intersect triangles
	//	//void mergeIntersectTriangles(std::vector<intersection::retessellatedTriangle>& aTriangles, std::vector<intersection::retessellatedTriangle>& bTriangles)
	//	//{
	//	//	#pragma omp parallel for
	//	//	for (int i = 0; i < aTriangles.size(); ++i)
	//	//	{
	//	//		for (int j = 0; j < bTriangles.size(); ++j)
	//	//		{
	//	//			if (i == j)
	//	//				continue;

	//	//			intersection::retessellatedTriangle& A = aTriangles[i];
	//	//			intersection::retessellatedTriangle& B = bTriangles[j];
	//	//			
	//	//			intersection::triangleTriangle triangleTriangleIntersection(A.getTriangle(), B.getTriangle(), B.getNormal());

	//	//			// do they intersect?
	//	//			if (triangleTriangleIntersection.notIntersecting())
	//	//				continue;
	//	//			
	//	//			// is the intersection coplanar?
	//	//			if (triangleTriangleIntersection.onPlane())
	//	//			{
	//	//				// caluclate the intersection points and edges for both triangles...
	//	//				std::pair<std::list<segment>, std::list<segment>> coplanarResult;
	//	//				triangleTriangleIntersection.calculateCoplanarIntersection(coplanarResult);

	//	//				if (!coplanarResult.first.empty() || !coplanarResult.second.empty())
	//	//				{
	//	//					#pragma omp critical
	//	//					{
	//	//						A.addIntersection(coplanarResult.first);
	//	//						B.addIntersection(coplanarResult.second);
	//	//					}
	//	//				}

	//	//				
	//	//			}
	//	//			// otherwise they intersect, but they are not coplanar...
	//	//			else
	//	//			{
	//	//				// Calculate the intersection edge... this should be the same for both triangles...
	//	//				segment result;
	//	//				if (triangleTriangleIntersection.calculateIntersection(result))
	//	//				{
	//	//					#pragma omp critical
	//	//					{
	//	//						A.addIntersection(result);
	//	//						B.addIntersection(result);
	//	//					}
	//	//					
	//	//				}
	//	//			}

	//	//		}
	//	//	}
	//	//}



	//	


	//	//void merge(const mesh& A, const mesh& B, mergeResult& result)
	//	//{
	//	//	std::vector<intersection::retessellatedTriangle> aTriangles(A.size(), intersection::retessellatedTriangle());
	//	//	std::vector<intersection::retessellatedTriangle> bTriangles(B.size(), intersection::retessellatedTriangle());

	//	//	//#pragma omp parallel for
	//	//	for (int t = 0; t < A.size(); ++t)
	//	//		aTriangles[t] = intersection::retessellatedTriangle(&A[t], normal(A[t]));

	//	//	//#pragma omp parallel for
	//	//	for (int t = 0; t < B.size(); ++t)
	//	//		bTriangles[t] = intersection::retessellatedTriangle(&B[t], normal(B[t]));

	//	//	mergeIntersectTriangles(aTriangles, bTriangles);

	//	//	// finalise all the triangles and get our total number of triangles...


	//	//	// for each triangle check if it is inside/outside or onplane...



	//	//	// put the triangles into the correct container...






	//	//	// We then finalise the triangles... and prep mesh results... look at this threading... need better streamlining...
	//	//	{
	//	//		bsp bspB(B);
	//	//		result.first.reset(new intersection::mergeIntersectResult());

	//	//		//#pragma omp parallel for
	//	//		for (int t = 0; t < A.size(); ++t)
	//	//		{
	//	//			if (aTriangles[t].needsFinalising())
	//	//				aTriangles[t].finalise(*result.first, bspB);
	//	//			else
	//	//				result.first->addTriangle(*aTriangles[t].getTriangle(), bspB);
	//	//		}
	//	//	}
	//	//	{
	//	//		bsp bspA(A);
	//	//		result.second.reset(new intersection::mergeIntersectResult());

	//	//		//#pragma omp parallel for
	//	//		for (int t = 0; t < B.size(); ++t)
	//	//		{
	//	//			if (bTriangles[t].needsFinalising())
	//	//				bTriangles[t].finalise(*result.second, bspA);
	//	//			else
	//	//				result.second->addTriangle(*bTriangles[t].getTriangle(), bspA);
	//	//		}
	//	//	}
	//	//}

	//	
	//}


	//

	
}

#endif // CSG_HPP