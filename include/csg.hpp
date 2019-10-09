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
#include <optional>
#include <limits>
#include <assert.h>

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
		
		// vector operations...
		static double magnitude(const vertex& v) { return sqrt((v.x_ * v.x_) + (v.y_ * v.y_) + (v.z_ * v.z_)); }
		static vertex unitise(const vertex& v) { double oneOverMagnitude = 1.0 / magnitude(v); return vertex(v.x_ * oneOverMagnitude, v.y_ * oneOverMagnitude, v.z_ * oneOverMagnitude); }
		static vertex cross(const vertex& a, const vertex& b) { return vertex((a.y_ * b.z_) - (a.z_ * b.y_), (a.z_ * b.x_) - (a.x_ * b.z_), (a.x_ * b.y_) - (a.y_ * b.x_)); }
		static double dot(const vertex& a, const vertex& b) { return (a.x_ * b.x_) + (a.y_ * b.y_) + (a.z_ * b.z_); }
		static bool equals(const vertex& a, const vertex& b, double e) { return (abs(a.x_ - b.x_) < e && abs(a.y_ - b.y_) < e && abs(a.z_ - b.z_) < e); }
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
		static bool pointOnLineSegment(const vertex& i, const vertex& j, const vertex& p) { return abs(magnitude(subtract(p, i)) + magnitude(subtract(p, j)) - magnitude(subtract(i, j))) < 0.001; }
		static vertex midPoint(const vertex& i, const vertex& j) { return vertex((i.x_ + j.x_) * 0.5, (i.y_ + j.y_) * 0.5, (i.z_ + j.z_) * 0.5); }
		static double arcCosine(double val)
		{
			if (val <= -1.0f)
				return pi;
			if (val >= 1.0f)
				return 0;
			return acos(val);
		}
		static vertex rodrigues(const vertex& v, const vertex& axis, double angle)
		{
			double c = cos(angle);
			vertex vRot(v.x_ * c, v.y_ * c, v.z_ * c);
			vRot = add(scale(cross(axis, v), sin(angle)), vRot);
			return add(scale(axis, dot(axis, v) * (1 - c)), vRot);
		}
		static void flip(triangle& t)
		{
			vertex tmp = t.c_; t.c_ = t.a_; t.a_ = tmp;
		}

		// bsp tree...
		class bsp
		{
		public:
			class node : public triangle
			{
			public:
				node(const triangle& t) : triangle(t.a_, t.b_, t.c_), normal_(normal(t)) { if (std::isnan(normal_.x_) || std::isnan(normal_.y_) || std::isnan(normal_.z_)) { throw std::exception("Unable to calculate normal correctly."); } }
				vertex triangleIntersection(const triangle& t) const { return vertex(directionFromPlane(p(), n(), t.a_), directionFromPlane(p(), n(), t.b_), directionFromPlane(p(), n(), t.c_)); }
				vertex calculateIntersection(const vertex& a, const vertex& b) { vertex rD = unitise(subtract(b, a)); return rayPlaneIntersection(a, rD, a_, normal_, dot(rD, normal_)); }
				const vertex& p() const { return a_; }
				const vertex& n() const { return normal_; }
				std::unique_ptr<node> inside_;
				std::unique_ptr<node> outside_;
			private:
				vertex normal_;
			};

			bsp(const mesh& m)
			{
				// create a working list of the triangles. We validate as we go along.
				std::list<triangle> workingTriangles(m.begin(), m.end());

				// get the first valid triangle...
				while (!validateTriangle(workingTriangles.front()))
					workingTriangles.pop_front();

				// create out node with it...
				root_.reset(new node(workingTriangles.front()));

				while (!workingTriangles.empty())
				{
					// get the next valid working triangle...
					while (!validateTriangle(workingTriangles.front()))
						workingTriangles.pop_front();
					std::list<triangle>::iterator currentTriangle = workingTriangles.begin();

					// traverse the tree to find the location to place this triangle/node.
					node* currentNode = root_.get();

					while (currentNode)
					{
						// calculate the currentTriangle intersection of the currentNode.
						vertex nodeIntersectionDirections = currentNode->triangleIntersection(*currentTriangle);

						// if the current triangle is outside the current node (triangle)...
						if (nodeIntersectionDirections.x_ > 0 && nodeIntersectionDirections.y_ > 0 && nodeIntersectionDirections.z_ > 0)
							currentNode = followOutsideNode(currentNode, *currentTriangle);
						// if the current triangle is inside or on the plane of the current node (triangle)...
						else if (nodeIntersectionDirections.x_ <= 0 && nodeIntersectionDirections.y_ <= 0 && nodeIntersectionDirections.z_ <= 0)
							currentNode = followInsideNode(currentNode, *currentTriangle);
						else
						{
							// otherwise the current triangle straddles the current node somehow so we need to split the triangle...
							splitResult splitTriangles = splitTriangle(nodeIntersectionDirections, currentNode, *currentTriangle);

							// invalid triangles won't be added, so we need to check there is only one triangle in the split...
							unsigned int workingTriangleCount = static_cast<unsigned int>(workingTriangles.size());

							if (splitTriangles.inside_.size() == 1 && splitTriangles.outside_.empty())
								currentNode = followInsideNode(currentNode, *currentTriangle);
							else if (splitTriangles.inside_.empty() && splitTriangles.outside_.size() == 1)
								currentNode = followOutsideNode(currentNode, *currentTriangle);
							else
							{
								// Add both these sets of triangles to our working list for tree traversal...
								workingTriangles.insert(workingTriangles.begin(), splitTriangles.inside_.begin(), splitTriangles.inside_.end());
								workingTriangles.insert(workingTriangles.begin(), splitTriangles.outside_.begin(), splitTriangles.outside_.end());

								// remove this triangle for our working list...
								currentNode = 0;
							}
						}

						// if the currentNode has been set to null, this means one or more working triangles have been added to the result.
						// We need to remove this current triangle from the working triangles list.
						if (!currentNode)
							workingTriangles.erase(currentTriangle);
					}
				}
			}

			enum IOOPflag
			{
				outside,
				inside,
				onPlane
			};

			IOOPflag ioop(const vertex& v)
			{
				// Traverse to find out if this point is inside infront or onplane...
				node* currentNode = root_.get();
				while (currentNode)
				{
					double direction = directionFromPlane(currentNode->p(), currentNode->n(), v);
					if (direction > 0)
					{
						if (currentNode->outside_)
							currentNode = currentNode->outside_.get();
						else
							return IOOPflag::outside;
					}
					else if (direction < 0)
					{
						if (currentNode->inside_)
							currentNode = currentNode->inside_.get();
						else
							return IOOPflag::inside;
					}
					else
					{
						if (pointWithinTriangle(currentNode->a_, currentNode->b_, currentNode->c_, v))
							return IOOPflag::onPlane;

						if (currentNode->inside_.get())
							currentNode = currentNode->inside_.get();
						else
							return IOOPflag::outside;
					}
				}
				throw std::exception("cannot deduce if point is inside or outside.");
			}

		private:
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

			struct splitResult
			{
				std::list<triangle> outside_;
				std::list<triangle> inside_;
			};

			bool validateTriangle(const triangle& t) const
			{
				return (!equals(t.a_, t.b_, 0.001) && !equals(t.a_, t.c_, 0.001) && !equals(t.c_, t.b_, 0.001));
			}

			splitResult splitTriangle(const vertex& splitInformation, const node* nde, const triangle& tri) const
			{
				splitResult result;

				// If there is at least one point on the plane... since this is split, this must mean that the triangle straddles...
				// calculate the single intersection point and the two triangles...
				if (!splitInformation.x_ || !splitInformation.y_ || !splitInformation.z_)
				{
					// if vertex A is on the plane, intersection point must be along BC...
					if (!splitInformation.x_)
					{
						vertex intersectionPoint = rayPlaneIntersection(tri.b_, unitise(subtract(tri.c_, tri.b_)), nde->p(), nde->n(), dot(tri.b_, nde->n()));
						if (splitInformation.y_ > 0)
						{
							result.outside_.push_back(triangle(tri.a_, tri.b_, intersectionPoint));
							result.inside_.push_back(triangle(intersectionPoint, tri.c_, tri.a_));
						}
						else
						{
							result.inside_.push_back(triangle(tri.a_, tri.b_, intersectionPoint));
							result.outside_.push_back(triangle(intersectionPoint, tri.c_, tri.a_));
						}
					}
					// if vertex B is on the plane, intersection point must be along CA...
					else if (!splitInformation.y_)
					{
						vertex intersectionPoint = rayPlaneIntersection(tri.c_, unitise(subtract(tri.a_, tri.c_)), nde->p(), nde->n(), dot(tri.b_, nde->n()));
						if (splitInformation.z_ > 0)
						{
							result.outside_.push_back(triangle(tri.b_, tri.c_, intersectionPoint));
							result.inside_.push_back(triangle(intersectionPoint, tri.a_, tri.b_));
						}
						else
						{
							result.inside_.push_back(triangle(tri.b_, tri.c_, intersectionPoint));
							result.outside_.push_back(triangle(intersectionPoint, tri.a_, tri.b_));
						}
					}
					// otherwise C must be on the plane, intersection point must be along AB...
					else if (!splitInformation.z_)
					{
						vertex intersectionPoint = rayPlaneIntersection(tri.a_, unitise(subtract(tri.b_, tri.a_)), nde->p(), nde->n(), dot(tri.b_, nde->n()));
						if (splitInformation.x_ > 0)
						{
							result.outside_.push_back(triangle(tri.c_, tri.a_, intersectionPoint));
							result.inside_.push_back(triangle(intersectionPoint, tri.b_, tri.c_));
						}
						else
						{
							result.inside_.push_back(triangle(tri.c_, tri.a_, intersectionPoint));
							result.outside_.push_back(triangle(intersectionPoint, tri.b_, tri.c_));
						}
					}
					else
						throw std::exception("Unable to split triangle");
				}
				else
				{
					// Otherwise this triangle straddles the plane along two edges... resulting in 3 triangles...
					if ((splitInformation.x_ > 0 && splitInformation.y_ > 0) || (splitInformation.x_ < 0 && splitInformation.y_ < 0))
					{
						vertex intersection1 = rayPlaneIntersection(tri.b_, unitise(subtract(tri.c_, tri.b_)), nde->p(), nde->n(), dot(tri.b_, nde->n()));
						vertex intersection2 = rayPlaneIntersection(tri.c_, unitise(subtract(tri.a_, tri.c_)), nde->p(), nde->n(), dot(tri.b_, nde->n()));

						result.outside_.push_back(triangle(tri.a_, tri.b_, intersection1));
						result.inside_.push_back(triangle(intersection1, tri.c_, intersection2));
						result.inside_.push_back(triangle(intersection2, tri.a_, intersection1));
					}
					else if ((splitInformation.y_ > 0 && splitInformation.z_ > 0) || (splitInformation.y_ < 0 && splitInformation.z_ < 0))
					{
						vertex intersection1 = rayPlaneIntersection(tri.c_, unitise(subtract(tri.a_, tri.c_)), nde->p(), nde->n(), dot(tri.b_, nde->n()));
						vertex intersection2 = rayPlaneIntersection(tri.a_, unitise(subtract(tri.b_, tri.a_)), nde->p(), nde->n(), dot(tri.b_, nde->n()));

						result.outside_.push_back(triangle(tri.b_, tri.c_, intersection1));
						result.inside_.push_back(triangle(intersection1, tri.a_, intersection2));
						result.inside_.push_back(triangle(intersection2, tri.b_, intersection1));
					}
					else if ((splitInformation.x_ > 0 && splitInformation.z_ > 0) || (splitInformation.x_ < 0 && splitInformation.z_ < 0))
					{
						vertex intersection1 = rayPlaneIntersection(tri.a_, unitise(subtract(tri.b_, tri.a_)), nde->p(), nde->n(), dot(tri.b_, nde->n()));
						vertex intersection2 = rayPlaneIntersection(tri.b_, unitise(subtract(tri.c_, tri.b_)), nde->p(), nde->n(), dot(tri.b_, nde->n()));

						result.outside_.push_back(triangle(tri.c_, tri.a_, intersection1));
						result.inside_.push_back(triangle(intersection1, tri.b_, intersection2));
						result.inside_.push_back(triangle(intersection2, tri.c_, intersection1));
					}
					else
						throw std::exception("Unable to split triangle");
				}

				for (std::list<triangle>::iterator itr = result.inside_.begin(); itr != result.inside_.end();)
				{
					if (!validateTriangle(*itr))
						itr = result.inside_.erase(itr);
					else
						++itr;
				}
				for (std::list<triangle>::iterator itr = result.outside_.begin(); itr != result.outside_.end();)
				{
					if (!validateTriangle(*itr))
						itr = result.outside_.erase(itr);
					else
						++itr;
				}
				return result;
			}

			
			std::unique_ptr<node> root_;
		};

		// delaunay...
		namespace Delaunay
		{
			const static double pi = 3.14159265358979323846;

			static bool almost_equal(const double& x, const double& y, int ulp = 2)
			{
				// the machine epsilon has to be scaled to the magnitude of the values used
				// and multiplied by the desired precision in ULPs (units in the last place)
				return std::abs(x - y) <= std::numeric_limits<double>::epsilon() * std::abs(x + y) * static_cast<double>(ulp)
					// unless the result is subnormal
					|| std::abs(x - y) < (std::numeric_limits<double>::min)();
			}

			static double half(const double x)
			{
				return 0.5 * x;
			}

			struct Vector2
			{
				Vector2() = default;
				Vector2(const Vector2 &v) = default;
				Vector2(Vector2&&) = default;
				Vector2(const double vx, const double vy)
					: x(vx), y(vy)
				{
				}

				double dist2(const Vector2 &v) const
				{
					const double dx = x - v.x;
					const double dy = y - v.y;
					return dx * dx + dy * dy;
				}

				double dist(const Vector2 &v) const
				{
					return hypot(x - v.x, y - v.y);
				}

				double norm2() const
				{
					return x * x + y * y;
				}

				Vector2 &operator=(const Vector2& v) = default;

				Vector2 &operator=(Vector2&&) = default;

				bool operator ==(const Vector2 &v) const
				{
					return (x == v.x) && (y == v.y);
				}

				double x;
				double y;
			};

			static bool almost_equal(const Vector2 &v1, const Vector2 &v2, int ulp = 2)
			{
				return almost_equal(v1.x, v2.x, ulp) && almost_equal(v1.y, v2.y, ulp);
			}

			struct Edge
			{
				Edge() = default;
				Edge(const Edge&) = default;
				Edge(Edge&&) = default;

				Edge &operator=(const Edge&) = default;
				Edge &operator=(Edge&&) = default;

				Edge(const Vector2 &v1, const Vector2 &v2)
					: v(&v1), w(&v2)
				{
				}

				Edge(Vector2* v1, Vector2* v2)
					: v(v1), w(v2)
				{
				}

				bool operator==(const Edge &e) const
				{
					return (*(this->v) == *e.v && *(this->w) == *e.w) ||
						(*(this->v) == *e.w && *(this->w) == *e.v);
				}

				const Vector2 *v;
				const Vector2 *w;
				bool isBad = false;
			};

			static bool almost_equal(const Edge &e1, const Edge &e2)
			{
				return	(almost_equal(*e1.v, *e2.v) && almost_equal(*e1.w, *e2.w)) ||
					(almost_equal(*e1.v, *e2.w) && almost_equal(*e1.w, *e2.v));
			}

			struct Triangle
			{
				Triangle() = default;
				Triangle(const Triangle&) = default;
				Triangle(Triangle&&) = default;
				Triangle(const Vector2 &v1, const Vector2 &v2, const Vector2 &v3)
					: a(&v1), b(&v2), c(&v3), isBad(false)
				{}

				bool containsVertex(const Vector2 &v) const
				{
					return almost_equal(*a, v) || almost_equal(*b, v) || almost_equal(*c, v);
				}

				bool constainsEdge(const Edge& e) const
				{
					return containsVertex(*e.v) && containsVertex(*e.w);
				}

				bool circumCircleContains(const Vector2 &v) const
				{
					const double ab = a->norm2();
					const double cd = b->norm2();
					const double ef = c->norm2();

					const double ax = a->x;
					const double ay = a->y;
					const double bx = b->x;
					const double by = b->y;
					const double cx = c->x;
					const double cy = c->y;

					const double circum_x = (ab * (cy - by) + cd * (ay - cy) + ef * (by - ay)) / (ax * (cy - by) + bx * (ay - cy) + cx * (by - ay));
					const double circum_y = (ab * (cx - bx) + cd * (ax - cx) + ef * (bx - ax)) / (ay * (cx - bx) + by * (ax - cx) + cy * (bx - ax));

					const Vector2 circum(half(circum_x), half(circum_y));
					const double circum_radius = a->dist2(circum);
					const double dist = v.dist2(circum);
					return dist <= circum_radius;
				}

				Triangle &operator=(const Triangle&) = default;
				Triangle &operator=(Triangle&&) = default;

				bool operator ==(const Triangle &t) const
				{
					return	(*this->a == *t.a || *this->a == *t.b || *this->a == *t.c) &&
						(*this->b == *t.a || *this->b == *t.b || *this->b == *t.c) &&
						(*this->c == *t.a || *this->c == *t.b || *this->c == *t.c);
				}

				const Vector2 *a;
				const Vector2 *b;
				const Vector2 *c;
				bool isBad = false;
			};

			static bool almost_equal(const Triangle &t1, const Triangle &t2)
			{
				return	(almost_equal(*t1.a, *t2.a) || almost_equal(*t1.a, *t2.b) || almost_equal(*t1.a, *t2.c)) &&
					(almost_equal(*t1.b, *t2.a) || almost_equal(*t1.b, *t2.b) || almost_equal(*t1.b, *t2.c)) &&
					(almost_equal(*t1.c, *t2.a) || almost_equal(*t1.c, *t2.b) || almost_equal(*t1.c, *t2.c));
			}

			static double angleDifference(const Vector2& a, const Vector2& b)
			{
				double angle = atan2(b.y, b.x) - atan2(a.y, a.x);

				if (angle > pi)
					angle -= 2 * pi;
				else if (angle <= -pi)
					angle += 2 * pi;

				return angle;
			}

			static std::optional<Delaunay::Vector2> segmentSegmentIntersection(const Vector2& a, const Vector2& b, const Vector2& i, const Vector2& j)
			{
				double s1_x, s1_y, s2_x, s2_y;
				s1_x = b.x - a.x;     s1_y = b.y - a.y;
				s2_x = j.x - i.x;     s2_y = j.y - i.y;

				double sDenom = (-s2_x * s1_y + s1_x * s2_y);
				double tDenom = (-s2_x * s1_y + s1_x * s2_y);

				if (!sDenom || !tDenom)
					return std::optional<Vector2>();

				double s = (-s1_y * (a.x - i.x) + s1_x * (a.y - i.y)) / sDenom;
				double t = (s2_x * (a.y - i.y) - s2_y * (a.x - i.x)) / tDenom;

				if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
					return std::optional<Vector2>(Vector2(a.x + (t * s1_x), a.y + (t * s1_y)));

				return std::optional<Vector2>();
			}

			class Tessellator
			{
			public:
				const std::list<Triangle>& triangulate(const std::vector<Vector2> &vertices)
				{
					// Store the vertices locally
					vertices_ = vertices;

					// Determinate the super triangle
					double minX = vertices_[0].x;
					double minY = vertices_[0].y;
					double maxX = minX;
					double maxY = minY;

					for (std::size_t i = 0; i < vertices_.size(); ++i)
					{
						if (vertices_[i].x < minX) minX = vertices_[i].x;
						if (vertices_[i].y < minY) minY = vertices_[i].y;
						if (vertices_[i].x > maxX) maxX = vertices_[i].x;
						if (vertices_[i].y > maxY) maxY = vertices_[i].y;
					}

					const double dx = maxX - minX;
					const double dy = maxY - minY;
					const double deltaMax = (std::max)(dx, dy);
					const double midx = half(minX + maxX);
					const double midy = half(minY + maxY);

					const Delaunay::Vector2 p1(midx - 20 * deltaMax, midy - deltaMax);
					const Delaunay::Vector2 p2(midx, midy + 20 * deltaMax);
					const Delaunay::Vector2 p3(midx + 20 * deltaMax, midy - deltaMax);

					// Create a list of triangles, and add the supertriangle in it
					triangles_.push_back(Triangle(p1, p2, p3));

					for (auto p = begin(vertices_); p != end(vertices_); p++)
					{
						std::vector<Edge> polygon;

						for (auto & t : triangles_)
						{
							if (t.circumCircleContains(*p))
							{
								t.isBad = true;
								polygon.push_back(Edge{ *t.a, *t.b });
								polygon.push_back(Edge{ *t.b, *t.c });
								polygon.push_back(Edge{ *t.c, *t.a });
							}
						}

						triangles_.erase(std::remove_if(begin(triangles_), end(triangles_), [](Triangle&t) {
							return t.isBad;
							}), end(triangles_));

						for (auto e1 = begin(polygon); e1 != end(polygon); ++e1)
						{
							for (auto e2 = e1 + 1; e2 != end(polygon); ++e2)
							{
								if (almost_equal(*e1, *e2))
								{
									e1->isBad = true;
									e2->isBad = true;
								}
							}
						}

						polygon.erase(std::remove_if(begin(polygon), end(polygon), [](Edge&e) {
							return e.isBad;
							}), end(polygon));

						for (const auto e : polygon)
							triangles_.push_back(Triangle(*e.v, *e.w, *p));
					}

					triangles_.erase(std::remove_if(begin(triangles_), end(triangles_), [p1, p2, p3](Triangle&t) {
						return t.containsVertex(p1) || t.containsVertex(p2) || t.containsVertex(p3);
						}), end(triangles_));

					for (const auto t : triangles_)
					{
						edges_.push_back(Edge{ *t.a, *t.b });
						edges_.push_back(Edge{ *t.b, *t.c });
						edges_.push_back(Edge{ *t.c, *t.a });
					}

					return triangles_;
				}

				const std::list<Delaunay::Triangle>& getTriangles() const { return triangles_; }
				const std::list<Delaunay::Edge>& getEdges() const { return edges_; }
				const std::vector<Delaunay::Vector2>& getVertices() const { return vertices_; }

				void addConstraint(const std::pair<unsigned int, unsigned int>& e)
				{
					// check if we already have this edge...
					Vector2* e1 = &vertices_[e.first];
					Vector2* e2 = &vertices_[e.second];

					if (std::find_if(edges_.begin(), edges_.end(), [&e1, &e2](const Edge& edge) { return ((e1 == edge.v && e2 == edge.w) || (e1 == edge.w && e2 == edge.v)); }) == edges_.end())
					{
						Vector2 constraintEdge(e2->x - e1->x, e2->y - e1->y);

						// find out which edges are intersected, and for the two vertices of the intersected edge, find out which side of the constraint they are on...
						std::list<Vector2> leftSide, rightSide;
						for (std::list<Edge>::iterator ee = edges_.begin(); ee != edges_.end();)
						{
							if (*ee->v == *e1 || *ee->w == *e1 || *ee->v == *e2 || *ee->w == *e2)
							{
								++ee;
								continue;
							}


							double vAngle = angleDifference(Vector2(ee->v->x - e1->x, ee->v->y - e1->y), constraintEdge);
							double wAngle = angleDifference(Vector2(ee->w->x - e2->x, ee->w->y - e2->y), constraintEdge);

							if ((vAngle < 0 && wAngle > 0) || (vAngle > 0 && wAngle < 0))
							{
								// do the addtional check to see if this constrain edge is intersected...
								if (segmentSegmentIntersection(*e1, *e2, *ee->v, *ee->w))
								{
									if (vAngle > 0)
										leftSide.push_back(*ee->v);
									else
										rightSide.push_back(*ee->v);

									if (wAngle > 0)
										leftSide.push_back(*ee->w);
									else
										rightSide.push_back(*ee->w);

									// and any triangles that use this edge...
									triangles_.erase(std::remove_if(begin(triangles_), end(triangles_), [&ee](Triangle& tri) { return tri.constainsEdge(*ee); }), end(triangles_));

									// remove the edge from our list...
									ee = edges_.erase(ee);

									// next edge...
									continue;
								}
							}

							++ee;
						}

						// add the constaint edge to both sides and remove the duplicates of both sides.
						leftSide.push_back(*e1);
						leftSide.push_back(*e2);
						rightSide.push_back(*e1);
						rightSide.push_back(*e2);

						// tessellate each subset...
						Tessellator leftTessellation, rightTessellation;

						// remove duplicates and tessellation the left side...
						leftSide.sort([](const Vector2& a, const Vector2& b) { if (a.x == b.x) { return a.y < b.y; } else { return a.x < b.x; } });
						leftSide.erase(std::unique(leftSide.begin(), leftSide.end()), leftSide.end());
						leftTessellation.triangulate(std::vector<Vector2>(leftSide.begin(), leftSide.end()));

						// remove duplicates and tessellation the right side...
						rightSide.sort([](const Vector2& a, const Vector2& b) { if (a.x == b.x) { return a.y < b.y; } else { return a.x < b.x; } });
						rightSide.erase(std::unique(rightSide.begin(), rightSide.end()), rightSide.end());
						rightTessellation.triangulate(std::vector<Vector2>(rightSide.begin(), rightSide.end()));

						// add these two subsets to the final results...
						append(leftTessellation);
						append(rightTessellation);
					}
				}

			private:
				Vector2* addVertex(const Vector2& v) { vertices_.push_back(v); return &vertices_.back(); }

				// destructive copy...
				void append(Tessellator& tessellation)
				{
					// create new container...
					std::vector<Vector2> newVertices;

					// copy the vertices across...
					newVertices.insert(newVertices.end(), vertices_.begin(), vertices_.end());
					newVertices.insert(newVertices.end(), tessellation.vertices_.begin(), tessellation.vertices_.end());

					// iterate through the original vertices of this tessellation and set the x value to the index of it.
					for (unsigned int v = 0; v < vertices_.size(); ++v)
						vertices_[v].x = static_cast<double>(v);

					// iterate through the original vertices of the tessellation to append and set the x value to the index of it + offset of original this tessellation vertices.
					unsigned int offset = static_cast<unsigned int>(vertices_.size());
					for (unsigned int v = 0; v < static_cast<unsigned int>(tessellation.vertices_.size()); ++v)
						tessellation.vertices_[v].x = static_cast<double>(v + offset);

					// set all the correct edge triangle pointers. new offset of new container is supplied by 

					// now copy the edges and triangles across to the new container, set the pointer to the new index supplied by x coord.
					triangles_.insert(triangles_.end(), tessellation.triangles_.begin(), tessellation.triangles_.end());
					std::for_each(triangles_.begin(), triangles_.end(),
						[&newVertices](Triangle& t)
						{
							t.a = &newVertices[static_cast<unsigned int>(t.a->x)];
							t.b = &newVertices[static_cast<unsigned int>(t.b->x)];
							t.c = &newVertices[static_cast<unsigned int>(t.c->x)];
						});

					// do the same for the edges...
					edges_.insert(edges_.end(), tessellation.edges_.begin(), tessellation.edges_.end());
					std::for_each(edges_.begin(), edges_.end(),
						[&newVertices](Edge& e)
						{
							e.v = &newVertices[static_cast<unsigned int>(e.v->x)];
							e.w = &newVertices[static_cast<unsigned int>(e.w->x)];
						});

					// swap vertices...
					vertices_.swap(newVertices);
				}

				std::list<Delaunay::Triangle> triangles_;
				std::list<Delaunay::Edge> edges_;
				std::vector<Delaunay::Vector2> vertices_;
			};

		}	// namespace Delaunay

		namespace intersection
		{
			struct mergeIntersectResult
			{
				std::list<triangle> inside_, outside_, onPlane_;

				void addTriangle(const triangle& tri, bsp& ioopCheck)
				{
					bsp::IOOPflag ioopFlag = ioopCheck.ioop(centroid(tri.a_, tri.b_, tri.c_));

					if (ioopFlag == bsp::IOOPflag::outside)
						outside_.push_back(tri);
					else if (ioopFlag == bsp::IOOPflag::inside)
						inside_.push_back(tri);
					else if (ioopFlag == bsp::IOOPflag::onPlane)
						onPlane_.push_back(tri);
					else
						throw std::exception("unable to deduce ioop.");
				}
			};


			class retessellatedTriangle
			{
			public:
				retessellatedTriangle() : triangle_(0) {}

				retessellatedTriangle(const triangle* tri, const vertex& normal) : triangle_(tri), normal_(normal) {}

				bool needsFinalising() const {
					//return intersections_.size() == 1;
					return !intersections_.empty();
				}

				

				void finalise(mergeIntersectResult& result, bsp& bsp)
				{
					// santise the edges, remove the duplicate points...
					for (std::list<segment>::iterator itr = intersections_.begin(); itr != intersections_.end();)
					{
						// check if the mid point of the segment is on the triangle edge...
						vertex segmentMidPoint = midPoint(itr->first, itr->second);
						if (pointOnLineSegment(triangle_->a_, triangle_->b_, segmentMidPoint) ||
							pointOnLineSegment(triangle_->b_, triangle_->c_, segmentMidPoint) ||
							pointOnLineSegment(triangle_->c_, triangle_->a_, segmentMidPoint))
							itr = intersections_.erase(itr);
						else
							++itr;
					}

					Projection projection(normal_, triangle_->a_);

					std::vector<Delaunay::Vector2> vertices;
					std::vector<std::pair<unsigned int, unsigned int>> constraints;
					vertices.push_back(projection.get2D(triangle_->a_));
					vertices.push_back(projection.get2D(triangle_->b_));
					vertices.push_back(projection.get2D(triangle_->c_));
					for (std::list<segment>::iterator itr = intersections_.begin(); itr != intersections_.end(); ++itr)
					{
						constraints.push_back(std::pair<unsigned int, unsigned int>(
							static_cast<unsigned int>(addDelaunayVertex2D(vertices, projection.get2D(itr->first))),
							static_cast<unsigned int>(addDelaunayVertex2D(vertices, projection.get2D(itr->second)))));
					}
						
					// Tesssellate...
					Delaunay::Tessellator tessellator;
					tessellator.triangulate(vertices);

					// Add the constriants...
					for (unsigned int c = 0; c < constraints.size(); ++c)
						tessellator.addConstraint(constraints[c]);

					// populate our result...
					const std::list<Delaunay::Triangle> triangles = tessellator.getTriangles();
					for (std::list<Delaunay::Triangle>::const_iterator tItr = triangles.begin(); tItr != triangles.end(); ++tItr)
					{
						triangle tri(projection.get3D(*tItr->a), projection.get3D(*tItr->b), projection.get3D(*tItr->c));
						if (!equals(normal(tri), normal_))
							flip(tri);
						result.addTriangle(tri, bsp);
					}

				}

				const vertex& getNormal() const { return normal_; }

				void addIntersection(const segment& intersectionEdge)
				{
					intersections_.push_back(intersectionEdge);
				}

				void addIntersection(const std::list<segment>& intersectionEdges)
				{
					intersections_.insert(intersections_.end(), intersectionEdges.begin(), intersectionEdges.end());
				}

				const triangle* getTriangle() const { return triangle_; }

			private:
				int addDelaunayVertex2D(std::vector<Delaunay::Vector2>& vertices, const Delaunay::Vector2& v)
				{
					for (int i = 0; i < vertices.size(); ++i)
					{
						if (vertices[i] == v)
							return i;
					}
					vertices.push_back(v);
					return static_cast<int>(vertices.size() - 1);
				}

				class Projection
				{
				public:
					Projection(const vertex& normal, const vertex& origin)
						:	origin_(origin), angle_(0)
					{
						axis_ = vertex(0, 0, 1.0);
						if (!equals(normal, axis_)&&!equals(vertex(normal.x_, normal.y_, normal.z_), axis_))
						{
							axis_ = cross(normal, vertex(0, 0, 1.0));
							angle_ = arcCosine(dot(normal, vertex(0, 0, 1.0)));
						}
					}

					Delaunay::Vector2 get2D(const vertex& v)
					{
						vertex rotated = rodrigues(subtract(v, origin_), axis_, angle_);
						assert(abs(rotated.z_) < 0.001);
						return Delaunay::Vector2(rotated.x_, rotated.y_);
					}

					vertex get3D(const Delaunay::Vector2& v)
					{
						return add(origin_, rodrigues(vertex(v.x, v.y, 0), axis_, -angle_));
					}

				private:
					vertex axis_, origin_;
					double angle_;
				};

				const triangle* triangle_;
				vertex normal_;
				std::list<segment> intersections_;
			};


			class triangleTriangle
			{
			public:
				triangleTriangle()
					: A_(0), B_(0)
				{
				}

				triangleTriangle(const triangle* A, const triangle* B, const vertex& normal)
					: A_(A), B_(B), Bnormal_(normal)
				{
					directions_ = calculateDirections(*A, *B);
				}

				bool notIntersecting() const 
				{
					return
					// if either there is only a single vertex of A on the B plane... 
						(((!directions_.x_ && directions_.y_ && directions_.z_) || (directions_.x_ && !directions_.y_ && directions_.z_) || (directions_.x_ && directions_.y_ && !directions_.z_))
					// or all directions are positive or all directions negative...
						|| (directions_.x_ > 0 && directions_.y_ > 0 && directions_.z_ > 0) || (directions_.x_ < 0 && directions_.y_ < 0 && directions_.z_ < 0));
				}

				bool onPlane() const { return (!directions_.x_ && !directions_.y_ && !directions_.z_); }

				bool calculateIntersection(segment& result) const
				{
					// what type of intersection is this? is it a single edge on the plane, or a clean two edges intersecting the plane...

					// Triangle A edge ab on plane...
					if (!directions_.x_ && !directions_.y_ && directions_.z_)
					{
						if (!calculateIntersectionEdge(A_->a_, A_->b_, result))
							return false;
					}
					// Triangle A edge bc on plane...
					else if (directions_.x_ && !directions_.y_ && !directions_.z_)
					{
						if (!calculateIntersectionEdge(A_->b_, A_->c_, result))
							return false;
					}
					// Triangle A edge ca on plane...
					else if (!directions_.x_ && directions_.y_ && !directions_.z_)
					{
						if (!calculateIntersectionEdge(A_->a_, A_->c_, result))
							return false;
					}
					// Or there is a clean two edges of A intersecting B plane...
					else
					{
						if ((directions_.x_ > 0 && directions_.y_ > 0 && directions_.z_ < 0) || (directions_.x_ < 0 && directions_.y_ < 0 && directions_.z_ > 0))
						{
							if (!calculateTwoEdgeIntersections(A_->a_, A_->b_, A_->c_, Bnormal_, B_->a_, result))
								return false;
						}
						else if ((directions_.x_ > 0 && directions_.y_ < 0 && directions_.z_ > 0) || (directions_.x_ < 0 && directions_.y_ > 0 && directions_.z_ < 0))
						{
							if (!calculateTwoEdgeIntersections(A_->a_, A_->c_, A_->b_, Bnormal_, B_->a_, result))
								return false;
						}
						else if ((directions_.x_ < 0 && directions_.y_ > 0 && directions_.z_ > 0) || (directions_.x_ > 0 && directions_.y_ < 0 && directions_.z_ < 0))
						{
							if (!calculateTwoEdgeIntersections(A_->b_, A_->c_, A_->a_, Bnormal_, B_->a_, result))
								return false;
						}
						else
							throw std::exception("intersection is not triangleTriangle intersection.");
					}

					return true;
				}


				void calculateCoplanarIntersection(std::pair<std::list<segment>, std::list<segment>>& result) const
				{
					// calculate the intersection of Aab, and B


					
					
				}


			private:
				vertex calculateDirections(const triangle& A, const triangle& B)
				{
					double d1 = directionFromPlane(B.a_, Bnormal_, A.a_);
					double d2 = directionFromPlane(B.a_, Bnormal_, A.b_);
					double d3 = directionFromPlane(B.a_, Bnormal_, A.c_);
					return vertex(d1, d2, d3);
				}

				bool segmentIsValid(const segment& s) const
				{
					return !equals(s.first, s.second);
				}

				bool calculateTwoEdgeIntersections(const vertex& major1, const vertex& major2, const vertex& minor, const vertex& pP, const vertex& pN, segment& result) const
				{
					// Calculate the intersection points...
					vertex i, j;

					{
						vertex rD = unitise(subtract(minor, major1));
						double dotrDpN = dot(rD, Bnormal_);
						if (std::isnan(dotrDpN) || !dotrDpN)
							throw std::exception("no intersection.");
						i = rayPlaneIntersection(major1, rD, B_->a_, Bnormal_, dotrDpN);
					}
					{
						vertex rD = unitise(subtract(minor, major2));
						double dotrDpN = dot(rD, Bnormal_);
						if (std::isnan(dotrDpN) || !dotrDpN)
							throw std::exception("no intersection.");
						j = rayPlaneIntersection(major2, rD, B_->a_, Bnormal_, dotrDpN);
					}

					// We have two intersections... we then need to create an intersection edge on the B plane...
					return calculateIntersectionEdge(i, j, result);
				}

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

					return segmentIsValid(result);
				}

				std::list<vertex> calculateTriangleEdgesSegmentIntersection(const triangle& t, const vertex& i, const vertex& j) const
				{
					std::list<vertex> intersections;

					vertex ij = unitise(subtract(j, i));

					// ab edge...
					{
						vertex ab = subtract(t.b_, t.a_);
						vertex abNormal = cross(ab, Bnormal_);
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
						vertex bcNormal = cross(bc, Bnormal_);
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
						vertex caNormal = cross(ca, Bnormal_);
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


				vertex directions_;
				vertex Bnormal_;

				const triangle* A_;
				const triangle* B_;
			};


		}	// namespace intersection
		

		// intersect triangles
		void mergeIntersectTriangles(std::vector<intersection::retessellatedTriangle>& aTriangles, std::vector<intersection::retessellatedTriangle>& bTriangles)
		{
			#pragma omp parallel for
			for (int i = 0; i < aTriangles.size(); ++i)
			{
				for (int j = 0; j < bTriangles.size(); ++j)
				{
					if (i == j)
						continue;

					intersection::retessellatedTriangle& A = aTriangles[i];
					intersection::retessellatedTriangle& B = bTriangles[j];
					
					intersection::triangleTriangle triangleTriangleIntersection(A.getTriangle(), B.getTriangle(), B.getNormal());

					// do they intersect?
					if (triangleTriangleIntersection.notIntersecting())
						continue;
					
					// is the intersection coplanar?
					if (triangleTriangleIntersection.onPlane())
					{
						// caluclate the intersection points and edges for both triangles...
						std::pair<std::list<segment>, std::list<segment>> coplanarResult;
						triangleTriangleIntersection.calculateCoplanarIntersection(coplanarResult);

						if (!coplanarResult.first.empty() || !coplanarResult.second.empty())
						{
							#pragma omp critical
							{
								A.addIntersection(coplanarResult.first);
								B.addIntersection(coplanarResult.second);
							}
						}

						
					}
					// otherwise they intersect, but they are not coplanar...
					else
					{
						// Calculate the intersection edge... this should be the same for both triangles...
						segment result;
						if (triangleTriangleIntersection.calculateIntersection(result))
						{
							#pragma omp critical
							{
								A.addIntersection(result);
								B.addIntersection(result);
							}
							
						}
					}

				}
			}
		}

		typedef std::pair<std::shared_ptr<intersection::mergeIntersectResult>, std::shared_ptr<intersection::mergeIntersectResult>> mergeResult;

		


		void merge(const mesh& A, const mesh& B, mergeResult& result)
		{
			std::vector<intersection::retessellatedTriangle> aTriangles(A.size(), intersection::retessellatedTriangle());
			std::vector<intersection::retessellatedTriangle> bTriangles(B.size(), intersection::retessellatedTriangle());

			//#pragma omp parallel for
			for (int t = 0; t < A.size(); ++t)
				aTriangles[t] = intersection::retessellatedTriangle(&A[t], normal(A[t]));

			//#pragma omp parallel for
			for (int t = 0; t < B.size(); ++t)
				bTriangles[t] = intersection::retessellatedTriangle(&B[t], normal(B[t]));

			mergeIntersectTriangles(aTriangles, bTriangles);

			// We then finalise the triangles... and prep mesh results...
			{
				bsp bspB(B);
				result.first.reset(new intersection::mergeIntersectResult());

				#pragma omp parallel for
				for (int t = 0; t < A.size(); ++t)
				{
					if (aTriangles[t].needsFinalising())
						aTriangles[t].finalise(*result.first, bspB);
					else
						result.first->addTriangle(*aTriangles[t].getTriangle(), bspB);
				}
			}
			{
				bsp bspA(A);
				result.second.reset(new intersection::mergeIntersectResult());

				#pragma omp parallel for
				for (int t = 0; t < B.size(); ++t)
				{
					if (bTriangles[t].needsFinalising())
						bTriangles[t].finalise(*result.second, bspA);
					else
						result.second->addTriangle(*bTriangles[t].getTriangle(), bspA);
				}
			}
		}

	}


	std::shared_ptr<mesh> Difference(const mesh& a, const mesh& b)
	{
		impl::mergeResult abMerge;
		impl::merge(a, b, abMerge);

		// We then need to construct the final mesh from the result...
		std::shared_ptr<mesh> result(new mesh());

		// outside of A...
		result->insert(result->end(), abMerge.first->outside_.begin(), abMerge.first->outside_.end());

		// inside of B (flipped)...
		std::for_each(abMerge.second->inside_.begin(), abMerge.second->inside_.end(), [](triangle& t) { impl::flip(t); });
		result->insert(result->end(), abMerge.second->inside_.begin(), abMerge.second->inside_.end());

		// and the A onplanes...
		result->insert(result->end(), abMerge.first->onPlane_.begin(), abMerge.first->onPlane_.end());

		return result;
	}

	/*std::shared_ptr<mesh> Union(const mesh& a, std::vector<mesh>& b)
	{
		return std::shared_ptr<mesh>();
	}

	std::shared_ptr<mesh> Intersection(const mesh& a, std::vector<mesh>& b)
	{
		return std::shared_ptr<mesh>();
	}*/

	/*std::shared_ptr<mesh> HalfSpace(const mesh& a, vertex& p, vertex& n)
	{
		return std::shared_ptr<mesh>();
	}*/

}

#endif // CSG_HPP