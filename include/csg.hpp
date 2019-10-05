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

namespace csg
{
	struct vertex { vertex() : x_(0), y_(0), z_(0) {} vertex(double x, double y, double z) : x_(x), y_(y), z_(z) {} double x_, y_, z_; };
	struct triangle { triangle(const vertex& a, const vertex& b, const vertex& c) : a_(a), b_(b), c_(c) {} vertex a_, b_, c_; };
	typedef std::vector<triangle> mesh;
	typedef std::pair<unsigned int, unsigned int> edge;
	typedef std::pair<vertex, vertex> segment;

	namespace impl
	{
		
		// vector operations...
		static double magnitude(const vertex& v) { return sqrt((v.x_ * v.x_) + (v.y_ * v.y_) + (v.z_ * v.z_)); }
		static vertex unitise(const vertex& v) { double oneOverMagnitude = 1.0 / magnitude(v); return vertex(v.x_ * oneOverMagnitude, v.y_ * oneOverMagnitude, v.z_ * oneOverMagnitude); }
		static vertex cross(const vertex& a, const vertex& b) { return vertex((a.y_ * b.z_) - (a.z_ * b.y_), (a.z_ * b.x_) - (a.x_ * b.z_), (a.x_ * b.y_) - (a.y_ * b.x_)); }
		static double dot(const vertex& a, const vertex& b) { return (a.x_ * b.x_) + (a.y_ * b.y_) + (a.z_ * b.z_); }
		static bool equals(const vertex& a, const vertex& b, double e) { return (abs(a.x_ - b.x_) < e && abs(a.y_ - b.y_) < e && abs(a.z_ - b.z_) < e); }
		static bool equals(const vertex& a, const vertex& b) { return (a.x_ == b.x_ && a.y_ == b.y_ && a.z_ == b.z_); }
		static vertex subtract(const vertex& a, const vertex& b) { return vertex(a.x_ - b.x_, a.y_ - b.y_, a.z_ - b.z_); }
		static vertex normal(const triangle& t) { return unitise(cross(subtract(t.b_, t.a_), subtract(t.c_, t.a_))); }
		static double directionFromPlane(const vertex& p, const vertex& n, const vertex& v) { return !equals(v, p) ? dot(unitise(subtract(v, p)), n) : 0; }
		static vertex rayPlaneIntersection(const vertex& rP, const vertex& rD, const vertex& pP, const vertex& pN, double dotrDpN) { double s = dot(subtract(pP, rP), pN) / dotrDpN; return vertex(rP.x_ + (rD.x_ * s), rP.y_ + (rD.y_ * s), rP.z_ + (rD.z_ * s)); }
		static vertex centroid(const vertex& a, const vertex& b, const vertex& c) { return vertex((a.x_ + b.x_ + c.x_) / 3, (a.y_ + b.y_ + c.y_) / 3, (a.z_ + b.z_ + c.z_) / 3); }
		static bool sameSide(const vertex& p1, const vertex& p2, const vertex& a, const vertex& b) { return dot(cross(subtract(b, a), subtract(p1, a)), cross(subtract(b, a), subtract(p2, a))); }
		static bool pointWithinTriangle(const vertex& a, const vertex& b, const vertex& c, const vertex& p) { return sameSide(p, a, b, c) && sameSide(p, b, a, c) && sameSide(p, c, a, b); }

		// bsp tree...
		class bsp
		{
		public:
			class node : public triangle
			{
			public:
				node(const triangle& t) : triangle(t.a_, t.b_, t.c_), normal_(normal(t)) { if (std::isnan(normal_.x_) || std::isnan(normal_.y_) || std::isnan(normal_.z_)) { throw std::exception("Unable to calculate normal correctly."); } }
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
						node::intersectionResult intersection = currentNode->calculateTriangleIntersection(*currentTriangle);

						// if the current triangle is outside the current node (triangle)...
						if (intersection.isOutside())
							currentNode = followOutsideNode(currentNode, *currentTriangle);
						// if the current triangle is inside or on the plane of the current node (triangle)...
						else if (intersection.isInside() || intersection.isOnPlane())
							currentNode = followInsideNode(currentNode, *currentTriangle);
						else
						{
							// otherwise the current triangle straddles the current node somehow so we need to split the triangle...
							splitResult splitTriangles = splitTriangle(intersection, currentNode, *currentTriangle);

							// invalid triangles won't be added, so we need to check there is only one triangle in the split...
							unsigned int workingTriangleCount = workingTriangles.size();

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
				return true;
				//return area(t) > 0.001;
			}

			splitResult splitTriangle(const node::intersectionResult& intersection, const node* nde, const triangle& tri) const
			{
				splitResult result;

				// If there is at least one point on the plane... since this is split, this must mean that the triangle straddles...
				// calculate the single intersection point and the two triangles...
				if (intersection.atLeastOneVertexIsOnPlane())
				{
					// if vertex A is on the plane, intersection point must be along BC...
					if (!intersection.A())
					{
						vertex intersectionPoint = rayPlaneIntersection(tri.b_, unitise(subtract(tri.c_, tri.b_)), nde->position(), nde->normal());
						if (intersection.B() > 0)
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
					else if (!intersection.B())
					{
						vertex intersectionPoint = rayPlaneIntersection(tri.c_, unitise(subtract(tri.a_, tri.c_)), nde->position(), nde->normal());
						if (intersection.C() > 0)
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
					else if (!intersection.C())
					{
						vertex intersectionPoint = rayPlaneIntersection(tri.a_, unitise(subtract(tri.b_, tri.a_)), nde->position(), nde->normal());
						if (intersection.A() > 0)
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
					if ((intersection.A() > 0 && intersection.B() > 0) || (intersection.A() < 0 && intersection.B() < 0))
					{
						vertex intersection1 = rayPlaneIntersection(tri.b_, unitise(subtract(tri.c_, tri.b_)), nde->position(), nde->normal());
						vertex intersection2 = rayPlaneIntersection(tri.c_, unitise(subtract(tri.a_, tri.c_)), nde->position(), nde->normal());

						result.outside_.push_back(triangle(tri.a_, tri.b_, intersection1));
						result.inside_.push_back(triangle(intersection1, tri.c_, intersection2));
						result.inside_.push_back(triangle(intersection2, tri.a_, intersection1));
					}
					else if ((intersection.B() > 0 && intersection.C() > 0) || (intersection.B() < 0 && intersection.C() < 0))
					{
						vertex intersection1 = rayPlaneIntersection(tri.c_, unitise(subtract(tri.a_, tri.c_)), nde->position(), nde->normal());
						vertex intersection2 = rayPlaneIntersection(tri.a_, unitise(subtract(tri.b_, tri.a_)), nde->position(), nde->normal());

						result.outside_.push_back(triangle(tri.b_, tri.c_, intersection1));
						result.inside_.push_back(triangle(intersection1, tri.a_, intersection2));
						result.inside_.push_back(triangle(intersection2, tri.b_, intersection1));
					}
					else if ((intersection.A() > 0 && intersection.C() > 0) || (intersection.A() < 0 && intersection.C() < 0))
					{
						vertex intersection1 = rayPlaneIntersection(tri.a_, unitise(subtract(tri.b_, tri.a_)), nde->position(), nde->normal());
						vertex intersection2 = rayPlaneIntersection(tri.b_, unitise(subtract(tri.c_, tri.b_)), nde->position(), nde->normal());

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


		

		namespace intersection
		{
			struct mergeIntersectResult
			{
				mesh inside_;
				mesh outside_;
				mesh onPlane_;
			};

			class retessellatedTriangle
			{
			public:
				retessellatedTriangle() : triangle_(0) {}

				retessellatedTriangle(const triangle* tri) : triangle_(tri) {}

				bool needsFinalising() const { return true; }

				void finalise()
				{
					// santise the edges, remove the duplicate points...

				}

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
				const triangle* triangle_;
				std::list<segment> intersections_;
			};


			class triangleTriangle
			{
			public:
				triangleTriangle()
					: A_(0), B_(0)
				{
				}

				triangleTriangle(const triangle* A, const triangle* B)
					: A_(A), B_(B), directions_(calculateDirections(*A, *B))
				{
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

				bool singleVertexOnPlane() const {
					return ;
				}
				
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
					Bnormal_ = normal(B);
					return vertex(directionFromPlane(B.a_, Bnormal_, A.a_), directionFromPlane(B.a_, Bnormal_, A.b_), directionFromPlane(B.a_, Bnormal_, A.c_));
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
							// no points, intersection edge is outside B entirely...
							if (triangleEdgeSegmentIntersections.empty())
								return false;

							if (triangleEdgeSegmentIntersections.size() != 2)
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
					return true;
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
							intersections.push_back(rayPlaneIntersection(i, ij, t.a_, abNormal, abDotrDpN));
					} 
					// bc edge...
					{
						vertex bc = subtract(t.c_, t.b_);
						vertex bcNormal = cross(bc, Bnormal_);
						double bcDotrDpN = dot(ij, bcNormal);
						if (!std::isnan(bcDotrDpN) && bcDotrDpN)
							intersections.push_back(rayPlaneIntersection(i, ij, t.b_, bcNormal, bcDotrDpN));
					}
					// ca edge...
					{
						vertex ca = subtract(t.a_, t.c_);
						vertex caNormal = cross(ca, Bnormal_);
						double caDotrDpN = dot(ij, caNormal);
						if (!std::isnan(caDotrDpN) && caDotrDpN)
							intersections.push_back(rayPlaneIntersection(i, ij, t.c_, caNormal, caDotrDpN));
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
					
					intersection::triangleTriangle triangleTriangleIntersection(A.getTriangle(), B.getTriangle());

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

		mergeResult merge(const mesh& a, const mesh& b)
		{
			std::vector<intersection::retessellatedTriangle> aTriangles(a.size(), intersection::retessellatedTriangle());
			std::vector<intersection::retessellatedTriangle> bTriangles(b.size(), intersection::retessellatedTriangle());

			#pragma omp parallel for
			for (int t = 0; t < a.size(); ++t)
				aTriangles[t] = intersection::retessellatedTriangle(&a[t]);

			#pragma omp parallel for
			for (int t = 0; t < b.size(); ++t)
				bTriangles[t] = intersection::retessellatedTriangle(&b[t]);

			mergeIntersectTriangles(aTriangles, bTriangles);

			// We then finalise the triangles... and prep mesh results...
			mergeResult result;

			#pragma omp parallel for
			for (int t = 0; t < a.size(); ++t)
			{
				if (aTriangles[t].needsFinalising())
				{

					aTriangles[t].finalise();

				}
					
			}
			
			#pragma omp parallel for
			for (int t = 0; t < b.size(); ++t)
			{
				if (bTriangles[t].needsFinalising())
					bTriangles[t].finalise();
			}

			// We then need to construct a BSP for b to see which triangles of a are in b...
			{
				std::list<triangle> inside, outside, onPlane;
				bsp bspB(b);




			}

			// We then need to construct a BSP for a to see which triangles of b are in a...
			





		}

	}


	std::shared_ptr<mesh> Difference(const mesh& a, const mesh& b)
	{
		impl::mergeResult abMerge = impl::merge(a, b);

		

		return std::shared_ptr<mesh>();
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