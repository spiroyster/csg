#ifndef CSG_HPP
#define CSG_HPP

#include <memory>
#include <vector>
#include <list>
#include <algorithm>

#include <assert.h>

namespace csg
{
	struct vertex { vertex(double x, double y, double z) : x_(x), y_(y), z_(z) {} double x_, y_, z_; };
	struct triangle { triangle(const vertex& a, const vertex& b, const vertex& c) : a_(a), b_(b), c_(c) {} vertex a_, b_, c_; };
	typedef std::vector<triangle> mesh;


	namespace impl
	{
		// vector operations...
		static double magnitude(const vertex& v) { return sqrt((v.x_ * v.x_) + (v.y_ * v.y_) + (v.z_ * v.z_)); }
		static vertex unitise(const vertex& v) { double oneOverMagnitude = 1.0 / magnitude(v); return vertex(v.x_ * oneOverMagnitude, v.y_ * oneOverMagnitude, v.z_ * oneOverMagnitude); }
		static vertex cross(const vertex& a, const vertex& b) { return vertex((a.y_ * b.z_) - (a.z_ * b.y_), (a.z_ * b.x_) - (a.x_ * b.z_), (a.x_ * b.y_) - (a.y_ * b.x_)); }
		static double dot(const vertex& a, const vertex& b) { return (a.x_ * b.x_) + (a.y_ * b.y_) + (a.z_ * b.z_); }
		static bool equals(const vertex& a, const vertex& b, float e) { return (abs(a.x_ - b.x_) < e && abs(a.y_ - b.y_) < e && abs(a.z_ - b.z_) < e); }
		static bool equals(const vertex& a, const vertex& b) { return (a.x_ == b.x_ && a.y_ == b.y_ && a.z_ == b.z_); }
		static vertex subtract(const vertex& a, const vertex& b) { return vertex(a.x_ - b.x_, a.y_ - b.y_, a.z_ - b.z_); }
		static vertex normal(const triangle& t) { return unitise(cross(subtract(t.b_, t.a_), subtract(t.c_, t.a_))); }

		// bsp



		// delaunay


	


		
		typedef std::pair<unsigned int, unsigned int> edge;
		

		struct workingTriangle
		{
			workingTriangle() : tri_(0), normal_(0,0,0) {}
			workingTriangle(const triangle& t) : tri_(&t), normal_(normal(t)) {}

			bool needsFinalising() const
			{
				return !intersectionEdges_.empty();
			}

			std::shared_ptr<mesh> finalise()
			{
				// remove duplicates...

				// project to 2D...

				// maybe sanitise to ensure the edges are not duplicates...

				// constrained retessellation...

				// convert back to 3D...

				return std::shared_ptr<mesh>();
			}

			void addIntersection(const std::list<vertex>& intersectionPoints)
			{
				if (intersectionPoints.size() != 2)
					return;
				intersectionEdges_.push_back(edge(intersectionPoints_.size(), intersectionPoints_.size() + 1));
				intersectionPoints_.insert(intersectionPoints_.end(), intersectionPoints.begin(), intersectionPoints.end());
			}

			void addCoplanarIntersection(const std::list<vertex>& intersectionPoints, std::list<edge>& edges)
			{
				intersectionPoints_.insert(intersectionPoints_.end(), intersectionPoints.begin(), intersectionPoints.end());
				intersectionEdges_.insert(intersectionEdges_.end(), edges.begin(), edges.end());
			}

			const triangle* tri_;
			vertex normal_;
			std::list<vertex> intersectionPoints_;
			std::list<edge> intersectionEdges_;
		};



		// result mesh...
		struct mergeIntersectResult
		{
			mesh inside_;
			mesh outside_;
			mesh onPlane_;
		};

		

		namespace triangleIntersection
		{
			static vertex centroid(const vertex& a, const vertex& b, const vertex& c) { return vertex((a.x_ + b.x_ + c.x_) / 3, (a.y_ + b.y_ + c.y_) / 3, (a.z_ + b.z_ + c.z_) / 3); }
			static bool sameSide(const vertex& p1, const vertex& p2, const vertex& a, const vertex& b) { return dot(cross(subtract(b, a), subtract(p1, a)), cross(subtract(b, a), subtract(p2, a))); }
			static bool pointWithinTriangle(const vertex& a, const vertex& b, const vertex& c, const vertex& p) { return sameSide(p, a, b, c) && sameSide(p, b, a, c) && sameSide(p, c, a, b); }

			static double directionFromPlane(const vertex& p, const vertex& n, const vertex& v)
			{
				if (!equals(v, p))
					return dot(unitise(subtract(v, p)), n);
				return 0;
			}

			static double dotrDpN(const vertex& rP, const vertex& rD, const vertex& pP, const vertex& pN)
			{
				return dot(rD, pN);
			}

			static vertex rayPlaneIntersection(const vertex& rP, const vertex& rD, const vertex& pP, const vertex& pN, double dotrDpN)
			{
				double s = dot(subtract(pP, rP), pN) / dotrDpN;
				return vertex(rP.x_ + (rD.x_ * s), rP.y_ + (rD.y_ * s), rP.z_ + (rD.z_ * s));
			}
			static vertex rayPlaneIntersection(const vertex& rP, const vertex& rD, const vertex& pP, const vertex& pN)
			{
				double dotrDpN = dot(rD, pN);
				if (dotrDpN)
				{
					double s = dot(subtract(pP, rP), pN) / dotrDpN;
					return vertex(rP.x_ + (rD.x_ * s), rP.y_ + (rD.y_ * s), rP.z_ + (rD.z_ * s));
				}
				throw std::exception("Ray and plane do not intersect.");
			}

			
			

			struct info
			{
				info(const double& a, const double& b, const double& c, const vertex& n)
					: directionsFromPlane_(a, b, c), n_(n)
				{
				}

				bool entirelyInFront() const { return directionsFromPlane_.x_ > 0 && directionsFromPlane_.y_ > 0 && directionsFromPlane_.z_ > 0; }
				bool entirelyBehind() const { return directionsFromPlane_.x_ < 0 && directionsFromPlane_.y_ < 0 && directionsFromPlane_.z_ < 0; }
				bool onPlane() const { return !directionsFromPlane_.x_ && !directionsFromPlane_.y_ && !directionsFromPlane_.z_; }
				bool edgeOnPlane() const { return ((!directionsFromPlane_.x_ && !directionsFromPlane_.y_) || (!directionsFromPlane_.y_ && !directionsFromPlane_.z_) || (!directionsFromPlane_.x_ && !directionsFromPlane_.z_)); }
				
				const vertex& normal() const { return n_; }

				double a() const { return directionsFromPlane_.x_; }
				double b() const { return directionsFromPlane_.y_; }
				double c() const { return directionsFromPlane_.z_; }

			private:
				vertex directionsFromPlane_;
				vertex n_;
			};

			typedef std::pair<std::list<vertex>, std::list<vertex>> result;

			// triangle plane intersection information... return a vertices in relation to b plane...
			static info trianglePlane(const workingTriangle& a, const workingTriangle& b)
			{
				vertex n = normal(*a.tri_);
				return info(directionFromPlane(a.tri_->a_, n, b.tri_->a_),
					directionFromPlane(a.tri_->a_, n, b.tri_->b_), directionFromPlane(a.tri_->a_, n, b.tri_->c_), n);
			}

			// coplanarSegmentTriangleIntersection, given two points on the triangle plane, calculate the triangle edge intersection points...
			std::list<vertex> coplanarSegmentTriangleEdge(const vertex& a, const vertex& b, const vertex& c, const vertex& n, const vertex& i, const vertex& j)
			{
				std::list<vertex> intersections;

				vertex ij = unitise(subtract(j, i));

				// ab edge...
				{
					vertex ab = subtract(b, a);
					vertex abNormal = cross(ab, n);
					double abDotrDpN = dot(ij, abNormal);
					if (!std::isnan(abDotrDpN) && abDotrDpN)
						intersections.push_back(rayPlaneIntersection(i, ij, a, abNormal, abDotrDpN));
				} 
				// bc edge...
				{
					vertex bc = subtract(c, b);
					vertex bcNormal = cross(bc, n);
					double bcDotrDpN = dot(ij, bcNormal);
					if (!std::isnan(bcDotrDpN) && bcDotrDpN)
						intersections.push_back(rayPlaneIntersection(i, ij, b, bcNormal, bcDotrDpN));
				}
				// ca edge...
				{
					vertex ca = subtract(a, c);
					vertex caNormal = cross(ca, n);
					double caDotrDpN = dot(ij, caNormal);
					if (!std::isnan(caDotrDpN) && caDotrDpN)
						intersections.push_back(rayPlaneIntersection(i, ij, c, caNormal, caDotrDpN));
				}
				
				return intersections;
			}

			result validateIntersectionPoints(const workingTriangle& B, const vertex& i, const vertex& j, const workingTriangle& A)
			{
				result r;

				// check if the points are within the intersecting triangle...
				bool iinB = pointWithinTriangle(B.tri_->a_, B.tri_->b_, B.tri_->c_, i);
				bool jinB = pointWithinTriangle(B.tri_->a_, B.tri_->b_, B.tri_->c_, j);

				// if both are in triangle...
				if (iinB && jinB)
				{
					r.first.push_back(i);
					r.first.push_back(j);
					r.second = r.first;
					return r;
				}
				// if neither are in triangle...
				else if (!iinB && !jinB)
				{
					// find the intersection points of triangleB in triangleA plane and use those...
					return performIntersection(triangleIntersection::trianglePlane(B, A), B, A);
				}
				// otherwise one is inside the triangle, the other outside...
				else if (!iinB && jinB)
				{
					// find the edge intersection...
					std::list<vertex> edgeIntersections = coplanarSegmentTriangleEdge(B.tri_->a_, B.tri_->b_, B.tri_->c_, B.normal_, i, j);

					if (edgeIntersections.size() != 1)
						throw std::exception("there should be only one intersection.");

					r.first.push_back(edgeIntersections.front());
					r.first.push_back(j);
					r.second = r.first;
					return r;
				}
				else if (iinB && !jinB)
				{
					// find the edge intersection...
					std::list<vertex> edgeIntersections = coplanarSegmentTriangleEdge(B.tri_->a_, B.tri_->b_, B.tri_->c_, B.normal_, i, j);

					if (edgeIntersections.size() != 1)
						throw std::exception("there should be only one intersection.");

					r.first.push_back(edgeIntersections.front());
					r.first.push_back(i);
					r.second = r.first;
					return r;
				}
			}


			// coplanar triangle intersection...
			result coplanarTriangles(const workingTriangle& a, const workingTriangle& b, const vertex& n)
			{
				

				return result();
			}
			
			// single edge intersection...
			result triangleEdge(const info& i, const workingTriangle& triA, const workingTriangle& triB)
			{
				assert((!i.a && !i.b && i.c) || (!i.a && !i.c && i.b) || (!i.b && !i.c && i.a));

				if (!i.a && !i.b && i.c)
					return validateIntersectionPoints(triB, i.a, i.b, triA);
				else if (!i.a && !i.c && i.b)
					return validateIntersectionPoints(triB, i.a, i.c, triA);
				else if (i.a && !i.c && !i.b)
					return validateIntersectionPoints(triB, i.b, i.c, triA);

				throw std::exception("intersection is not triangleEdge intersection.");
			}

			result calculateDualEdgeIntersection(const vertex& oneSide1, const vertex& oneSide2, const vertex& otherSide, const workingTriangle& B, const workingTriangle& A)
			{
				vertex intersection1 = rayPlaneIntersection(oneSide1, subtract(otherSide, oneSide1), B.tri_->a_, B.normal_);
				vertex intersection2 = rayPlaneIntersection(oneSide2, subtract(otherSide, oneSide2), B.tri_->a_, B.normal_);
				return validateIntersectionPoints(A, intersection1, intersection2, B);
			}

			// triangle triangle intersection...
			result triangleTriangle(const info& i, const workingTriangle& A, const workingTriangle& B)
			{
				// calculate the intersection points between the triangle A and the plane...
				assert((i.a > 0 && i.b > 0 && i.c < 0) || (i.a > 0 && i.c > 0 && i.b < 0) || (i.b > 0 && i.c > 0 && i.a < 0)
					(i.a < 0 && i.b < 0 && i.c > 0) || (i.a < 0 && i.c < 0 && i.b > 0) || (i.b < 0 && i.c < 0 && i.a > 0));

				if ((i.a > 0 && i.b > 0 && i.c < 0) || (i.a < 0 && i.b < 0 && i.c > 0))
					return calculateDualEdgeIntersection(A.tri_->a_, A.tri_->b_, A.tri_->c_, B, A);
				else if ((i.a > 0 && i.b < 0 && i.c > 0) || (i.a < 0 && i.b > 0 && i.c < 0))
					return calculateDualEdgeIntersection(A.tri_->a_, A.tri_->c_, A.tri_->b_, B, A);
				else if ((i.a < 0 && i.b > 0 && i.c > 0) || (i.a > 0 && i.b < 0 && i.c < 0))
					return calculateDualEdgeIntersection(A.tri_->b_, A.tri_->c_, A.tri_->a_, B, A);

				throw std::exception("intersection is not triangleTriangle intersection.");
			}

			result performIntersection(const info& i, const workingTriangle& A, const workingTriangle& B)
			{
				// check if triangles are coplanar...
				if (i.onPlane())
					return coplanarTriangles(A, B, i.normal());

				// check if a single edge is on the plane of triangleB...
				else if (i.edgeOnPlane())
					return triangleEdge(i, A, B);
				
				// otherwise it is standard triangel/triangle intersection...
				return triangleTriangle(i, A, B);
			}

		}


		

		// intersect triangles
		void mergeIntersectTriangles(std::vector<workingTriangle>& aTriangles, std::vector<workingTriangle>& bTriangles)
		{
			for (int i = 0; i < aTriangles.size(); ++i)
			{
				for (int j = 0; j < bTriangles.size(); ++j)
				{
					if (i == j)
						continue;

					workingTriangle& A = aTriangles[i];
					workingTriangle& B = bTriangles[j];
					
					// Calculate the intersection between the triangles... info is A vertices in relation to B plane...
					triangleIntersection::info intersectionInfo = triangleIntersection::trianglePlane(A, B);

					if (intersectionInfo.entirelyBehind() || intersectionInfo.entirelyInFront())
						continue;
					
					triangleIntersection::result r = triangleIntersection::performIntersection(intersectionInfo, A, B);
					
					if (!r.first.empty() || !r.second.empty())
					{
						if (r.first.empty())
							A.addIntersection(r.first);
						if (r.first.empty())
							B.addIntersection(r.second);
					}
				}
			}
		}

		typedef std::pair<std::shared_ptr<mergeIntersectResult>, std::shared_ptr<mergeIntersectResult>> mergeResult;

		mergeResult merge(const mesh& a, const mesh& b)
		{
			std::vector<workingTriangle> aTriangles(a.size(), workingTriangle());
			std::vector<workingTriangle> bTriangles(b.size(), workingTriangle());

			for (int t = 0; t < a.size(); ++t)
				aTriangles[t] = workingTriangle(a[t]);
			for (int t = 0; t < b.size(); ++t)
				bTriangles[t] = workingTriangle(b[t]);

			mergeIntersectTriangles(aTriangles, bTriangles);

			// We then finalise the triangles... and prep mesh results...
			mergeResult result;

			for (int t = 0; t < a.size(); ++t)
			{
				if (aTriangles[t].needsFinalising())
					aTriangles[t].finalise();
			}
			for (int t = 0; t < a.size(); ++t)
			{
				if (bTriangles[t].needsFinalising())
					bTriangles[t].finalise();
			}
				
			





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