



#ifndef CSG2_HPP
#define CSG2_HPP

#include <cmath>
#include <functional>
#include <vector>
#include <list>
#include <algorithm>
#include <memory>


namespace csg 
{
	// vertex
	struct vertex { vertex() : x_(0), y_(0), z_(0) {} vertex(double x, double y, double z) : x_(x), y_(y), z_(z) {} double x_, y_, z_; };
	
	// segment
	typedef std::pair<vertex, vertex> segment;

	// triangle (user is custom callback)
	struct triangle
	{
		triangle(const vertex& a, const vertex& b, const vertex& c, void* user = nullptr) :	a_(a), b_(b), c_(c), user_(user) {}
		vertex a_, b_, c_;
		void* user_;
	};

	// callback functions...

	// triangle callback for population
	typedef std::function<triangle(unsigned int index)> populator;

	// callback for extracting mesh triangles from merge
	typedef std::function<void(unsigned int index, const triangle& tri)> meshExtractor;
	
	//  callback for extracting loops from merge
	typedef std::function<void(unsigned int loop, unsigned int index, const segment& seg)> loopExtractor;

	namespace impl
	{
		const static double pi = 3.14159265358979323846264338327950288;
		const static double superTriangleScaling = 2000.0;
		const static double defaultTolerance = 0.01;

		// vertex operations...
		static bool isValid(const triangle& t, double tolerance) { return (!equals(t.a_, t.b_, tolerance) && !equals(t.a_, t.c_, tolerance) && !equals(t.b_, t.c_, tolerance) && !std::isnan(normal(t).x_)); }
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
		static void flip(triangle& t) { vertex tmp = t.c_; t.c_ = t.a_; t.a_ = tmp; }
		static triangle flipped(const triangle& t) { return triangle(t.c_, t.b_, t.a_, t.user_); }
		
		// Intersection operations...
		namespace intersect
		{
			struct splitResult { std::vector<triangle> outside_; std::vector<triangle> inside_; vertex i_, j_; };

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
				std::for_each(result.inside_.begin(), result.inside_.end(), [&validTriangles, &tolerance](triangle& t) { if (isValid(t, tolerance)) { validTriangles.inside_.push_back(t); } });
				std::for_each(result.outside_.begin(), result.outside_.end(), [&validTriangles, &tolerance](triangle& t) { if (isValid(t, tolerance)) { validTriangles.outside_.push_back(t); } });
				result.inside_.swap(validTriangles.inside_);
				result.outside_.swap(validTriangles.outside_);
			}

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

					// if any of the vertices are 'close' to plane, clamp...
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

			protected:
				vertex directions_;		// directions information of the triangle vertices wrt to the plane
				vertex normal_;			// normal of the plane
				triangle A_;			// the triange

				bool isClamped_;		// true if we clamped the triangle A_ to the plane in any way.
				double tolerance_;
			};

			// Split a triangle...
			splitResult splitTriangle(const trianglePlane& intersectionInfo, const triangle& t, const vertex& tN, const vertex& pP, const vertex& pN)
			{
				//assert(!intersectionInfo.entirelyInside());
				//assert(!intersectionInfo.entirelyOutside());
				//assert(!intersectionInfo.onPlane());
				//assert(!intersectionInfo.edgeABOnPlane());
				//assert(!intersectionInfo.edgeBCOnPlane());
				//assert(!intersectionInfo.edgeCAOnPlane());

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
						i = t.a_; j = segmentPlaneIntersection(t.c_, t.b_, pP, pN);
						result.inside_.push_back(triangle(i, t.b_, j)); result.outside_.push_back(triangle(j, t.c_, i));
					}
					else if (!intersectionInfo.b())
					{
						i = t.b_; j = segmentPlaneIntersection(t.c_, t.a_, pP, pN);
						result.inside_.push_back(triangle(i, t.c_, j)); result.outside_.push_back(triangle(j, t.a_, i));
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

				// ascertain the correct winding...
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

						//assert(triangleEdgeSegmentIntersections.size() < 3);
						
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
				
				vertex closest(const triangle& t, const vertex& v) const
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

				const triangle* B_;		// B triangle of a triangle triangle intersection.
			};

		}

		namespace ioop
		{
			enum ioop { inside, outside, onPlane };
		}

		// bsp 
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
			bsp(const std::vector<triangle>& m, double tolerance) { construct(m, tolerance); }

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
				}
				throw std::runtime_error("cannot deduce if point is inside or outside.");
			}

		private:
			void construct(const std::vector<triangle>& m, double tolerance)
			{
				// create a working list of the triangles. We validate as we go along.
				std::list<triangle> workingTriangles(m.begin(), m.end());
				while (!workingTriangles.empty() && !isValid(workingTriangles.front(), tolerance))
					workingTriangles.pop_front();

				if (workingTriangles.empty())
					return;

				root_.reset(new node(workingTriangles.front()));
				while (!workingTriangles.empty())
				{
					std::list<triangle>::iterator currentTriangle = workingTriangles.begin();
					if (!isValid(*currentTriangle, tolerance))
					{
						workingTriangles.erase(currentTriangle);
						continue;
					}

					node* currentNode = root_.get();
					while (currentNode)
					{
						intersect::trianglePlane intersectionInfo(&(*currentTriangle), currentNode->p(), currentNode->n(), tolerance);

						const triangle* workingTriangle = intersectionInfo.IsClamped() ? &intersectionInfo.ClampedTriangle() : &(*currentTriangle);
						
						if (intersectionInfo.onPlane())
							currentNode = followInsideNode(currentNode, *workingTriangle);
						else if (intersectionInfo.outside())
							currentNode = followOutsideNode(currentNode, *workingTriangle);
						else if (intersectionInfo.inside())
							currentNode = followInsideNode(currentNode, *workingTriangle);
						else
						{
							intersect::splitResult splitTriangles = intersect::splitTriangle(intersectionInfo, *workingTriangle, normal(*workingTriangle), currentNode->p(), currentNode->n());
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

		namespace tessellate
		{
			

		}


		// working csg triangle...
		class working
		{
			public:

			working(const triangle& t, double tolerance)
				: triangle_(t), isValid_(impl::isValid(t, tolerance)), isClamped_(false)
			{
			}

			// Add intersection edge
			void addIntersection(const segment& intersectionEdge)
			{

			}

			// Add coplanar intersection edges
			void addIntersection(int triangleIndex, const std::list<segment>& intersectionEdges)
			{

			}


			void retessellate()
			{
				// Create a single container of all the points, and calculate associated segments (as constraints)...

				// Sanitise and prep for tessellation...


				// Tessellate...



				

			}

			bool isValid_; 				// true if this is a valid triangle...
			bool isClamped_;			// true if any vertex of this triangle is clamped...
			triangle triangle_;			// the original triangle...
			vertex normal_;				// the normal of this triangle...
			
			std::vector<triangle> tessellation_;
			
			//std::list<unsigned int> intersections_;		// indexees to the current intersection edges
		};
	}

	// barycentric...
	vertex barycentric(const triangle& t, const vertex& v)
	{
		// given a triangle, calculate the barycentric coordinates of the given v.
	}

	vertex interpolate(const vertex& barycentric, const vertex& a, const vertex& b, const vertex& c)
	{
		// given a barycentric coordinate and 3 vertices, interpolate to find the vertex at barycentric location...
	}

	// e.g.  interpolate(barycentric(t, v), na, nb, nc);

	// merge class
	class merge
	{
		//std::vector<triangle> aInside_, aOutside_, aOnPlane_, bInside_, bOutside_, bOnPlane_;

		// working triangle buffers...
		std::vector<impl::working> a_, b_;

		// mesh BSP's...

		
		// intersection edges/segments...
		void calculateIntersections(std::vector<impl::working>& aTriangles, std::vector<impl::working>& bTriangles, double tolerance)
		{
			// Process to find the intersections...
			//#pragma omp for
			for (int a = 0; a < static_cast<int>(aTriangles.size()); ++a)
			{
				impl::working& A = aTriangles[a];
				if (!A.isValid_)
					continue;
				
				for (unsigned int b = a; b < bTriangles.size(); ++b)
				{
					impl::working& B = bTriangles[b];
					if (!B.isValid_)
						continue;
					
					// Calculate the triangle/triangle intersection...
					impl::intersect::triangleTriangle tt(A.triangle_, B.triangle_, B.normal_, tolerance);
					B.isClamped_ = tt.IsClamped();

					// If no intersection, nothing to do...
					if (!tt.intersects())
						continue;

					// Otherwise check coplanar intersections...
					else if (tt.onPlane())
					{
						std::pair<std::list<segment>, std::list<segment>> coplanarResult;
						if (tt.calculateCoplanarIntersection(coplanarResult))
						{
							//#pragma omp critical
							{
								A.addIntersection(b, coplanarResult.second);
								B.addIntersection(a, coplanarResult.first);
							}
						}
					}

					// Otherwise it should be a 'clean' intersection (edge)...
					else
					{
						segment intersectionEdge;
						if (tt.calculateIntersection(intersectionEdge))
						{
							//#pragma omp critical
							{
								A.addIntersection(intersectionEdge);
								B.addIntersection(intersectionEdge);
							}
						}
					}
				}
			}

			// Retessllate the required triangles...
			for (int a = 0; a < a_.size(); ++a)
				a_[a].retessellate();
			for (int b = 0; b < b_.size(); ++b)
				b_[b].retessellate();

		}

		public:

		// policy to deduce which onplane triangles to include.
		enum OnPlaneIncludePolicy { Always, Never, NormalsEqual, NormalsNotEqual };

		// result statistics... counts of how many triangles are inside/outside
		struct meshStats
		{
			meshStats(unsigned int aInside, unsigned int aOutside, unsigned int bInside, unsigned int bOutside)
			:	aInside_(aInside), bInside_(bInside), aOutside_(aOutside), bOutside_(bOutside), count_(aInside + aOutside + bInside + bOutside) {}
			unsigned int aInside_, aOutside_, bInside_, bOutside_, count_;
		};

		// return stats regarding the 
		struct loopStats
		{
			loopStats(const std::vector<unsigned int>& loops) : loops_(loops) { /* accumulate */  }
			std::vector<unsigned int> loops_;
			unsigned int count_;
		};

		// Constructor from explicit 
		merge(const std::vector<triangle>& meshA, const std::vector<triangle>& meshB, double tolerance = 0.001)
		{
			// create our working triangle buffers...
			a_.reserve(meshA.size());
			b_.reserve(meshB.size());
			for (unsigned int a = 0; a < meshA.size(); ++a)
				a_.push_back(impl::working(meshA[a], tolerance));
			for (unsigned int b = 0; b < meshB.size(); ++b)
				b_.push_back(impl::working(meshB[b], tolerance));

			calculateIntersections(a_, b_, tolerance);
		}

		// Constructor with custom callback populators...
		merge(unsigned int triangleCountA, populator meshA, 
			  unsigned int triangleCountB, populator meshB, 
			  double tolerance = 0.001)
		{

			// create our working triangle buffers...
			a_.reserve(triangleCountA);
			b_.reserve(triangleCountB);
			for (unsigned int a = 0; a < triangleCountA; ++a)
				a_.push_back(impl::working(meshA(a), tolerance));
			for (unsigned int b = 0; b < triangleCountB; ++b)
				b_.push_back(impl::working(meshB(b), tolerance));

			calculateIntersections(a_, b_, tolerance);
		}

		// // union
		// meshStats Union() { return meshStats(0, aOutside_.size(), 0, bOutside_.size()); }
		// void Union(meshExtractor callback)
		// {
		// 	unsigned int index = 0;
		// 	for (unsigned int ao = 0; ao < aOutside_.size(); ++ao, ++index) { callback(index, aOutside_[ao]); }
		// 	for (unsigned int bo = 0; bo < bOutside_.size(); ++bo, ++index) { callback(index, bOutside_[bo]); }
		// }

		// // difference
		// meshStats Difference() { return meshStats(0, aOutside_.size(), bInside_.size(), 0); }
		// void Difference(meshExtractor callback)
		// {
		// 	unsigned int index = 0;
		// 	for (unsigned int ao = 0; ao < aOutside_.size(); ++ao, ++index) { callback(index, aOutside_[ao]); }
		// 	for (unsigned int bi = 0; bi < bInside_.size(); ++bi, ++index) { callback(index, impl::flipped(bInside_[bi])); }
		// }

		// // intersection
		// meshStats Intersection() { return meshStats(aInside_.size(), 0, bInside_.size(), 0); }
		// void Intersection(meshExtractor callback)
		// {
		// 	unsigned int index = 0;
		// 	for (unsigned int ai = 0; ai < aInside_.size(); ++ai, ++index) { callback(index, aInside_[ai]); }
		// 	for (unsigned int bi = 0; bi < bInside_.size(); ++bi, ++index) { callback(index, bInside_[bi]); }
		// }

		// // edges
		// loopStats IntersectionLoops()
		// {

		// }

		// void IntersectionLoops(loopExtractor callback)
		// {
		// 	// We need to return the intersection loops that have been calculated...

		// 	// These need to be processed into the correct ordering and winding...

		// }

	};

	class halfSpace
	{
		public:

		halfSpace(const std::vector<impl::working>& trianges, const vertex& point, const vertex& normal, double tolerance)
		{

		}

		halfSpace(unsigned int triCount, populator mesh, const vertex& point, const vertex& normal, double tolerance)
		{
			// Create our working triangles...

			// Cacluate the intersections of the triangles...

			// Retessellate the triangles, 

			// Caluclate the intersection loops...

			// tessellate the intersection loops...


		}

		
	};

	// Use the vertices of the original triangle to calculate the barycentric coordinate of any given triangle vertex...

	// Then user can pass the coordinates to interpolate of the original tex/normal to evaluate what the interpoated tex/normal will be.








}


#endif // CSG2_HPP
