#ifndef CSG3_HPP
#define CSG3_HPP

#include <functional>
#include <vector>
#include <list>
#include <assert.h>
#include <math.h>
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

	// callback functions for population and extraction...

	// triangle callback for population
	typedef std::function<triangle(unsigned int index)> triangle_populator_fn;

	// callback for extracting mesh triangles from merge
	typedef std::function<void(unsigned int index, const triangle& tri)> triangle_extractor_fn;
	
    namespace ancillary
    {
        vertex subtract(const vertex& a, const vertex& b) { return vertex(a.x_ - b.x_, a.y_ - b.y_, a.z_ - b.z_); }
        double magnitude_squared(const vertex& v) { return (v.x_ * v.x_) + (v.y_ * v.y_) + (v.z_ * v.z_); }
        double magnitude(const vertex& v) { return sqrt(magnitude_squared(v)); }
        vertex unitise(const vertex& v) { double mag_recip = 1.0 / magnitude(v); return vertex(v.x_ * mag_recip, v.y_ * mag_recip, v.z_ * mag_recip); }
        double distance(const vertex& a, const vertex& b) { return magnitude(subtract(b, a)); }
        double distance_squared(const vertex& a, const vertex& b) { return magnitude_squared(subtract(b, a)); }
        vertex cross(const vertex& a, const vertex& b) { return vertex((a.y_ * b.z_) - (a.z_ * b.y_), (a.z_ * b.x_) - (a.x_ * b.z_), (a.x_ * b.y_) - (a.y_ * b.x_)); }
		double dot(const vertex& a, const vertex& b) { return (a.x_ * b.x_) + (a.y_ * b.y_) + (a.z_ * b.z_); }
        bool equal(const vertex& a, const vertex& b, double tolerance) { return distance_squared(a, b) <= (tolerance * tolerance); }
        vertex centroid(const triangle& t) { return vertex((t.a_.x_ + t.b_.x_ + t.c_.x_) / 3, (t.a_.y_ + t.b_.y_ + t.c_.y_) / 3, (t.a_.z_ + t.b_.z_ + t.c_.z_) / 3); }
        vertex normal(const triangle& t) { return unitise(cross(subtract(t.b_, t.a_), subtract(t.c_, t.a_))); }
        double plane_distance(const vertex& p, const vertex& n, const vertex& v) { return dot(n, subtract(v, p)); }
        bool point_in_triangle(const triangle& t, const vertex& p) 
        { 
            csg::vertex n = cross(subtract(t.b_, t.a_), subtract(t.c_, t.a_));
            if (std::isnan(n.x_) || (!n.x_ && !n.y_ && !n.z_)) { return false; }
            return (plane_distance(t.a_, cross(subtract(t.b_, t.a_), n), p) <= 0 && 
                plane_distance(t.b_, cross(subtract(t.c_, t.b_), n), p) <= 0 && 
                plane_distance(t.c_, cross(subtract(t.a_, t.c_), n), p) <= 0);
        }
        double area(const triangle& t)
        {
            csg::vertex ab = subtract(t.b_, t.a_); csg::vertex ac = subtract(t.c_, t.a_);
            double cos_theta = dot(unitise(ab), unitise(ac));
            return 0.5 * magnitude(ab) * magnitude(ac) * sqrt(1.0 - (cos_theta * cos_theta));
        }
        bool point_on_segment(const segment& s, const vertex& p)
        {
            double d = distance_squared(s.first, s.second);
            return (distance_squared(s.first, p) + distance_squared(s.second, p)) <= d;
        }
        
        

        namespace intersect
        {

            double ray_plane_distance(const vertex& r, const vertex& d, const vertex& p, const vertex& n)
            {
                double n_dot_d = dot(n, d);
                if (n_dot_d != 0)
                    return dot(subtract(p, r), n) / n_dot_d;
                throw std::runtime_error("Ray and plane do not intersect.");
            }

            vertex ray_plane(const vertex& r, const vertex& d, const vertex& p, const vertex& n)
            {
                double t = ray_plane_distance(r, d, p, n);
                if (t >= 0)
                    return vertex(r.x_ + (d.x_ * t), r.y_ + (d.y_ * t), r.z_ + (d.z_ * t));
                throw std::runtime_error("Ray and plane do not intersect.");
            }

            vertex line_plane(const segment& s, const vertex& p, const vertex& n)
            {
                csg::vertex d = unitise(subtract(s.second, s.first));
                double n_dot_d = dot(n, d);
                if (n_dot_d != 0 && !std::isnan(n_dot_d))
                {
                    double t = dot(subtract(p, s.first), n) / n_dot_d;
                    return vertex(s.first.x_ + (d.x_ * t), s.first.y_ + (d.y_ * t), s.first.z_ + (d.z_ * t));
                }
                throw std::runtime_error("Line and plane do not intersect.");
            }

            // more than one implies intersection points of overlapping segments...
            std::vector<vertex> coplanar_segment_segment(const segment& a, const segment& b)
            {
                std::vector<vertex> result;
                
                // check for overlap...
                double a_dot_b = dot(subtract(a.first, a.second), subtract(b.first, b.second));
                if (a_dot_b != 0)
                {
                    vertex intersection_point;
                    if (point_on_segment(a, intersection_point) && point_on_segment(b, intersection_point)) { result.push_back(intersection_point); }
                }
                else
                {
                    if (point_on_segment(a, b.first)) { result.push_back(b.first); }
                    if (point_on_segment(a, b.second)) { result.push_back(b.second); }
                    if (point_on_segment(b, a.first)) { result.push_back(a.first); }
                    if (point_on_segment(b, a.second)) { result.push_back(a.second); }
                }
                return result;
            }

            // clip a segment which overlaps a triangle...
            std::vector<vertex> coplanar_triangle_clip_segment(const triangle& t, const segment& s)
            {
                std::vector<vertex> result, intersections;
                if (point_in_triangle(t, s.first)) { result.push_back(s.first); }
                if (point_in_triangle(t, s.second)) { result.push_back(s.second); }
                if (result.size() == 2) { return result; }
                intersections = coplanar_segment_segment(segment(t.a_, t.b_), s); result.insert(result.end(), intersections.begin(), intersections.end());
                intersections = coplanar_segment_segment(segment(t.b_, t.c_), s); result.insert(result.end(), intersections.begin(), intersections.end());
                intersections = coplanar_segment_segment(segment(t.c_, t.a_), s); result.insert(result.end(), intersections.begin(), intersections.end());
                assert(result.size() < 3);
                return result;
            }

            class triangle_plane_info
            {
            public:
                triangle_plane_info(const triangle& t, const vertex& p, const vertex& n, double clamp_tolerance)
                :   distances_(plane_distance(p, n, t.a_), plane_distance(p, n, t.b_), plane_distance(p, n, t.c_)), p_(p), n_(n)
                {
                    if (fabs(distances_.x_) < clamp_tolerance) { distances_.x_ = 0.0; }
                    if (fabs(distances_.y_) < clamp_tolerance) { distances_.y_ = 0.0; }
                    if (fabs(distances_.z_) < clamp_tolerance) { distances_.z_ = 0.0; }
                }

                bool outside() const { return distances_.x_ >= 0 && distances_.y_ >= 0 && distances_.z_ >= 0; }
                bool inside() const { return distances_.x_ <= 0 && distances_.y_ <= 0 && distances_.z_ <= 0; }
                bool entirely_outside() const { return distances_.x_ > 0 && distances_.y_ > 0 && distances_.z_ > 0; }
                bool entirely_inside() const { return distances_.x_ < 0 && distances_.y_ < 0 && distances_.z_ < 0; }
                bool point_on() const { return !distances_.x_ || !distances_.x_ || !distances_.x_; }
                bool intersects() const { return (!distances_.x_ || !distances_.y_ || !distances_.z_) || (!entirely_outside() && !entirely_inside()); } 
                bool coplanar() const { return !distances_.x_ && !distances_.y_ && !distances_.z_; }
                bool edge_ab() const { return !distances_.x_ && !distances_.y_ && distances_.z_; }
                bool edge_bc() const { return distances_.x_ && !distances_.y_ && !distances_.z_; }
                bool edge_ca() const { return !distances_.x_ && distances_.y_ && !distances_.z_; }
                bool straddle_ab_major() const { return (distances_.x_ > 0 && distances_.y_ > 0 && distances_.z_ <= 0) && (distances_.x_ < 0 && distances_.y_ < 0 && distances_.z_ >= 0); }
                bool straddle_bc_major() const { return (distances_.x_ <= 0 && distances_.y_ > 0 && distances_.z_ > 0) && (distances_.x_ >= 0 && distances_.y_ < 0 && distances_.z_ < 0); }
                bool straddle_ca_major() const { return (distances_.x_ > 0 && distances_.y_ <= 0 && distances_.z_ > 0) && (distances_.x_ < 0 && distances_.y_ >= 0 && distances_.z_ < 0); }
                bool straddle_bc() const { return ((!distances_.x_ && distances_.y_ > 0 && distances_.z_ < 0) || (!distances_.x_ && distances_.y_ < 0 && distances_.z_ > 0)); }
                bool straddle_ca() const { return ((distances_.x_ < 0 && !distances_.y_ && distances_.z_ > 0) || (distances_.x_ > 0 && !distances_.y_ && distances_.z_ < 0)); }
                bool straddle_ab() const { return ((distances_.x_ < 0 && distances_.y_ > 0 && !distances_.z_) || (distances_.x_ > 0 && distances_.y_ < 0 && !distances_.z_)); }

                vertex distances_;
                vertex p_, n_;
                
            };

            segment triangle_plane_edge(const triangle_plane_info& info, const triangle& t)
            {
                // assert not coplanar, and intersects...
                assert(!info.coplanar() && info.intersects());

                // check for case of triangle edge or point on plane...
                if (info.point_on())
                {
                    // check if triangle edge is on plane...
                    if (info.edge_ab()) { return segment(t.a_, t.b_); }
                    if (info.edge_bc()) { return segment(t.b_, t.c_); }
                    if (info.edge_ca()) { return segment(t.c_, t.a_); }
                    // Otherwise it is just a single point that is on plane....
                    if (!info.distances_.x_) { return segment(t.a_, t.a_); }
                    else if (!info.distances_.y_) { return segment(t.b_, t.b_); }
                    else if (!info.distances_.z_) { return segment(t.c_, t.c_); }
                    throw std::runtime_error("Failed to calculate intersection of single point/edge on.");
                }
                // assume straddling...
                else
                {
                    //if (info.straddle_ab_major()) { return segment(line_plane(segment(t.a_, t.c_), info.p_, info.n_), line_plane(segment(t.b_, t.c_), info.p_, info.n_)); }
                    //if (info.straddle_bc_major()) { return segment(line_plane(segment(t.b_, t.a_), info.p_, info.n_), line_plane(segment(t.c_, t.a_), info.p_, info.n_)); }
                    //if (info.straddle_ca_major()) { return segment(line_plane(segment(t.a_, t.c_), info.p_, info.n_), line_plane(segment(t.b_, t.c_), info.p_, info.n_)); }
                    //throw std::runtime_error("Failed to calculate intersection edge.");
                }
            }

            typedef std::pair<std::vector<segment>, std::vector<segment>> coplanar_intersections;

            // Calculate coplanar intersection between two triangles...
            coplanar_intersections coplanar_triangle_triangle_edges(const triangle& a, const triangle& b)
            {
                coplanar_intersections result; std::vector<vertex> intersection_points;
                intersection_points = coplanar_triangle_clip_segment(b, segment(a.a_, a.b_));
                if (intersection_points.size() == 2) { result.first.push_back(segment(intersection_points.front(), intersection_points.back())); }
                intersection_points = coplanar_triangle_clip_segment(b, segment(a.b_, a.c_));
                if (intersection_points.size() == 2) { result.first.push_back(segment(intersection_points.front(), intersection_points.back())); }
                intersection_points = coplanar_triangle_clip_segment(b, segment(a.c_, a.a_));
                if (intersection_points.size() == 2) { result.first.push_back(segment(intersection_points.front(), intersection_points.back())); }
                intersection_points = coplanar_triangle_clip_segment(a, segment(b.a_, b.b_));
                if (intersection_points.size() == 2) { result.second.push_back(segment(intersection_points.front(), intersection_points.back())); }
                intersection_points = coplanar_triangle_clip_segment(a, segment(b.b_, b.c_));
                if (intersection_points.size() == 2) { result.second.push_back(segment(intersection_points.front(), intersection_points.back())); }
                intersection_points = coplanar_triangle_clip_segment(a, segment(b.c_, b.a_));
                if (intersection_points.size() == 2) { result.second.push_back(segment(intersection_points.front(), intersection_points.back())); }
                return result;
            }

            // Calculate intersection edge between two triangles...
            std::vector<vertex> triangle_triangle_edge(const triangle_plane_info& info, const triangle& a, const triangle& b)
            {
                return coplanar_triangle_clip_segment(b, triangle_plane_edge(info, a));
            }
        }

        namespace tessellate
        {
            // Split triangle result (first is outside, second is inside)
            typedef std::pair<std::vector<triangle>, std::vector<triangle>> split_result;

            split_result split_triangle(const intersect::triangle_plane_info& info, const triangle& t)
            {
                assert(!info.coplanar() && info.intersects());
                if (info.point_on())
                {
                    if (info.edge_ab()) { return info.distances_.z_ > 0 ? split_result({t}, {}) : split_result({}, {t}); }
                    else if (info.edge_bc()) { return info.distances_.x_ > 0 ? split_result({t}, {}) : split_result({}, {t}); }
                    else if (info.edge_ca()) { return info.distances_.y_ > 0 ? split_result({t}, {}) : split_result({}, {t}); }
                    if (info.straddle_bc()) { csg::vertex intersection = intersect::line_plane(segment(t.b_, t.c_), info.p_, info.n_); return split_result({ triangle(t.b_, intersection, t.a_) }, { triangle(t.a_, intersection, t.c_) }); }
                    else if (info.straddle_ca()) { csg::vertex intersection = intersect::line_plane(segment(t.c_, t.a_), info.p_, info.n_); return split_result({ triangle(t.c_, intersection, t.b_) }, { triangle(t.b_, intersection, t.a_) }); }
                    else if (info.straddle_ab()) { csg::vertex intersection = intersect::line_plane(segment(t.a_, t.b_), info.p_, info.n_); return split_result({ triangle(t.a_, intersection, t.c_) }, { triangle(t.c_, intersection, t.b_) }); }
                    throw std::runtime_error("Unable to split point-on triangle");
                }
                else
                {
                    if (info.straddle_ab_major())
                    {


                    }
                }
                throw std::runtime_error("Unable to split triangle.");
            }

            // Delaunay

        }


        class bsp
        {
            class node : public triangle
            {
            public:
                node(const triangle& t) : triangle(t.a_, t.b_, t.c_), normal_(ancillary::normal(t)) 
                { 
                    if (std::isnan(normal_.x_))
                        std::runtime_error("Unable to calculate bsp node.");
                }
                vertex normal_;
                std::unique<node> inside_;
                std::unique<node> outside_;
            };

            std::unique_ptr<node> root_;

        public:
            bsp(const std::vector<working>& mesh, double tolerance)
            {

            }

            enum ioop
            {
                undefined,
                inside,
                outside,
                on_plane
            };
            
            ioop query(const vertex& v, double on_plane_tolerance)
            {


            }
        };


        
        // class bsp
		// {
		// public:
		// 	class node : public triangle
		// 	{
		// 	public:
		// 		node(const triangle& t) : triangle(t.a_, t.b_, t.c_), normal_(normal(t))
		// 		{
		// 			if (std::isnan(normal_.x_) || std::isnan(normal_.y_) || std::isnan(normal_.z_))
		// 				throw std::runtime_error("unable to calculate bsp node normal.");
		// 		}
		// 		const vertex& p() const { return a_; }
		// 		const vertex& n() const { return normal_; }
		// 		std::unique_ptr<node> inside_;
		// 		std::unique_ptr<node> outside_;
		// 	private:
		// 		vertex normal_;
		// 	};

        //     enum ioop
        //     {
        //         undefined,
        //         inside,
        //         outside,
        //         on_plane
        //     }

		// 	// assume all triangles are well-formed...
		// 	bsp(const mesh& m, double tolerance) { construct(m, tolerance); }
			
		// 	ioop query(const vertex& v, double onPlaneTolerance) const
		// 	{
		// 		node* currentNode = root_.get();
		// 		while (currentNode)
		// 		{
		// 			// first we check if on the plane... project point onto plane if distance to plane is < tol.
		// 			double d = distanceFromPlane(currentNode->p(), currentNode->n(), v, onPlaneTolerance);
		// 			if (d == 0)
		// 			{
		// 				if (pointWithinTriangle(currentNode->a_, currentNode->b_, currentNode->c_, v)) { return on_plane; }
		// 				else if (currentNode->inside_.get()) { currentNode = currentNode->inside_.get(); }
		// 				else { return outside; }
		// 			}
		// 			else if (d > 0)
		// 			{
		// 				if (currentNode->outside_) { currentNode = currentNode->outside_.get(); }
		// 				else { return outside; }
		// 			}
		// 			else if (d < 0)
		// 			{
		// 				if (currentNode->inside_) { currentNode = currentNode->inside_.get(); }
		// 				else { return inside; }
		// 			}
		// 		}
		// 		throw std::runtime_error("cannot deduce if point is inside or outside.");
		// 	}

		// private:
		// 	void construct(const mesh& m, double tolerance)
		// 	{
		// 		// create a working list of the triangles. We validate as we go along.
		// 		std::list<triangle> workingTriangles(m.begin(), m.end());
		// 		while (!workingTriangles.empty() && !sanitise::isValid(workingTriangles.front(), tolerance))
		// 			workingTriangles.pop_front();

		// 		if (workingTriangles.empty())
		// 			return;

		// 		root_.reset(new node(workingTriangles.front()));
		// 		while (!workingTriangles.empty())
		// 		{
		// 			std::list<triangle>::iterator currentTriangle = workingTriangles.begin();
		// 			if (!sanitise::isValid(*currentTriangle, tolerance))
		// 			{
		// 				workingTriangles.erase(currentTriangle);
		// 				continue;
		// 			}

		// 			node* currentNode = root_.get();
		// 			while (currentNode)
		// 			{
		// 				intersection::trianglePlane intersectionInfo(&(*currentTriangle), currentNode->p(), currentNode->n(), tolerance);

		// 				const triangle* workingTriangle = intersectionInfo.IsClamped() ? &intersectionInfo.ClampedTriangle() : &(*currentTriangle);
						
		// 				if (intersectionInfo.onPlane())
		// 					currentNode = followInsideNode(currentNode, *workingTriangle);
		// 				else if (intersectionInfo.outside())
		// 					currentNode = followOutsideNode(currentNode, *workingTriangle);
		// 				else if (intersectionInfo.inside())
		// 					currentNode = followInsideNode(currentNode, *workingTriangle);
		// 				else
		// 				{
		// 					intersection::splitResult splitTriangles = intersection::splitTriangle(intersectionInfo, *workingTriangle, normal(*workingTriangle), currentNode->p(), currentNode->n());
		// 					workingTriangles.insert(workingTriangles.begin(), splitTriangles.inside_.begin(), splitTriangles.inside_.end());
		// 					workingTriangles.insert(workingTriangles.begin(), splitTriangles.outside_.begin(), splitTriangles.outside_.end());
		// 					currentNode = 0;
		// 				}
		// 				if (!currentNode)
		// 					workingTriangles.erase(currentTriangle);
		// 			}
		// 		}
		// 	}
		// 	node* followOutsideNode(node* nde, const triangle& tri)
		// 	{
		// 		if (nde->outside_)
		// 			return nde->outside_.get();
		// 		nde->outside_.reset(new node(tri));
		// 		return 0;
		// 	}
		// 	node* followInsideNode(node* nde, const triangle& tri)
		// 	{
		// 		if (nde->inside_)
		// 			return nde->inside_.get();
		// 		nde->inside_.reset(new node(tri));
		// 		return 0;
		// 	}
		// 	std::unique_ptr<node> root_;
		// };

        class working
        {
        public:
            working(const triangle& t, double tolerance) : triangle_(t), normal_(normal(t)), valid_(area(t) < tolerance) {}

            // retessellate...
            void retessellate()
            {
                // Collate all the points into a single array, and create segment indexes accordinlly...

                // Sanitise...

                // tessellate...

                // Assert windings...

                // Populate mesh...
            }

            std::vector<vertex> intersection_vertices() const
            {
                std::vector<vertex> result(intersections_.size() * 2);
                for (unsigned int s = 0; s < intersections_.size(); ++s)
                {
                    result[s * 2] = intersections_[s].first;
                    result[(s * 2)+1] = intersections_[s].second;
                }
                return result;
            }

            triangle                triangle_;
            vertex                  normal_;
            std::vector<segment>    intersections_;
            std::vector<triangle>   mesh_;
            bool                    valid_;

        };
        
    }



    class merge
    {
    public:

        merge(const std::vector<triangle>& a_mesh, const std::vector<triangle>& b_mesh, double tolerance)
        {
            a_.reserve(a_mesh.size()); b_.reserve(b_mesh.size());
            for (int a = 0; a < a_mesh.size(); ++a) { a_.push_back(ancillary::working(a_mesh[a], tolerance)); }
            for (int b = 0; b < b_mesh.size(); ++b) { b_.push_back(ancillary::working(b_mesh[b], tolerance)); }
            calculate_intersections(tolerance);
        }

        merge(unsigned int a_count, triangle_populator_fn a_triangles, unsigned int b_count, triangle_populator_fn b_triangles, double tolerance)
        {
            a_.reserve(a_count); b_.reserve(b_count);
            for (int a = 0; a < a_count; ++a) { a_.push_back(ancillary::working(a_triangles(a), tolerance)); }
            for (int b = 0; b < b_count; ++b) { b_.push_back(ancillary::working(b_triangles(b), tolerance)); }
            calculate_intersections(tolerance);
        }

        std::vector<std::vector<vertex>> intersection_loops() const
        {
            // Collate all the intersection edges...
            std::list<segment> intersection_loop_segments;

            for (int a = 0; a < a_.size(); ++a)
                intersection_loop_segments.insert(intersection_loop_segments.end(), a_[a].intersections_.begin(), a_[a].intersections_.end());
            for (int b = 0; b < b_.size(); ++b)
                intersection_loop_segments.insert(intersection_loop_segments.end(), b_[b].intersections_.begin(), b_[b].intersections_.end());
            
            // Order them into looping orders, split different polyloops...
            std::vector<std::vector<vertex>> result;





            return result;

        }

    protected:
        struct mesh_triangles
        {
            std::vector<triangle> outside_, inside_, on_plane_;
        };

        mesh_triangles a_triangles() { return triangles(a_), bspB_); }
        mesh_triangles b_triangles() { return triangles(b_), bspA_); }

    private:
        mesh_triangles triangles(const std::vector<ancillary::working>& mesh, const ancillary::bsp& bsp)
        {
            mesh_triangles result;
            for (auto itr = mesh.begin(); itr != mesh.end(); ++itr)
            {
                vertex centroid = ancillary::centroid(*itr);
                    ancillary::bsp::ioop ioop = bspB_.query(centroid, tolerance);
                    switch (ioop)
                    {
                    case ancillary::bsp::ioop::inside:
                        result.inside_.push_back(*itr);
                        break;
                    case ancillary::bsp::ioop::outside:
                        result.outside_.push_back(*itr);
                        break;
                    case ancillary::bsp::ioop::on_plane:
                        result.on_plane_.push_back(*itr);
                        break;
                    }
            }
            return result;
        }

        void calculate_intersections(double tolerance)
        {
            for (int a = 0; a < a_.size(); ++a)
            {
                if (a_[a].valid_)
                {
                    for (int b = 0; b < b_.size(); ++b)
                    {
                        if (b_[b].valid_)
                        {
                            ancillary::intersect::triangle_plane_info tpi(a_[a].triangle_, b_[b].triangle_.a_, b_[b].normal_, tolerance);
                            if (tpi.coplanar())
                            {
                                auto intersection_edges = ancillary::intersect::coplanar_triangle_triangle_edges(a_[a].triangle_, b_[b].triangle_);
                                a_[a].intersections_.insert(a_[a].intersections_.end(), intersection_edges.first.begin(), intersection_edges.first.end());
                                b_[b].intersections_.insert(b_[b].intersections_.end(), intersection_edges.second.begin(), intersection_edges.second.end());
                            }
                            else if (!tpi.entirely_outside() && !tpi.entirely_inside())
                            {
                                auto intersection_edge = ancillary::intersect::triangle_triangle_edge(tpi, a_[a].triangle_, b_[b].triangle_);
                                if (intersection_edge.size() == 2)
                                {
                                    a_[a].intersections_.push_back(segment(intersection_edge.front(), intersection_edge.back()));
                                    b_[b].intersections_.push_back(segment(intersection_edge.front(), intersection_edge.back()));
                                }
                            }
                        }
                    }
                }
            }

            for (int a = 0; a < a_.size(); ++a) { a_[a].retessellate(); }
            for (int b = 0; b < b_.size(); ++b) { b_[b].retessellate(); }

            //bspA_ = ancillary::bsp(a_);

        }

        std::vector<ancillary::working> a_, b_;
        ancillary::bsp bspA_, bspB_;
    };

    class mesh_union : public merge
    {
        mesh_triangles a_triangles_, b_triangles_;
        
    public:
        mesh_union(const merge& m)
        :   merge(m), a_triangles_(a_triangles()), b_triangles_(b_triangles())
        {
        }
    };


    class slice
    {
    public:
        slice(const std::vector<triangle>& mesh, const vertex& p, const vertex& n, double tolerance)
        {
            for (int t = 0; t < mesh.size(); ++t)
            {
                ancillary::intersect::triangle_plane_info tpi(mesh[t], p, n, tolerance);
                if (tpi.coplanar())
                    triangles_.first.push_back(mesh[t]);
                else if (tpi.intersects())
                {
                    ancillary::tessellate::split_result triangle_split;
                    if (tpi.point_on())
                    {

                    }
                    else
                        triangle_split = ancillary::tessellate::split_triangle(tpi, mesh[t]);
                    triangles_.first.insert(triangles_.first.end(), triangle_split.first.begin(), triangle_split.first.end());
                    triangles_.second.insert(triangles_.second.end(), triangle_split.second.begin(), triangle_split.second.end());
                }
                else
                {
                    if (tpi.outside())
                        triangles_.first.push_back(mesh[t]);
                    else if (tpi.outside())
                        triangles_.second.push_back(mesh[t]);
                }
            }

            // tessellate the outline intersection...


            // compare with mesh to see inside/outside...

            // keep these triangles for the cap...

        }

        // get the half space triangles...

        // get the cap


        // get the intersection_loops

        ancillary::tessellate::split_result triangles_;
        std::vector<triangle> cap_;
    }


    

}

#endif // CSG3_HPP
