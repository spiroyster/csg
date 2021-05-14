#ifndef CSG3_HPP
#define CSG3_HPP

#include <functional>
#include <vector>
#include <assert.h>
#include <math.h>

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
	
	//  callback for extracting loops from merge
	typedef std::function<void(unsigned int loop, unsigned int index, const vertex& point)> loop_extractor_fn;


    namespace ancillary
    {
        vertex subtract(const vertex& a, const vertex& b) { return vertex(a.x_ - b.x_, a.y_ - b.y_, a.z_ - b.z_); }
        double magnitude(const vertex& v) { return sqrt((v.x_ * v.x_) + (v.y_ * v.y_) + (v.z_ * v.z_)); }
        vertex unitise(const vertex& v) { double mag_recip = 1.0 / magnitude(v); return vertex(v.x_ * mag_recip, v.y_ * mag_recip, v.z_ * mag_recip); }
        double distance(const vertex& a, const vertex& b) { return magnitude(subtract(b, a)); }
        vertex cross(const vertex& a, const vertex& b) { return vertex((a.y_ * b.z_) - (a.z_ * b.y_), (a.z_ * b.x_) - (a.x_ * b.z_), (a.x_ * b.y_) - (a.y_ * b.x_)); }
		double dot(const vertex& a, const vertex& b) { return (a.x_ * b.x_) + (a.y_ * b.y_) + (a.z_ * b.z_); }
        bool equal(const vertex& a, const vertex& b, double tolerance) { return distance(a, b) < tolerance; }
        vertex centroid(const triangle& t) { return vertex((t.a_.x_ + t.b_.x_ + t.c_.x_) / 3, (t.a_.y_ + t.b_.y_ + t.c_.y_) / 3, (t.a_.z_ + t.b_.z_ + t.c_.z_) / 3); }
        vertex normal(const triangle& t) { return unitise(cross(subtract(t.b_, t.a_), subtract(t.c_, t.a_))); }
        double plane_distance(const vertex& p, const vertex& n, const vertex& v) { return dot(n, subtract(v, p)); }
        bool overlap(const segment& a, const segment& b)
        {

        }
        bool triangle_point(const triangle& t, const vertex& p)
        {
            return false;
        }

        double area(const triangle& t)
        {
            return 0.0;
        }

        
        

        namespace intersect
        {
            
            vertex ray_plane(const vertex& v, const vertex& d, const vertex& p, const vertex& n)
            {
                return vertex();
            }

            // more than one implies intersection points of overllaping segments...
            std::vector<vertex> segment_segment(const segment& a, const segment& b)
            {
                if (overlap(a, b))
                {

                }
                else
                {


                }
            }

            // clip a segment which overlaps a triangle...
            std::vector<vertex> triangle_clip_segment(const triangle& t, const segment& s)
            {
                std::vector<vertex> result(2), intersections;
                intersections = segment_segment(segment(t.a_, t.b_), s); result.insert(result.end(), intersections.begin(), intersections.end());
                intersections = segment_segment(segment(t.b_, t.c_), s); result.insert(result.end(), intersections.begin(), intersections.end());
                intersections = segment_segment(segment(t.c_, t.a_), s); result.insert(result.end(), intersections.begin(), intersections.end());
                return result;
            }

            class triangle_plane_info
            {
            public:
                triangle_plane_info(const triangle& t, const vertex& p, const vertex& n, double clamp_tolerance)
                :   distances_(plane_distance(p, n, t.a_), plane_distance(p, n, t.b_), plane_distance(p, n, t.c_)), p_(p), n_(n)
                {
                    if (abs(distances_.x_) < clamp_tolerance) { distances_.x_ = 0.0; }
                    if (abs(distances_.y_) < clamp_tolerance) { distances_.y_ = 0.0; }
                    if (abs(distances_.z_) < clamp_tolerance) { distances_.z_ = 0.0; }
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

                vertex distances_;
                vertex p_, n_;
                
            };

            segment triangle_plane_edge(const triangle_plane_info& info, const triangle& t)
            {
                // assert not coplanar, and intersects...
                assert(!info.coplanar() && info.intersects());

                // check for case of triangle edge on plane, in which case we return edge...
                if (info.point_on())
                {
                    if (info.edge_ab()) { return segment(t.a_, t.b_); }
                    if (info.edge_bc()) { return segment(t.b_, t.c_); }
                    if (info.edge_ca()) { return segment(t.c_, t.a_); }
                    
                    // Otherwise it is just a single point that intersects....
                }

                // assume straddling...
                if (info.straddle_ab_major())
                {
                    return segment(ray_plane(t.a_, unitise(subtract(t.a_, t.c_)), info.p_, info.n_),
                        ray_plane(t.b_, unitise(subtract(t.b_, t.c_)), info.p_, info.n_));
                }
                else if (info.straddle_bc_major())
                {
                    return segment(ray_plane(t.b_, unitise(subtract(t.b_, t.a_)), info.p_, info.n_),
                        ray_plane(t.c_, unitise(subtract(t.c_, t.a_)), info.p_, info.n_));
                }
                else if (info.straddle_ab_major())
                {
                    return segment(ray_plane(t.a_, unitise(subtract(t.a_, t.c_)), info.p_, info.n_),
                        ray_plane(t.b_, unitise(subtract(t.b_, t.c_)), info.p_, info.n_));
                }

                // throw... this may be a single intersection point, however we ignore this as donot consider it an intersection...
                throw std::runtime_error("Failed to calculate intersection edge.");
            }

            typedef std::pair<std::vector<segment>, std::vector<segment>> coplanar_intersections;

            // Calculate coplanar intersection between two triangles...
            coplanar_intersections coplanar_triangle_triangle_edges(const triangle& a, const triangle& b)
            {
                coplanar_intersections result;
                result.first.push_back(triangle_clip_segment(b, segment(a.a_, a.b_)));
                result.first.push_back(triangle_clip_segment(b, segment(a.b_, a.c_)));
                result.first.push_back(triangle_clip_segment(b, segment(a.c_, a.a_)));
                result.second.push_back(triangle_clip_segment(a, segment(b.a_, b.b_)));
                result.second.push_back(triangle_clip_segment(a, segment(b.b_, b.c_)));
                result.second.push_back(triangle_clip_segment(a, segment(b.c_, b.a_)));
                return result;
            }

            // Calculate intersection edge between two triangles...
            segment triangle_triangle_edge(const triangle_plane_info& info, const triangle& a, const triangle& b)
            {
                return triangle_clip_segment(b, triangle_plane_edge(info, a));
            }
        }

        namespace tessellate
        {
            // Split triangle

            // Delaunay

        }
        
        class bsp
        {

        };

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

        std::vector<std::vector<vertex>> intersection_loops()
        {
            // Collate all the intersection edges...

            // Order them into looping orders, split different polyloops...


            return std::vector<std::vector<vertex>>();

        }

    private:

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
                                segment intersection_edge = ancillary::intersect::triangle_triangle_edge(tpi, a_[a].triangle_, b_[b].triangle_);
                                a_[a].intersections_.push_back(intersection_edge);
                                b_[b].intersections_.push_back(intersection_edge);
                            }
                        }
                    }
                }
            }

            for (int a = 0; a < a_.size(); ++a) { a_[a].retessellate(); }
            for (int b = 0; b < b_.size(); ++b) { b_[b].retessellate(); }
        }

        std::vector<ancillary::working> a_, b_;
    };

    class mesh_union : public merge
    {
    public:
        mesh_union(const merge& m)
        :   merge(m)
        {

        }
        



    };

}

#endif // CSG3_HPP
