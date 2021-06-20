#include "../include/csg3.hpp"
#include "LogiQA.hpp"

LOGIQA_TEST_RUNNER { RUNALL }

namespace test_data
{
    struct vector_equal
    {
        vector_equal(const csg::vertex& a, const csg::vertex& b, double tolerance, bool equal)
        :   a_(a), b_(b), tolerance_(tolerance), equal_(equal) {}
        
        csg::vertex a_, b_;
        double tolerance_;
        bool equal_;
    };

    struct plane_distance
    {
        plane_distance(const csg::vertex& p, const csg::vertex& n, const csg::vertex& v, double tolerance, double result)
            : p_(p), n_(n), v_(v), tolerance_(tolerance), result_(result) {}

        csg::vertex p_, n_, v_;
        double tolerance_, result_;
    };

    struct centroid
    {
        centroid(const csg::triangle& t, const csg::vertex& result)
            :   t_(t), result_(result) {}

        csg::triangle t_;
        csg::vertex result_;
    };

    struct point_on_segment
    {
        point_on_segment(const csg::segment& s, const csg::vertex& p) : s_(s), p_(p) {}

        csg::segment s_;
        csg::vertex p_;
    };

    struct point_in_triangle
    {
        point_in_triangle(const csg::triangle& t, const csg::vertex& v) : t_(t), v_(v) {}

        csg::triangle t_;
        csg::vertex v_;
    };

    struct area
    {
        area(const csg::triangle& t, double area)
        :   t_(t), area_(area) {}

        csg::triangle t_;
        double area_;
    };

    struct ray_plane
    {
        ray_plane(const csg::vertex& r, const csg::vertex& d, const csg::vertex& p, const csg::vertex& n, const csg::vertex& intersection)
        :   r_(r), d_(d), p_(p), n_(n), intersection_(intersection) {}

        csg::vertex r_, d_, p_, n_, intersection_;
    };

    struct line_plane
    {
        line_plane(const csg::segment& l, const csg::vertex& p, const csg::vertex& n, const csg::vertex& intersection)
            : l_(l), p_(p), n_(n), intersection_(intersection) {}

        csg::segment l_;
        csg::vertex p_, n_, intersection_;
    };

    
    struct coplanar_segment_segment
    {
        coplanar_segment_segment(const csg::segment& i, const csg::segment& j, const std::vector<csg::vertex>& intersections)
            :   i_(i), j_(j), intersections_(intersections) {}

        csg::segment i_, j_;
        std::vector<csg::vertex> intersections_;
    };

    struct coplanar_triangle_clip_segment
    {
        coplanar_triangle_clip_segment(const csg::triangle& t, const csg::segment& s, const std::vector<csg::vertex>& intersections)
            :   t_(t), s_(s), intersections_(intersections) {}

        csg::triangle t_;
        csg::segment s_;
        std::vector<csg::vertex> intersections_;
    };

    // triangle plane info
    

    // coplanar triangle triangle intersection


    // triangle triangle intersection


    // triangle area
    
    
}

// vector equals
LOGIQA_TEST_PARAMS(vector_equals, "", logiqa::param<test_data::vector_equal>({
    test_data::vector_equal(csg::vertex(), csg::vertex(), 0, true),
    test_data::vector_equal(csg::vertex(1.0, 1.0, 1.0), csg::vertex(1.0, 1.0, 1.0), 0, true),
    test_data::vector_equal(csg::vertex(-1.0, -1.0, -1.0), csg::vertex(-1.0, -1.0, -1.0), 0, true),

    test_data::vector_equal(csg::vertex(), csg::vertex(), 0.001, true),
    test_data::vector_equal(csg::vertex(1.0, 1.0, 1.0), csg::vertex(1.0, 1.0, 1.0), 0.001, true),
    test_data::vector_equal(csg::vertex(-1.0, -1.0, -1.0), csg::vertex(-1.0, -1.0, -1.0), 0.001, true),

    test_data::vector_equal(csg::vertex(), csg::vertex(0.001, 0, 0), 0.001, true),
    test_data::vector_equal(csg::vertex(), csg::vertex(0, 0.001, 0), 0.001, true),
    test_data::vector_equal(csg::vertex(), csg::vertex(0, 0, 0.001), 0.001, true),

    test_data::vector_equal(csg::vertex(), csg::vertex(0.001 + 0.001, 0, 0), 0.001, false),
    test_data::vector_equal(csg::vertex(), csg::vertex(0, 0.001 + 0.001, 0), 0.001, false),
    test_data::vector_equal(csg::vertex(), csg::vertex(0, 0, 0.001 + 0.001), 0.001, false),

    test_data::vector_equal(csg::vertex(), csg::vertex(1.0, 0, 0), 0.001, false),
    test_data::vector_equal(csg::vertex(), csg::vertex(0.1, 0.1, 0.1), 0.001, false),
    test_data::vector_equal(csg::vertex(), csg::vertex(1.0, 0, 0), 0.001, false),
    test_data::vector_equal(csg::vertex(), csg::vertex(0.1, 0.1, 0.1), 0.001, false)
})) 
{ 
    auto v = logiqa_param(); 
    bool r = csg::ancillary::equal(v.a_, v.b_, v.tolerance_);
    ASSERT_EQ(r, v.equal_) 
}


// plane distance
LOGIQA_TEST_PARAMS(plane_distance, "", logiqa::param<test_data::plane_distance>({
    test_data::plane_distance(csg::vertex(), csg::vertex(1.0, 0, 0), csg::vertex(1.0, 0, 0), 0, 1.0),
    test_data::plane_distance(csg::vertex(), csg::vertex(1.0, 0, 0), csg::vertex(-1.0, 0, 0), 0, -1.0),
    test_data::plane_distance(csg::vertex(), csg::vertex(1.0, 0, 0), csg::vertex(), 0, 0)
    
})) { auto v = logiqa_param(); ASSERT_NEAR(csg::ancillary::plane_distance(v.p_, v.n_, v.v_), v.result_, v.tolerance_) }

// Point on segment
LOGIQA_TEST_PARAMS(point_on_segment_pass, "", logiqa::param<test_data::point_on_segment>({
    test_data::point_on_segment(csg::segment(), csg::vertex()),
    test_data::point_on_segment(csg::segment(csg::vertex(), csg::vertex(1.0, 0, 0)), csg::vertex()),
    test_data::point_on_segment(csg::segment(csg::vertex(), csg::vertex(1.0, 0, 0)), csg::vertex(1.0, 0, 0)),
    test_data::point_on_segment(csg::segment(csg::vertex(), csg::vertex(1.0, 0, 0)), csg::vertex(0.5, 0, 0))
})) { auto v = logiqa_param(); ASSERT_EQ(csg::ancillary::point_on_segment(v.s_, v.p_), true) }

LOGIQA_TEST_PARAMS(point_on_segment_fail, "", logiqa::param<test_data::point_on_segment>({
    test_data::point_on_segment(csg::segment(), csg::vertex(1.0, 0, 0)),
    test_data::point_on_segment(csg::segment(), csg::vertex(-1.0, 0, 0))
})) { auto v = logiqa_param(); ASSERT_EQ(csg::ancillary::point_on_segment(v.s_, v.p_), false) }


// point in triangle
LOGIQA_TEST_PARAMS(point_in_triangle_pass, "", logiqa::param<test_data::point_in_triangle>({
    test_data::point_in_triangle(csg::triangle(csg::vertex(), csg::vertex(1.0, 0, 0), csg::vertex(1.0, 0, 0)), csg::vertex(1.0, 0, 0)),
    test_data::point_in_triangle(csg::triangle(csg::vertex(), csg::vertex(0, 0, 1.0), csg::vertex(0, 0, 1.0)), csg::vertex(0, 0, 1.0)),
    test_data::point_in_triangle(csg::triangle(csg::vertex(), csg::vertex(0, 1.0, 0), csg::vertex(0, 1.0, 0)), csg::vertex(0, 1.0, 0)),
    
    // right angle tri
    test_data::point_in_triangle(csg::triangle(csg::vertex(), csg::vertex(1.0, 0, 0), csg::vertex(0, 1.0, 0)), csg::vertex(1.0, 0, 0)),
    
    
    test_data::point_in_triangle(csg::triangle(csg::vertex(), csg::vertex(), csg::vertex()), csg::vertex(1.0, 0, 0))
})) { auto v = logiqa_param(); ASSERT_EQ(csg::ancillary::point_in_triangle(v.t_, v.v_), true) }

LOGIQA_TEST_PARAMS(point_in_triangle_fail, "", logiqa::param<test_data::point_in_triangle>({
    test_data::point_in_triangle(csg::triangle(csg::vertex(), csg::vertex(), csg::vertex()), csg::vertex(1.0, 0, 0)),
    test_data::point_in_triangle(csg::triangle(csg::vertex(), csg::vertex(1.0, 0, 0), csg::vertex(1.0, 0, 0)), csg::vertex(-1.0, 0, 0))
})) { auto v = logiqa_param(); ASSERT_EQ(csg::ancillary::point_in_triangle(v.t_, v.v_), false) }

// ray plane intersection
LOGIQA_TEST_PARAMS(ray_plane_pass, "", logiqa::param<test_data::ray_plane>({
    test_data::ray_plane(csg::vertex(-2.0, 0, 0), csg::vertex(1.0, 0, 0), csg::vertex(), csg::vertex(-1.0, 0, 0), csg::vertex()),
    test_data::ray_plane(csg::vertex(2.0, 0, 0), csg::vertex(-1.0, 0, 0), csg::vertex(), csg::vertex(1.0, 0, 0), csg::vertex()),
})) 
{ 
    auto v = logiqa_param();
    csg::vertex i = csg::ancillary::intersect::ray_plane(v.r_, v.d_, v.p_, v.n_);
    ASSERT(csg::ancillary::equal(i, v.intersection_, 0.001)) 
}

LOGIQA_TEST_PARAMS(ray_plane_fail, "", logiqa::param<test_data::ray_plane>({
    test_data::ray_plane(csg::vertex(), csg::vertex(), csg::vertex(), csg::vertex(), csg::vertex()),
    test_data::ray_plane(csg::vertex(-2.0, 0, 0), csg::vertex(0, 0, 1.0), csg::vertex(), csg::vertex(-1.0, 0, 0), csg::vertex()),
    test_data::ray_plane(csg::vertex(2.0, 0, 0), csg::vertex(1.0, 0, 0), csg::vertex(), csg::vertex(-1.0, 0, 0), csg::vertex()),
    test_data::ray_plane(csg::vertex(2.0, 0, 0), csg::vertex(1.0, 0, 0), csg::vertex(), csg::vertex(1.0, 0, 0), csg::vertex()),
    test_data::ray_plane(csg::vertex(-2.0, 0, 0), csg::vertex(-1.0, 0, 0), csg::vertex(), csg::vertex(-1.0, 0, 0), csg::vertex()),
    test_data::ray_plane(csg::vertex(-2.0, 0, 0), csg::vertex(-1.0, 0, 0), csg::vertex(), csg::vertex(1.0, 0, 0), csg::vertex())
})) { auto v = logiqa_param(); try { csg::ancillary::intersect::ray_plane(v.r_, v.d_, v.p_, v.n_); ASSERT_FAIL } catch (...) { ASSERT_PASS } }

// line plane intersection
LOGIQA_TEST_PARAMS(line_plane_pass, "", logiqa::param<test_data::line_plane>({
    test_data::line_plane(csg::segment(csg::vertex(-2.0, 0, 0), csg::vertex(1.0, 0, 0)), csg::vertex(), csg::vertex(-1.0, 0, 0), csg::vertex()),
    test_data::line_plane(csg::segment(csg::vertex(2.0, 0, 0), csg::vertex(-1.0, 0, 0)), csg::vertex(), csg::vertex(1.0, 0, 0), csg::vertex()),
    test_data::line_plane(csg::segment(csg::vertex(2.0, 0, 0), csg::vertex(1.0, 0, 0)), csg::vertex(), csg::vertex(-1.0, 0, 0), csg::vertex())
})) { auto v = logiqa_param(); ASSERT(csg::ancillary::equal(csg::ancillary::intersect::line_plane(v.l_, v.p_, v.n_), v.intersection_, 0.001)) }

LOGIQA_TEST_PARAMS(line_plane_fail, "", logiqa::param<test_data::line_plane>({
    test_data::line_plane(csg::segment(csg::vertex(), csg::vertex()), csg::vertex(), csg::vertex(), csg::vertex()),
    test_data::line_plane(csg::segment(csg::vertex(0, 1.0, 0), csg::vertex()), csg::vertex(), csg::vertex(1.0, 0, 0), csg::vertex()),
    test_data::line_plane(csg::segment(csg::vertex(1.0, 1.0, 0), csg::vertex(1.0, 0, 0)), csg::vertex(), csg::vertex(1.0, 0, 0), csg::vertex()),
    test_data::line_plane(csg::segment(csg::vertex(1.0, 1.0, 0), csg::vertex(1.0, 0, 0)), csg::vertex(), csg::vertex(-1.0, 0, 0), csg::vertex()),
    test_data::line_plane(csg::segment(csg::vertex(-1.0, 1.0, 0), csg::vertex(-1.0, 0, 0)), csg::vertex(), csg::vertex(1.0, 0, 0), csg::vertex()),
    test_data::line_plane(csg::segment(csg::vertex(-1.0, 1.0, 0), csg::vertex(-1.0, 0, 0)), csg::vertex(), csg::vertex(-1.0, 0, 0), csg::vertex())
})) { auto v = logiqa_param(); try { csg::ancillary::intersect::line_plane(v.l_, v.p_, v.n_); ASSERT_FAIL } catch (...) { ASSERT_PASS } }

