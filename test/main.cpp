#include "../include/csg3.hpp"
#include "jahoutf.hpp"

JAHOUTF_TEST_RUNNER { RUNALL }


struct vertex_test_data
{
    csg::vertex a_, b_, expected_;
    vertex_test_data(const csg::vertex& a, const csg::vertex& b, const csg::vertex& expected) : a_(a), b_(b), expected_(expected) {}
};

TEST_VALUES(impl, add, jahoutf::values<vertex_test_data>({
    vertex_test_data(csg::vertex(), csg::vertex(), csg::vertex()),
    vertex_test_data(csg::vertex(1.0, 0, 0), csg::vertex(), csg::vertex(1.0, 0, 0)),
    vertex_test_data(csg::vertex(), csg::vertex(), csg::vertex())
}))
{
    auto v = jahoutf_value();
    EXPECT(csg::impl::equals(csg::impl::add(v.a_, v.b_), v.expected_))
}

