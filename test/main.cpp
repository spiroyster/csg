#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "..\include\csg.hpp"
#include "..\examples\obj2csg.hpp"

namespace
{
	/*std::shared_ptr<csg::mesh> read(const std::string& filename)
	{
		std::shared_ptr<objio::mesh> m = objio::readFile(filename);

		std::shared_ptr<csg::mesh> result(new csg::mesh);

		std::for_each(m->faces_.begin(), m->faces_.end(),
			[&result, &m](const objio::face& f)
			{
				const objio::vector3& a = m->p_[f.vertices_[0].p_];
				const objio::vector3& b = m->p_[f.vertices_[1].p_];
				const objio::vector3& c = m->p_[f.vertices_[2].p_];
				result->push_back(csg::triangle(csg::vertex(a.x_, a.y_, a.z_), csg::vertex(b.x_, b.y_, b.z_), csg::vertex(c.x_, c.y_, c.z_)));
			});

		return result;
	}*/


}

TEST_CASE("CSG offsetCube", "[csg_offsetCube_difference]")
{
	//std::shared_ptr<csg::mesh> A = obj2csg::read("cube.obj");
	//std::shared_ptr<csg::mesh> B = obj2csg::read("cube.obj");



}


//#define CATCH_CONFIG_MAIN
//#include "catch.hpp"
//#include "..\include\delaunay.hpp"
//
//namespace
//{
//	// Nothing to see here, cba to type ;)
//	void outputSyntax(const delaunay::tessellator& t)
//	{
//		{
//			std::ofstream ofs("edges.txt");
//			std::for_each(t.getEdges().begin(), t.getEdges().end(), [&ofs](const delaunay::edge& e) { ofs << "delaunay::edge{" << e.i_ << ", " << e.j_ << "}," << '\n'; });
//		}
//		{
//			std::ofstream ofs("triangles.txt");
//			std::for_each(t.getTriangles().begin(), t.getTriangles().end(), [&ofs](const delaunay::triangle& t) { ofs << "delaunay::triangle{" << t.a_ << ", " << t.b_ << ", " << t.c_ << "}," << '\n'; });
//		}
//	}
//	
//	bool approxEqual(const double& x, const double& y, int ulp = 2)
//	{
//		// https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
//		return std::abs(x - y) <= std::numeric_limits<double>::epsilon() * std::abs(x + y) * static_cast<double>(ulp) || std::abs(x - y) < (std::numeric_limits<double>::min)();
//	}
//
//	bool approxEqual(const delaunay::vector2& a, const delaunay::vector2& b)
//	{
//		return approxEqual(a.x_, b.x_) && approxEqual(a.y_, b.y_);
//	}
//
//	bool equal(const delaunay::edge& e, const delaunay::edge& ee)
//	{
//		return (e.i_ == ee.i_ && e.j_ == ee.j_) || (e.i_ == ee.j_ && e.j_ == ee.i_);
//	}
//
//	bool equal(const delaunay::triangle& t, const delaunay::triangle& tt)
//	{
//		return (t.a_ == tt.a_ || t.a_ == tt.b_ || t.a_ == tt.c_) && (t.b_ == tt.a_ || t.b_ == tt.b_ || t.b_ == tt.c_) && (t.c_ == tt.a_ || t.c_ == tt.b_ || t.c_ == tt.c_);
//	}
//}
//
//
//void check(const delaunay::tessellator& tessellator, const std::vector<delaunay::vector2>& vertices, const std::vector<delaunay::edge>& edges, const std::vector<delaunay::triangle>& triangles)
//{
//	// Check the vertices...
//	REQUIRE(tessellator.getVertices().size() == vertices.size());
//	for (unsigned int i = 0; i < vertices.size(); ++i)
//	{
//		const delaunay::vector2& v = vertices[i];
//		REQUIRE(std::find_if(tessellator.getVertices().begin(), tessellator.getVertices().end(), [&v](const delaunay::vector2& vv) { return approxEqual(v, vv); }) != tessellator.getVertices().end());
//	}
//		
//	REQUIRE(tessellator.getEdges().size() == edges.size());
//	for (unsigned int i = 0; i < edges.size(); ++i)
//	{
//		const delaunay::edge& e = edges[i];
//		REQUIRE(std::find_if(tessellator.getEdges().begin(), tessellator.getEdges().end(), [&e](const delaunay::edge& ee) { return equal(e, ee); }) != tessellator.getEdges().end());
//	}
//
//	REQUIRE(tessellator.getTriangles().size() == triangles.size());
//	for (unsigned int i = 0; i < triangles.size(); ++i)
//	{
//		const delaunay::triangle& t = triangles[i];
//		REQUIRE(std::find_if(tessellator.getTriangles().begin(), tessellator.getTriangles().end(), [&t](const delaunay::triangle& tt) { return equal(t, tt); }) != tessellator.getTriangles().end());
//	}
//
//}
//
//TEST_CASE("Tessellate triangle", "[delaunay_tessellate_triangle]")
//{
//	//Delaunay::Tessellator tessellator;
//
//	// Vertices...
//	std::vector<delaunay::vector2> vertices({
//		delaunay::vector2(0, 0),
//		delaunay::vector2(1.0, 0),
//		delaunay::vector2(0.5, 1.0)
//		});
//
//	// Edges...
//	std::vector<delaunay::edge> edges({
//		delaunay::edge{0, 1},
//		delaunay::edge{1, 2},
//		delaunay::edge{2, 0}
//		});
//	
//	// Triangles...
//	std::vector<delaunay::triangle> triangles({
//		delaunay::triangle{0, 1, 2}
//		});
//
//	delaunay::tessellator t(vertices);
//
//	check(t, vertices, edges, triangles);
//}
//
//TEST_CASE("Tessellate square", "[delaunay_tessellate_square]")
//{
//	// Vertices...
//	std::vector<delaunay::vector2> vertices({
//			delaunay::vector2(0, 0),
//			delaunay::vector2(1.0, 0),
//			delaunay::vector2(1.0, 1.0),
//			delaunay::vector2(0, 1.0)
//		});
//
//	// Edges...
//	std::vector<delaunay::edge> edges({
//		delaunay::edge{0, 1},
//		delaunay::edge{1, 2},
//		delaunay::edge{2, 3},
//		delaunay::edge{3, 0},
//		delaunay::edge{1, 3}
//		});
//
//	// Triangles...
//	std::vector<delaunay::triangle> triangles({
//		delaunay::triangle{0, 1, 3},
//		delaunay::triangle{1, 2, 3}
//		});
//
//	delaunay::tessellator t(vertices);
//
//	check(t, vertices, edges, triangles);
//}
//
//TEST_CASE("Tessellate ellipse", "[delaunay_tessellate_ellipse]")
//{
//	// Vertices...
//	unsigned int segments = 16;
//	double width = 2.0 / 2.0;
//	double height = 1.0 / 2.0;
//	double twoPI = 2.0 * delaunay::pi;
//
//	std::vector<delaunay::vector2> vertices;
//
//	for (unsigned int d = 0; d < segments; ++d)
//	{
//		double angle = d * twoPI / segments;
//		vertices.push_back(delaunay::vector2(width * cos(angle), height * sin(angle)));
//	}
//
//	// Edges...
//	std::vector<delaunay::edge> edges({
//		delaunay::edge{0, 1},
//		delaunay::edge{0, 15},
//		delaunay::edge{1, 2},
//		delaunay::edge{1, 15},
//		delaunay::edge{2, 3},
//		delaunay::edge{2, 14},
//		delaunay::edge{2, 15},
//		delaunay::edge{3, 4},
//		delaunay::edge{3, 13},
//		delaunay::edge{3, 14},
//		delaunay::edge{4, 5},
//		delaunay::edge{4, 12},
//		delaunay::edge{4, 13},
//		delaunay::edge{5, 6},
//		delaunay::edge{5, 11},
//		delaunay::edge{5, 12},
//		delaunay::edge{6, 7},
//		delaunay::edge{6, 9},
//		delaunay::edge{6, 10},
//		delaunay::edge{6, 11},
//		delaunay::edge{7, 8},
//		delaunay::edge{7, 9},
//		delaunay::edge{8, 9},
//		delaunay::edge{9, 10},
//		delaunay::edge{10, 11},
//		delaunay::edge{11, 12},
//		delaunay::edge{12, 13},
//		delaunay::edge{13, 14},
//		delaunay::edge{14, 15}
//		});
//
//	// Triangles...
//	std::vector<delaunay::triangle> triangles({
//		delaunay::triangle{7, 6, 9},
//		delaunay::triangle{8, 7, 9},
//		delaunay::triangle{9, 6, 10},
//		delaunay::triangle{6, 5, 11},
//		delaunay::triangle{10, 6, 11},
//		delaunay::triangle{5, 4, 12},
//		delaunay::triangle{11, 5, 12},
//		delaunay::triangle{4, 3, 13},
//		delaunay::triangle{12, 4, 13},
//		delaunay::triangle{3, 2, 14},
//		delaunay::triangle{13, 3, 14},
//		delaunay::triangle{1, 0, 15},
//		delaunay::triangle{2, 1, 15},
//		delaunay::triangle{14, 2, 15}
//		});
//
//	delaunay::tessellator t(vertices);
//
//	check(t, vertices, edges, triangles);
//}
//
//// test constrained1
//TEST_CASE("Tessellate constrained1", "[delaunay_tessellate_constrained1]")
//{
//	// Vertices...
//	unsigned int segments = 16;
//	double width = 2.0 / 2.0;
//	double height = 1.0 / 2.0;
//	double twoPI = 2.0 * delaunay::pi;
//
//	std::vector<delaunay::vector2> vertices;
//
//	for (unsigned int d = 0; d < segments; ++d)
//	{
//		double angle = d * twoPI / segments;
//		vertices.push_back(delaunay::vector2(width * cos(angle), height * sin(angle)));
//	}
//
//	// Edges...
//	std::vector<delaunay::edge> edges({
//		delaunay::edge{0, 1},
//		delaunay::edge{0, 2},
//		delaunay::edge{0, 3},
//		delaunay::edge{0, 4},
//		delaunay::edge{0, 8},
//		delaunay::edge{0, 12},
//		delaunay::edge{0, 13},
//		delaunay::edge{0, 14},
//		delaunay::edge{0, 15},
//		delaunay::edge{1, 2},
//		delaunay::edge{2, 3},
//		delaunay::edge{3, 4},
//		delaunay::edge{4, 5},
//		delaunay::edge{4, 8},
//		delaunay::edge{5, 6},
//		delaunay::edge{5, 8},
//		delaunay::edge{6, 7},
//		delaunay::edge{6, 8},
//		delaunay::edge{7, 8},
//		delaunay::edge{8, 9},
//		delaunay::edge{8, 10},
//		delaunay::edge{8, 11},
//		delaunay::edge{8, 12},
//		delaunay::edge{9, 10},
//		delaunay::edge{10, 11},
//		delaunay::edge{11, 12},
//		delaunay::edge{12, 13},
//		delaunay::edge{13, 14},
//		delaunay::edge{14, 15}
//		});
//
//	// Triangles...
//	std::vector<delaunay::triangle> triangles({
//		delaunay::triangle{1, 0, 2},
//		delaunay::triangle{2, 0, 3},
//		delaunay::triangle{3, 0, 4},
//		delaunay::triangle{4, 0, 8},
//		delaunay::triangle{5, 4, 8},
//		delaunay::triangle{6, 5, 8},
//		delaunay::triangle{7, 6, 8},
//		delaunay::triangle{9, 8, 10},
//		delaunay::triangle{10, 8, 11},
//		delaunay::triangle{8, 0, 12},
//		delaunay::triangle{11, 8, 12},
//		delaunay::triangle{12, 0, 13},
//		delaunay::triangle{13, 0, 14},
//		delaunay::triangle{14, 0, 15}
//		});
//
//	delaunay::tessellator t(vertices);
//	t.addConstraint(delaunay::edge{ 0, segments / 2 });
//
//	check(t, vertices, edges, triangles);
//}
//
//// test constrained2
//TEST_CASE("Tessellate constrained2", "[delaunay_tessellate_constrained2]")
//{
//	// Vertices...
//	unsigned int segments = 16;
//	double width = 2.0 / 2.0;
//	double height = 1.0 / 2.0;
//	double twoPI = 2.0 * delaunay::pi;
//
//	std::vector<delaunay::vector2> vertices;
//
//	for (unsigned int d = 0; d < segments; ++d)
//	{
//		double angle = d * twoPI / segments;
//		vertices.push_back(delaunay::vector2(width * cos(angle), height * sin(angle)));
//	}
//
//	// Edges...
//	std::vector<delaunay::edge> edges({
//		delaunay::edge{0, 1},
//		delaunay::edge{0, 2},
//		delaunay::edge{0, 3},
//		delaunay::edge{0, 4},
//		delaunay::edge{0, 12},
//		delaunay::edge{0, 13},
//		delaunay::edge{0, 14},
//		delaunay::edge{0, 15},
//		delaunay::edge{1, 2},
//		delaunay::edge{2, 3},
//		delaunay::edge{3, 4},
//		delaunay::edge{4, 5},
//		delaunay::edge{4, 8},
//		delaunay::edge{4, 12},
//		delaunay::edge{5, 6},
//		delaunay::edge{5, 8},
//		delaunay::edge{6, 7},
//		delaunay::edge{6, 8},
//		delaunay::edge{7, 8},
//		delaunay::edge{8, 9},
//		delaunay::edge{8, 10},
//		delaunay::edge{8, 11},
//		delaunay::edge{8, 12},
//		delaunay::edge{9, 10},
//		delaunay::edge{10, 11},
//		delaunay::edge{11, 12},
//		delaunay::edge{12, 13},
//		delaunay::edge{13, 14},
//		delaunay::edge{14, 15}
//		});
//
//	// Triangles...
//	std::vector<delaunay::triangle> triangles({
//		delaunay::triangle{1, 0, 2},
//		delaunay::triangle{2, 0, 3},
//		delaunay::triangle{3, 0, 4},
//		delaunay::triangle{5, 4, 8},
//		delaunay::triangle{6, 5, 8},
//		delaunay::triangle{7, 6, 8},
//		delaunay::triangle{9, 8, 10},
//		delaunay::triangle{10, 8, 11},
//		delaunay::triangle{11, 8, 12},
//		delaunay::triangle{12, 0, 13},
//		delaunay::triangle{13, 0, 14},
//		delaunay::triangle{14, 0, 15},
//		delaunay::triangle{8, 4, 12},
//		delaunay::triangle{4, 0, 12}
//		});
//
//	delaunay::tessellator t(vertices);
//	t.addConstraint(delaunay::edge{ 0, segments / 2 });
//	t.addConstraint(delaunay::edge{ segments / 4, 3 * (segments / 4) });
//
//	check(t, vertices, edges, triangles);
//}
//
//// test constrained with steiner
//TEST_CASE("Tessellate constrained with steiner", "[delaunay_tessellate_constrained_with_steiner]")
//{
//	// Vertices...
//	unsigned int segments = 16;
//	double width = 2.0 / 2.0;
//	double height = 1.0 / 2.0;
//	double twoPI = 2.0 * delaunay::pi;
//
//	std::vector<delaunay::vector2> vertices;
//
//	// vertices...
//	for (unsigned int d = 0; d < segments; ++d)
//	{
//		double angle = d * twoPI / segments;
//		vertices.push_back(delaunay::vector2(width * cos(angle), height * sin(angle)));
//	}
//
//	// insert a steiner point in the centre of the ellipse...
//	vertices.push_back(delaunay::vector2(0, 0));
//	unsigned int steinerIndex = vertices.size() - 1;
//
//
//	// Edges...
//	std::vector<delaunay::edge> edges({
//		delaunay::edge{0, 1},
//		delaunay::edge{0, 2},
//		delaunay::edge{0, 14},
//		delaunay::edge{0, 15},
//		delaunay::edge{0, 16},
//		delaunay::edge{1, 2},
//		delaunay::edge{2, 3},
//		delaunay::edge{2, 16},
//		delaunay::edge{3, 4},
//		delaunay::edge{3, 16},
//		delaunay::edge{4, 5},
//		delaunay::edge{4, 16},
//		delaunay::edge{5, 6},
//		delaunay::edge{5, 16},
//		delaunay::edge{6, 7},
//		delaunay::edge{6, 8},
//		delaunay::edge{6, 16},
//		delaunay::edge{7, 8},
//		delaunay::edge{8, 9},
//		delaunay::edge{8, 10},
//		delaunay::edge{8, 16},
//		delaunay::edge{9, 10},
//		delaunay::edge{10, 11},
//		delaunay::edge{10, 16},
//		delaunay::edge{11, 12},
//		delaunay::edge{11, 16},
//		delaunay::edge{12, 13},
//		delaunay::edge{12, 16},
//		delaunay::edge{13, 14},
//		delaunay::edge{13, 16},
//		delaunay::edge{14, 15},
//		delaunay::edge{14, 16}
//		});
//
//	// Triangles...
//	std::vector<delaunay::triangle> triangles({
//		delaunay::triangle{6, 5, 16},
//		delaunay::triangle{11, 10, 16},
//		delaunay::triangle{5, 4, 16},
//		delaunay::triangle{12, 11, 16},
//		delaunay::triangle{4, 3, 16},
//		delaunay::triangle{13, 12, 16},
//		delaunay::triangle{3, 2, 16},
//		delaunay::triangle{14, 13, 16},
//		delaunay::triangle{1, 0, 2},
//		delaunay::triangle{2, 0, 16},
//		delaunay::triangle{14, 0, 15},
//		delaunay::triangle{0, 14, 16},
//		delaunay::triangle{7, 6, 8},
//		delaunay::triangle{8, 6, 16},
//		delaunay::triangle{9, 8, 10},
//		delaunay::triangle{10, 8, 16}
//		});
//
//	delaunay::tessellator t(vertices);
//	t.addConstraint(delaunay::edge{ 0, steinerIndex });
//	t.addConstraint(delaunay::edge{ steinerIndex, segments / 2 });
//
//	check(t, vertices, edges, triangles);
//}