#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "..\include\csg.hpp"
#include "..\examples\obj2csg.hpp"

//namespace
//{
//	class check
//	{
//	public:
//
//		static void Difference(const std::string& objA, const std::string& objB, const std::string& objExpected)
//		{
//			check c(objA, objB, objExpected);
//			std::shared_ptr<csg::mesh> result = csg::Difference(*c.a_, *c.b_);
//			c.validate(*result);
//		}
//		static void Union(const std::string& objA, const std::string& objB, const std::string& objExpected)
//		{
//			check c(objA, objB, objExpected);
//			std::shared_ptr<csg::mesh> result = csg::Union(*c.a_, *c.b_);
//			c.validate(*result);
//		}
//		static void Intersection(const std::string& objA, const std::string& objB, const std::string& objExpected)
//		{
//			check c(objA, objB, objExpected);
//			std::shared_ptr<csg::mesh> result = csg::Intersection(*c.a_, *c.b_);
//			c.validate(*result);
//		}
//
//	private:
//
//		check(const std::string& objA, const std::string& objB, const std::string& objExpected)
//		{
//			// load in the meshes...
//			a_ = obj2csg::read(objA);
//			b_ = obj2csg::read(objB);
//			expected_ = obj2csg::read(objExpected);
//		}
//
//		void validate(const csg::mesh& testMesh)
//		{
//			// check the triangles are the same...
//			REQUIRE(testMesh.size() == expected_->size());
//
//			// check the vertices...
//			std::for_each(expected_->begin(), expected_->end(), 
//				[](const csg::vertex& v) 
//				{
//
//				
//				});
//
//		
//		}
//
//		std::shared_ptr<csg::mesh> a_;
//		std::shared_ptr<csg::mesh> b_;
//		std::shared_ptr<csg::mesh> expected_;
//	};
//
//
//}

//TEST_CASE("CSG offsetCube", "[csg_offsetCube_difference]") { check::Difference("cube.obj", "cubeOffset.obj", "cubeOffsetDifference.obj"); }
