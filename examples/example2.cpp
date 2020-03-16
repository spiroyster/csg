#include "..\include\csg.hpp"
#include "obj2csg.hpp"

#include <iostream>

int main(int argc, char** argv)
{

	try
	{
		// load in two cubes...
		std::shared_ptr<csg::mesh> A = obj2csg::read("teapot.obj");
		std::shared_ptr<csg::mesh> B = obj2csg::read("sphere.obj");

		// translate the B 
		std::for_each(B->begin(), B->end(),
			[](csg::triangle& t)
			{
				double scalar = 0.5;
				t.a_.x_ *= scalar; t.a_.y_ *= scalar; t.a_.z_ *= scalar;
				t.b_.x_ *= scalar; t.b_.y_ *= scalar; t.b_.z_ *= scalar;
				t.c_.x_ *= scalar; t.c_.y_ *= scalar; t.c_.z_ *= scalar;

				t.a_.x_ += 0.5; t.a_.y_ += 0.5; t.a_.z_ += 0.5;
				t.b_.x_ += 0.5; t.b_.y_ += 0.5; t.b_.z_ += 0.5;
				t.c_.x_ += 0.5; t.c_.y_ += 0.5; t.c_.z_ += 0.5;
			});

		obj2csg::write("A-B.obj", *csg::Difference(*A, *B));
		obj2csg::write("AuB.obj", *csg::Union(*A, *B));
		obj2csg::write("AnB.obj", *csg::Intersection(*A, *B));


	}
	catch (const std::exception& e)
	{
		std::cout << e.what();
	}

	return 0;
}