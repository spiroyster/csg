#include "..\include\csg.hpp"
#include "obj2csg.hpp"

#include <iostream>

int main(int argc, char** argv)
{
	
	try
	{
		// load in two cubes...
		std::shared_ptr<csg::mesh> A = obj2csg::read("cube.obj");
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


		std::ofstream file("A-B.obj");
		if (file)
			file << csg::convenience::obj(*csg::Difference(*A, *B));

		std::ofstream file("AnB.obj");
		if (file)
			file << csg::convenience::obj(*csg::Intersection(*A, *B));

		std::ofstream file("AuB.obj");
		if (file)
			file << csg::convenience::obj(*csg::Union(*A, *B));

	}
	catch (const std::exception& e)
	{
		std::cout << e.what();
	}
	
	return 0;
}