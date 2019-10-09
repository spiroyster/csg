#include "..\include\csg.hpp"
#include "obj2csg.hpp"

int main(int argc, char** argv)
{
	// load in two cubes...
	std::shared_ptr<csg::mesh> A = obj2csg::read("cube.obj");
	std::shared_ptr<csg::mesh> B = obj2csg::read("cube.obj");

	// translate the B 
	std::for_each(B->begin(), B->end(), 
		[](csg::triangle& t) 
		{
			t.a_.x_ += 0.5;
			t.a_.y_ += 0.5;
			t.a_.z_ += 0.5;
			t.b_.x_ += 0.5;
			t.b_.y_ += 0.5;
			t.b_.z_ += 0.5;
			t.c_.x_ += 0.5;
			t.c_.y_ += 0.5;
			t.c_.z_ += 0.5;
		});

	// spit out the translated...
	//A->insert(A->end(), B->begin(), B->end());
	//obj2csg::write("mid.obj", *A);

	std::shared_ptr<csg::mesh> difference = csg::Difference(*A, *B);
	obj2csg::write("AdiffB.obj", *difference);

	return 0;
}