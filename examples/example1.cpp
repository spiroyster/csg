#include "../include/csg3.hpp"

#include <iostream>
#include <fstream>

int main(int argc, char** argv)
{
	
	try
	{

		

		csg::merge m(
			{
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5))
			}, 
			{
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5)),
				csg::triangle(csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5), csg::vertex(0.5, 0.5, 0.5))
			});
		
		
		
		std::vector<csg::triangle> result;
		
		csg::mesh_union u(m);
		
		// result.reserve(m.Union().count_);

		

		// m.Union([&result](unsigned int index, const csg::triangle& t) 
		// {
		// 	//csg::interpolate(csg::barycentric(t, ), )

		// 	result.push_back(t); 
		// });


		

	}
	catch (const std::exception& e)
	{
		std::cout << e.what();
	}
	
	return 0;
}