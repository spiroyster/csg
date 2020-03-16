#include "..\include\csg.hpp"
#include "obj2csg.hpp"

#include <iostream>
#include <cctype>

int main(int argc, char** argv)
{
	try
	{
		enum command { u, n, diff };
		command operation = command::n;

		std::vector<std::string> inputFilepaths;

		// parse the arguments...
		for (int i = 1; i < argc; ++i)
		{
			std::string arg(*argv+i);

			if (arg[0] == '-')
			{
				switch (std::tolower(arg[1]))
				{
				case 'u':
					operation = command::u;
					break;
				case 'n':
					operation = command::n;
					break;
				case '-':
					operation = command::diff;
					break;
				default:
					std::runtime_error("Unknown switch " + arg);
				}
			}

			inputFilepaths.push_back(arg);
		}
			
		if (inputFilepaths.empty())
			std::runtime_error("No input files.");

		if (inputFilepaths.size() > 2)
			std::runtime_error("Too many input files.");

		std::shared_ptr<csg::mesh> result;

		if (inputFilepaths.size() == 1)
			result = obj2csg::read(inputFilepaths.front());
		else
		{
			std::shared_ptr<csg::mesh> A = obj2csg::read(inputFilepaths.front());
			std::shared_ptr<csg::mesh> B = obj2csg::read(inputFilepaths.back());
			switch (operation)
			{
			case command::u:
				{
					result = csg::Union(*A, *B);
					break;
				}
			case command::n:
				{
					result = csg::Intersection(*A, *B);
					break;
				}
			case command::diff:
				{
					result = csg::Difference(*A, *B);
					break;
				}	
			}
		}
		
		if (!result)
			std::runtime_error("Error occured with result.");

		std::cout << obj2csg::write(*result);
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what();
		std::cout << '\n';
		std::cout << "csg command line tool. https://github.com/spiroyster/csg/examples/example3.cpp\n";
		std::cout << "Usage \"> csg -[u,n,-] {fileA.obj} {fileB.obj}\"\n";
		std::cout << "e.g. > csg -- cube.obj sphere.obj > output.obj\n";
		std::cout << "	performs CSG difference (cube.obj - sphere.obj) and pipes to file output.obj.\n";
	}

	return 1;
}