/*
	MIT License

	Copyright (c) 2020 Cordell Barron

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

*/

#ifndef QDCSG_OBJIO
#define QDCSG_OBJIO

#include "../include/csg.hpp"

#include <fstream>
#include <sstream>

/// Read and write obj (alias wavefront) file format...

namespace objio
{
	namespace impl
	{
		static std::vector<std::string> split(const std::string& str, const std::string& seperator)
		{
			std::list<std::string> tokens;

			if (str.empty())
				return std::vector<std::string>();

			std::size_t start = 0;
			std::size_t itr = str.find(seperator, start);

			while (itr != std::string::npos)
			{
				std::string token = str.substr(start, itr - start);
				if (!token.empty())
					tokens.push_back(token);
				start = itr + seperator.size();
				itr = str.find(seperator, start);
			}
			tokens.push_back(str.substr(start));

			return std::vector<std::string>(tokens.begin(), tokens.end());
		}

		static std::string rightTrim(const std::string& str, const std::string& chars = "\t\n\v\f\r ")
		{
			std::string newStr = str;
			newStr.erase(newStr.find_last_not_of(chars) + 1);
			return newStr;
		}

		static std::string leftTrim(const std::string& str, const std::string& chars = "\t\n\v\f\r ")
		{
			std::string newStr = str;
			newStr.erase(0, newStr.find_first_not_of(chars));
			return newStr;
		}

		static std::string trim(const std::string& str, const std::string& chars = "\t\n\v\f\r ")
		{
			return leftTrim(rightTrim(str, chars), chars);
		}
	}

	struct vector2
	{
		//vector2() : x_(0), y_(0) {}
		vector2(double x, double y) : x_(x), y_(y) {}

		double x_, y_;
	};
	struct vector3
	{
		//vector3() : x_(0), y_(0), z_(0) {}
		vector3(double x, double y, double z) : x_(x), y_(y), z_(z) {}

		double x_, y_, z_;
	};
	struct vertex
	{
		//vertex() : p_(0), n_(0), uv_(0), c_(0) {}
		vertex(int p, int n, int uv, int c) : p_(p), n_(n), uv_(uv), c_(c) {}

		static vertex p(int p) { return vertex(p, -1, -1, -1); }
		static vertex pc(int p, int c) { return vertex(p, -1, -1, c); }
		static vertex pn(int p, int n) { return vertex(p, n, -1, -1); }
		static vertex pt(int p, int t) { return vertex(p, -1, t, -1); }
		static vertex ptc(int p, int t, int c) { return vertex(p, -1, t, c); }
		static vertex pnt(int p, int n, int t) { return vertex(p, n, t, -1); }
		static vertex pnc(int p, int n, int c) { return vertex(p, n, -1, c); }
		static vertex pntc(int p, int n, int t, int c) { return vertex(p, n, t, c); }

		static vertex parse(const std::string& str)
		{
			std::vector<std::string> vertexIndexes = impl::split(str, "/");
			if (vertexIndexes.size() == 1)
				return vertex::p(stoi(vertexIndexes[0]) - 1);
			else if (vertexIndexes.size() == 2)
				return str.find("//") != std::string::npos ? vertex::pn(stoi(vertexIndexes[0]) - 1, stoi(vertexIndexes[1]) - 1) : vertex::pt(stoi(vertexIndexes[0]) - 1, stoi(vertexIndexes[1]) - 1);
			else if (vertexIndexes.size() == 3)
				return vertex::pnt(stoi(vertexIndexes[0]) - 1, stoi(vertexIndexes[2]) - 1, stoi(vertexIndexes[1]) - 1);			
			throw std::runtime_error("Incorrect number of vertex indexes.");
		}

		int p_, n_, uv_, c_;
	};
	struct face
	{
		face(const std::vector<vertex>& vertices)
			:	vertices_(vertices)
		{
		}

		static face parse(const std::string& str)
		{
			std::vector<vertex> vertices;
			std::vector<std::string> faceSyntax = impl::split(impl::trim(str), " ");
			std::for_each(faceSyntax.begin(), faceSyntax.end(),
				[&vertices](const std::string& faceStr)
				{
					vertices.push_back(vertex::parse(faceStr));
				});
			
			return face(vertices);
		}

		const vertex& operator[](unsigned int i) const { return vertices_[i]; }

		std::vector<vertex> vertices_;
	};
	struct mesh
	{
		std::vector<vector3> p_;
		std::vector<vector3> n_;
		std::vector<vector2> uv_;
		std::vector<vector3> c_;
		std::vector<face> faces_;
	};

	static std::shared_ptr<mesh> read(const std::string& str)
	{
		std::istringstream syntax(str);

		std::list<vector3> points;
		std::list<vector3> normals;
		std::list<vector2> uvs;
		std::list<vector3> colours;
		std::list<face> faces;

		std::string line;

		while (std::getline(syntax, line))
		{
			// if this line has a comment.. ignore it.
			if (line.find('#') != std::string::npos)
				continue;

			// look for vt (texture coordinates)
			std::size_t vt = line.find("vt");
			if (vt != std::string::npos)
			{
				std::istringstream iss(line.substr(vt + 2));

				double x = 0, y = 0;
				iss >> std::ws >> x >> std::ws >> y;
				uvs.push_back(vector2(x, y));
				continue;
			}

			// look for vn (normals)
			std::size_t vn = line.find("vn");
			if (vn != std::string::npos)
			{
				std::istringstream iss(line.substr(vn + 2));

				double x = 0, y = 0, z = 0;
				iss >> std::ws >> x >> std::ws >> y >> std::ws >> z;
				normals.push_back(vector3(x, y, z));
				continue;
			}

			// look for v
			std::size_t v = line.find("v");
			if (v != std::string::npos)
			{
				// tokenize...
				std::vector<std::string> vertex = impl::split(impl::trim(line.substr(v + 1)), " ");
				unsigned int vertexComponentCount = static_cast<unsigned int>(vertex.size());

				// standard vertex x,y,z (w)...
				if (vertexComponentCount == 3)
					points.push_back(vector3(stod(vertex[0]), stod(vertex[1]), stod(vertex[2])));
				else if (vertexComponentCount == 4)
					points.push_back(vector3(stod(vertex[0]), stod(vertex[1]), stod(vertex[2])));
				else if (vertexComponentCount == 6)
				{
					points.push_back(vector3(stod(vertex[0]), stod(vertex[1]), stod(vertex[2])));
					colours.push_back(vector3(stod(vertex[0]), stod(vertex[1]), stod(vertex[2])));
				}
				else
					throw std::runtime_error("Number of components in vertex \"v\" not supported.");

				continue;
			}

			// look for f
			std::size_t f = line.find("f");
			if (f != std::string::npos)
			{
				faces.push_back(face::parse(impl::trim(line.substr(f + 1))));
				continue;
			}
		}

		std::shared_ptr<mesh> result(new mesh);
		result->p_.insert(result->p_.end(),points.begin(), points.end());
		result->n_.insert(result->n_.end(), normals.begin(), normals.end());
		result->c_.insert(result->c_.end(), colours.begin(), colours.end());
		result->uv_.insert(result->uv_.end(), uvs.begin(), uvs.end());
		result->faces_.insert(result->faces_.end(), faces.begin(), faces.end());
		
		return result;
	}

	static std::shared_ptr<mesh> readFile(const std::string& filename)
	{
		std::ifstream file(filename);
		if (!file)
			throw std::runtime_error("Unable to read file");

		std::string str;

		file.seekg(0, std::ios::end);
		str.reserve(static_cast<size_t>(file.tellg()));
		file.seekg(0, std::ios::beg);

		str.assign((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());

		return read(str);
	}

	static std::string write(const mesh& mesh)
	{
		std::ostringstream oss;

		oss << "# obj file. https://github.com/spiroyster/csg\n";

		oss << "\n# points...\n";
		if (mesh.c_.empty())
		{
			std::for_each(mesh.p_.begin(), mesh.p_.end(),
				[&oss](const vector3& v)
				{
					oss << "v " << v.x_ << " " << v.y_ << " " << v.z_ << '\n';
				});
		}
		else if (mesh.p_.size() == mesh.c_.size())
		{
			for (unsigned int i = 0; i < mesh.p_.size(); ++i)
			{
				const vector3& v = mesh.p_[i];
				const vector3& c = mesh.c_[i];
				oss << "v " << v.x_ << " " << v.y_ << " " << v.z_ << " " << c.x_ << " " << c.y_ << " " << c.z_ << '\n';
			}
		}

		if (!mesh.n_.empty())
		{
			oss << "\n# normals...\n";
			std::for_each(mesh.n_.begin(), mesh.n_.end(),
				[&oss](const vector3& v)
				{
					oss << "vn " << v.x_ << " " << v.y_ << " " << v.z_ << '\n';
				});
		}

		if (!mesh.uv_.empty())
		{
			oss << "\n# texture coordinates...\n";
			std::for_each(mesh.uv_.begin(), mesh.uv_.end(),
				[&oss](const vector2& v)
				{
					oss << "vn " << v.x_ << " " << v.y_ << '\n';
				});
		}

		oss << "\n# faces...\n";
		std::for_each(mesh.faces_.begin(), mesh.faces_.end(), 
			[&oss](const face& f) 
			{
				oss << "f ";
				std::for_each(f.vertices_.begin(), f.vertices_.end(),
					[&oss](const vertex& v) 
					{
						oss << v.p_ + 1;
						if (v.n_ != -1 && v.uv_ != -1)
							oss << "/" << v.uv_ + 1 << "/" << v.n_ + 1;
						else if (v.n_ != -1)
							oss << "//" << v.n_ + 1;
						oss << " ";
					});
				oss << '\n';
			});


		return oss.str();
	}

	static void writeFile(const std::string& filename, const mesh& mesh)
	{
		std::ofstream file(filename);
		if (file)
			file << write(mesh);
		else
			std::runtime_error("Unable to save obj file.");
	}

}

#endif // QDCSG_OBJIO