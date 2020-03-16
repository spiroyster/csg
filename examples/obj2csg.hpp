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

#ifndef OBJ2CSG_HPP
#define OBJ2CSG_HPP

#include "objio.hpp"

/// Convert csg meshes to objio::meshes for output and input...

namespace obj2csg
{
	std::shared_ptr<csg::mesh> read(const std::string& filename)
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
	}

	std::string write(const csg::mesh& m)
	{
		std::list<objio::vector3> vertices;
		std::list<objio::face> faces;

		std::for_each(m.begin(), m.end(),
			[&vertices, &faces](const csg::triangle& t)
			{
				unsigned int currentSize = static_cast<unsigned int>(vertices.size());
				vertices.push_back(objio::vector3(t.a_.x_, t.a_.y_, t.a_.z_));
				vertices.push_back(objio::vector3(t.b_.x_, t.b_.y_, t.b_.z_));
				vertices.push_back(objio::vector3(t.c_.x_, t.c_.y_, t.c_.z_));
				faces.push_back(objio::face({ objio::vertex::p(currentSize), objio::vertex::p(currentSize + 1) , objio::vertex::p(currentSize + 2) }));
			});

		objio::mesh objMesh;
		objMesh.p_.insert(objMesh.p_.end(), vertices.begin(), vertices.end());
		objMesh.faces_.insert(objMesh.faces_.end(), faces.begin(), faces.end());

		return objio::write(objMesh);
	}

}

#endif // OBJ2CSG_HPP
