#ifndef OBJ2CSG_HPP
#define OBJ2CSG_HPP

#include "objio.hpp"

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

	void write(const std::string& filename, const csg::mesh& m)
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

		objio::writeFile(filename, objMesh);

	}
}

#endif // OBJ2CSG_HPP
