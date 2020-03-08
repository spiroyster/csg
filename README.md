# About

Single header only CSG (Constructive Solid Geometry), C++14 & STL only. 

# Usage

Simply include csg.hpp and you're off!

    #include "csg.hpp"

The csg namespace provides 



## Difference

To perform difference operation...

    std::shared_ptr<csg::mesh> csg::difference(A, B);



## Union

## Intersection

## Half space

## Threading





# Examples

Use obj

# Tests


# Bugs

Convenience function for outputting

# Future

Normals (smooth)
UV mapping
Maybe other hardware support.





The Delaunay namespace provides a Tessellator class which you can use to create the tessellation of triangles given the input vertices. 

    std::vector<delaunay::vector2> vertices({
			delaunay::vector2(0, 0),
			delaunay::vector2(1.0, 0),
			delaunay::vector2(0.5, 1.0)
		});

    delaunay::tessellator tess(vertices);
    std::list<delaunay::triangle> result = tess.getTriangles();

No _Steiner_ points are inserted so the number and ordering of vertices are preserved. The result is a list of triangles with indexes of the original vertices container used for tessellation.

![Alt text](triangle.PNG?raw=true "Triangle")

Input vertices must be unique and not duplicated, otherwise this will cause erroneous tessellations. A function is provided for convinience for this.

    delaunay::convenience::unique(vertices);

*N.B* since this function sorts these vertices before uniquing them, the ordering may not be preservered for the original vertices. 

Triangle winding is not enforced, this can be achieved by...

For examples of usage. See **examples/examples1.cpp**.

**examples/examples2.cpp** is a simple command line program that reads CSV files [vertex definitions (double, double), and edge definitions (int, int) to be used as constraints supplimenting the vertex definitions], and outputs (stdout) Wavefront OBJ valid syntax.

## Constrained tessellation

Constraints can be used to enforce edges in the resultant tessellation. First tessellate the vertices as per usual.

e.g This snippet tessellated vertices defining a ellipse which is 2 units wide, 1 unit high, with 16 segments.

    unsigned int segments = 16;
    double width = 2.0;
    double height = 1.0;

    std::vector<delaunay::vector2> vertices;
    
    for (unsigned int d = 0; d < segments; ++d)
    {
        double angle = d * (2.0 * Delaunay::pi) / segments;
        vertices.push_back(Delaunay::Vector2(width * cos(angle), height * sin(angle)));
    }

    delaunay::tessellator tess(vertices);
    std::list<delaunay::triangle> result = tess.getTriangles();

![Alt text](ellipse.PNG?raw=true "Ellipse")

Constraints are added as edges (vertex index pairs). Constraints are added individually, a single constraint at a time. If the edge defined by the two vertex indexes is already present in the tessellation, it is ignored.

    tessellator.addConstraint(delaunay::edge(0, segments / 2));

Adding a constraint updates the triangles and edges within the tessellator instance. Vertices are unaffected.

![Alt text](constrained.PNG?raw=true "Constrained")

When adding multiple constraints, edges defining the constraints should ideally not intersect each other, however since they are added individually, one constraining edge will overwrite the other.

e.g Adding another constraint to the above tessellation which crosses the original constraint will result in the second constraint given priority, effectively loosing the previous one.

    tessellator.addConstraint(std::pair<unsigned int, unsigned int>(segments / 4, 3 * (segments / 4)));

![Alt text](constrained2.PNG?raw=true "Constrained2")

For examples of usage. See **examples/examples1.cpp**.

# Bugs

Please submit bug data (if applicable) as .csv, ideally this will be added as a test to identify regression in future development. A CSV output function is provided for convenience.

    delaunay::convenience::csv(std::vector<delaunay::vector2>)

This does not create a csv file, only the syntax for a csv file, so the caller is reposonsible for writing a file.

e.g
    std::ofstream file("bugData.csv");
    file << delaunary::convenience::csv(vertices);


# Future

* Doxygenate

![Alt text](footer.PNG?raw=true "Footer")