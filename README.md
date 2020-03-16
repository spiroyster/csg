# About

Single header only CSG (Constructive Solid Geometry), C++14 & STL only. 

# Usage

Simply include csg.hpp and you're off!

    #include "csg.hpp"

The csg namespace contains everything required to perform the CSG functions. CSG functions are performed on 3D geometry only, and the input meshes are required to be closed manifolds. The input meshes are not validated so it is up to the caller to ensure this.

The CSG routine calculates retessellates intersecting triangles of the two input meshes and then uses a BSP tree constructed for both meshes to check which retessellated triangles are inside or outside the other mesh. If the meshes are not closed manifolds, it can become UB when deducing if triangles are inside or outside, although in some cases it would still work... YMMV. 

## Difference

![Alt text](difference.PNG?raw=true "Difference")

    std::shared_ptr<csg::mesh> csg::difference(A, B);

The resultant mesh will have all triangles from mesh A that outside mesh B, and flipped triangles from mesh B which are inside mesh A.

## Union

![Alt text](union.PNG?raw=true "Union")

## Intersection

![Alt text](intersection.PNG?raw=true "Intersection")

The resultant mesh will have all triangles from mesh A that outside mesh B, and flipped triangles from mesh B which are inside mesh A.


# Output

The resulting mesh(es) will be closed manfiolds. In some cases, there can be more than one resultant mesh...

![Alt text](difference.PNG?raw=true "Difference")

These are not reduced to unique manifolds, rather all vertices and triangles would be in the same mesh result container.

In order to combat FP precision, a user defined tolerance can be passed to the routines (default is 0.01).

    csg::Difference(A, B, 0.02);

Alternatively a static value can be set... 

    csg::defaultEpsilon = 0.01;

although the former is recommended as the latter will only be valid is the translation unit that uses this file.

The tessellation routine is a constrained delaunay and this can vary depending on the input vertices. Missing triangles can be mitigated by creating a larger super triangle which can be set...

    csg::superTriangleSize = 20000;

Multi-threading is supported through OpenMP where applicable and so this needs to be enabled when compiling with csg.hpp. To disable OpenMP support for csg.hpp even if enabled for compilation, define the preprocessor directive CSG_HPP_DISABLE_OPENMP before including csg.hpp.

    #define CSG_HPP_DISABLE_OPENMP
    #include "csg.hpp"

Enjoy!

# Examples

Example1 outputs the above images...

Example2 shows usage on a slightly more complex mesh...

Example3 is a small program to perform the CSG routines on obj files from the command line...

# Bugs

Please submit bug data (if applicable) as .obj (Alias wavefront) files, ideally this will be added as a test to identify regression in future development. An obj output function is provided for convenience. Ideally two obj files should be submitted, one for each input mesh (e.g. A.obj and B.obj).

    csg::convenience::obj(csg::mesh)

This does not create an obj file, only the syntax for an obj file, so the caller is reposonsible for writing a file.

    std::ofstream file("bugData.obj");
    file << csg::convenience::obj(mesh);


# Future

* Normals (smooth)
* UV mapping
* Maybe other hardware support.
* More unit tests
* Doxygenate

