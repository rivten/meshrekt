// TODO(hugo) : Choice of the kernel is important ! Think about it later or test with differents kernels how differents the results are
#include <CGAL/Simple_cartesian.h>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>

#include <fstream>
#include <vector>

/*
 * TODO(hugo)
 *      * Perform one basic mesh decimation
 *      * List all possible mesh contraction according to GH
 *      * Compute quadric errors
 *      * Perform GH algorithm
 *      * Measure error between input and output meshes
 */

typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Polyhedron_3<K> mesh;

typedef K::Point_3 v3;

struct plane_from_facet
{
	mesh::Plane_3 operator()(mesh::Facet& f)
	{
		mesh::Halfedge_handle h = f.halfedge();
		return(mesh::Plane_3(h->vertex()->point(), 
 							h->next()->vertex()->point(), 
							h->opposite()->vertex()->point()));
	}
};

int main(int ArgumentCount, char** Arguments)
{
	if(ArgumentCount < 2)
	{
		std::cout << "You should give more arguments.\nTerminating." << std::endl;
		return(1);
	}
	mesh InputMesh;
	std::ifstream InputMeshFile;
	InputMeshFile.open(Arguments[1]);
	if(!InputMeshFile.good())
	{
		std::cout << "File not found" << std::endl;
		return(1);
	}
	InputMeshFile >> InputMesh;
	InputMeshFile.close();
	std::cout << "The input mesh contains " << InputMesh.size_of_vertices() << " vertices." << std::endl;

	CGAL::set_ascii_mode(std::cout);
	std::ofstream OFFFile;
	OFFFile.open("convex_hull.off");
	OFFFile << InputMesh;
	OFFFile.close();
	std::cout << "Done !" << std::endl;

	return(0);
}
