#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>

#include <fstream>
#include <vector>

#define Assert(x) do{if(!(x)){*(int*)0=0;}}while(0)
typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Polyhedron_3<K> mesh;

typedef K::Point_3 v3;

/*
 * TODO(hugo)
 *      * Perform one basic mesh decimation
 *      * List all possible edge contraction according to GH
 *      * Compute quadric errors for each edge contraction
 *      * Solve minimization problem for a given edge contraction
 *      * Perform GH algorithm
 *      * Measure error between input and output meshes
 *      * Compute one face planarity score
 *      * Compute planar proxies
 *      * Display planar proxies
 *      * Compute quadric error according to SAMD
 */

mesh ReadOFF(char* Filename)
{
	mesh Result;
	std::ifstream File;
	File.open(Filename);
	if(!File.good())
	{
		std::cout << "File not found" << std::endl;
		Assert(false);
	}
	File >> Result;
	File.close();

	return(Result);
}

void SaveOFF(mesh* Mesh, const char* Filename)
{
	CGAL::set_ascii_mode(std::cout);
	std::ofstream OFFFile;
	OFFFile.open(Filename);
	OFFFile << *Mesh;
	OFFFile.close();
}

int main(int ArgumentCount, char** Arguments)
{
	if(ArgumentCount < 2)
	{
		std::cout << "You should give more arguments.\nTerminating." << std::endl;
		return(1);
	}

	mesh InputMesh = ReadOFF(Arguments[1]);
	std::cout << "The input mesh contains " << InputMesh.size_of_vertices() << " vertices." << std::endl;

	SaveOFF(&InputMesh, "output.off");

	return(0);
}
