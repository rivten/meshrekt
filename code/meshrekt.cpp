#include <stdio.h>
#include <string.h>
#include <time.h>
#include <float.h>

#include "rivten.h"
#include "rivten_math.h"

/*
 * TODO(hugo)
 *      * Compute one face planarity score
 *      * Compute planar proxies
 *      * Display planar proxies
 *      * Compute SAMD quadric heuristic
 *
 *      BUG:
 *      * there seems to be problems such as edge flipping or connectivity. Investigate
 */

#include "mesh.cpp"
#include "mesh_parser.cpp" 
#include "decimation.cpp"

int main(int ArgumentCount, char** Arguments)
{
	if(ArgumentCount < 2)
	{
		printf("You should give more arguments.\nTerminating.");
		return(1);
	}

	mesh Mesh = ParseOFF(Arguments[1]);
	PreProcessMesh(&Mesh);

	// NOTE(hugo): Copying the mesh to compute the distance between our mesh and the input one
	mesh InputMesh = CopyMesh(&Mesh);
	printf("The input mesh contains %d vertices.\n", Mesh.VertexCount);

	// NOTE(hugo) : +1 because we push the new vertex before deletex the others
	triangle_index_list* ReverseTriangleIndices = AllocateArray(triangle_index_list, Mesh.VertexCount + 1);	
	memset(ReverseTriangleIndices, 0, sizeof(triangle_index_list) * (Mesh.VertexCount + 1));
	ComputeReverseTriangleIndices(&Mesh, ReverseTriangleIndices);

	triangles_properties Properties = {};
	ComputeProperties(&Mesh, &Properties, ReverseTriangleIndices);
	u32 ContractionGoal = 5000;
	//contraction_queue Queue = CreateQueue();
	contraction_queue Queue = {};
	//InitQueue(&Mesh, &Queue, ReverseTriangleIndices, Properties.InnerQuadrics);

	Assert(AreTrianglesValid(&Mesh));

	while(ContractionGoal > 0)
	{
		printf("Computing contractions\n");

		contraction C = GetBestContraction(&Mesh, Properties.InnerQuadrics, 
				ReverseTriangleIndices, &Queue);
		ContractEdge(&Mesh, C.Edge, C.OptimalVertex, 
				&Queue, 
				&Properties, ReverseTriangleIndices,
				Properties.Proxies, Properties.ProxyCount, Properties.Lambda);
		printf("Contraction cost was %f.\n", C.Cost);

		--ContractionGoal;
		printf("Contraction goal : %i.\n", ContractionGoal);
	}

	Free(ReverseTriangleIndices);

#if 1
	float DistanceBetweenMeshes = MeanDistance(&InputMesh, &Mesh);
	printf("Distance between meshes is : %f\n", DistanceBetweenMeshes);
#endif

	SaveOFF(&Mesh, "output.off");

	FreeProperties(&Properties);
	Free(Mesh.Vertices);
	Free(Mesh.Triangles);
	Free(InputMesh.Vertices);
	Free(InputMesh.Triangles);

	return(0);
}
