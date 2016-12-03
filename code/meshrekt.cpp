#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <float.h>

#define Assert(x) do{if(!(x)){*(int*)0=0;}}while(0)
#define ArrayCount(x) (sizeof(x)/sizeof((x)[0]))
#define InvalidCodePath Assert(!"InvalidCodePath");
#define InvalidDefaultCase default: {InvalidCodePath;} break

#include <cstdint>

#ifdef __unix__
#include <sys/types.h>
#endif

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef bool b32;
typedef float r32;
typedef double r64;

typedef size_t memory_index;

#include "rivten_math.h"

/*
 * TODO(hugo)
 *      * List all possible edge contraction according to GH
 *      * Compute quadric errors for each edge contraction
 *      * Solve minimization problem for a given edge contraction
 *      * Perform GH algorithm
 *      * Measure error between input and output meshes
 *      * Compute one face planarity score
 *      * Compute planar proxies
 *      * Display planar proxies
 *      * Compute SAMD quadric heuristic
 */

typedef v3 vertex;

struct edge
{
	u32 Vertex0Index;
	u32 Vertex1Index;
};

struct triangle
{
	union
	{
		struct
		{
			u32 Vertex0Index;
			u32 Vertex1Index;
			u32 Vertex2Index;
		};
		u32 VertexIndices[3];
	};
};

#define MAX_VERTEX_COUNT 1000
struct mesh
{
	vertex Vertices[MAX_VERTEX_COUNT];
	u32 VertexCount;
	triangle Triangles[MAX_VERTEX_COUNT];
	u32 TriangleCount;
};

void PushVertex(mesh* Mesh, vertex V)
{
	Assert(Mesh->VertexCount < ArrayCount(Mesh->Vertices));
	Mesh->Vertices[Mesh->VertexCount] = V;
	Mesh->VertexCount++;
}

void PushTriangle(mesh* Mesh, triangle T)
{
	Assert(Mesh->TriangleCount < ArrayCount(Mesh->Triangles));
	Mesh->Triangles[Mesh->TriangleCount] = T;
	Mesh->TriangleCount++;
}

void DeleteTriangle(mesh* Mesh, u32 TriangleIndex)
{
	Assert(TriangleIndex < Mesh->TriangleCount);
	Assert(Mesh->TriangleCount > 0);
	Mesh->Triangles[TriangleIndex] = Mesh->Triangles[Mesh->TriangleCount - 1];
	Mesh->TriangleCount--;
}

void DeleteVertex(mesh* Mesh, u32 VertexIndex)
{
	Assert(VertexIndex < Mesh->VertexCount);
	Assert(Mesh->VertexCount > 0);
	Mesh->Vertices[VertexIndex] = Mesh->Vertices[Mesh->VertexCount - 1];
	Mesh->VertexCount--;
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;
		if(T->Vertex0Index == Mesh->VertexCount)
		{
			T->Vertex0Index = VertexIndex;
		}
		else if(T->Vertex1Index == Mesh->VertexCount)
		{
			T->Vertex1Index = VertexIndex;
		}
		else if(T->Vertex2Index == Mesh->VertexCount)
		{
			T->Vertex2Index = VertexIndex;
		}
	}
}

edge RandomEdge(mesh* Mesh)
{
	u32 RandomTriangleIndex = rand() % Mesh->TriangleCount;
	triangle RandomTriangle = Mesh->Triangles[RandomTriangleIndex];
	u32 RandomVertexIndex = rand() % 3;
	edge Result = {};

	Result.Vertex0Index = RandomTriangle.VertexIndices[RandomVertexIndex];
	Result.Vertex1Index = RandomTriangle.VertexIndices[(RandomVertexIndex + 1) % 3];

	return(Result);
}

bool TriangleContainsEdge(triangle T, edge E)
{
	u32 V0Index = E.Vertex0Index;
	u32 V1Index = E.Vertex1Index;
	bool V0Found = false;
	for(u32 i = 0; i < ArrayCount(T.VertexIndices); ++i)
	{
		if(V0Index == T.VertexIndices[i])
		{
			V0Found = true;
			break;
		}
	}
	if(!V0Found)
	{
		return(false);
	}
	
	for(u32 i = 0; i < ArrayCount(T.VertexIndices); ++i)
	{
		if(V1Index == T.VertexIndices[i])
		{
			return(true);
		}
	}
	return(false);
}

void FindTrianglesIncidentToEdge(mesh* Mesh, edge E, u32* T0Index, u32* T1Index)
{
	bool Found0 = false;
	bool Found1 = false;
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle T = Mesh->Triangles[TriangleIndex];
		if(TriangleContainsEdge(T, E))
		{
			if(!Found0)
			{
				Found0 = true;
				*T0Index = TriangleIndex;
			}
			else if(!Found1)
			{
				Found1 = true;
				*T1Index = TriangleIndex;
				return;
			}
			else
			{
				InvalidCodePath;
			}
		}
	}
	InvalidCodePath;
}

void ContractEdge(mesh* Mesh, edge E)
{
	vertex V0 = Mesh->Vertices[E.Vertex0Index];
	vertex V1 = Mesh->Vertices[E.Vertex1Index];
	u32 T0Index = 0;
	u32 T1Index = 0;
	FindTrianglesIncidentToEdge(Mesh, E, &T0Index, &T1Index);

	if(T1Index == Mesh->TriangleCount - 1)
	{
		DeleteTriangle(Mesh, T0Index);
		DeleteTriangle(Mesh, T0Index);
	}
	else
	{
		DeleteTriangle(Mesh, T0Index);
		DeleteTriangle(Mesh, T1Index);
	}

	vertex VFinal = {};
	VFinal.x = 0.5f * (V0.x + V1.x);
	VFinal.y = 0.5f * (V0.y + V1.y);
	VFinal.z = 0.5f * (V0.z + V1.z);

	PushVertex(Mesh, VFinal);

	// NOTE(hugo) : Remplacing all occurences of V0 and V1 in triangles by VF (the last point added to the mesh)
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;
		if(T->Vertex0Index == E.Vertex0Index)
		{
			T->Vertex0Index = Mesh->VertexCount - 1;
		}
		else if(T->Vertex1Index == E.Vertex0Index)
		{
			T->Vertex1Index = Mesh->VertexCount - 1;
		}
		else if(T->Vertex2Index == E.Vertex0Index)
		{
			T->Vertex2Index = Mesh->VertexCount - 1;
		}

		if(T->Vertex0Index == E.Vertex1Index)
		{
			T->Vertex0Index = Mesh->VertexCount - 1;
		}
		else if(T->Vertex1Index == E.Vertex1Index)
		{
			T->Vertex1Index = Mesh->VertexCount - 1;
		}
		else if(T->Vertex2Index == E.Vertex1Index)
		{
			T->Vertex2Index = Mesh->VertexCount - 1;
		}
	}

	if(E.Vertex1Index == Mesh->VertexCount - 1)
	{
		DeleteVertex(Mesh, E.Vertex0Index);
		DeleteVertex(Mesh, E.Vertex0Index);
	}
	else
	{
		DeleteVertex(Mesh, E.Vertex0Index);
		DeleteVertex(Mesh, E.Vertex1Index);
	}
}


float Distance(vertex V, mesh* Mesh)
{
	float MinDistSqrFound = FLT_MAX;

	for(u32 VertexIndex = 0; VertexIndex < Mesh->VertexCount; ++VertexIndex)
	{
		MinDistSqrFound = Minf(MinDistSqrFound, LengthSqr(V - Mesh->Vertices[VertexIndex]));
		if(MinDistSqrFound == 0.0f)
		{
			return(0.0f);
		}
	}

	return(sqrt(MinDistSqrFound));
}

float MeanDistance(mesh* A, mesh* B)
{
	float DistFromAToB = 0.0f;
	for(u32 VAIndex = 0; VAIndex < A->VertexCount; ++VAIndex)
	{
		vertex VA = A->Vertices[VAIndex];
		DistFromAToB += Distance(VA, B);
	}
	DistFromAToB /= A->VertexCount;

	float DistFromBToA = 0.0f;
	for(u32 VBIndex = 0; VBIndex < B->VertexCount; ++VBIndex)
	{
		vertex VB = B->Vertices[VBIndex];
		DistFromBToA += Distance(VB, A);
	}
	DistFromBToA /= B->VertexCount;

	return(Maxf(DistFromAToB, DistFromBToA));
}

mesh ParseOFF(const char* Filename)
{
	FILE* File = 0;
	fopen_s(&File, Filename, "r");
	Assert(File);

	mesh Result = {};
	u32 RealVertexCount = 0;
	u32 RealTriangleCount = 0;

	char Line[512];
	Assert(fgets(Line, ArrayCount(Line), File));
	char* Token;
	Token = strtok(Line, " ");
	Assert(Token[0] == 'O' && Token[1] == 'F' && Token[2] == 'F');
	while(fgets(Line, ArrayCount(Line), File))
	{
		Token = strtok(Line, " ");
		if(Token[0] != '#')
		{
			RealVertexCount = strtol(Token, 0, 10);
			Token = strtok(0, " ");
			RealTriangleCount = strtol(Token, 0, 10);
			break;
		}
	}

	for(u32 VertexIndex = 0; VertexIndex < RealVertexCount; ++VertexIndex)
	{
		Assert(fgets(Line, ArrayCount(Line), File));
		vertex V = {};

		Token = strtok(Line, " ");
		V.x = strtof(Token, 0);
		Token = strtok(0, " ");
		V.y = strtof(Token, 0);
		Token = strtok(0, " ");
		V.z = strtof(Token, 0);

		PushVertex(&Result, V);
	}

	for(u32 TriangleIndex = 0; TriangleIndex < RealTriangleCount; ++TriangleIndex)
	{
		Assert(fgets(Line, ArrayCount(Line), File));
		triangle T = {};

		Token = strtok(Line, " ");
		Assert(strtol(Token, 0, 10) == 3);
		Token = strtok(0, " ");
		T.Vertex0Index = strtol(Token, 0, 10);
		Token = strtok(0, " ");
		T.Vertex1Index = strtol(Token, 0, 10);
		Token = strtok(0, " ");
		T.Vertex2Index = strtol(Token, 0, 10);

		PushTriangle(&Result, T);
	}

	Assert((RealVertexCount == Result.VertexCount) && (RealTriangleCount == Result.TriangleCount));

	fclose(File);
	return(Result);
}

void SaveOFF(mesh* Mesh, const char* Filename)
{
	FILE* File = 0;
	fopen_s(&File, Filename, "w");
	Assert(File);

	fprintf(File, "OFF\n#This is a OFF File generated by the program MeshRekt by Hugo Viala\n#Please do not touch it unless you know what you are doing. Thanks.\n");
	fprintf(File, "%d %d 0\n", Mesh->VertexCount, Mesh->TriangleCount);

	for(u32 VertexIndex = 0; VertexIndex < Mesh->VertexCount; ++VertexIndex)
	{
		vertex V = Mesh->Vertices[VertexIndex];
		fprintf(File, "%f %f %f\n", V.x, V.y, V.z);
	}

	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle T = Mesh->Triangles[TriangleIndex];
		fprintf(File, "3 %d %d %d\n", T.Vertex0Index, T.Vertex1Index, T.Vertex2Index);
	}

	fclose(File);

}

int main(int ArgumentCount, char** Arguments)
{
	if(ArgumentCount < 2)
	{
		printf("You should give more arguments.\nTerminating.");
		return(1);
	}
	srand(time(0));

	mesh Mesh = ParseOFF(Arguments[1]);

	// NOTE(hugo): Copying the mesh to compute the distance between our mesh and the input one
	mesh InputMesh = Mesh;
	printf("The input mesh contains %d vertices.\n", Mesh.VertexCount);

	edge E = RandomEdge(&Mesh);
	ContractEdge(&Mesh, E);

	float DistanceBetweenMeshes = MeanDistance(&InputMesh, &Mesh);
	printf("Distance between meshes is : %f\n", DistanceBetweenMeshes);

	SaveOFF(&Mesh, "output.off");

	return(0);
}
