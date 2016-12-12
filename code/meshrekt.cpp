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

#define ReAllocateArray(Buffer, Type, Size) (Type *)ReAllocate_(Buffer, Size * sizeof(Type))
#define AllocateArray(Type, Size) (Type *)Allocate_(Size * sizeof(Type))
#define Free(Buffer) free(Buffer)
#define CopyArray(Dest, Source, Type, Size) memcpy(Dest, Source, Size * sizeof(Type))

void* Allocate_(size_t Size)
{
	void* Result = malloc(Size);
	Assert(Result);

	return(Result);
}

void* ReAllocate_(void* Buffer, size_t Size)
{
	void* Result = realloc(Buffer, Size);
	Assert(Result);

	return(Result);
}

/*
 * TODO(hugo)
 *      * Compute one face planarity score
 *      * Compute planar proxies
 *      * Display planar proxies
 *      * Compute SAMD quadric heuristic
 *
 *      BUG:
 *      * there seems to be problems such as edge flipping or connectivity. Investigate
 *      * handle bigger meshes (for now, memory allocation issues when the vertex count is too big)
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

struct contraction
{
	edge Edge;
	float Cost;
	vertex OptimalVertex;
};

struct contraction_queue
{
	contraction* Contractions;
	u32 ContractionCount;
	u32 ContractionPoolSize;
};

void DeleteContraction(contraction_queue* Queue, u32 DeletedIndex)
{
	for(u32 ContractionIndex = DeletedIndex; ContractionIndex < Queue->ContractionCount - 1; ++ContractionIndex)
	{
		Queue->Contractions[ContractionIndex] = Queue->Contractions[ContractionIndex + 1];
	}
	Queue->ContractionCount--;
}

struct mesh
{
	vertex* Vertices;
	u32 VertexCount;
	u32 VertexPoolSize;

	triangle* Triangles;
	u32 TriangleCount;
	u32 TrianglePoolSize;
};

mesh CopyMesh(mesh* Mesh)
{
	mesh Result = {};

	Result.Vertices = AllocateArray(vertex, Mesh->VertexCount);
	Result.VertexCount = Mesh->VertexCount;
	Result.VertexPoolSize = Mesh->VertexPoolSize;
	CopyArray(Result.Vertices, Mesh->Vertices, vertex, Result.VertexCount);

	Result.Triangles = AllocateArray(triangle, Mesh->TriangleCount);
	Result.TriangleCount = Mesh->TriangleCount;
	Result.TrianglePoolSize = Mesh->TrianglePoolSize;
	CopyArray(Result.Triangles, Mesh->Triangles, triangle, Result.TriangleCount);

	return(Result);
}

void PushVertex(mesh* Mesh, vertex V)
{
	if(Mesh->VertexCount == Mesh->VertexPoolSize)
	{
		Mesh->Vertices = ReAllocateArray(Mesh->Vertices, vertex, 2 * Mesh->VertexPoolSize);
		Mesh->VertexPoolSize *= 2;
	}
	Mesh->Vertices[Mesh->VertexCount] = V;
	Mesh->VertexCount++;
}

void PushTriangle(mesh* Mesh, triangle T)
{
	if(Mesh->TriangleCount == Mesh->TrianglePoolSize)
	{
		Mesh->Triangles = ReAllocateArray(Mesh->Triangles, triangle, 2 * Mesh->TrianglePoolSize);
		Mesh->TrianglePoolSize *= 2;
	}
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

bool FindTrianglesIncidentToEdge(mesh* Mesh, edge E, u32* T0Index, u32* T1Index)
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
				return(true);
			}
			else
			{
				InvalidCodePath;
			}
		}
	}
	return(false);
}

void ContractEdge(mesh* Mesh, edge E, vertex OptimalPos, contraction_queue* Queue)
{
	vertex V0 = Mesh->Vertices[E.Vertex0Index];
	vertex V1 = Mesh->Vertices[E.Vertex1Index];
	u32 T0Index = 0;
	u32 T1Index = 0;

	if(!FindTrianglesIncidentToEdge(Mesh, E, &T0Index, &T1Index))
	{
		// TODO(hugo) : Here is a simplification : if we couldn't find the triangle (for example, we are using a edge border), then we don't perform the contraction
		Queue->ContractionCount--;
		return;
	}

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

	PushVertex(Mesh, OptimalPos);

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
		InvalidCodePath; // TODO(hugo) : Handle this. We need to delete the proper contraction in the queue
		DeleteVertex(Mesh, E.Vertex0Index);
		DeleteVertex(Mesh, E.Vertex0Index);
	}
	else
	{
		DeleteVertex(Mesh, E.Vertex0Index);
		for(u32 ContractionIndex = 0; ContractionIndex < Queue->ContractionCount; ++ContractionIndex)
		{
			contraction C = Queue->Contractions[ContractionIndex];
			if(C.Edge.Vertex0Index == E.Vertex0Index || C.Edge.Vertex1Index == E.Vertex0Index)
			{
				DeleteContraction(Queue, ContractionIndex);
				ContractionIndex--;
			}
		}

		DeleteVertex(Mesh, E.Vertex1Index);

		// NOTE(hugo) : need to update the contraction queue
		// The previous vertex at index Mesh->VertexCount (last one) is now at the place of E.Vertex1Index
		for(u32 ContractionIndex = 0; ContractionIndex < Queue->ContractionCount; ++ContractionIndex)
		{
			contraction C = Queue->Contractions[ContractionIndex];
			if(C.Edge.Vertex0Index == Mesh->VertexCount)
			{
				C.Edge.Vertex0Index = E.Vertex1Index;
			}
			else if(C.Edge.Vertex1Index == Mesh->VertexCount)
			{
				C.Edge.Vertex1Index = E.Vertex1Index;
			}
			else if(C.Edge.Vertex0Index == E.Vertex1Index || C.Edge.Vertex1Index == E.Vertex1Index)
			{
				DeleteContraction(Queue, ContractionIndex);
				ContractionIndex--;
			}
		}
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

void PushContraction(contraction_queue* Queue, contraction C)
{
	if(Queue->ContractionCount + 1 == Queue->ContractionPoolSize)
	{
		Queue->Contractions = ReAllocateArray(Queue->Contractions, contraction, 2 * Queue->ContractionPoolSize);
		Queue->ContractionPoolSize *= 2;
	}

	if(Queue->ContractionCount == 0)
	{
		Queue->Contractions[Queue->ContractionCount] = C;
		Queue->ContractionCount++;
	}
	else if(C.Cost <= Queue->Contractions[Queue->ContractionCount - 1].Cost)
	{
		Queue->Contractions[Queue->ContractionCount] = C;
		Queue->ContractionCount++;
	}
	else
	{
		for(u32 ContractionIndex = Queue->ContractionCount - 1; ContractionIndex >= 1; --ContractionIndex)
		{
			Queue->Contractions[ContractionIndex + 1] = Queue->Contractions[ContractionIndex];
			if(C.Cost <= Queue->Contractions[ContractionIndex - 1].Cost)
			{
				Queue->Contractions[ContractionIndex] = C;
				Queue->ContractionCount++;
				return;
			}
		}
		Queue->Contractions[1] = Queue->Contractions[0];
		Queue->Contractions[0] = C;
		Queue->ContractionCount++;
	}
}

bool IsEdgeInMesh(mesh* Mesh, u32 Vertex0Index, u32 Vertex1Index)
{
	edge E = {Vertex0Index, Vertex1Index};
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		if(TriangleContainsEdge(Mesh->Triangles[TriangleIndex], E))
		{
			return(true);
		}
	}
	return(false);
}

mat4 ComputeQuadric(mesh* Mesh, u32 VertexIndex)
{
	mat4 Quadric = {};
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;
		if((T->Vertex0Index == VertexIndex) || (T->Vertex1Index == VertexIndex) || (T->Vertex2Index == VertexIndex))
		{
			// NOTE(hugo) : Computing the plane equation of the triangle : ax + by + cz + d = 0 
			// with a*a + b*b + c*c = 1
			vertex A = Mesh->Vertices[T->VertexIndices[0]];
			vertex B = Mesh->Vertices[T->VertexIndices[1]];
			vertex C = Mesh->Vertices[T->VertexIndices[2]];
			vertex N = Normalized(Cross(B - A, C - A));
			float d = - Dot(N, A);
			mat4 PlaneQuadric = {};
			SetValue(&PlaneQuadric, 0, 0, N.x * N.x);
			SetValue(&PlaneQuadric, 0, 1, N.x * N.y);
			SetValue(&PlaneQuadric, 0, 2, N.x * N.z);
			SetValue(&PlaneQuadric, 0, 3, N.x * d);

			SetValue(&PlaneQuadric, 1, 0, N.y * N.x);
			SetValue(&PlaneQuadric, 1, 1, N.y * N.y);
			SetValue(&PlaneQuadric, 1, 2, N.y * N.z);
			SetValue(&PlaneQuadric, 1, 3, N.y * d);

			SetValue(&PlaneQuadric, 2, 0, N.z * N.x);
			SetValue(&PlaneQuadric, 2, 1, N.z * N.y);
			SetValue(&PlaneQuadric, 2, 2, N.z * N.z);
			SetValue(&PlaneQuadric, 2, 3, N.z * d);

			SetValue(&PlaneQuadric, 3, 0, d * N.x);
			SetValue(&PlaneQuadric, 3, 1, d * N.y);
			SetValue(&PlaneQuadric, 3, 2, d * N.z);
			SetValue(&PlaneQuadric, 3, 3, d * d);

			Quadric += PlaneQuadric;
		}
	}

	return(Quadric);
}

void ComputeCostAndVertexOfContraction(mesh* Mesh, contraction* C, mat4 Quadric)
{
	if(Det(Quadric) == 0)
	{
		// NOTE(hugo) : The quadric is not invertible, fall back to a simple plan
		C->OptimalVertex = 0.5f * (Mesh->Vertices[C->Edge.Vertex0Index] + Mesh->Vertices[C->Edge.Vertex1Index]);
	}
	else
	{
		v4 Solution = Inverse(Quadric) * V4(0.0f, 0.0f, 0.0f, 1.0f);
		C->OptimalVertex = {Solution.x, Solution.y, Solution.z};
		if(Solution.w != 0.0f)
		{
			C->OptimalVertex /= Solution.w;
		}
		else
		{
			// TODO(hugo) : Find out why I fall into this case sometimes
			C->OptimalVertex = 0.5f * (Mesh->Vertices[C->Edge.Vertex0Index] + Mesh->Vertices[C->Edge.Vertex1Index]);
		}
	}
	v4 V = ToV4(C->OptimalVertex);
	V.w = 0.0f;
	C->Cost = Dot(V, Quadric * V);
}

bool IsEdgeInQueue(edge E, contraction_queue* Queue)
{
	for(u32 ContractionIndex = 0; ContractionIndex < Queue->ContractionCount; ++ContractionIndex)
	{
		contraction* C = Queue->Contractions + ContractionIndex;
		u32 ContVertex0Index = C->Edge.Vertex0Index;
		u32 ContVertex1Index = C->Edge.Vertex1Index;

		if((ContVertex0Index == E.Vertex0Index && ContVertex1Index == E.Vertex1Index)
				|| (ContVertex0Index == E.Vertex1Index && ContVertex1Index == E.Vertex0Index))
		{
			return(true);
		}
	}

	return(false);
}

void ComputeContractions(mesh* Mesh, contraction_queue* Queue)
{
	mat4* Quadrics = AllocateArray(mat4, Mesh->VertexCount);
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;

		for(u32 i = 0; i < 3; ++i)
		{
			edge E = {T->VertexIndices[i], T->VertexIndices[(i + 1) % 3]};
			if(!IsEdgeInQueue(E, Queue))
			{
				contraction C = {};
				C.Edge = E;
				if(!(Quadrics + E.Vertex0Index))
				{
					Quadrics[E.Vertex0Index] = ComputeQuadric(Mesh, E.Vertex0Index);
				}
				if(!(Quadrics + E.Vertex1Index))
				{
					Quadrics[E.Vertex1Index] = ComputeQuadric(Mesh, E.Vertex1Index);
				}

				mat4 Quadric = Quadrics[E.Vertex0Index] + Quadrics[E.Vertex1Index];
				ComputeCostAndVertexOfContraction(Mesh, &C, Quadric);

				// NOTE(hugo) : inserting the contraction in the list (the list is in decreasing order)
				PushContraction(Queue, C);
			}
		}
	}
	Free(Quadrics);
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

	Result.Vertices = AllocateArray(vertex, RealVertexCount);
	Result.VertexPoolSize = RealVertexCount;

	Result.Triangles = AllocateArray(triangle, RealTriangleCount);
	Result.TrianglePoolSize = RealTriangleCount;

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
	mesh InputMesh = CopyMesh(&Mesh);
	printf("The input mesh contains %d vertices.\n", Mesh.VertexCount);

	contraction_queue Queue = {};
	Queue.ContractionPoolSize = 1;
	Queue.Contractions = AllocateArray(contraction, Queue.ContractionPoolSize);
	Queue.ContractionCount = 0;
	printf("Computing contractions\n");
	ComputeContractions(&Mesh, &Queue);

	//u32 ContractionGoal = 1000;
	printf("Contracting\n");
	for(u32 ContractionIndex = 0; Queue.ContractionCount > 0; ++ContractionIndex)
	{
		contraction C = Queue.Contractions[Queue.ContractionCount - 1];
		// TODO(hugo) : Here the contraction deletes a lot of other possible contractions.
		// When we contract an edge, this creates numerous other edges that could be added in the queue to be 
		// processed at a further time.
		ContractEdge(&Mesh, C.Edge, C.OptimalVertex, &Queue);
	}

#if 0
	float DistanceBetweenMeshes = MeanDistance(&InputMesh, &Mesh);
	printf("Distance between meshes is : %f\n", DistanceBetweenMeshes);
#endif

	SaveOFF(&Mesh, "output.off");

	Free(Queue.Contractions);
	Free(Mesh.Vertices);
	Free(Mesh.Triangles);

	return(0);
}
