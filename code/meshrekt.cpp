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

void PushContraction(contraction_queue* Queue, contraction C)
{
	if(Queue->ContractionCount == Queue->ContractionPoolSize)
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

void ComputeCostAndVertexOfContraction(mesh* Mesh, 
		contraction* C, 
		mat4 Quadric)
{
#if 0
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
#else
	// NOTE(hugo) : We are solving the quadric equation given by
	//     Av = f
	v4 f = {};
	f.x = - GetValue(Quadric, 0, 3);
	f.y = - GetValue(Quadric, 1, 3);
	f.z = - GetValue(Quadric, 2, 3);
	f.w = 1.0f;

	mat4 A = {};
	for(u32 i = 0; i < 16; ++i)
	{
		A.Data_[i] = Quadric.Data_[i];
	}

	SetValue(&A, 0, 3, 0.0f);
	SetValue(&A, 1, 3, 0.0f);
	SetValue(&A, 2, 3, 0.0f);
	SetValue(&A, 3, 3, 1.0f);

	SetValue(&A, 3, 0, 0.0f);
	SetValue(&A, 3, 1, 0.0f);
	SetValue(&A, 3, 2, 0.0f);

	if(Det(A) == 0)
	{
		// NOTE(hugo) : The quadric is not invertible, fall back to a simple plan
		C->OptimalVertex = 0.5f * (Mesh->Vertices[C->Edge.Vertex0Index] + Mesh->Vertices[C->Edge.Vertex1Index]);
	}
	else
	{
		v4 Solution = Inverse(A) * f;
		Assert(Solution.w != 0.0f);

		C->OptimalVertex = Solution.xyz;
	}

#endif
	v4 V = ToV4(C->OptimalVertex);
	V.w = 1.0f;
	C->Cost = Dot(V, Quadric * V);
	//Assert(C->Cost >= 0.0f);
}

bool IsMeshValid(mesh* Mesh, vertex OptimalPos)
{
	bool Found = false;
	u32 FirstIndexFound = 0;
	for(u32 VertexIndex = 0; VertexIndex < Mesh->VertexCount; ++VertexIndex)
	{
		vertex V = Mesh->Vertices[VertexIndex];
		if(LengthSqr(V - OptimalPos) == 0.0f)
		{
			if(!Found)
			{
				Found = true;
				FirstIndexFound = VertexIndex;
			}
			else
			{
				return(false);
			}
		}
	}
	return(true);
}

bool IsMeshValid(mesh* Mesh)
{
	for(u32 VertexIndex = 0; VertexIndex < Mesh->VertexCount; ++VertexIndex)
	{
		if(!IsMeshValid(Mesh, Mesh->Vertices[VertexIndex]))
		{
			return(false);
		}
	}

	return(true);
}

void ContractEdge(
		mesh* Mesh, 
		edge E, 
		vertex OptimalPos, 
		contraction_queue* Queue, 
		mat4* Quadrics)
{
	vertex V0 = Mesh->Vertices[E.Vertex0Index];
	vertex V1 = Mesh->Vertices[E.Vertex1Index];
	u32 T0Index = 0;
	u32 T1Index = 0;

	if(!FindTrianglesIncidentToEdge(Mesh, E, &T0Index, &T1Index))
	{
		// TODO(hugo) : Here is a simplification : if we couldn't find the triangle 
		// (for example, we are using a edge border), then we don't perform the contraction
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

	//Assert(IsMeshValid(Mesh, OptimalPos));

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
		// NOTE(hugo) : We can compute the new point's quadric right
		// now since the triangle topology of the new mesh
		// has been updates just before
		Quadrics[E.Vertex0Index] = ComputeQuadric(Mesh, E.Vertex0Index);

		contraction_queue AddingQueue = {};
		AddingQueue.ContractionPoolSize = 1;
		AddingQueue.Contractions = AllocateArray(contraction, AddingQueue.ContractionPoolSize);
		AddingQueue.ContractionCount = 0;

		for(u32 ContractionIndex = 0; ContractionIndex < Queue->ContractionCount; ++ContractionIndex)
		{
			contraction C = Queue->Contractions[ContractionIndex];
			if(C.Edge.Vertex0Index == E.Vertex0Index 
					|| C.Edge.Vertex1Index == E.Vertex0Index)
			{
#if 0
				contraction UpdatedContraction = {};
				UpdatedContraction.Edge = C.Edge;
				mat4 Quadric = Quadrics[UpdatedContraction.Edge.Vertex0Index] 
					+ Quadrics[UpdatedContraction.Edge.Vertex1Index];
				ComputeCostAndVertexOfContraction(Mesh, 
						&UpdatedContraction, Quadric);

				// NOTE(hugo): Adding the new contraction the list of 
				// contractions to be added.
				if(AddingQueue.ContractionCount == AddingQueue.ContractionPoolSize)
				{
					AddingQueue.Contractions = ReAllocateArray(AddingQueue.Contractions, contraction, 2 * AddingQueue.ContractionPoolSize);
					AddingQueue.ContractionPoolSize *= 2;
				}
				AddingQueue.Contractions[AddingQueue.ContractionCount] = UpdatedContraction;
				AddingQueue.ContractionCount++;
#endif

				DeleteContraction(Queue, ContractionIndex);
				ContractionIndex--;
			}
		}

#if 0
		for(u32 AddingContractionIndex = 0; AddingContractionIndex < AddingQueue.ContractionCount; ++AddingContractionIndex)
		{
			contraction* C = AddingQueue.Contractions + AddingContractionIndex;
			PushContraction(Queue, *C);
		}
		AddingQueue.ContractionCount = 0;
#endif

		DeleteVertex(Mesh, E.Vertex1Index);
		Quadrics[E.Vertex1Index] = Quadrics[Mesh->VertexCount];

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
#if 0
				contraction UpdatedContraction = {};
				if(C.Edge.Vertex0Index == E.Vertex1Index)
				{
					UpdatedContraction.Edge = {E.Vertex0Index, C.Edge.Vertex1Index}; // NOTE(hugo): C.Edge.Vertex0Index was v1 which became v' which is now at the old position of v0
				}
				else if(C.Edge.Vertex1Index == E.Vertex1Index)
				{
					UpdatedContraction.Edge = {C.Edge.Vertex0Index, E.Vertex0Index};
				}
				else
				{
					InvalidCodePath;
				}

				mat4 Quadric = Quadrics[UpdatedContraction.Edge.Vertex0Index] 
					+ Quadrics[UpdatedContraction.Edge.Vertex1Index];
				ComputeCostAndVertexOfContraction(Mesh, 
						&UpdatedContraction, Quadric);

				// NOTE(hugo): Adding the new contraction the list of 
				// contractions to be added.
				if(AddingQueue.ContractionCount == AddingQueue.ContractionPoolSize)
				{
					AddingQueue.Contractions = ReAllocateArray(AddingQueue.Contractions, contraction, 2 * AddingQueue.ContractionPoolSize);
					AddingQueue.ContractionPoolSize *= 2;
				}
				AddingQueue.Contractions[AddingQueue.ContractionCount] = UpdatedContraction;
				AddingQueue.ContractionCount++;
#endif

				DeleteContraction(Queue, ContractionIndex);
				ContractionIndex--;
			}
		}

#if 0
		for(u32 AddingContractionIndex = 0; AddingContractionIndex < AddingQueue.ContractionCount; ++AddingContractionIndex)
		{
			contraction* C = AddingQueue.Contractions + AddingContractionIndex;
			PushContraction(Queue, *C);
		}
#endif
		Free(AddingQueue.Contractions);
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

// NOTE(hugo) : Not used at the moment
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


bool IsZeroMatrix(mat4 M)
{
	for(u32 i = 0; i < 16; ++i)
	{
		if(M.Data_[i] != 0.0f)
		{
			return(false);
		}
	}
	return(true);
}

void ComputeContractions(mesh* Mesh, contraction_queue* Queue, mat4* Quadrics)
{
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
				if(IsZeroMatrix(Quadrics[E.Vertex0Index]))
				{
					Quadrics[E.Vertex0Index] = ComputeQuadric(Mesh, E.Vertex0Index);
				}
				if(IsZeroMatrix(Quadrics[E.Vertex1Index]))
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
}

contraction GetBestContraction(mesh* Mesh, mat4* Quadrics)
{
	contraction Result = {};
	Result.Cost = FLT_MAX;
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;

		for(u32 i = 0; i < 3; ++i)
		{
			edge E = {T->VertexIndices[i], T->VertexIndices[(i + 1) % 3]};
			contraction C = {};
			C.Edge = E;
			if(IsZeroMatrix(Quadrics[E.Vertex0Index]))
			{
				Quadrics[E.Vertex0Index] = ComputeQuadric(Mesh, E.Vertex0Index);
			}
			if(IsZeroMatrix(Quadrics[E.Vertex1Index]))
			{
				Quadrics[E.Vertex1Index] = ComputeQuadric(Mesh, E.Vertex1Index);
			}

			mat4 Quadric = Quadrics[E.Vertex0Index] + Quadrics[E.Vertex1Index];
			ComputeCostAndVertexOfContraction(Mesh, &C, Quadric);

			if(C.Cost < Result.Cost)
			{
				Result = C;
			}
		}
	}

	return(Result);
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

void ComputeQuadrics(mesh* Mesh, mat4* Quadrics)
{
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;

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

		for(u32 i = 0; i < 3; ++i)
		{
			Quadrics[T->VertexIndices[i]] += PlaneQuadric;
		}
	}
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
	//Assert(IsMeshValid(&Mesh));

	// NOTE(hugo): Copying the mesh to compute the distance between our mesh and the input one
	mesh InputMesh = CopyMesh(&Mesh);
	printf("The input mesh contains %d vertices.\n", Mesh.VertexCount);

#if 0
	contraction_queue Queue = {};
	Queue.ContractionPoolSize = 1;
	Queue.Contractions = AllocateArray(contraction, Queue.ContractionPoolSize);
	Queue.ContractionCount = 0;
	printf("Computing contractions\n");
	mat4* Quadrics = AllocateArray(mat4, Mesh.VertexCount);
	memset(Quadrics, 0, sizeof(mat4) * Mesh.VertexCount);

	ComputeContractions(&Mesh, &Queue, Quadrics);
	printf("Contractions computed. There are %d contractions.\n", Queue.ContractionCount);

	printf("Contracting\n");
	for(u32 ContractionIndex = 0; (Queue.ContractionCount > 0); ++ContractionIndex)
	{
		contraction C = Queue.Contractions[Queue.ContractionCount - 1];
		ContractEdge(&Mesh, C.Edge, C.OptimalVertex, &Queue, Quadrics);
	}

	Free(Quadrics);
	Free(Queue.Contractions);
#else
	u32 ContractionGoal = 500;
	contraction_queue Queue = {};
	while(ContractionGoal > 0)
	{
		printf("Computing contractions\n");
		mat4* Quadrics = AllocateArray(mat4, Mesh.VertexCount);
		memset(Quadrics, 0, sizeof(mat4) * Mesh.VertexCount);
		ComputeQuadrics(&Mesh, Quadrics);

		contraction C = GetBestContraction(&Mesh, Quadrics);
		ContractEdge(&Mesh, C.Edge, C.OptimalVertex, &Queue, Quadrics);
		printf("Contraction cost was %f.\n", C.Cost);
		Free(Quadrics);

		--ContractionGoal;
		printf("Contraction goal : %i.\n", ContractionGoal);
	}
#endif

#if 0
	float DistanceBetweenMeshes = MeanDistance(&InputMesh, &Mesh);
	printf("Distance between meshes is : %f\n", DistanceBetweenMeshes);
#endif

	SaveOFF(&Mesh, "output.off");

	Free(Mesh.Vertices);
	Free(Mesh.Triangles);
	Free(InputMesh.Vertices);
	Free(InputMesh.Triangles);

	return(0);
}
