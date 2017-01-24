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

struct triangle_index_list
{
	u32 TriangleCount;
	// TODO(hugo): Make this variable
	u32 TriangleIndices[40];
};

void ComputeReverseTriangleIndices(mesh* Mesh, 
		triangle_index_list* ReverseTriangleIndices)
{
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;
		for(u32 i = 0; i < 3; ++i)
		{
			u32 VertexIndex = T->VertexIndices[i];
			u32 TriangleCount = ReverseTriangleIndices[VertexIndex].TriangleCount;
			Assert(TriangleCount < 40);
			ReverseTriangleIndices[VertexIndex].TriangleIndices[TriangleCount] = TriangleIndex;
			++ReverseTriangleIndices[VertexIndex].TriangleCount;
		}
	}
}

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

mat4 ComputeQuadric(u32 VertexIndex, mat4* Quadrics, triangle_index_list* ReverseTriangleIndices)
{
	mat4 Result = {};
	triangle_index_list* TriangleList = ReverseTriangleIndices + VertexIndex;
	for(u32 TriangleIndexIndex = 0; TriangleIndexIndex < TriangleList->TriangleCount; ++TriangleIndexIndex)
	{
		u32 TriangleIndex = TriangleList->TriangleIndices[TriangleIndexIndex];
		Result += Quadrics[TriangleIndex];
	}

	return(Result);
}

void ComputeCostAndVertexOfContraction(mesh* Mesh, 
		contraction* C, 
		mat4 Quadric)
{
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

	v4 V = ToV4(C->OptimalVertex);
	V.w = 1.0f;
	C->Cost = Dot(V, Quadric * V);
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

bool IsBoundary(edge E,
		triangle_index_list* ReverseTriangleIndices,
		u32* T0Index = 0, u32* T1Index = 0)
{
	u32 Vertex0Index = E.Vertex0Index;
	u32 Vertex1Index = E.Vertex1Index;
	bool FoundFirstTriangle = false;
	bool IsBoundary = true;
	for(u32 Triangle0Index = 0; (IsBoundary) && (Triangle0Index < ReverseTriangleIndices[Vertex0Index].TriangleCount); ++Triangle0Index)
	{
		u32 T0 = ReverseTriangleIndices[Vertex0Index].TriangleIndices[Triangle0Index];
		for(u32 Triangle1Index = 0; Triangle1Index < ReverseTriangleIndices[Vertex1Index].TriangleCount; ++Triangle1Index)
		{
			u32 T1 = ReverseTriangleIndices[Vertex1Index].TriangleIndices[Triangle1Index];
			if(T0 == T1)
			{
				if(!FoundFirstTriangle)
				{
					FoundFirstTriangle = true;
					if(T0Index)
					{
						*T0Index = T0;
					}
				}
				else
				{
					IsBoundary = false;
					if(T1Index)
					{
						*T1Index = T1;
					}
				}
				break;
			}
		}
	}
	Assert(FoundFirstTriangle);
	return(IsBoundary);
}

// TODO(hugo) : Can be improved because we know
// the vertices of the deleted triangle via the mesh
// TODO(hugo) : This code is wrooooong
#if 0
void UpdateReverseTriangleListAfterDeletion(
		triangle_index_list* ReverseTriangleIndices, 
		u32 DeletedIndex, 
		u32 VertexCount,
		u32 TriangleCount)
{
	for(u32 VertexIndex = 0; VertexIndex < VertexCount; ++VertexIndex)
	{
		triangle_index_list* TriangleList = ReverseTriangleIndices + VertexIndex;
		for(u32 TriangleIndexIndex = 0; TriangleIndexIndex < TriangleList->TriangleCount; ++TriangleIndexIndex)
		{
			u32 TriangleIndex = TriangleList->TriangleIndices[TriangleIndexIndex];
			if(TriangleIndex == DeletedIndex)
			{
				--TriangleList->TriangleCount;
				TriangleList->TriangleIndices[TriangleIndexIndex] = TriangleList->TriangleIndices[TriangleList->TriangleCount];
				--TriangleIndexIndex;
			}
			else if(TriangleIndex == TriangleCount)
			{
				// NOTE(hugo) : If a vertex points to the last triangle
				// this triangle is now at the place of the deleted triangle
				TriangleList->TriangleIndices[TriangleIndexIndex] = DeletedIndex;
			}
		}
	}
}
#endif

mat4 GetQuadricOfTriangle(mesh* Mesh, triangle* T)
{
	vertex A = Mesh->Vertices[T->VertexIndices[0]];
	vertex B = Mesh->Vertices[T->VertexIndices[1]];
	vertex C = Mesh->Vertices[T->VertexIndices[2]];
	vertex N = Normalized(Cross(B - A, C - A));
	float d = - Dot(N, A);
	mat4 Result = {};
	SetValue(&Result, 0, 0, N.x * N.x);
	SetValue(&Result, 0, 1, N.x * N.y);
	SetValue(&Result, 0, 2, N.x * N.z);
	SetValue(&Result, 0, 3, N.x * d);

	SetValue(&Result, 1, 0, N.y * N.x);
	SetValue(&Result, 1, 1, N.y * N.y);
	SetValue(&Result, 1, 2, N.y * N.z);
	SetValue(&Result, 1, 3, N.y * d);

	SetValue(&Result, 2, 0, N.z * N.x);
	SetValue(&Result, 2, 1, N.z * N.y);
	SetValue(&Result, 2, 2, N.z * N.z);
	SetValue(&Result, 2, 3, N.z * d);

	SetValue(&Result, 3, 0, d * N.x);
	SetValue(&Result, 3, 1, d * N.y);
	SetValue(&Result, 3, 2, d * N.z);
	SetValue(&Result, 3, 3, d * d);

	return(Result);
}

bool IsTriangleValid(triangle* T)
{
	bool Result = !((T->Vertex0Index == T->Vertex1Index) ||
			(T->Vertex0Index == T->Vertex2Index) ||
			(T->Vertex1Index == T->Vertex2Index));
	return(Result);
}

bool AreTrianglesCorrect(mesh* Mesh, u32* WrongTriangle)
{
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;
		if(!IsTriangleValid(T))
		{
			if(WrongTriangle)
			{
				*WrongTriangle = TriangleIndex;
			}
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
		mat4* Quadrics,
		triangle_index_list* ReverseTriangleIndices)
{
	vertex V0 = Mesh->Vertices[E.Vertex0Index];
	vertex V1 = Mesh->Vertices[E.Vertex1Index];
	u32 T0Index = 0;
	u32 T1Index = 0;

	// TODO(hugo) : Optim : this is computed previously, reuse and pass it 
	// to ContractEdge
	Assert(!IsBoundary(E, ReverseTriangleIndices, &T0Index, &T1Index));

	if(T1Index == Mesh->TriangleCount - 1)
	{
		DeleteTriangle(Mesh, T0Index);
		Quadrics[T0Index] = Quadrics[Mesh->TriangleCount];
		DeleteTriangle(Mesh, T0Index);
		Quadrics[T0Index] = Quadrics[Mesh->TriangleCount];
	}
	else
	{
		DeleteTriangle(Mesh, T0Index);
		Quadrics[T0Index] = Quadrics[Mesh->TriangleCount];
		DeleteTriangle(Mesh, T1Index);
		Quadrics[T1Index] = Quadrics[Mesh->TriangleCount];
	}

	PushVertex(Mesh, OptimalPos);

	// NOTE(hugo) : Remplacing all occurences of V0 and V1 in triangles by VF (the last point added to the mesh)
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;
		Assert(IsTriangleValid(T));
		if(T->Vertex0Index == E.Vertex0Index)
		{
			T->Vertex0Index = Mesh->VertexCount - 1;
			Assert(IsTriangleValid(T));
			Quadrics[TriangleIndex] = GetQuadricOfTriangle(Mesh, T);
		}
		else if(T->Vertex1Index == E.Vertex0Index)
		{
			T->Vertex1Index = Mesh->VertexCount - 1;
			Assert(IsTriangleValid(T));
			Quadrics[TriangleIndex] = GetQuadricOfTriangle(Mesh, T);
		}
		else if(T->Vertex2Index == E.Vertex0Index)
		{
			T->Vertex2Index = Mesh->VertexCount - 1;
			Assert(IsTriangleValid(T));
			Quadrics[TriangleIndex] = GetQuadricOfTriangle(Mesh, T);
		}

		if(T->Vertex0Index == E.Vertex1Index)
		{
			T->Vertex0Index = Mesh->VertexCount - 1;
			Assert(IsTriangleValid(T));
			Quadrics[TriangleIndex] = GetQuadricOfTriangle(Mesh, T);
		}
		else if(T->Vertex1Index == E.Vertex1Index)
		{
			T->Vertex1Index = Mesh->VertexCount - 1;
			Assert(IsTriangleValid(T));
			Quadrics[TriangleIndex] = GetQuadricOfTriangle(Mesh, T);
		}
		else if(T->Vertex2Index == E.Vertex1Index)
		{
			T->Vertex2Index = Mesh->VertexCount - 1;
			Assert(IsTriangleValid(T));
			Quadrics[TriangleIndex] = GetQuadricOfTriangle(Mesh, T);
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
		DeleteVertex(Mesh, E.Vertex1Index);

		memset(ReverseTriangleIndices, 0, sizeof(triangle_index_list) * (Mesh->VertexCount + 1));
		ComputeReverseTriangleIndices(Mesh, ReverseTriangleIndices);
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

	return(MinDistSqrFound);
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
					//Quadrics[E.Vertex0Index] = ComputeQuadric(Mesh, E.Vertex0Index);
				}
				if(IsZeroMatrix(Quadrics[E.Vertex1Index]))
				{
					//Quadrics[E.Vertex1Index] = ComputeQuadric(Mesh, E.Vertex1Index);
				}

				mat4 Quadric = Quadrics[E.Vertex0Index] + Quadrics[E.Vertex1Index];
				ComputeCostAndVertexOfContraction(Mesh, &C, Quadric);

				// NOTE(hugo) : inserting the contraction in the list (the list is in decreasing order)
				PushContraction(Queue, C);
			}
		}
	}
}

contraction GetBestContraction(mesh* Mesh, 
		mat4* Quadrics, 
		triangle_index_list* ReverseTriangleIndices)
{
	contraction Result = {};
	Result.Cost = FLT_MAX;
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;

		for(u32 i = 0; i < 3; ++i)
		{
			edge E = {T->VertexIndices[i], T->VertexIndices[(i + 1) % 3]};
			if(!IsBoundary(E, ReverseTriangleIndices))
			{
				contraction C = {};
				C.Edge = E;

				//mat4 Quadric = ComputeQuadrics[E.Vertex0Index] + Quadrics[E.Vertex1Index];
				mat4 Quadric = ComputeQuadric(E.Vertex0Index, Quadrics, ReverseTriangleIndices) + ComputeQuadric(E.Vertex1Index, Quadrics, ReverseTriangleIndices);
				ComputeCostAndVertexOfContraction(Mesh, &C, Quadric);

				if(C.Cost < Result.Cost)
				{
					Result = C;
				}
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
		mat4 PlaneQuadric = GetQuadricOfTriangle(Mesh, T);

		Quadrics[TriangleIndex] = PlaneQuadric;
	}
}

struct plane
{
	// NOTE(hugo) : defined by the equation 
	//           <N, v> + d = 0
	v3 N;
	float D;
};

struct proxy
{
	u32 TriangleCount;
	u32 TriangleIndices[256];

	// NOTE(hugo) : defined by the equation 
	//           <N, v> + d = 0
	v3 N;
	float D;
};

void PushTriangle(proxy* P, u32 TriangleIndex)
{
	Assert(P->TriangleCount < ArrayCount(P->TriangleIndices));
	P->TriangleIndices[P->TriangleCount] = TriangleIndex;
	++P->TriangleCount;
}

bool IsTriangleInProxy(u32 TriangleIndex, proxy* Proxy)
{
	bool Found = false;
	for(u32 Index = 0; (!Found) && (Index < Proxy->TriangleCount); ++Index)
	{
		if(Proxy->TriangleIndices[Index] == TriangleIndex)
		{
			Found = true;
		}
	}
	return(Found);
}

bool IsTriangleInProxies(u32 TriangleIndex, s32* TriangleProxyMap)
{
	bool Result = (TriangleProxyMap[TriangleIndex] != -1);
	return(Result);
}

s32 FindBestPlanarTriangle(float* PlanarityScore, u32 Count, s32* TriangleProxyMap)
{
	Assert(Count >= 1);
	float BestScoreFound = PlanarityScore[0];
	u32 IndexBestFound = 0;
	bool FoundOne = false;

	for(u32 Index = 0; Index < Count; ++Index)
	{
		if(!IsTriangleInProxies(Index, TriangleProxyMap))
		{
			FoundOne = true;
			float Score = PlanarityScore[Index];
			if(Score > BestScoreFound)
			{
				BestScoreFound = Score;
				IndexBestFound = Index;
			}
		}
	}

	if(!FoundOne)
	{
		return(-1);
	}
	return(IndexBestFound);
}

bool IsTriangleBoundaryOfProxy(mesh* Mesh, u32 TriangleIndex, proxy* Proxy, triangle_index_list* ReverseTriangleIndices)
{
	for(u32 Index = 0; Index < Proxy->TriangleCount; ++Index)
	{
		u32 ProxyTriangleIndex = Proxy->TriangleIndices[Index];
		triangle* T = Mesh->Triangles + ProxyTriangleIndex;
		for(u32 i = 0; i < 3; ++i)
		{
			edge E = {T->VertexIndices[i], T->VertexIndices[(i + 1) % 3]};
			u32 T0Index;
			u32 T1Index;
			if(!IsBoundary(E, ReverseTriangleIndices, &T0Index, &T1Index))
			{
				if(T0Index == ProxyTriangleIndex)
				{
					if(T1Index == TriangleIndex)
					{
						return(true);
					}
				}
				else if(T1Index == ProxyTriangleIndex)
				{
					if(T0Index == TriangleIndex)
					{
						return(true);
					}
				}
				else
				{
					InvalidCodePath;
				}
			}
		}
	}

	return(false);
}

s32 FindBestPlanarTriangleWithConstraints(mesh* Mesh, float* PlanarityScore, u32 Count, 
		proxy* Proxies, u32 ProxyCount, triangle_index_list* ReverseTriangleIndices,
		s32* TriangleProxyMap)
{
	float BestScoreFound = -1.0f;
	u32 BestIndexFound = 0;
	bool FoundOneNotInProxies = false;
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		if(!IsTriangleInProxies(TriangleIndex, TriangleProxyMap))
		{
			if(IsTriangleBoundaryOfProxy(Mesh, TriangleIndex, Proxies + (ProxyCount - 1), ReverseTriangleIndices))
			{
				FoundOneNotInProxies = true;
				float TriangleScore = PlanarityScore[TriangleIndex];
				if(TriangleScore > BestScoreFound)
				{
					BestScoreFound = TriangleScore;
					BestIndexFound = TriangleIndex;
				}
			}
		}
	}
	if(!FoundOneNotInProxies)
	{
		return(-1);
	}
	return(BestIndexFound);
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

	u32 ContractionGoal = 5000;
	contraction_queue Queue = {};

	mat4* Quadrics = AllocateArray(mat4, Mesh.TriangleCount);
	memset(Quadrics, 0, sizeof(mat4) * Mesh.TriangleCount);
	ComputeQuadrics(&Mesh, Quadrics);

	// NOTE(hugo) : +1 because we push the new vertex before deletex the others
	triangle_index_list* ReverseTriangleIndices = AllocateArray(triangle_index_list, Mesh.VertexCount + 1);	
	memset(ReverseTriangleIndices, 0, sizeof(triangle_index_list) * (Mesh.VertexCount + 1));
	ComputeReverseTriangleIndices(&Mesh, ReverseTriangleIndices);

	// NOTE(hugo) : Planarity score computation
	v3* TrianglesNormal = AllocateArray(v3, Mesh.TriangleCount);
	memset(TrianglesNormal, 0, sizeof(v3) * (Mesh.TriangleCount));
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh.TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh.Triangles + TriangleIndex;
		v3 P0 = Mesh.Vertices[T->Vertex0Index];
		v3 P1 = Mesh.Vertices[T->Vertex1Index];
		v3 P2 = Mesh.Vertices[T->Vertex2Index];
		v3 N = Normalized(Cross(P1 - P0, P2 - P0));
		TrianglesNormal[TriangleIndex] = N;
	}

	float* PlanarityScore = AllocateArray(float, Mesh.TriangleCount);
	memset(PlanarityScore, 0, sizeof(float) * Mesh.TriangleCount);
	u32* PlanarityCount = AllocateArray(u32, Mesh.TriangleCount);
	memset(PlanarityCount, 0, sizeof(u32) * Mesh.TriangleCount);
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh.TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh.Triangles + TriangleIndex;
		for(u32 Index = 0; Index < 3; ++Index)
		{
			u32 PointID = T->VertexIndices[Index];
			triangle_index_list* ReverseTriangleIndex = ReverseTriangleIndices + PointID;
			for(u32 NeighbourIndex = 0; NeighbourIndex < ReverseTriangleIndex->TriangleCount; ++NeighbourIndex)
			{
				// TODO(hugo) : Not perfect since I can consider the same neighbour twice.
				// Indeed, two vertices can share the same neighbour.
				u32 NeighbourID = ReverseTriangleIndex->TriangleIndices[NeighbourIndex];
				if(NeighbourID != TriangleIndex)
				{
					PlanarityScore[TriangleIndex] += 
						Dot(TrianglesNormal[TriangleIndex], TrianglesNormal[NeighbourID]);
					PlanarityCount[TriangleIndex] += 1;
				}
			}
		}
		Assert(PlanarityCount[TriangleIndex] != 0);
		PlanarityScore[TriangleIndex] /= (float(PlanarityCount[TriangleIndex]));
	}

	s32* TriangleProxyMap = AllocateArray(s32, Mesh.TriangleCount);
	memset(TriangleProxyMap, -1, sizeof(s32) * Mesh.TriangleCount);
	float NormalTolerance = 1.5f;
	float DistanceTolerance = 1.5f;
	proxy Proxies[128];
	u32 ProxyCount = 0;

	bool TrianglesLeft = true;
	while(TrianglesLeft)
	{
		u32 BestPlanarTriangleIndex = FindBestPlanarTriangle(PlanarityScore, Mesh.TriangleCount, 
				TriangleProxyMap);
		if(BestPlanarTriangleIndex != -1)
		{
			Assert(ProxyCount < ArrayCount(Proxies));
			proxy Proxy = {};
			Proxy.TriangleCount = 1;
			Proxy.TriangleIndices[0] = BestPlanarTriangleIndex;
			Proxy.N = TrianglesNormal[BestPlanarTriangleIndex];
			Proxy.D = - Dot(Proxy.N, Mesh.Vertices[Mesh.Triangles[BestPlanarTriangleIndex].Vertex0Index]);
			Proxies[ProxyCount] = Proxy;
			++ProxyCount;

			bool KeepGrowing = true;
			while(KeepGrowing)
			{
				s32 BestIndex = FindBestPlanarTriangleWithConstraints(&Mesh, PlanarityScore, 
						Mesh.TriangleCount, &Proxies[0], ProxyCount, ReverseTriangleIndices,
						TriangleProxyMap);
				if(BestIndex == -1)
				{
					TrianglesLeft = false;
					break;
				}
				v3 TNormal = TrianglesNormal[BestIndex];
				// TODO(hugo) : Can be improved by selecting the min or the max of the three vertices
				float DistanceToProxy = Abs(Dot(Proxy.N, Mesh.Vertices[Mesh.Triangles[BestIndex].Vertex0Index]));
				if((DistanceToProxy < DistanceTolerance) &&
						(LengthSqr(TNormal - Proxy.N) < NormalTolerance))
				{
					PushTriangle(&Proxies[ProxyCount - 1], BestIndex);
					TriangleProxyMap[BestIndex] = ProxyCount - 1;
				}
				else
				{
					KeepGrowing = false;
				}
			}
#if 0
			// TODO(hugo) : Reject proxy if its area is too small
			float A = Area(Proxy);
			if(A < AreaThreshold)
			{
			}
			else
			{
			}
#endif
		}
		else
		{
			TrianglesLeft = false;
		}
	}

	while(ContractionGoal > 0)
	{
		printf("Computing contractions\n");

		contraction C = GetBestContraction(&Mesh, Quadrics, ReverseTriangleIndices);
		ContractEdge(&Mesh, C.Edge, C.OptimalVertex, &Queue, Quadrics, ReverseTriangleIndices);
		printf("Contraction cost was %f.\n", C.Cost);

		--ContractionGoal;
		printf("Contraction goal : %i.\n", ContractionGoal);
	}

	Free(ReverseTriangleIndices);
	Free(Quadrics);

#if 1
	float DistanceBetweenMeshes = MeanDistance(&InputMesh, &Mesh);
	printf("Distance between meshes is : %f\n", DistanceBetweenMeshes);
#endif

	SaveOFF(&Mesh, "output.off");

	Free(TrianglesNormal);
	Free(Mesh.Vertices);
	Free(Mesh.Triangles);
	Free(InputMesh.Vertices);
	Free(InputMesh.Triangles);

	return(0);
}
