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

struct proxy
{
	u32 TriangleCount;
	u32 TriangleIndices[256];

	// NOTE(hugo) : defined by the equation 
	//           <N, v> + d = 0
	v3 N;
	float D;
};

struct triangles_properties
{
	u32 TriangleCount;
	float* Areas;
	v3* Normals;
	float* PlanarityScore;
	s32* ProxyMap;
	mat4* InnerQuadrics;

	proxy Proxies[128];
	u32 ProxyCount;
	float Lambda;
};

void SwitchProperties(triangles_properties* Properties, u32 AIndex, u32 BIndex)
{
	Properties->Areas[AIndex] = Properties->Areas[BIndex];
	Properties->Normals[AIndex] = Properties->Normals[BIndex];
	Properties->PlanarityScore[AIndex] = Properties->PlanarityScore[BIndex];
	Properties->ProxyMap[AIndex] = Properties->ProxyMap[BIndex];
	Properties->InnerQuadrics[AIndex] = Properties->InnerQuadrics[BIndex];
}

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

mat4 QuadricFromPlane(v3 N, float d)
{
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

mat4 GetQuadricOfTriangle(mesh* Mesh, triangle* T)
{
	vertex A = Mesh->Vertices[T->VertexIndices[0]];
	vertex B = Mesh->Vertices[T->VertexIndices[1]];
	vertex C = Mesh->Vertices[T->VertexIndices[2]];
	vertex N = Normalized(Cross(B - A, C - A));
	float d = - Dot(N, A);
	mat4 Result = QuadricFromPlane(N, d);

	return(Result);
}

void ComputeQuadrics(mesh* Mesh, mat4* TriangleQuadrics, 
		proxy* Proxies, u32 ProxyCount, s32* TriangleProxyMap, 
		float* TriangleAreas,
		float Lambda)
{
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;
		mat4 PlaneQuadric = GetQuadricOfTriangle(Mesh, T);
		if(TriangleProxyMap[TriangleIndex] != -1)
		{
			proxy Proxy = Proxies[TriangleProxyMap[TriangleIndex]];
			mat4 ProxyQuadric = QuadricFromPlane(Proxy.N, Proxy.D);

			TriangleQuadrics[TriangleIndex] = ((1.0f - Lambda) * PlaneQuadric + Lambda * ProxyQuadric);
		}
		else
		{
			TriangleQuadrics[TriangleIndex] = PlaneQuadric;
		}

		float Area = TriangleAreas[TriangleIndex];
		TriangleQuadrics[TriangleIndex] *= Area;
	}
}


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

bool ArePermutation(u32 A[], u32 B[], u32 Count)
{
	if((A[0] == B[0]) && (A[1] == B[1]) && (A[2] == B[2]))
	{
		return(true);
	}
	if((A[0] == B[0]) && (A[1] == B[2]) && (A[2] == B[1]))
	{
		return(true);
	}
	if((A[0] == B[1]) && (A[1] == B[0]) && (A[2] == B[2]))
	{
		return(true);
	}
	if((A[0] == B[2]) && (A[1] == B[1]) && (A[2] == B[0]))
	{
		return(true);
	}
	if((A[0] == B[1]) && (A[1] == B[2]) && (A[2] == B[0]))
	{
		return(true);
	}
	if((A[0] == B[2]) && (A[1] == B[0]) && (A[2] == B[1]))
	{
		return(true);
	}

	return(false);
}

void PreProcessMesh(mesh* Mesh)
{
	for(u32 TriangleAIndex = 0; TriangleAIndex < (Mesh->TriangleCount - 1); ++TriangleAIndex)
	{
		triangle* A = Mesh->Triangles + TriangleAIndex;
		for(u32 TriangleBIndex = TriangleAIndex + 1; TriangleBIndex < Mesh->TriangleCount; ++TriangleBIndex)
		{
			triangle* B = Mesh->Triangles + TriangleBIndex;
			if(ArePermutation(A->VertexIndices, B->VertexIndices, ArrayCount(A->VertexIndices)))
			{
				DeleteTriangle(Mesh, TriangleBIndex);
				--TriangleBIndex;
			}
		}
	}
}

bool AreTrianglesValid(mesh* Mesh)
{
	for(u32 TriangleAIndex = 0; TriangleAIndex < (Mesh->TriangleCount - 1); ++TriangleAIndex)
	{
		triangle* A = Mesh->Triangles + TriangleAIndex;
		for(u32 TriangleBIndex = TriangleAIndex + 1; TriangleBIndex < Mesh->TriangleCount; ++TriangleBIndex)
		{
			triangle* B = Mesh->Triangles + TriangleBIndex;
			if(ArePermutation(A->VertexIndices, B->VertexIndices, ArrayCount(A->VertexIndices)))
			{
				return(false);
			}
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

float GetTriangleArea(mesh* Mesh, triangle* T)
{
	vertex P0 = Mesh->Vertices[T->VertexIndices[0]];
	vertex P1 = Mesh->Vertices[T->VertexIndices[1]];
	vertex P2 = Mesh->Vertices[T->VertexIndices[2]];
	float A = SquareRoot(LengthSqr(P0 - P1));
	float B = SquareRoot(LengthSqr(P1 - P2));
	float C = SquareRoot(LengthSqr(P2 - P0));
	float S = 0.5f * (A + B + C);
	
	float Result = SquareRoot(S * (S - A) * (S - B) * (S - C));

	return(Result);
}

v3 GetTriangleNormal(mesh* Mesh, triangle* T)
{
	v3 P0 = Mesh->Vertices[T->Vertex0Index];
	v3 P1 = Mesh->Vertices[T->Vertex1Index];
	v3 P2 = Mesh->Vertices[T->Vertex2Index];
	v3 N = Normalized(Cross(P1 - P0, P2 - P0));

	return(N);
}

void ContractEdge(
		mesh* Mesh, 
		edge E, 
		vertex OptimalPos, 
		contraction_queue* Queue, 
		triangles_properties* Properties,
		triangle_index_list* ReverseTriangleIndices,
		proxy* Proxies,
		u32 ProxyCount,
		float Lambda)
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
		//Properties->InnerQuadrics[T0Index] = Properties->InnerQuadrics[Mesh->TriangleCount];
		SwitchProperties(Properties, T0Index, Mesh->TriangleCount);
		DeleteTriangle(Mesh, T0Index);
		//Properties->InnerQuadrics[T0Index] = Properties->InnerQuadrics[Mesh->TriangleCount];
		SwitchProperties(Properties, T0Index, Mesh->TriangleCount);
	}
	else
	{
		DeleteTriangle(Mesh, T0Index);
		//Properties->InnerQuadrics[T0Index] = Properties->InnerQuadrics[Mesh->TriangleCount];
		SwitchProperties(Properties, T0Index, Mesh->TriangleCount);
		DeleteTriangle(Mesh, T1Index);
		//Properties->InnerQuadrics[T1Index] = Properties->InnerQuadrics[Mesh->TriangleCount];
		SwitchProperties(Properties, T1Index, Mesh->TriangleCount);
	}
	PreProcessMesh(Mesh);

	PushVertex(Mesh, OptimalPos);

	// NOTE(hugo) : Remplacing all occurences of V0 and V1 in triangles by VF (the last point added to the mesh)
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;
		bool DeletionHappened = false;
		if(T->Vertex0Index == E.Vertex0Index)
		{
			T->Vertex0Index = Mesh->VertexCount - 1;
			if(IsTriangleValid(T))
			{
				Properties->InnerQuadrics[TriangleIndex] = GetQuadricOfTriangle(Mesh, T);
				Properties->Areas[TriangleIndex] = GetTriangleArea(Mesh, T);
				Properties->Normals[TriangleIndex] = GetTriangleNormal(Mesh, T);
			}
			else
			{
				DeleteTriangle(Mesh, TriangleIndex);
				--TriangleIndex;
				DeletionHappened = true;
			}
		}
		else if(T->Vertex1Index == E.Vertex0Index)
		{
			T->Vertex1Index = Mesh->VertexCount - 1;
			if(IsTriangleValid(T))
			{
				Properties->InnerQuadrics[TriangleIndex] = GetQuadricOfTriangle(Mesh, T);
				Properties->Areas[TriangleIndex] = GetTriangleArea(Mesh, T);
				Properties->Normals[TriangleIndex] = GetTriangleNormal(Mesh, T);
			}
			else
			{
				DeleteTriangle(Mesh, TriangleIndex);
				--TriangleIndex;
				DeletionHappened = true;
			}
		}
		else if(T->Vertex2Index == E.Vertex0Index)
		{
			T->Vertex2Index = Mesh->VertexCount - 1;
			if(IsTriangleValid(T))
			{
				Properties->InnerQuadrics[TriangleIndex] = GetQuadricOfTriangle(Mesh, T);
				Properties->Areas[TriangleIndex] = GetTriangleArea(Mesh, T);
				Properties->Normals[TriangleIndex] = GetTriangleNormal(Mesh, T);
			}
			else
			{
				DeleteTriangle(Mesh, TriangleIndex);
				--TriangleIndex;
				DeletionHappened = true;
			}
		}

		if(!DeletionHappened)
		{
			if(T->Vertex0Index == E.Vertex1Index)
			{
				T->Vertex0Index = Mesh->VertexCount - 1;
				if(IsTriangleValid(T))
				{
					Properties->InnerQuadrics[TriangleIndex] = GetQuadricOfTriangle(Mesh, T);
					Properties->Areas[TriangleIndex] = GetTriangleArea(Mesh, T);
					Properties->Normals[TriangleIndex] = GetTriangleNormal(Mesh, T);
				}
				else
				{
					DeleteTriangle(Mesh, TriangleIndex);
					--TriangleIndex;
					DeletionHappened = true;
				}
			}
			else if(T->Vertex1Index == E.Vertex1Index)
			{
				T->Vertex1Index = Mesh->VertexCount - 1;
				if(IsTriangleValid(T))
				{
					Properties->InnerQuadrics[TriangleIndex] = GetQuadricOfTriangle(Mesh, T);
					Properties->Areas[TriangleIndex] = GetTriangleArea(Mesh, T);
					Properties->Normals[TriangleIndex] = GetTriangleNormal(Mesh, T);
				}
				else
				{
					DeleteTriangle(Mesh, TriangleIndex);
					--TriangleIndex;
					DeletionHappened = true;
				}
			}
			else if(T->Vertex2Index == E.Vertex1Index)
			{
				T->Vertex2Index = Mesh->VertexCount - 1;
				if(IsTriangleValid(T))
				{
					Properties->InnerQuadrics[TriangleIndex] = GetQuadricOfTriangle(Mesh, T);
					Properties->Areas[TriangleIndex] = GetTriangleArea(Mesh, T);
					Properties->Normals[TriangleIndex] = GetTriangleNormal(Mesh, T);
				}
				else
				{
					DeleteTriangle(Mesh, TriangleIndex);
					--TriangleIndex;
					DeletionHappened = true;
				}
			}
		}
	}
	PreProcessMesh(Mesh);

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
		ComputeQuadrics(Mesh, Properties->InnerQuadrics, Proxies, ProxyCount,
				Properties->ProxyMap, Properties->Areas, Lambda);
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

struct plane
{
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

void FreeProperties(triangles_properties* Properties)
{
	Free(Properties->Areas);
	Free(Properties->Normals);
	Free(Properties->PlanarityScore);
	Free(Properties->ProxyMap);
	Free(Properties->InnerQuadrics);
}

void ComputeProperties(mesh* Mesh, triangles_properties* Properties,
		triangle_index_list* ReverseTriangleIndices)
{
	Properties->TriangleCount = Mesh->TriangleCount;
	Properties->Areas = AllocateArray(float, Mesh->TriangleCount);
	memset(Properties->Areas, 0, sizeof(float) * Mesh->TriangleCount);
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;
		Properties->Areas[TriangleIndex] = GetTriangleArea(Mesh, T);
	}

	// NOTE(hugo) : Planarity score computation
	// {
	Properties->Normals = AllocateArray(v3, Mesh->TriangleCount);
	memset(Properties->Normals, 0, sizeof(v3) * (Mesh->TriangleCount));
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;
		Properties->Normals[TriangleIndex] = GetTriangleNormal(Mesh, T);
	}

	Properties->PlanarityScore = AllocateArray(float, Mesh->TriangleCount);
	memset(Properties->PlanarityScore, 0, sizeof(float) * Mesh->TriangleCount);
	u32* PlanarityCount = AllocateArray(u32, Mesh->TriangleCount);
	memset(PlanarityCount, 0, sizeof(u32) * Mesh->TriangleCount);
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;
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
					Properties->PlanarityScore[TriangleIndex] += 
						Dot(Properties->Normals[TriangleIndex], Properties->Normals[NeighbourID]);
					PlanarityCount[TriangleIndex] += 1;
				}
			}
		}
		Assert(PlanarityCount[TriangleIndex] != 0);
		Properties->PlanarityScore[TriangleIndex] /= (float(PlanarityCount[TriangleIndex]));
	}
	Free(PlanarityCount);

	Properties->ProxyMap = AllocateArray(s32, Mesh->TriangleCount);
	memset(Properties->ProxyMap, -1, sizeof(s32) * Mesh->TriangleCount);
	float NormalTolerance = 0.5f;
	float DistanceTolerance = 0.5f;

	bool TrianglesLeft = true;
	while(TrianglesLeft)
	{
		u32 BestPlanarTriangleIndex = FindBestPlanarTriangle(Properties->PlanarityScore, Mesh->TriangleCount, 
				Properties->ProxyMap);
		if(BestPlanarTriangleIndex != -1)
		{
			Assert(Properties->ProxyCount < ArrayCount(Properties->Proxies));
			proxy Proxy = {};
			Proxy.TriangleCount = 1;
			Proxy.TriangleIndices[0] = BestPlanarTriangleIndex;
			Proxy.N = Properties->Normals[BestPlanarTriangleIndex];
			Proxy.D = - Dot(Proxy.N, Mesh->Vertices[Mesh->Triangles[BestPlanarTriangleIndex].Vertex0Index]);
			Properties->Proxies[Properties->ProxyCount] = Proxy;
			++Properties->ProxyCount;
			Properties->ProxyMap[BestPlanarTriangleIndex] = Properties->ProxyCount - 1;

			bool KeepGrowing = true;
			while(KeepGrowing)
			{
				s32 BestIndex = FindBestPlanarTriangleWithConstraints(Mesh, Properties->PlanarityScore, 
						Mesh->TriangleCount, &Properties->Proxies[0], Properties->ProxyCount, ReverseTriangleIndices,
						Properties->ProxyMap);
				if(BestIndex == -1)
				{
					TrianglesLeft = false;
					break;
				}
				v3 TNormal = Properties->Normals[BestIndex];
				// TODO(hugo) : Can be improved by selecting the min or the max of the three vertices
				float DistanceToProxy = Abs(Proxy.D + Dot(Proxy.N, Mesh->Vertices[Mesh->Triangles[BestIndex].Vertex0Index]));
				if((DistanceToProxy < DistanceTolerance) &&
						(LengthSqr(TNormal - Proxy.N) < NormalTolerance))
				{
					PushTriangle(&Properties->Proxies[Properties->ProxyCount - 1], BestIndex);
					Properties->ProxyMap[BestIndex] = Properties->ProxyCount - 1;
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
	// }

	Properties->Lambda = 0.8f;
	Properties->Lambda = 0.0f;
	Properties->InnerQuadrics = AllocateArray(mat4, Mesh->TriangleCount);
	memset(Properties->InnerQuadrics, 0, sizeof(mat4) * Mesh->TriangleCount);
	ComputeQuadrics(Mesh, Properties->InnerQuadrics, Properties->Proxies, Properties->ProxyCount, Properties->ProxyMap, 
			Properties->Areas, Properties->Lambda);
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
	u32 ContractionGoal = 2000;
	contraction_queue Queue = {};

	Assert(AreTrianglesValid(&Mesh));

	while(ContractionGoal > 0)
	{
		printf("Computing contractions\n");

		contraction C = GetBestContraction(&Mesh, Properties.InnerQuadrics, ReverseTriangleIndices);
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
