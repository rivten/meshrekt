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

