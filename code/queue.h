#pragma once

struct contraction_queue
{
	contraction* Contractions;
	u32 Count;
	u32 PoolSize;
};

contraction_queue CreateQueue(void)
{
	contraction_queue Result = {};

	Result.Count = 0;
	Result.PoolSize = 1;
	Result.Contractions = AllocateArray(contraction, 1);
	memset(Result.Contractions, 0, sizeof(contraction) * Result.PoolSize);

	return(Result);
}

#if 1
void PushContraction(contraction_queue* Queue, contraction C)
{
	if(Queue->Count == Queue->PoolSize)
	{
		Queue->Contractions = ReAllocateArray(Queue->Contractions, contraction, 2 * Queue->PoolSize);
		Queue->PoolSize *= 2;
	}

	if(Queue->Count == 0)
	{
		Queue->Contractions[Queue->Count] = C;
		Queue->Count++;
	}
	else if(C.Cost <= Queue->Contractions[Queue->Count - 1].Cost)
	{
		Queue->Contractions[Queue->Count] = C;
		Queue->Count++;
	}
	else
	{
		for(u32 ContractionIndex = Queue->Count - 1; ContractionIndex >= 1; --ContractionIndex)
		{
			Queue->Contractions[ContractionIndex + 1] = Queue->Contractions[ContractionIndex];
			if(C.Cost <= Queue->Contractions[ContractionIndex - 1].Cost)
			{
				Queue->Contractions[ContractionIndex] = C;
				Queue->Count++;
				return;
			}
		}
		Queue->Contractions[1] = Queue->Contractions[0];
		Queue->Contractions[0] = C;
		Queue->Count++;
	}
}
#endif

bool IsEdgeInQueue(edge E, contraction_queue* Queue)
{
	for(u32 ContractionIndex = 0; ContractionIndex < Queue->Count; ++ContractionIndex)
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

void InitQueue(mesh* Mesh, 
		contraction_queue* Queue, 
		triangle_index_list* ReverseTriangleIndices,
		mat4* Quadrics)
{
	for(u32 TriangleIndex = 0; TriangleIndex < Mesh->TriangleCount; ++TriangleIndex)
	{
		triangle* T = Mesh->Triangles + TriangleIndex;
		for(u32 i = 0; i < 3; ++i)
		{
			edge E = {T->VertexIndices[i], T->VertexIndices[(i + 1) % 3]};
			if(!IsBoundary(E, ReverseTriangleIndices))
			{
				if(!IsEdgeInQueue(E, Queue))
				{
					contraction C = {};
					C.Edge = E;

					mat4 EdgeQuadric = ComputeQuadricFromEdge(Mesh, E, Quadrics, ReverseTriangleIndices);
					ComputeCostAndVertexOfContraction(Mesh, &C, EdgeQuadric);

					PushContraction(Queue, C);
				}
			}
		}
	}
}

void DeleteContraction(contraction_queue* Queue, u32 DeletedIndex)
{
	for(u32 ContractionIndex = DeletedIndex; ContractionIndex < Queue->Count - 1; ++ContractionIndex)
	{
		Queue->Contractions[ContractionIndex] = Queue->Contractions[ContractionIndex + 1];
	}
	Queue->Count--;
}

bool IsEdgeInTriangleList(mesh* Mesh,
		edge E,
		triangle_index_list* TriangleIndices)
{
	for(u32 Index = 0; Index < TriangleIndices->TriangleCount; ++Index)
	{
		u32 TriangleIndex = TriangleIndices->TriangleIndices[Index];
		triangle* T = Mesh->Triangles + TriangleIndex;
		for(u32 i = 0; i < 3; ++i)
		{
			u32 V0Index = T->VertexIndices[i];
			u32 V1Index = T->VertexIndices[(i + 1) % 3];
			if(((V0Index == E.Vertex0Index) && (V1Index == E.Vertex1Index)) ||
					((V0Index == E.Vertex1Index) && (V1Index == E.Vertex0Index)))
			{
				return(true);
			}
		}
	}

	return(false);
}

void DeleteContractionsFromAffectedTriangles(mesh* Mesh, 
		contraction_queue* Queue, 
		triangle_index_list* AffectedTriangles)
{
	for(u32 ContractionIndex = 0; ContractionIndex < Queue->Count; ++ContractionIndex)
	{
		contraction* C = Queue->Contractions + ContractionIndex;
		edge E = C->Edge;
		if(IsEdgeInTriangleList(Mesh, E, AffectedTriangles))
		{
			DeleteContraction(Queue, ContractionIndex);
			--ContractionIndex;
		}
	}
}

void AddNewEdgesToQueue(mesh* Mesh, 
		contraction_queue* Queue, 
		triangle_index_list* AffectedTriangles, 
		triangle_index_list* ReverseTriangleIndices,
		mat4* Quadrics)
{
	for(u32 Index = 0; Index < AffectedTriangles->TriangleCount; ++Index)
	{
		u32 TriangleIndex = AffectedTriangles->TriangleIndices[Index];
		triangle* T = Mesh->Triangles + TriangleIndex;
		for(u32 i = 0; i < 3; ++i)
		{
			edge E = {T->VertexIndices[i], T->VertexIndices[(i + 1) % 3]};
			if(!IsEdgeInQueue(E, Queue))
			{
				if(!IsBoundary(E, ReverseTriangleIndices))
				{
					contraction C = {};
					C.Edge = E;

					mat4 Quadric = ComputeQuadricFromEdge(Mesh, 
							E, 
							Quadrics, 
							ReverseTriangleIndices);
					ComputeCostAndVertexOfContraction(Mesh, &C, Quadric);

					PushContraction(Queue, C);
				}
			}
		}
	}
}

void UpdateQueue(contraction_queue* Queue, 
		u32 OldIndex, u32 NewIndex)
{
	for(u32 ContractionIndex = 0; 
			ContractionIndex < Queue->Count; 
			++ContractionIndex)
	{
		contraction* C = Queue->Contractions + ContractionIndex;
		edge E = C->Edge;

		// NOTE(hugo): Updating the queue with the new value
		if(E.Vertex0Index == OldIndex)
		{
			E.Vertex0Index = NewIndex;
		}
		if(E.Vertex1Index == OldIndex)
		{
			E.Vertex1Index = NewIndex;
		}
	}
}

void DeleteVertexFromQueue(contraction_queue* Queue,
		u32 DeletedIndex)
{
	for(u32 ContractionIndex = 0; 
			ContractionIndex < Queue->Count;
			++ContractionIndex)
	{
		contraction* C = Queue->Contractions + ContractionIndex;
		edge E = C->Edge;
		if((E.Vertex0Index == DeletedIndex) ||
				(E.Vertex1Index == DeletedIndex))
		{
			DeleteContraction(Queue, ContractionIndex);
			--ContractionIndex;
		}

	}
}
