#pragma once
#include <list>
#include <vector>

#include "Debug/ReporterGraph.h"
#include "GeometryCollection/GeometryCollectionComponent.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"


struct Cell final
{
	Cell(float Left,float Bottom, float Width, float Height);
	
	std::vector<FVector2D> GetRectPoints() const;
	
	std::list<ASteeringAgent*> Agents;
	FRect BoundingBox;	
};

class CellSpace final
{
public:
	CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities);
	
	void AddAgent(ASteeringAgent& pAgent);
	void UpdateAgentCell(ASteeringAgent& pAgent, const FVector2D& OldPos);
	
	void RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius);
	const TArray<ASteeringAgent*>& GetNeighbors() const {return Neighbors;}
	int GetNrOfNeighbors() const {return NrOfNeighbors;};
	
	void EmptyCell();
	void RenderCells() const;
	
private:
	UWorld* pWorld;
	
	std::vector<Cell> Cells;
	FVector2D CellOrigin{};
	
	float SpaceWidth;
	float SpaceHeight;
	
	int NrOfRows;
	int NrOfCols;
	
	float CellWidth;
	float CellHeight;
	
	TArray<ASteeringAgent*> Neighbors;
	int NrOfNeighbors;
	
	int PositionToIndex(FVector2D const& Pos) const;
	bool DoRectsOverlap(FRect const& RectA, FRect const& RectB) const;
};