#include "SpacePartitioning.h"

Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = {Left, Bottom};
	BoundingBox.Max = {Left + Width, Bottom + Height};
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const auto Height{BoundingBox.Max.Y - BoundingBox.Min.Y};
	const auto Width{BoundingBox.Max.X - BoundingBox.Min.X};

	return {
		{BoundingBox.Min.X, BoundingBox.Min.Y},
		{BoundingBox.Min.X, BoundingBox.Min.Y + Height},
		{BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height},
		{BoundingBox.Min.X + Width, BoundingBox.Min.Y}
	};
}

CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	  , SpaceWidth{Width}
	  , SpaceHeight{Height}
	  , NrOfRows{Rows}
	  , NrOfCols{Cols}
	  , NrOfNeighbors{0}
	  , CellWidth{Width / Cols}
	  , CellHeight{Height / Rows}
{
	Neighbors.SetNum(MaxEntities);
	Cells.reserve(NrOfRows * NrOfCols);
	CellOrigin = FVector2D(-Width / 2.f, -Height / 2.f);

	for (int Row{0}; Row < NrOfRows; Row++)
	{
		for (int Col{0}; Col < NrOfCols; Col++)
		{
			Cells.emplace_back(CellOrigin.X + (Col * CellWidth),
			                   CellOrigin.Y + (Row * CellHeight),
			                   CellWidth,
			                   CellHeight);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& pAgent)
{
	Cells[PositionToIndex(pAgent.GetPosition())].Agents.emplace_back(&pAgent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& pAgent, const FVector2D& OldPos)
{
	const int OldIdx{PositionToIndex(OldPos)};
	const int NewIdx{PositionToIndex(pAgent.GetPosition())};
	
	if (OldIdx != NewIdx)
	{
		Cells[OldIdx].Agents.remove(&pAgent);
		Cells[NewIdx].Agents.emplace_back(&pAgent);
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;

	FRect BoundingBox;
	BoundingBox.Min = Agent.GetPosition() - FVector2D(QueryRadius, QueryRadius);
	BoundingBox.Max = Agent.GetPosition() + FVector2D(QueryRadius, QueryRadius);

	const int MinCol{static_cast<int>(FMath::Clamp(FMath::FloorToInt((BoundingBox.Min.X - CellOrigin.X) / CellWidth), 0, NrOfCols - 1))};
	const int MaxCol{static_cast<int>(FMath::Clamp(FMath::FloorToInt((BoundingBox.Max.X - CellOrigin.X) / CellWidth), 0, NrOfCols - 1))};

	const int MinRow{static_cast<int>(FMath::Clamp(FMath::FloorToInt((BoundingBox.Min.Y - CellOrigin.Y) / CellHeight), 0, NrOfRows - 1))};
	const int MaxRow{static_cast<int>(FMath::Clamp(FMath::FloorToInt((BoundingBox.Max.Y - CellOrigin.Y) / CellHeight), 0, NrOfRows - 1))};

	for (auto Row{MinRow}; Row < MaxRow + NrOfRows; Row++)
	{
		for (auto Col{MinCol}; Col < MaxCol + NrOfCols; Col++)
		{
			const int CellIdx = (Row * NrOfCols) + Col;
			if (CellIdx < 0 || CellIdx >= static_cast<int>(Cells.size()))
				continue;

			
			for (ASteeringAgent* pAgent : Cells[CellIdx].Agents)
			{
				if (!pAgent || pAgent == &Agent) continue;

				if (FVector2D::DistSquared(Agent.GetPosition(), pAgent->GetPosition()) < QueryRadius * QueryRadius)
				{
					if (NrOfNeighbors < Neighbors.Num())
					{
						Neighbors[NrOfNeighbors] = pAgent;
						++NrOfNeighbors;
					}
				}
			}
		}
	}
}

void CellSpace::EmptyCell()
{
	for (Cell& Cell : Cells)
	{
		Cell.Agents.clear();
	}
}

void CellSpace::RenderCells() const
{
	for (const Cell& Cell : Cells)
	{
		FVector Center{Cell.BoundingBox.Min.X + CellWidth / 2.f, Cell.BoundingBox.Min.Y + CellHeight / 2.f,0};
		FVector Radius{CellWidth / 2.f, CellHeight / 2.f,0};
		DrawDebugBox(pWorld, Center, Radius, FColor::Purple, false,-1,0,5.f);
		
		auto Text{FString::Printf(TEXT("%llu"),Cell.Agents.size())};
		DrawDebugString(pWorld, Center,Text,nullptr,FColor::Purple,0,true);
	}
}

int CellSpace::PositionToIndex(FVector2D const& Pos) const
{
	const int Col{static_cast<int>(FMath::Clamp(FMath::FloorToInt((Pos.X - CellOrigin.X) / CellWidth), 0, NrOfCols - 1))};
	const int Row{static_cast<int>(FMath::Clamp(FMath::FloorToInt((Pos.Y - CellOrigin.Y) / CellHeight), 0, NrOfRows - 1))};
	
	return (Row * NrOfCols) + Col;
}

bool CellSpace::DoRectsOverlap(FRect const& RectA, FRect const& RectB) const
{
	if (RectA.Min.X < RectB.Min.X || RectA.Max.X > RectB.Max.X) return false;
	if (RectA.Min.Y < RectB.Min.Y || RectA.Max.Y > RectB.Max.Y) return false;
	return true;
}
