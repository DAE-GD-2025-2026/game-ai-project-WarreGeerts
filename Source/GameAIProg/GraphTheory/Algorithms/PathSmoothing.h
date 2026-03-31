#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
	{
	public:
		//=== SSFA Functions ===
		//--- References ---
		//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
		//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
		static std::vector<NavLine> FindPortals(std::vector<Node*> const& Path, TriPolygon const& NavPoly)
		{
			std::vector<NavLine> Portals = {};

			if (Path.size() < 2) return Portals;

			Portals.push_back(NavLine{Path[0]->GetPosition(), Path[0]->GetPosition()});

			for (size_t i = 1; i < Path.size() - 1; ++i)
			{
				NavGraphNode* pNavNode = static_cast<NavGraphNode*>(Path[i]);
				int edgeIdx = pNavNode->GetEdgeIdx();

				const auto& edge = NavPoly.GetEdges()[edgeIdx];
				FVector2D p1 = FVector2D{edge.GetP1(NavPoly).X, edge.GetP1(NavPoly).Y};
				FVector2D p2 = FVector2D{edge.GetP2(NavPoly).X, edge.GetP2(NavPoly).Y};


				FVector2D currentPos = Path[i]->GetPosition();
				FVector2D prevPos = Path[i - 1]->GetPosition();
				FVector2D direction = (currentPos - prevPos).GetSafeNormal();

				float cross = FVector2D::CrossProduct(direction, p1 - currentPos);
				if (cross > 0)
				{
					Portals.push_back(NavLine{p2, p1});
				}
				else
				{
					Portals.push_back(NavLine{p1, p2});
				}
			}

			FVector2D endPos = Path.back()->GetPosition();
			Portals.push_back(NavLine{endPos, endPos});

			return Portals;
		}

		static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const& Portals, TriPolygon const& NavPoly)
		{
			std::vector<FVector2D> Path{};
			if (Portals.empty()) return Path;

			FVector2D apexPos = Portals[0].P1;
			FVector2D leftLeg = Portals[1].P2 - apexPos;
			FVector2D rightLeg = Portals[1].P1 - apexPos;

			int leftLegIndex = 1;
			int rightLegIndex = 1;
			int apexIndex = 0;

			Path.push_back(apexPos);

			for (int i = 1; i < Portals.size(); ++i)
			{
				const auto& portal = Portals[i];

				// --- RIGHT CHECK ---
				FVector2D newRightLeg = portal.P1 - apexPos;

				if (FVector2D::CrossProduct(rightLeg, newRightLeg) >= 0)
				{
					if (FVector2D::CrossProduct(leftLeg, newRightLeg) < 0)
					{
						rightLeg = newRightLeg;
						rightLegIndex = i;
					}
					else
					{
						apexPos += leftLeg;
						apexIndex = leftLegIndex;
						Path.push_back(apexPos);

						i = apexIndex;
						if (i + 1 < Portals.size())
						{
							leftLeg = Portals[i + 1].P2 - apexPos;
							rightLeg = Portals[i + 1].P1 - apexPos;
							leftLegIndex = i + 1;
							rightLegIndex = i + 1;
						}
						continue;
					}
				}

				// --- LEFT CHECK ---
				FVector2D newLeftLeg = portal.P2 - apexPos;

				if (FVector2D::CrossProduct(leftLeg, newLeftLeg) <= 0)
				{
					if (FVector2D::CrossProduct(rightLeg, newLeftLeg) > 0)
					{
						leftLeg = newLeftLeg;
						leftLegIndex = i;
					}
					else
					{
						apexPos += rightLeg;
						apexIndex = rightLegIndex;
						Path.push_back(apexPos);

						i = apexIndex;
						if (i + 1 < Portals.size())
						{
							leftLeg = Portals[i + 1].P2 - apexPos;
							rightLeg = Portals[i + 1].P1 - apexPos;
							leftLegIndex = i + 1;
							rightLegIndex = i + 1;
						}
						continue;
					}
				}
			}

			//Add last path point		
			Path.push_back(Portals.back().P1);

			return Path;
		}

	private:
		SSFA()
		{
		};

		~SSFA()
		{
		};
	};
}
