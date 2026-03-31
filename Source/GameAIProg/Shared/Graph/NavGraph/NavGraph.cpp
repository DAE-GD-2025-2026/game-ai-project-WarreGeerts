#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon>&& NavPoly)
	: Graph{false}
	  , pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const& OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}

	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const& OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const& pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}

	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	const auto& allEdges = pNavPoly->GetEdges();
	const auto& allTriangles = pNavPoly->GetTriangles();

	for (int edgeIdx = 0; edgeIdx < (int)allEdges.size(); ++edgeIdx)
	{
		const auto& currentEdge = allEdges[edgeIdx];

		int connectedTriCount = 0;
		for (const auto& tri : allTriangles)
		{
			if (tri.HasEdge(currentEdge))
			{
				connectedTriCount++;
			}
		}

		if (connectedTriCount > 1)
		{
			const FVector p1 = currentEdge.GetP1(*pNavPoly);
			const FVector p2 = currentEdge.GetP2(*pNavPoly);
			const FVector midpoint = (p1 + p2) * 0.5f;

			const FVector2D pos2D{midpoint.X, midpoint.Y};

			auto pNewNode = std::make_unique<NavGraphNode>(pos2D, edgeIdx);
			this->AddNode(std::move(pNewNode));
		}
	}

	//2. Create connections now that every node is created	
	for (const auto& tri : pNavPoly->GetTriangles())
	{
		std::vector<int> nodeIdsOnTriangle;
		
		auto triEdges = tri.GetEdges();

		for (const auto& triEdge : triEdges)
		{
			std::optional<int> globalEdgeIdx = pNavPoly->FindEdgeIndex(triEdge);

			if (globalEdgeIdx.has_value())
			{
				int nodeId = this->GetNodeIdFromEdgeIndex(globalEdgeIdx.value());

				if (nodeId != -1)
				{
					nodeIdsOnTriangle.push_back(nodeId);
				}
			}
		}

		auto CreateTwoWayConnection = [&](int id1, int id2)
		{
			//3. Set the connections cost to the actual distance
			float distance = FVector2D::Distance(this->GetNode(id1)->GetPosition(), this->GetNode(id2)->GetPosition());

			auto connectionAB{std::make_unique<Connection>(id1, id2)};
			connectionAB->SetWeight(distance);
			this->AddConnection(std::move(connectionAB));

			auto connectionBA{std::make_unique<Connection>(id2, id1)};
			connectionBA->SetWeight(distance);
			this->AddConnection(std::move(connectionBA));
		};

		if (nodeIdsOnTriangle.size() == 2)
		{
			//2 valid nodes -> 1 path
			CreateTwoWayConnection(nodeIdsOnTriangle[0], nodeIdsOnTriangle[1]);
		}
		else if (nodeIdsOnTriangle.size() == 3)
		{
			//3 valid nodes -> 3 paths
			CreateTwoWayConnection(nodeIdsOnTriangle[0], nodeIdsOnTriangle[1]);
			CreateTwoWayConnection(nodeIdsOnTriangle[1], nodeIdsOnTriangle[2]);
			CreateTwoWayConnection(nodeIdsOnTriangle[2], nodeIdsOnTriangle[0]);
		}
	}
}
