#include "AStar.h"

#include <map>

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	  , HeuristicFunction(hFunction)
{
}

std::vector<Node*> AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	//Node, Cost of node f(n) = g(n) + h(n)
	std::vector<Node*> ClosedList{};
	std::map<Node*, int> OpenList{};

	auto H{GetHeuristicCost(pStartNode, pGoalNode)};

	Node* CurrentNode{pStartNode};
	OpenList.insert({CurrentNode, 0});

	while (CurrentNode->GetId() != pGoalNode->GetId())
	{
		for (const auto& Neighbor : pGraph->FindConnectionsFrom(CurrentNode->GetId()))
		{
			float G = FVector2D::Distance(
				pGraph->GetNode(CurrentNode->GetId())->GetPosition(), pGraph->GetNode(Neighbor->GetToId())->GetPosition()
			);

			OpenList.insert({pGraph->GetNode(Neighbor->GetToId()).get(), G});
		}
		ClosedList.emplace_back(CurrentNode);
		OpenList.erase(CurrentNode);


		float LowestCost{FLT_MAX};
		Node* LowestNode{nullptr};
		for (const auto& [node,G] : OpenList)
		{
			H = GetHeuristicCost(node, pGoalNode);
			auto F{G + H};
			if (F < LowestCost)
			{
				LowestCost = F;
				LowestNode = node;
			}
		}
		CurrentNode = LowestNode;
	}
	ClosedList.emplace_back(pGoalNode);
	return ClosedList;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->
		GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}
