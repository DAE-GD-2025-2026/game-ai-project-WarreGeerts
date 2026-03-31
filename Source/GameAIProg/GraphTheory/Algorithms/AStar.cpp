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
	std::map<Node*, int> OpenList{};
	std::map<Node*, int> ClosedList{};
	std::vector<Node*> Path{};

	std::map<Node*, Node*> cameFrom{};

	int StartG{0};
	OpenList.insert({pStartNode, StartG});
	Node* CurrentNode{pStartNode};

	while (!OpenList.empty())
	{
		float LowestCost{FLT_MAX};
		Node* LowestNode{nullptr};
		for (const auto& [node,G] : OpenList)
		{
			const auto H = GetHeuristicCost(node, pGoalNode);
			const auto F{G + H};
			if (F < LowestCost)
			{
				LowestCost = F;
				LowestNode = node;
			}
		}
		CurrentNode = LowestNode;

		if (CurrentNode->GetId() == pGoalNode->GetId()) break;

		for (const auto& Neighbor : pGraph->FindConnectionsFrom(CurrentNode->GetId()))
		{
			auto pNextNode = pGraph->GetNode(Neighbor->GetToId()).get();

			const int PrevG = OpenList[CurrentNode];
			const int EdgeCost = Neighbor->GetWeight();
			const int TotalG = PrevG + EdgeCost;

			const auto ClosedIt = ClosedList.find(pNextNode);
			if (ClosedIt != ClosedList.end())
			{
				if (TotalG >= ClosedIt->second) continue;
				ClosedList.erase(ClosedIt);
			}

			const auto OpenIt = OpenList.find(pNextNode);
			if (OpenIt != OpenList.end())
			{
				if (TotalG >= OpenIt->second) continue;
				OpenList.erase(OpenIt);
			}
			
			OpenList.insert({pNextNode, TotalG});
			cameFrom[pNextNode] = CurrentNode;
		}
		const auto CurG = OpenList.find(CurrentNode)->second;
		OpenList.erase(CurrentNode);
		ClosedList.insert({CurrentNode, CurG});
	}

	if (CurrentNode->GetId() != pGoalNode->GetId())
	{
		Node* closestNode = nullptr;
		float closestDist = FLT_MAX;

		for (const auto& [node, g] : ClosedList)
		{
			float dist = GetHeuristicCost(node, pGoalNode);
			if (dist < closestDist)
			{
				closestDist = dist;
				closestNode = node;
			}
		}

		if (closestNode)
		{
			CurrentNode = closestNode;
		}
		else
		{
			return {};
		}
	}

	Node* Current = CurrentNode;
	while (Current != pStartNode)
	{
		Path.push_back(Current);
		auto it = cameFrom.find(Current);
		if (it == cameFrom.end()) break;
		Current = it->second;
	}
	Path.push_back(pStartNode);
	std::reverse(Path.begin(), Path.end());

	return Path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->
		GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}
