#include "BFS.h"

#include <map>
#include <queue>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::queue<Node*> Queue;
	Queue.push(pStartNode);
	//current node, prev node
	std::map<Node*, Node*> Visited;
	Visited.insert({pStartNode, pStartNode});

	while (!Queue.empty())
	{
		const auto Node = Queue.front();
		Queue.pop();

		if (Node->GetId() == pDestinationNode->GetId())
			return Reconstruct_path(Visited, pStartNode, pDestinationNode);

		for (const auto& Connection : pGraph->FindConnectionsFrom(Node->GetId()))
		{
			if (!Visited.contains(pGraph->GetNode(Connection->GetToId()).get()))
			{
				Visited.insert({pGraph->GetNode(Connection->GetToId()).get(), pGraph->GetNode(Connection->GetFromId()).get()});
				Queue.push(pGraph->GetNode(Connection->GetToId()).get());
			}
		}
	}
	return {};
}

std::vector<Node*> BFS::Reconstruct_path(const std::map<Node*, Node*>& Visited, Node* const pStartNode,
                                         Node* const pDestinationNode) const
{
	std::vector<Node*> path;
	auto Current = pDestinationNode;
	
	while (Current->GetId() != pStartNode->GetId())
	{
		path.push_back(Current);
		Current = Visited.find(pGraph->GetNode(Current->GetId()).get())->second;
	}
	
	path.push_back(pStartNode);
	std::reverse(path.begin(), path.end());
	return path;
}
