#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
                                                    NavGraph* const pNavGraph,
                                                    std::vector<FVector2D>& debugNodePositions,
                                                    std::vector<NavLine>& debugPortals)
{
	//Create the path to return
	std::vector<FVector2D> finalPath{};
	auto pNavPoly = pNavGraph->GetNavPolygon();

	//Get the start and endTriangle
	auto pStartTri = pNavPoly->GetTriangleAtPosition(startPos, true);
	auto pEndTri = pNavPoly->GetTriangleAtPosition(endPos, true);

	//We have valid start/end triangles, and they are not the same
	if (!pStartTri || !pEndTri) return finalPath;
	// If they are in the same triangle, just go straight there
	if (pStartTri == pEndTri)
	{
		finalPath.push_back(startPos);
		finalPath.push_back(endPos);
		return finalPath;
	}

	//=> Start looking for a path
	//Copy the graph
	auto pCloneGraph = pNavGraph->Clone();

	//Create Extra node for the Start Node (Agent's position
	int startNodeId = static_cast<int>(pCloneGraph->GetNodes().size());
	auto pStartNode = std::make_unique<NavGraphNode>(startPos, -1);
	pCloneGraph->AddNode(std::move(pStartNode));

	for (auto& edge : pStartTri->GetEdges())
	{
		auto edgeIdxOpt = pNavPoly->FindEdgeIndex(edge);
		if (edgeIdxOpt.has_value())
		{
			int nodeIdOnEdge = pCloneGraph->GetNodeIdFromEdgeIndex(edgeIdxOpt.value());
			if (nodeIdOnEdge != -1)
			{
				float dist = FVector2D::Distance(startPos, pCloneGraph->GetNode(nodeIdOnEdge)->GetPosition());
				auto pConn = std::make_unique<Connection>(startNodeId, nodeIdOnEdge);
				pConn->SetWeight(dist);
				pCloneGraph->AddConnection(std::move(pConn));
			}
		}
	}

	//Create extra node for the endNode
	int endNodeId = static_cast<int>(pCloneGraph->GetNodes().size());
	auto pEndNode = std::make_unique<NavGraphNode>(endPos, -1);
	pCloneGraph->AddNode(std::move(pEndNode));

	for (auto& edge : pEndTri->GetEdges())
	{
		auto edgeIdxOpt = pNavPoly->FindEdgeIndex(edge);
		if (edgeIdxOpt.has_value())
		{
			int nodeIdOnEdge = pCloneGraph->GetNodeIdFromEdgeIndex(edgeIdxOpt.value());
			if (nodeIdOnEdge != -1)
			{
				float dist = FVector2D::Distance(endPos, pCloneGraph->GetNode(nodeIdOnEdge)->GetPosition());
				auto pConn = std::make_unique<Connection>(nodeIdOnEdge, endNodeId);
				pConn->SetWeight(dist);
				pCloneGraph->AddConnection(std::move(pConn));
			}
		}
	}

	//Run A star on new graph
	AStar pathfinder = AStar(pCloneGraph.get(), HeuristicFunctions::Euclidean);

	std::vector<Node*> nodePath = pathfinder.FindPath(pCloneGraph->GetNode(startNodeId).get(),
	                                                  pCloneGraph->GetNode(endNodeId).get());

	//Debug Visualisation
	for (auto pNode : nodePath)
	{
		finalPath.push_back(pNode->GetPosition());
		debugNodePositions.push_back(pNode->GetPosition());
	}

	// Extra: Run optimizer on new graph (First check if everything works without SSFA!)
	debugPortals = SSFA::FindPortals(nodePath, *pNavGraph->GetNavPolygon());
	finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());

	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
                                                    NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}
