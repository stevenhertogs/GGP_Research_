#pragma once
#include <framework\EliteAI\EliteGraphs\EGraphNodeTypes.h>
#include <framework\EliteAI\EliteGraphs\EGraphConnectionTypes.h>
#include <framework/EliteAI/EliteGraphs\EGridGraph.h>
#include <list>

namespace Elite
{
	class JPS
	{
	public:
		JPS(GridGraph<GridTerrainNode, GraphConnection>* pGraph, Heuristic hFunction) 
			: m_pGraph(pGraph)
			, m_HeuristicFunction(hFunction)
		{
		}

		// stores the optimal connection to a node and its total costs related to the start and end node of the path
		struct NodeRecord
		{
			GridTerrainNode* pNode = nullptr;
			GridTerrainNode* pParent = nullptr;
			int xDir;
			int yDir;
			float costSoFar = 0.f; // accumulated g-costs of all the connections leading up to this one
			float estimatedTotalCost = 0.f; // f-cost (= costSoFar + h-cost)

			bool operator==(const NodeRecord& other) const
			{
				return pNode == other.pNode
					&& costSoFar == other.costSoFar
					&& estimatedTotalCost == other.estimatedTotalCost;
			};

			bool operator<(const NodeRecord& other) const
			{
				return estimatedTotalCost < other.estimatedTotalCost;
			};
		};



		std::vector<GridTerrainNode*> FindPath(GridTerrainNode* pStartNode, GridTerrainNode* pGoalNode);

	private:
		GridGraph<GridTerrainNode, GraphConnection>* m_pGraph;
		Heuristic m_HeuristicFunction;

		float GetHeuristicCost(GridTerrainNode* pStartNode, GridTerrainNode* pEndNode) const
		{
			Vector2 toDestination = m_pGraph->GetNodePos(pEndNode) - m_pGraph->GetNodePos(pStartNode);
			return m_HeuristicFunction(abs(toDestination.x), abs(toDestination.y));
		}


		void ScanJumpPoints(NodeRecord& nodeRec, GridTerrainNode* pGoalNode, std::list<NodeRecord>& openList, std::map<GridTerrainNode*, NodeRecord>& closed_list);
		bool HorizontalScan(NodeRecord start, GridTerrainNode* pGoalNode, std::list<NodeRecord>& openList);
		bool VerticalScan(NodeRecord start, GridTerrainNode* pGoalNode, std::list<NodeRecord>& openList);
		void DiagonalScan(NodeRecord start, GridTerrainNode* pGoalNode, std::list<NodeRecord>& openList, std::map<GridTerrainNode*, NodeRecord>& closed_list);
		bool GetNextDiagonalNodeRecord(NodeRecord start, NodeRecord& nextRec);
		bool IsWaterNode(int index) { return m_pGraph->GetNode(index)->GetTerrainType() == TerrainType::Water; }
	
	};
}