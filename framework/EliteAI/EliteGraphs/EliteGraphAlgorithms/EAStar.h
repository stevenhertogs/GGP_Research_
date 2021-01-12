#pragma once

namespace Elite
{
	template <class T_NodeType, class T_ConnectionType>
	class AStar
	{
	public:
		AStar(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction);

		// stores the optimal connection to a node and its total costs related to the start and end node of the path
		struct NodeRecord
		{
			T_NodeType* pNode = nullptr;
			T_ConnectionType* pConnection = nullptr;
			float costSoFar = 0.f; // accumulated g-costs of all the connections leading up to this one
			float estimatedTotalCost = 0.f; // f-cost (= costSoFar + h-cost)

			bool operator==(const NodeRecord& other) const
			{
				return pNode == other.pNode
					&& pConnection == other.pConnection
					&& costSoFar == other.costSoFar
					&& estimatedTotalCost == other.estimatedTotalCost;
			};

			bool operator<(const NodeRecord& other) const
			{
				return estimatedTotalCost < other.estimatedTotalCost;
			};
		};

		std::vector<T_NodeType*> FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode);

	private:
		float GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const;

		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
		Heuristic m_HeuristicFunction;
	};

	template <class T_NodeType, class T_ConnectionType>
	AStar<T_NodeType, T_ConnectionType>::AStar(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction)
		: m_pGraph(pGraph)
		, m_HeuristicFunction(hFunction)
	{
	}

	template <class T_NodeType, class T_ConnectionType>
	std::vector<T_NodeType*> AStar<T_NodeType, T_ConnectionType>::FindPath(T_NodeType* pStartNode, T_NodeType* pGoalNode)
	{
		std::list<NodeRecord> openList;
		std::map<T_NodeType*,NodeRecord> closedList;
		NodeRecord startRecord{};
		startRecord.pNode = pStartNode;
		openList.push_back(startRecord);
		while (!openList.empty())
		{
			auto nextRecordIt = std::min_element(openList.begin(), openList.end());
			NodeRecord nextRecord = *nextRecordIt;
			openList.erase(nextRecordIt);
			closedList[nextRecord.pNode] = nextRecord;

			if (nextRecord.pNode == pGoalNode)
			{
				break;
			}
			else
			{
				for (T_ConnectionType* con : m_pGraph->GetNodeConnections(nextRecord.pNode->GetIndex()))
				{
					NodeRecord newRecord{};
					newRecord.pNode = m_pGraph->GetNode(con->GetTo());
					newRecord.pConnection = con;
					newRecord.costSoFar = nextRecord.costSoFar + con->GetCost();
					newRecord.estimatedTotalCost = newRecord.costSoFar + GetHeuristicCost(newRecord.pNode, pGoalNode);
					if (closedList.find(newRecord.pNode) != closedList.end())
					{
						if (closedList[newRecord.pNode].estimatedTotalCost > newRecord.estimatedTotalCost)
						{
							closedList[newRecord.pNode] = newRecord;
						}
						continue;
					}
					auto it = std::find_if(openList.begin(), openList.end(),[&newRecord](const NodeRecord& n) {return n.pNode == newRecord.pNode; });
					if (it != openList.end())
					{
						if (it->estimatedTotalCost > newRecord.estimatedTotalCost)
						{
							openList.remove(*it);
							openList.push_back(newRecord);
						}
					}
					else
					{
						openList.push_back(newRecord);
					}
					
				}
				
			}
		}

		std::vector<T_NodeType*> path;
		T_NodeType* currentNode = pGoalNode;
		while (currentNode != pStartNode)
		{
			path.push_back(currentNode);
			auto con = closedList[currentNode].pConnection;
			if (con == nullptr)
			{
				return {};
			}
			currentNode = m_pGraph->GetNode(con->GetFrom());
		}
		std::reverse(path.begin(), path.end());
		return path;
	}

	template <class T_NodeType, class T_ConnectionType>
	float Elite::AStar<T_NodeType, T_ConnectionType>::GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const
	{
		Vector2 toDestination = m_pGraph->GetNodePos(pEndNode) - m_pGraph->GetNodePos(pStartNode);
		return m_HeuristicFunction(abs(toDestination.x), abs(toDestination.y));
	}
}