#include "stdafx.h"
#include "EJPS.h"


void Elite::JPS::ScanJumpPoints(NodeRecord& nodeRec, GridTerrainNode* pGoalNode, list<NodeRecord>& openList, std::map<GridTerrainNode*, NodeRecord>& closed_list)
{
	if (nodeRec.xDir == 0)
	{
		VerticalScan(nodeRec, pGoalNode, openList);
	}
	else if (nodeRec.yDir == 0)
	{
		HorizontalScan(nodeRec, pGoalNode, openList);
	}
	else
	{
		DiagonalScan(nodeRec, pGoalNode, openList, closed_list);
	}
}

std::vector<Elite::GridTerrainNode*> Elite::JPS::FindPath(GridTerrainNode* pStartNode, GridTerrainNode* pGoalNode)
{
	std::list<NodeRecord> openList;
	std::map<GridTerrainNode*, NodeRecord> closedList;
	NodeRecord start{};
	start.pNode = pStartNode;
	start.costSoFar = 0;
	start.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);
	start.pParent = nullptr;

	//setup start
	for (int x{ -1 }; x <= 1; ++x)
	{
		for (int y{ -1 }; y <= 1; ++y)
		{
			start.xDir = x;
			start.yDir = y;
			openList.push_back(start);
		}
	}

	//begin algorithm

	while (!openList.empty())
	{
		auto nextRecordIt = std::min_element(openList.begin(), openList.end());
		auto nextRecord = *nextRecordIt;
		openList.erase(nextRecordIt);
		PushNodeToClosedListIfBetterHeuristic(nextRecord.pNode, nextRecord, closedList);

		//check if goal found
		if (nextRecord.pNode == pGoalNode)
		{
			break;
		}

		ScanJumpPoints(nextRecord, pGoalNode, openList, closedList);
	}

	//check if goal found
	if (closedList.find(pGoalNode) == closedList.end())
	{
		return {};
	}
	std::vector<GridTerrainNode*> path;
	GridTerrainNode* currentNode = pGoalNode;

	while (currentNode != pStartNode)
	{
		path.push_back(currentNode);

		auto currentNodeRec = closedList.at(currentNode);

		int fromIdx = currentNode->GetIndex();
		GridTerrainNode* nextNode = currentNodeRec.pParent;
		int toIdx = nextNode->GetIndex();
		//determine direction
		int xDir{}, yDir{};

		int toRow = m_pGraph->GetRow(toIdx);
		int fromRow = m_pGraph->GetRow(fromIdx);

		int toCol = m_pGraph->GetCol(toIdx);
		int fromCol = m_pGraph->GetCol(fromIdx);
		if (toCol - fromCol > 0)
			xDir = 1;
		else if (toCol - fromCol < 0)
			xDir = -1;
		if (toRow - fromRow > 0)
			yDir = 1;
		else if (toRow - fromRow < 0)
			yDir = -1;


		int nextIdx = m_pGraph->GetNextIndex(fromIdx, xDir, yDir);
		while (nextIdx != toIdx)
		{
			path.push_back(m_pGraph->GetNode(nextIdx));
			nextIdx = m_pGraph->GetNextIndex(nextIdx, xDir, yDir);
		}
		currentNode = nextNode;

	}
	std::reverse(path.begin(), path.end());
	return path;

}

bool Elite::JPS::HorizontalScan(NodeRecord start, GridTerrainNode* pGoalNode, std::list<NodeRecord>& openList)
{
	//Get next horizontal node
	int startIndex = start.pNode->GetIndex();
	int nextIndex = startIndex + start.xDir;

	if (!m_pGraph->IsValidIndex(nextIndex) || IsWaterNode(nextIndex))
		return false;

	GraphConnection* pCon = m_pGraph->GetConnection(startIndex, nextIndex);
	if (pCon == nullptr || !pCon->IsValid())
		return false;
	float conCost = pCon->GetCost();
	bool nodeFound{ false };
	while (true)
	{
		//If no next node
		if (!m_pGraph->IsValidIndex(nextIndex))
			return false;
		if (IsWaterNode(nextIndex))
			return false;

		NodeRecord nextNodeRec{};
		nextNodeRec.pNode = m_pGraph->GetNode(nextIndex);
		nextNodeRec.costSoFar = start.costSoFar + conCost;
		nextNodeRec.estimatedTotalCost = nextNodeRec.costSoFar + GetHeuristicCost(nextNodeRec.pNode, pGoalNode);
		nextNodeRec.xDir = start.xDir;
		nextNodeRec.pParent = start.pNode;

		//if next node is goal node
		if (nextNodeRec.pNode == pGoalNode)
		{
			nextNodeRec.yDir = 0;
			openList.push_back(nextNodeRec);
			return true;
		}

		//if next node is interesting

		//down
		int obstacleIndex = m_pGraph->GetIndexBelow(nextIndex);
		int freeIndex = obstacleIndex + start.xDir;

		if (m_pGraph->IsValidIndex(freeIndex) && m_pGraph->IsValidIndex(obstacleIndex))
		{

			if (!IsWaterNode(freeIndex) && IsWaterNode(obstacleIndex))
			{
				nextNodeRec.yDir = -1;
				openList.push_front(nextNodeRec);
				nodeFound = true;
			}
		}
		//up
		obstacleIndex = m_pGraph->GetIndexAbove(nextIndex);
		freeIndex = obstacleIndex + start.xDir;
		if (m_pGraph->IsValidIndex(freeIndex) && m_pGraph->IsValidIndex(obstacleIndex))
		{
			if (!IsWaterNode(freeIndex) && IsWaterNode(obstacleIndex))
			{
				nextNodeRec.yDir = 1;
				openList.push_front(nextNodeRec);
				nodeFound = true;
			}
		}

		//repeat
		startIndex = nextIndex;
		nextIndex = startIndex + start.xDir;
		if (!m_pGraph->IsValidIndex(nextIndex))
			return nodeFound;
		if (IsWaterNode(nextIndex))
			return nodeFound;
		auto pCon = m_pGraph->GetConnection(startIndex, nextIndex);
		if (!pCon || !pCon->IsValid())
			return nodeFound;
		conCost += pCon->GetCost();
	}
	return nodeFound;
}

bool Elite::JPS::VerticalScan(NodeRecord start, GridTerrainNode* pGoalNode, std::list<NodeRecord>& openList)
{
	//Get next vertical node
	int startIndex = start.pNode->GetIndex();
	int nextIndex{};
	if (start.yDir == 1)
	{
		nextIndex = m_pGraph->GetIndexAbove(startIndex);
	}
	else if (start.yDir == -1)
	{
		nextIndex = m_pGraph->GetIndexBelow(startIndex);
	}

	if (!m_pGraph->IsValidIndex(nextIndex) || IsWaterNode(nextIndex))
		return false;
	GraphConnection* pCon = m_pGraph->GetConnection(startIndex, nextIndex);

	if (pCon == nullptr || !pCon->IsValid())
		return false;
	float conCost = pCon->GetCost();
	bool nodeFound = false;
	while (true)
	{
		//If no right node
		if (pCon == nullptr || !pCon->IsValid())
		{
			return nodeFound;
		}

		if (IsWaterNode(nextIndex))
			return nodeFound;

		NodeRecord nextNodeRec{};
		nextNodeRec.pNode = m_pGraph->GetNode(nextIndex);
		nextNodeRec.costSoFar = start.costSoFar + conCost;
		nextNodeRec.estimatedTotalCost = nextNodeRec.costSoFar + GetHeuristicCost(nextNodeRec.pNode, pGoalNode);
		nextNodeRec.pParent = start.pNode;
		nextNodeRec.yDir = start.yDir;

		if (nextNodeRec.pNode == pGoalNode)
		{
			nextNodeRec.xDir = 0;
			openList.push_back(nextNodeRec);
			return true;
		}

		//if node is interesting
		//right
		int obstacleIndex = nextIndex + 1;
		int freeIndex{};
		if (start.yDir == 1)
		{
			freeIndex = m_pGraph->GetIndexAbove(obstacleIndex);
		}
		else if (start.yDir == -1)
		{
			freeIndex = m_pGraph->GetIndexBelow(obstacleIndex);
		}
		if (m_pGraph->IsValidIndex(obstacleIndex) && m_pGraph->IsValidIndex(freeIndex))
		{

			if (!IsWaterNode(freeIndex) && IsWaterNode(obstacleIndex))
			{
				nextNodeRec.xDir = 1;
				openList.push_front(nextNodeRec);
				nodeFound = true;
			}
		}

		//left
		obstacleIndex = nextIndex - 1;
		if (start.yDir == 1)
		{
			freeIndex = m_pGraph->GetIndexAbove(obstacleIndex);
		}
		else if (start.yDir == -1)
		{
			freeIndex = m_pGraph->GetIndexBelow(obstacleIndex);
		}

		if (m_pGraph->IsValidIndex(obstacleIndex) && m_pGraph->IsValidIndex(freeIndex))
		{
			if (!IsWaterNode(freeIndex) && IsWaterNode(obstacleIndex))
			{
				nextNodeRec.xDir = -1;
				openList.push_front(nextNodeRec);
				nodeFound = true;
			}
		}

		//repeat
		startIndex = nextIndex;
		if (start.yDir == 1)
		{
			nextIndex = m_pGraph->GetIndexAbove(startIndex);
		}
		else if (start.yDir == -1)
		{
			nextIndex = m_pGraph->GetIndexBelow(nextIndex);
		}

		if (!m_pGraph->IsValidIndex(nextIndex))
			return nodeFound;
		if (IsWaterNode(nextIndex))
			return nodeFound;

		pCon = m_pGraph->GetConnection(startIndex, nextIndex);
		if (pCon == nullptr || !pCon->IsValid())
		{
			return nodeFound;
		}
		conCost += pCon->GetCost();
	}
	return nodeFound;
}

void Elite::JPS::DiagonalScan(NodeRecord start, GridTerrainNode* pGoalNode, std::list<NodeRecord>& openList, std::map<GridTerrainNode*, NodeRecord>& closed_list)
{
	NodeRecord currentNode = start;
	bool nodeFound = false;

	while (true)
	{
		NodeRecord nextNodeRec;
		if (!GetNextDiagonalNodeRecord(currentNode, nextNodeRec))
			return;
		nextNodeRec.pParent = start.pNode;
		nextNodeRec.estimatedTotalCost = nextNodeRec.costSoFar + GetHeuristicCost(nextNodeRec.pNode, pGoalNode);

		//if goal node
		if (nextNodeRec.pNode == pGoalNode)
		{
			openList.push_back(nextNodeRec);
			return;
		}

		//check if interesting node
		//check vertical
		int nextIndex = nextNodeRec.pNode->GetIndex();
		int obstacleIndex = nextIndex - start.xDir;
		int freeIndex{};
		if (start.yDir == 1)
		{
			freeIndex = m_pGraph->GetIndexAbove(obstacleIndex);
		}
		else if (start.yDir == -1)
		{
			freeIndex = m_pGraph->GetIndexBelow(obstacleIndex);
		}
		if (m_pGraph->IsValidIndex(freeIndex) && m_pGraph->IsValidIndex(obstacleIndex))
		{
			if (!IsWaterNode(freeIndex) && IsWaterNode(obstacleIndex))
			{
				if (closed_list.find(m_pGraph->GetNode(freeIndex)) != closed_list.end())
					return;
				NodeRecord newNode = nextNodeRec;
				newNode.xDir = -start.xDir;
				newNode.yDir = start.yDir;
				openList.push_front(newNode);
				nodeFound = true;
			}
		}

		//check horizontal
		if (start.yDir == 1)
		{
			obstacleIndex = m_pGraph->GetIndexBelow(nextIndex);
		}
		else if (start.yDir == -1)
		{
			obstacleIndex = m_pGraph->GetIndexAbove(nextIndex);
		}
		freeIndex = obstacleIndex + start.xDir;

		if (m_pGraph->IsValidIndex(freeIndex) && m_pGraph->IsValidIndex(obstacleIndex))
		{
			if (!IsWaterNode(freeIndex) && IsWaterNode(obstacleIndex))
			{
				if (closed_list.find(m_pGraph->GetNode(freeIndex)) != closed_list.end())
					return;
				NodeRecord newNode = nextNodeRec;
				newNode.xDir = start.xDir;
				newNode.yDir = -start.yDir;
				openList.push_front(newNode);
				nodeFound = true;
			}
		}

		//hor scan

		if (HorizontalScan(nextNodeRec, pGoalNode, openList))
		{
			PushNodeToClosedListIfBetterHeuristic(nextNodeRec.pNode, nextNodeRec, closed_list);
		}

		//vert scan
		if (VerticalScan(nextNodeRec, pGoalNode, openList))
		{
			PushNodeToClosedListIfBetterHeuristic(nextNodeRec.pNode, nextNodeRec, closed_list);
		}

		//repeat
		currentNode = nextNodeRec;
	}
}


bool Elite::JPS::GetNextDiagonalNodeRecord(NodeRecord prev, NodeRecord& nextRec)
{
	int prevIndex = prev.pNode->GetIndex();
	int nextIndex{};
	if (prev.yDir == 1)
	{
		nextIndex = m_pGraph->GetIndexAbove(prevIndex) + prev.xDir;
	}
	else if (prev.yDir == -1)
	{
		nextIndex = m_pGraph->GetIndexBelow(prevIndex) + prev.xDir;
	}

	if (!m_pGraph->IsValidIndex(nextIndex))
		return false;

	GraphConnection* pCon = m_pGraph->GetConnection(prevIndex, nextIndex);

	if (pCon == nullptr || !pCon->IsValid())
		return false;

	if (pCon == nullptr || !pCon->IsValid())
	{
		return false;
	}

	nextRec.pNode = m_pGraph->GetNode(nextIndex);
	float conCost = m_pGraph->GetConnection(prevIndex, nextIndex)->GetCost();
	nextRec.costSoFar = prev.costSoFar + conCost;
	nextRec.xDir = prev.xDir;
	nextRec.yDir = prev.yDir;

	return true;
}

bool Elite::JPS::PushNodeToClosedListIfBetterHeuristic(GridTerrainNode* node, NodeRecord rec, std::map<GridTerrainNode*, NodeRecord>& closed_list)
{
	if (closed_list.find(node) == closed_list.end())
	{
		closed_list[node] = rec;
		return true;
	}
	else
	{
		if (closed_list[node].estimatedTotalCost >= rec.estimatedTotalCost)
		{
			closed_list[node] = rec;
			return true;
		}
		return false;
	}
	return false;

}
