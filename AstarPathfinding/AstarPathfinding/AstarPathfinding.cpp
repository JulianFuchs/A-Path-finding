// AstarPathfinding.cpp : Defines the entry point for the console application.
//

// comment out the following line if on Linux
#include "stdafx.h"
#include <iostream>
#include <vector>
//#include <concurrent_priority_queue.h>
#include <queue>
#include <math.h> 
#include <algorithm>

using namespace std;

int FindPath(const int nStartX, const int nStartY,
	const int nTargetX, const int nTargetY,
	const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
	int* pOutBuffer, const int nOutBufferSize);

struct node {
	int id;
	int fValue;
	int predecessor;
	int distance;
};

// needs to be reversed, since priority queue per default has largest element on top.
// But we want the smallest fValue on top.
bool operator <(const node& x, const node& y) {
	//this would be the real smaller:
	////return x.fValue > y.fValue;
	return x.fValue > y.fValue;
}

int findX(int id);
int findY(int id);
int findId(int x, int y);
int computeHeuristic(int x, int y);
void investigate(node n);
void findShortestPath(int dist, int* pOutBuffer, const int nOutBufferSize);

//variables
int mapWidth, mapHeight;

const unsigned char* map;
vector<int> distances;
vector<int> predecessors;

priority_queue<node> prioQueue;

int targetId, startId;
int targetX, targetY;
bool targetFound = false;

// uncomment main to submit to kattis
int main()
{
	cout << "A* Pathfinding" << endl;

	// first example:
	unsigned char pMap[] = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
	int pOutBuffer[12];
	int shortestDist = FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12);

	//// second example:
	//unsigned char pMap[] = { 0, 0, 1, 0, 1, 1, 1, 0, 1 };
	//int pOutBuffer[7];
	//int shortestDist = FindPath(2, 0, 0, 2, pMap, 3, 3, pOutBuffer, 7);

	cout << "Found shortest distance: " << shortestDist << endl;
	cout << "now printing shortest path... " << endl;

	for (int i = 0; i < shortestDist; i++) {

		cout << pOutBuffer[i] << ", ";
	}

	cout << endl;
	cin.get();

    return 0;
}


int FindPath(const int nStartX, const int nStartY,
	const int nTargetX, const int nTargetY,
	const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
	int* pOutBuffer, const int nOutBufferSize)
{
	mapWidth = nMapWidth;
	mapHeight = nMapHeight;
	map = pMap;

	distances.assign(nMapWidth*nMapHeight, -1);
	predecessors.assign(nMapWidth*nMapHeight, -1);
	
	priority_queue<node> p;
	prioQueue = p;

	targetId = findId(nTargetX, nTargetY);
	targetX = nTargetX;
	targetY = nTargetY;

	node startingNode;
	startId = findId(nStartX, nStartY);
	startingNode.id = startId;
	startingNode.fValue = 0;
	startingNode.distance = 0;
	startingNode.predecessor = -1;

	prioQueue.push(startingNode);
	targetFound = false;

	while (!prioQueue.empty() && !targetFound) {
		node n = prioQueue.top();
		prioQueue.pop();
		investigate(n);
	}

	if (distances[targetId] != -1 && distances[targetId] <= nOutBufferSize) {
		findShortestPath(distances[targetId],pOutBuffer, nOutBufferSize);
		return distances[targetId];
	}
	// if we found a shortest path but it's longer than the buffer size, return the size of the path but don't write to the buffer.
	else if (distances[targetId] > nOutBufferSize) {
		return distances[targetId];
	}
	else {
		return -1;
	}
}

int findX(int id) {
	return id % mapWidth;
}

int findY(int id) {
	return floor(id / mapWidth);
}

int findId(int x, int y) {
	return y*mapWidth + x;
}

/*
heuristic is the manhattan distance from point z = (x,y) to the target if there were no obstacles.
The function which the priority queue is sorted for is f() = distance(z)+heuristic(z).
*/
int computeHeuristic(int x, int y) {
	return max(x - targetX, targetX - x) + max(y - targetY, targetY - y);
}

void investigate(node n) {

	// check if the node is the target
	if (n.id == targetId) {
		targetFound = true;
		distances[n.id] = n.distance;
		predecessors[n.id] = n.predecessor;
		return;
	}

	// check wether the node was already visited
	if (distances[n.id] != -1) {
		return;
	}

	// if this is the shortest path from the source to this node:
	int x = findX(n.id);
	int y = findY(n.id);

	// set predecessor and distance
	distances[n.id] = n.distance;
	predecessors[n.id] = n.predecessor;
	
	// visit node on the left
	if (0 <= x - 1) {
		int newId = findId(x - 1, y);
		if (distances[newId] == -1 && map[newId] == 1) {
			node newNode;
			newNode.id = newId;
			newNode.predecessor = n.id;
			newNode.distance = n.distance + 1;
			newNode.fValue = (n.distance + 1) + computeHeuristic(x - 1, y);
			prioQueue.push(newNode);
		}
	}
	// visit node on the right
	if (x + 1 < mapWidth) {
		int newId = findId(x + 1, y);
		if (distances[newId] == -1 && map[newId] == 1) {
			node newNode;
			newNode.id = newId;
			newNode.predecessor = n.id;
			newNode.distance = n.distance + 1;
			newNode.fValue = (n.distance + 1) + computeHeuristic(x + 1, y);
			prioQueue.push(newNode);
		}
	}
	// visit node below
	if (0 <= y - 1) {
		int newId = findId(x, y - 1);
		if (distances[newId] == -1 && map[newId] == 1) {
			node newNode;
			newNode.id = newId;
			newNode.predecessor = n.id;
			newNode.distance = n.distance + 1;
			newNode.fValue = (n.distance + 1) + computeHeuristic(x, y - 1);
			prioQueue.push(newNode);
		}
	}
	if (y + 1 < mapHeight) {
		int newId = findId(x, y + 1);
		if (distances[newId] == -1 && map[newId] == 1) {
			node newNode;
			newNode.id = newId;
			newNode.predecessor = n.id;
			newNode.distance = n.distance + 1;
			newNode.fValue = (n.distance + 1) + computeHeuristic(x, y + 1);
			prioQueue.push(newNode);
		}
	}
	
}

void findShortestPath(int dist, int* pOutBuffer, const int nOutBufferSize) {

	int i = 1;
	int index = targetId;

	while (index != startId) {
		*(pOutBuffer + dist - i) = index;
		index = predecessors[index];
		++i;
	}
}
