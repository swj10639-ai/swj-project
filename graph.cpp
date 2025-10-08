#include <algorithm>
#include <set>
#include "graph.h"
#include <map>
#include <limits>
#include <queue>
#include <unordered_map>
#include <cstdint>
#include <iostream>


Graph::Graph(int vertexCount): adjList(vertexCount), vertexCount(vertexCount){}

Graph::Graph(): adjList(), vertexCount(0) {}

const Edge& Graph::addEdge(int from, int to, Edge* edge) {
    for(Edge* neighbour: adjList[from]) {
        edge->addNeighbour(neighbour)->addNeighbour(edge);
    }
    for(Edge* neighbour: adjList[to]) { // 遍历目标节点的邻接边
        edge->addNeighbour(neighbour)->addNeighbour(edge);
    }
    //edge->addNeighbour(edge);
    adjList[from].push_back(edge);
    adjList[to].push_back(edge);
    edges.push_back(edge);
    return *adjList[from].back();
}

const Edge& Graph::addEdge(int from, int to, int length) {
    static int edgeId = 0;
    Edge *newEdge = new Edge(edgeId++, from, to, length);
    return addEdge(from, to, newEdge);
}

void Graph::addNode() {
    adjList.push_back(std::vector<Edge*>());
    vertexCount++;
}

const Edge* Graph::getEdge(int from, int to) {
    for(int i = 0; i < adjList.size(); i++) {
        if(adjList[from][i]->connectsTo(to)) {
            return (adjList[from][i]);
        }
    }
    throw "Edge not found!";
}

void Graph::prohibitTransition(int left, int mid, int right) {
    Edge* leftEdge;
    Edge* rightEdge;
    for(Edge* edge: adjList[mid]) {
        if(edge->connectsTo(left)) leftEdge = edge;
        if(edge->connectsTo(right)) rightEdge = edge;
    }
    leftEdge->neighbours.erase(std::find(leftEdge->neighbours.begin(), leftEdge->neighbours.end(), rightEdge));
    rightEdge->neighbours.erase(std::find(rightEdge->neighbours.begin(), rightEdge->neighbours.end(), leftEdge));
}


EdgeSet Graph::reachableEdges(int from, int to) {
    const Edge* start = getEdge(from, to);
    return reachableEdges(start);
}


EdgeSet Graph::reachableEdges(const Edge* start) {
    EdgeSet reachable;      // 存储可达的边集合
    for (Edge* currentEdge : start->neighbours) {
        reachable.insert(currentEdge);  // 将当前边加入可达边集合
    }
    return reachable;
}


size_t Graph::reachableInPath(const pathType& path, size_t startIndex) {
    startIndex++;
    return startIndex;
}





std::vector<std::vector<const Edge*>> Graph::search(const Edge* start, const Edge* stop) {
    std::vector<std::vector<const Edge*>> paths;
    std::vector<std::pair<int, const Edge*>> dfsStack;
    dfsStack.push_back({0, start});
    std::vector<const Edge*> currPath;
    int dfsDepth = 0;

    while(dfsStack.size() > 0) {
        auto [curDepth, currEdge] = dfsStack.back();
        dfsStack.pop_back();
        while(dfsDepth > curDepth) {
            currPath.pop_back();
            dfsDepth--;
        }
        dfsDepth++;

        int sharedNode = -1;
        if(currPath.size() > 0) {
            const Edge* prev = currPath.back();
            sharedNode = currEdge->sharedVertex(prev);

        }
        currPath.push_back(currEdge);

        if(currEdge == stop) {
            std::vector<const Edge*> finishedPath(currPath);
            paths.push_back(finishedPath);
            continue;
        }

        for(const Edge* neighbour: currEdge->neighbours) {
            bool onPath = false;

            //cant make sudden turns in a fork
            if(neighbour->connectsTo(sharedNode)) {
                continue;
            }

            for(const Edge* pathEdge: currPath) {
                if(pathEdge == neighbour) {
                    onPath = true;
                    break;
                }
            }
            if(onPath)
                continue;

            dfsStack.push_back({dfsDepth, neighbour});
        }
    }
    return paths; // 返回存储所有路径的二维向量
}