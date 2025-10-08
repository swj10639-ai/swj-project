#ifndef SATCBS_TRAIN_GRAPH_H
#define SATCBS_TRAIN_GRAPH_H

#include "edge.h"
#include <vector>
#include <unordered_set>
#include <cstdint>


typedef std::vector<std::vector<Edge*>> AdjacencyList;
typedef std::unordered_set<const Edge*> EdgeSet;
typedef std::vector<const Edge*> pathType;





class Graph {
private:

    const Edge& addEdge(int from, int to, Edge* edge);

public:

    AdjacencyList adjList;




    int vertexCount;

    std::vector<Edge*> edges;

    Graph(int vertexCount);
    Graph();


    const Edge* getEdge(int from, int to);

    const Edge& addEdge(int from, int to, int length);


    void addNode();


    void prohibitTransition(int left, int mid, int right);

    EdgeSet reachableEdges(int startFrom, int startTo);
    EdgeSet reachableEdges(const Edge* start);

    size_t reachableInPath(const pathType& path, size_t startIndex);

    //std::vector<std::vector<const Edge *>> A_star(const Edge *start, const Edge *stop,uint32_t maxTimeSteps);

    //int dijkstra(const Edge *start, const Edge *stop);
    //std::vector<std::vector<const Edge*>> A_star1(const Edge* start, const Edge* stop, uint32_t maxTimeSteps,std::vector<_Constraint> _Consts);


    //std::vector<std::vector<const Edge *>>A_star2(const Edge *start, const Edge *stop, uint32_t maxTimeSteps, std::vector<_Constraint> _Consts);
    std::vector<std::vector<const Edge*>> search(const Edge* start, const Edge* stop) ;

    };


#endif //SATCBS_TRAIN_GRAPH_H
