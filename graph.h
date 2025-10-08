#ifndef SATCBS_TRAIN_GRAPH_H
#define SATCBS_TRAIN_GRAPH_H

#include "edge.h"
#include <vector>
#include <unordered_set>
#include <cstdint>


typedef std::vector<std::vector<Edge*>> AdjacencyList;  //图的邻接表。二维向量。外层向量的每个元素代表一个顶点（即from），内层向量存储从该顶点出发的边的指针。因此，AdjacencyList 可以用来表示图的结构，通过遍历邻接表的元素可以获取每个顶点的相邻边。
typedef std::unordered_set<const Edge*> EdgeSet;    //无序集合，存储了指向 const Edge* 的指针。通过 EdgeSet 可以方便地进行边的查找、插入和删除操作。
typedef std::vector<const Edge*> pathType;  //pathType 用于表示路径，其中每个元素都是指向边的指针。通过 pathType 可以按顺序记录路径上的边，以便进行路径的处理和分析。





class Graph {
private:

    const Edge& addEdge(int from, int to, Edge* edge);    // 添加一条边到图中，并返回对应的边对象的引用

public:

    AdjacencyList adjList;     // 邻接表，用于存储图的边和相邻节点信息




    int vertexCount;  // 图中节点的数量

    std::vector<Edge*> edges; // 存储图中的所有边对象

    Graph(int vertexCount);   // 构造函数，创建具有指定节点数量的图
    Graph();  // 构造函数，创建空图

    // 返回邻接表从“from”到“to”的边，如果不存在则返回空指针
    const Edge* getEdge(int from, int to);

    const Edge& addEdge(int from, int to, int length);// 添加一条边到图中，并返回对应的边对象的引用


    void addNode();   // 添加一个节点到图中，默认不是边界节点


    void prohibitTransition(int left, int mid, int right);// 禁止从左节点到右节点通过中间节点的过渡

    EdgeSet reachableEdges(int startFrom, int startTo);    // 返回从指定起点（起点由from和to指定）在一个时间步内可到达的边的集合
    EdgeSet reachableEdges(const Edge* start);  // 返回从指定起始边在一个时间步可到达的边的集合

    size_t reachableInPath(const pathType& path, size_t startIndex);

    //std::vector<std::vector<const Edge *>> A_star(const Edge *start, const Edge *stop,uint32_t maxTimeSteps);

    //int dijkstra(const Edge *start, const Edge *stop);
    //std::vector<std::vector<const Edge*>> A_star1(const Edge* start, const Edge* stop, uint32_t maxTimeSteps,std::vector<_Constraint> _Consts);


    //std::vector<std::vector<const Edge *>>A_star2(const Edge *start, const Edge *stop, uint32_t maxTimeSteps, std::vector<_Constraint> _Consts);
    std::vector<std::vector<const Edge*>> search(const Edge* start, const Edge* stop) ;

    };


#endif //SATCBS_TRAIN_GRAPH_H
