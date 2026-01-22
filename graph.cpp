#include <algorithm>
#include <set>
#include "graph.h"
#include <map>
#include <limits>
#include <queue>
#include <unordered_map>
#include <cstdint>
#include <iostream>

//有参构造函数。。adjList 被初始化为具有 vertexCount 大小的邻接表，。
Graph::Graph(int vertexCount): adjList(vertexCount), vertexCount(vertexCount){}

Graph::Graph(): adjList(), vertexCount(0) {}//这意味着创建的图没有任何节点，邻接表和边界节点列表都为空。

const Edge& Graph::addEdge(int from, int to, Edge* edge) {
    for(Edge* neighbour: adjList[from]) { // 遍历起始节点的邻接边
        edge->addNeighbour(neighbour)->addNeighbour(edge);// 建立了新边与起始节点的邻接边之间的邻居关系。
    }
    for(Edge* neighbour: adjList[to]) { // 遍历目标节点的邻接边
        edge->addNeighbour(neighbour)->addNeighbour(edge);// 将新边edge与目标节点的邻接边互相添加为邻居。
    }
    //edge->addNeighbour(edge);//每条边的邻居边包括自己
    adjList[from].push_back(edge);     // 将新边添加到起始节点的邻接边列表中
    adjList[to].push_back(edge);   // 将新边添加到起始节点的邻接边列表中
    edges.push_back(edge);    // 将新边添加到图的边列表中
    return *adjList[from].back();  // 返回起始节点的最后一条邻接边的引用
}

//这个重载版本的addEdge函数用于创建一条新的边，并将其添加到图中。它通过调用另一个重载版本的addEdge函数来完成实际的添加操作，并返回添加的边的引用。
const Edge& Graph::addEdge(int from, int to, int length) {
    static int edgeId = 0;    //静态局部变量edgeId用于在每次调用addEdge时生成唯一的边ID。它在第一次调用时初始化为0，之后每次函数调用都会自增。
    Edge *newEdge = new Edge(edgeId++, from, to, length);
    return addEdge(from, to, newEdge);
}

void Graph::addNode() {  //向图(邻接表)中添加一个节点。。
    adjList.push_back(std::vector<Edge*>());  //在adjList中添加一个空的std::vector<Edge*>，用于存储与该节点相关联的边。
    vertexCount++;    //节点计数器vertexCount递增
}

//获取指定起点和终点之间的边（给定起点和终点，这在邻接表中是哪条边，返回该边）
const Edge* Graph::getEdge(int from, int to) {
    for(int i = 0; i < adjList.size(); i++) {
        if(adjList[from][i]->connectsTo(to)) {
            return (adjList[from][i]);
        }
    }
    throw "Edge not found!";
}

void Graph::prohibitTransition(int left, int mid, int right) {// 禁止从左节点到右节点通过中间节点的过渡//这块又不能删除邻接表对应那条边，用于解决道岔，是个小bug哦
    Edge* leftEdge;
    Edge* rightEdge;
    for(Edge* edge: adjList[mid]) {   // 遍历中间顶点的邻接列表，查找与左顶点和右顶点相连的边
        if(edge->connectsTo(left)) leftEdge = edge; // 找到与左顶点相连的边
        if(edge->connectsTo(right)) rightEdge = edge;   // 找到与右顶点相连的边
    }
    leftEdge->neighbours.erase(std::find(leftEdge->neighbours.begin(), leftEdge->neighbours.end(), rightEdge));   // 从左边的邻居列表中移除右边的边
    rightEdge->neighbours.erase(std::find(rightEdge->neighbours.begin(), rightEdge->neighbours.end(), leftEdge)); // 从右边的邻居列表中移除左边的边
}

// 计算从某边一个时间步可达的边集合，给定起点
EdgeSet Graph::reachableEdges(int from, int to) {
    const Edge* start = getEdge(from, to);
    return reachableEdges(start);
}

// 返回从指定起始边在一个时间步可到达的边的集合
EdgeSet Graph::reachableEdges(const Edge* start) {
    EdgeSet reachable;      // 存储可达的边集合
    for (Edge* currentEdge : start->neighbours) {
        reachable.insert(currentEdge);  // 将当前边加入可达边集合
    }
    return reachable;
}

//它计算在能够到达的下一个时间步的索引。（这条最远边是在已经确定的路径上，索引是这条路径每条边的索引）
size_t Graph::reachableInPath(const pathType& path, size_t startIndex) {
    startIndex++;
    return startIndex;
}




//返回从起点到终点的所有路径（使用了深度优先搜索）
std::vector<std::vector<const Edge*>> Graph::search(const Edge* start, const Edge* stop) {
    std::vector<std::vector<const Edge*>> paths;  // 存储所有路径的二维向量
    std::vector<std::pair<int, const Edge*>> dfsStack; // 深度优先搜索栈，存储已发现的边及其深度，以了解需要回溯的次数
    dfsStack.push_back({0, start});   //     作为初始状态压入栈
    std::vector<const Edge*> currPath;    // 当前路径向量
    int dfsDepth = 0;  // 栈深度初始化为0

    while(dfsStack.size() > 0) {  // 当栈非空时循环
        auto [curDepth, currEdge] = dfsStack.back();    // 取出栈顶元素，即当前深度和当前边。。
        dfsStack.pop_back();    // 弹出栈顶元素
        while(dfsDepth > curDepth) {
            currPath.pop_back();  // 回溯路径，直到深度匹配
            dfsDepth--;
        }
        dfsDepth++; // 增加深度表示进入下一层级

        int sharedNode = -1;
        if(currPath.size() > 0) {
            const Edge* prev = currPath.back();    // 获取当前路径的最后一条边
            sharedNode = currEdge->sharedVertex(prev);    // 找到与当前边共享顶点的前一条边

        }
        currPath.push_back(currEdge);   // 将当前边添加到当前路径中

        if(currEdge == stop) { // 如果当前边等于终止边（stop），表示找到了一条完整的路径
            std::vector<const Edge*> finishedPath(currPath);  // 将当前路径复制到一个新的向量中  //创建了一个名为 finishedPath 的 std::vector，并将 currPath 中的元素复制到这个新的向量中。
            paths.push_back(finishedPath);    // 将完整路径添加到路径集合中
            continue;  // 继续下一次循环
        }

        for(const Edge* neighbour: currEdge->neighbours) {  // 遍历当前边的邻边
            bool onPath = false;  //onPath的作用是检查当前邻边是否已经在当前路径中。这个检查是通过遍历当前路径中的边来完成的，如果找到了与邻边相等的边，则将 onPath 设置为 true，表示邻边已经在路径中。

            //cant make sudden turns in a fork
            if(neighbour->connectsTo(sharedNode)) {//检查邻边 neighbour 是否与前一条边 prev 共享顶点。
                continue;   //如果共享顶点，表示当前处于一个分叉点，不能突然转向。（无论是不是刀叉，只要是一个路口，都不能转向，所以这条边舍弃）
            }

            for(const Edge* pathEdge: currPath) {//遍历当前路径 currPath 中的边 pathEdge
                if(pathEdge == neighbour) { //如果找到了相等的边，即neighbour邻边已经在当前路径中，将 onPath 设置为 true，并使用 break 语句跳出循环。
                    onPath = true;
                    break;
                }
            }
            if(onPath)    // 如果邻边已经在当前路径中，则跳过继续下一次循环
                continue;

            dfsStack.push_back({dfsDepth, neighbour});// 将邻边的深度和邻边本身作为新的状态压入栈（之前都是如果，这才是找到了真正的邻边）
        }
    }
    return paths; // 返回存储所有路径的二维向量
}