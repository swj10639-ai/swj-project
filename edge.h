
#ifndef SATCBS_TRAIN_EDGE_H
#define SATCBS_TRAIN_EDGE_H

#include <vector>
#include <string>

class Edge {
    public:
        const int from, to, length ,id;
    // 边的属性：id、起始顶点、目标顶点
        std::vector <Edge*> neighbours;  // 存储与该边相邻的边的指针列表

        Edge(int id, int from, int to, int length=1); // 构造函数，用于初始化边的属性

        int getOtherVertex(int vertex) const;   // 获取边上除给定顶点外的另一个顶点

        int operator[] (int vertex) const; // 重载运算符[]，用于访问边的另一个顶点

        Edge *addNeighbour(Edge *neighbour);   // 添加与该边相邻的边,并返回添加的边的指针
        bool connectsTo(int vertex) const;    // 检查该边是否与给定顶点相连
        int sharedVertex(const Edge *otherEdge) const;    // 获取与另一条边共享的顶点
        bool containsNeighbour(const Edge *otherEdge) const;    //该边的所有邻边是否包含otherEdge

        std::string toString() const; // 将边的属性转换为字符串表示

};


#endif //SATCBS_TRAIN_EDGE_H
