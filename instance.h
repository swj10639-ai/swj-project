#ifndef SATCBS_TRAIN_INSTANCE_H
#define SATCBS_TRAIN_INSTANCE_H

#include "graph.h"
#include "train.h"
#include <vector>
#include "z3++.h"
#include <tuple>
#include <map>
#include <set>

using namespace z3;

//typedef std::vector<const Edge*> trainPathType; //它是常量指针 Edge 对象的向量。该类型用于表示 Instance 类中火车所经过的边的路径
typedef std::vector<std::vector<expr_vector>> OccupiedType; //expr_vector 是用于存储 Z3 表达式（expr）的向量（用于存储占位变量，即布尔变量）..OccupiedType是一个二维向量，其中的元素是 expr_vector 对象的向量。该类型用于表示在 Instance 类中的被占用变量。





typedef std::tuple<const Edge*, int> tup; // (边的指针, 深度)
typedef std::set<tup> s_tup; // 存储边指针和深度的集合

class trainMdd {
public:
    trainMdd(const int _trainId,const Edge* startEdge, const Edge* stopEdge)
            : start(startEdge), goal(stopEdge),_trainId(_trainId) {}

    // 添加一条路径
    void add_path(const std::vector<const Edge*>& path) {
        if (path.empty()) return;

        // 遍历路径中的每条边并存储在 MDD 中
        for (size_t depth = 0; depth < path.size(); ++depth) {
            const Edge* currEdge = path[depth];
            tup state = std::make_tuple(currEdge, depth); // 使用边的指针和深度

            // 如果当前状态还没有后继集合，则初始化（不覆盖现有的后继）
            if (mdd.find(state) == mdd.end()) {
                mdd[state] = s_tup(); // 初始化后继集合
            }

            // 如果不是最后一条边，添加下一个状态为后继
            if (depth < path.size() - 1) {
                const Edge* nextEdge = path[depth + 1];
                tup next_state = std::make_tuple(nextEdge, depth + 1);
                mdd[state].insert(next_state);  // 添加后继状态
            }

            // 如果不是第一条边，添加前一个状态的后继
            if (depth > 0) {
                const Edge* previousEdge = path[depth - 1];
                tup previous_state = std::make_tuple(previousEdge, depth - 1);
                if (mdd.find(previous_state) == mdd.end()) {
                    mdd[previous_state] = s_tup(); // 初始化后继集合
                }
                mdd[previous_state].insert(state);  // 将当前状态添加为前一状态的后继
            }
        }
    }

    // 获取所有路径
    std::map<tup, s_tup> get_mdd() const {
        return mdd;
    }

    // 获取起始边
    const Edge* get_start() const {
        return start;
    }

    // 获取目标边
    const Edge* get_goal() const {
        return goal;
    }
private:
    const int _trainId;
    const Edge* start; // 起始边
    const Edge* goal;  // 目标边
    std::map<tup, s_tup> mdd; // 存储 MDD 结构
};




class Instance {
public:
    Instance(Graph graph, std::vector<Train> trains, uint32_t maxTimeSteps,
             context *c);//构造函数，用给定的图、列车、最大时间步数和Z3上下文初始化实例。c是上下文对象指针，用于与Z3求解器进行交互
    Instance(const Instance &otherInstance);  //拷贝构造函数

    void creatMDD(std::vector<std::vector<pathType>> &trainPaths, int &makespan,
                  std::vector<std::vector<std::vector<const Edge *>>> &MDD);

    std::vector<std::vector<std::vector<const Edge *>>> MDD; // MDD结构，三维向量，表示每个列车的路径决策图MDD
    std::vector<expr> constraints;    //expr对象的向量，表示实例的布尔约束条件。 //Z3 表达式（expr）

    std::vector<std::unordered_set<const Edge*>> mddSet;
    std::vector<std::vector<std::vector<const Edge *>>> TEG; // TEG结构，三维向量，表示每个列车的TEG图


    std::vector<trainMdd> MDDs;


    //std::vector<Collision> Collisions;  //冲突的集合
    //Constraints _Constraints;
    double time_t = 0;  //测量计算所用时间
    int sumOfCost;

protected:
    Graph graph;  //表示图结构的Graph类对象
    std::vector<Train> trains;    //包含了Train对象的向量，表示实例中的列车。
    uint32_t maxTimeSteps;    //最大时间步数。


    std::vector<std::vector<pathType>> trainPaths;   // 二维向量，表示每个列车的路径。   //pathType 用于表示路径，其中每个元素都是边的指针。//std::vector<pathType>表示列车的几条路径，不止一条


    OccupiedType occupiedVars;    //mapping train -> time -> edge  -----> variable  //occupiedVars：三维向量，表示列车、时间步和边到变量的映射。


    context *c;   //指向Z3库中的context对象的指针。
    z3::solver solver;  //Z3库中的solver类的实例，用于求解优化问题。

    std::unordered_map<int, int> trainMinLength;   //

    std::vector<std::unordered_map<const Edge*, int>> EdgeTime;
    void computeEdgeTime();



    std::vector<std::vector<EdgeSet>> reachableFrom;
    void computePathNextEdge();

    void computeEachCollisionConstraint(Train &train, int &makespan);


    void computeReachableConstraint(Train &train, int &makespan);//4.1计算可达性约束

    void computeCollisionConstraint(Train &train, int &makespan);//4.2计算冲突约束

    bool solve();

    void addConstraintsToSolver();//添加约束到solver求解器


    void printTrainRoute(int &makespan);    //打印列车路径


    //void parseName(const std::string &name, int &trainId, int &time, int &edgeId);//unsat core 的string的解析
    void parseName(const std::string &name, int &trainId1, int &time1, int &edgeId1, int &trainId2, int &time2,
                   int &edgeId2);//unsat core 的string的解析

    //std::vector<std::vector<const Edge *>> A_star1(const Edge *start, const Edge *stop, uint32_t maxTimeSteps, std::vector<_Constraint> _Consts);
    //int dijskra(const Edge* start, const Edge* stop);
    void computeEachOtherCollisionConstraint(Train &train, int &makespan);//4.3计算相向而行直接相撞的约束
    void computeCrossCollisionConstraint(Train &train, int &makespan,bool _nextMakespan);  //4.4计算交叉碰撞
};

#endif //SATCBS_TRAIN_INSTANCE_H
