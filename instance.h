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

typedef std::vector<std::vector<expr_vector>> OccupiedType;



typedef std::tuple<const Edge*, int> tup;
typedef std::set<tup> s_tup;

class trainMdd {
public:
    trainMdd(const int _trainId,const Edge* startEdge, const Edge* stopEdge)
            : start(startEdge), goal(stopEdge),_trainId(_trainId) {}

    void add_path(const std::vector<const Edge*>& path) {
        if (path.empty()) return;

        for (size_t depth = 0; depth < path.size(); ++depth) {
            const Edge* currEdge = path[depth];
            tup state = std::make_tuple(currEdge, depth);

            if (mdd.find(state) == mdd.end()) {
                mdd[state] = s_tup(); // 初始化后继集合
            }

            if (depth < path.size() - 1) {
                const Edge* nextEdge = path[depth + 1];
                tup next_state = std::make_tuple(nextEdge, depth + 1);
                mdd[state].insert(next_state);
            }

            if (depth > 0) {
                const Edge* previousEdge = path[depth - 1];
                tup previous_state = std::make_tuple(previousEdge, depth - 1);
                if (mdd.find(previous_state) == mdd.end()) {
                    mdd[previous_state] = s_tup();
                }
                mdd[previous_state].insert(state);
            }
        }
    }

    std::map<tup, s_tup> get_mdd() const {
        return mdd;
    }

    const Edge* get_start() const {
        return start;
    }

    const Edge* get_goal() const {
        return goal;
    }
private:
    const int _trainId;
    const Edge* start;
    const Edge* goal;
    std::map<tup, s_tup> mdd;
};




class Instance {
public:
    Instance(Graph graph, std::vector<Train> trains, uint32_t maxTimeSteps,
             context *c);
    Instance(const Instance &otherInstance);

    void creatMDD(std::vector<std::vector<pathType>> &trainPaths, int &makespan,
                  std::vector<std::vector<std::vector<const Edge *>>> &MDD);

    std::vector<std::vector<std::vector<const Edge *>>> MDD;
    std::vector<expr> constraints;

    std::vector<std::unordered_set<const Edge*>> mddSet;
    std::vector<std::vector<std::vector<const Edge *>>> TEG;


    std::vector<trainMdd> MDDs;



    double time_t = 0;
    int sumOfCost;

protected:
    Graph graph;
    std::vector<Train> trains;
    uint32_t maxTimeSteps;


    std::vector<std::vector<pathType>> trainPaths;

    OccupiedType occupiedVars;

    context *c;
    z3::solver solver;

    std::unordered_map<int, int> trainMinLength;

    std::vector<std::unordered_map<const Edge*, int>> EdgeTime;
    void computeEdgeTime();



    std::vector<std::vector<EdgeSet>> reachableFrom;
    void computePathNextEdge();

    void computeEachCollisionConstraint(Train &train, int &makespan);


    void computeReachableConstraint(Train &train, int &makespan);

    void computeCollisionConstraint(Train &train, int &makespan);

    bool solve();

    void addConstraintsToSolver();


    void printTrainRoute(int &makespan);


    //void parseName(const std::string &name, int &trainId, int &time, int &edgeId);//unsat core 的string的解析
    void parseName(const std::string &name, int &trainId1, int &time1, int &edgeId1, int &trainId2, int &time2,
                   int &edgeId2);

    //std::vector<std::vector<const Edge *>> A_star1(const Edge *start, const Edge *stop, uint32_t maxTimeSteps, std::vector<_Constraint> _Consts);
    //int dijskra(const Edge* start, const Edge* stop);
    void computeEachOtherCollisionConstraint(Train &train, int &makespan);
    void computeCrossCollisionConstraint(Train &train, int &makespan,bool _nextMakespan);
};

#endif //SATCBS_TRAIN_INSTANCE_H
