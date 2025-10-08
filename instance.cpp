#include <sstream>
#include "instance.h"
#include <z3++.h>
#include <z3_api.h>
#include <iostream>
#include <set>
#include <regex>
#include <queue>

Instance::Instance(Graph graph, std::vector<Train> trains, uint32_t maxTimeSteps,context *c)
        : graph(graph), trains(trains), maxTimeSteps(maxTimeSteps), trainPaths(trains.size()),c(c), solver(*c),
        occupiedVars(trains.size(), std::vector<expr_vector>(maxTimeSteps, *c))
        {

    std::clock_t c_start = std::clock();

    int start_Makespan=0;
    for (size_t train = 0; train < trains.size(); train++) {
        //trainPaths[train] = graph.A_star(trains[train].start.stopEdge,   // 用A*获取了每辆列车的路径
                                        //trains[train].stops.back().stopEdge,maxTimeSteps);//初始_Constraints为空
        trainPaths[train] = graph.search(trains[train].start.stopEdge,   // 用A*获取了每辆列车的路径
                                         trains[train].stops.back().stopEdge);//初始_Constraints为空
        int pathLength=maxTimeSteps+1;
        for(int path_i=0;path_i<trainPaths[train].size();path_i++) {
            if (trainPaths[train][path_i].size() <pathLength) {
                pathLength = trainPaths[train][path_i].size();
                //std::cout << "路径长度为" << pathLength << std::endl;

            }
        }
        trainMinLength[train]=pathLength;
    }
            sumOfCost=0;
            for (size_t train = 0; train < trains.size(); train++) {
                sumOfCost+=(trainMinLength[train]-1);
                if(trainMinLength[train]-1>start_Makespan){
                    start_Makespan=trainMinLength[train]-1;//Makespan为为路径长度减1
                }
            }



            computePathNextEdge();//计算路径中每条边的下一条边
            computeEdgeTime();//计算路径中每条边的到终边需要多少时间
            for (Train &train : trains) { // 遍历每个列车对象
                trainMdd mdd(train.id, train.start.stopEdge, train.stops.back().stopEdge); // 创建 MDD 实例
                for (const auto& path : trainPaths[train.id]) {
                    mdd.add_path(path); // 将每条路径添加到 MDD 中
                }
                MDDs.push_back(mdd);



            }
    for (int soc = sumOfCost; soc < maxTimeSteps*trains.size(); ++soc) {      //总迭代soc

        int extra_cost=soc-sumOfCost;
        int makespan = start_Makespan+1+extra_cost;

        bool innerLoopCompleted = false;
        bool nextMakespan = false;


        while (true){




            for (Train &train : trains) {
                mddSet.emplace_back();
                auto all_paths = MDDs[train.id].get_mdd();

                for (const auto &entry: all_paths) {
                    const tup &current_state = entry.first;
                    const s_tup &successors = entry.second;

                    const Edge *current_edge = std::get<0>(current_state); // 获取边指针
                    mddSet[train.id].insert(current_edge);


                    int depth = std::get<1>(current_state);



                }
            }





            TEG.clear();
            // 初始化MDD
            TEG.resize(trains.size()); // 根据列车数量调整第一维大小
            for (size_t train = 0; train < trains.size(); train++) {
                TEG[train].resize(makespan); // 根据最大时间步数调整第二维大小
            }
            for (size_t train = 0; train < trains.size(); train++) {

                //TEG[train][0].push_back(trains[train].start.stopEdge);
                for (size_t time = 0; time < makespan; time++) {
                    if(time==0){
                        TEG[train][time].push_back(trains[train].start.stopEdge);
                        continue;
                    }

                    for (const auto& mddEgge : mddSet[train]) {
                        if((EdgeTime[train][mddEgge]+time)>(makespan-1))
                            continue;
                        bool isNextEdge = false;
                        for (int u = 0; u < TEG[train][time-1].size(); ++u) {// 对于当前MDD时间步中的每个边

                            for (const Edge *reachable: reachableFrom[train][TEG[train][time - 1][u]->id]) {
                                if (reachable == mddEgge) {
                                    TEG[train][time].push_back(mddEgge);
                                    isNextEdge = true;
                                    break;
                                }
                            }
                            if (isNextEdge) break;
                        }

                        //TEG[train][time].push_back(mddEgge);
                    }
                }
            }





            solver.reset();


            for (size_t train = 0; train < trains.size(); train++) {
                //occupiedVars.clear();
                //occupiedVars(trains.size(), std::vector<expr_vector>(maxTimeSteps, *c));
                for (size_t time = trains[train].start.arrivalTime; time < TEG[train].size(); time++) {
                    //std::cout << "curr:"<<time<<std::endl;
                    occupiedVars[train][time] = expr_vector(*c);

                    for (int v=0;v< TEG[train][time].size();++v) {
                        std::stringstream oVarName;
                        oVarName << "occupies_" << train << "_" << time << "_" << TEG[train][time][v];  ////要不要加->id
                        occupiedVars[train][time].push_back(
                                c->bool_const(oVarName.str().c_str()));


                    }

                }
            }


            z3::expr sum = c->int_val(0);
            for (size_t train = 0; train < trains.size(); train++) {
                // 遍历时间步
                for (size_t time = trainMinLength[train]-1; time < makespan; time++) {

                    for (size_t edge = 0; edge < TEG[train][time].size(); edge++) {
                        if(TEG[train][time][edge]!=trains[train].stops.back().stopEdge)
                        sum = sum + z3::ite(occupiedVars[train][time][edge], c->int_val(1), c->int_val(0));
                    }

                }
            }
            solver.add(sum <= extra_cost);





            for (Train &train : trains) {
                computeReachableConstraint(train,makespan);
                //computeEachCollisionConstraint(train, makespan);
                computeCollisionConstraint(train,makespan);
                //computeCrossCollisionConstraint(train,makespan,_nextMakespan);
                //computeEachOtherCollisionConstraint(train,makespan);
            }



            if(solve()){



                std::cout << "SAT" << std::endl;

                std::clock_t c_end = std::clock();
                time_t = 1.0 * (c_end - c_start) / CLOCKS_PER_SEC;
                std::cout << std::endl;
                printTrainRoute(makespan);
                std::cout << "SOLVE TIME: " << time_t << std::endl;

                innerLoopCompleted = true;
                break ;


            } else {
                nextMakespan= true;



                break;









            }


        }
        if (innerLoopCompleted) {
            break;
        }
        if (nextMakespan) {
            //std::cout << "进入下一个makespan"<<std::endl;

            continue;
        }


    }





}








Instance::Instance(const Instance &otherInstance)
        : graph(otherInstance.graph), trains(otherInstance.trains),
          maxTimeSteps(otherInstance.maxTimeSteps),
          trainPaths(otherInstance.trainPaths),c(otherInstance.c), solver(*c),occupiedVars(otherInstance.occupiedVars),constraints(otherInstance.constraints)
{}

/*void Instance::computeReachableConstraint(Train &train, int &makespan) {
    std::vector<EdgeSet> reachableFrom(mddSet[train.id].size());   // 用来存储从每个边可达的边集合
    for (pathType path : trainPaths[train.id]) {  // 遍历列车的每条路径
        for (size_t startIndex = 0; startIndex < path.size() - 1; startIndex++) {   // 遍历路径中的每个起始顶点索引
            size_t endIndex = startIndex+1;   // 计算可达的终止顶点索引
            for (size_t pathIndex = startIndex; pathIndex <= endIndex; pathIndex++) {
                reachableFrom[path[startIndex]->id].insert(path[pathIndex]);    // 将可达的边添加到reachableFrom中
            }
        }
    }

}*/





void Instance::computeReachableConstraint(Train &train, int &makespan) {
    /*std::vector<EdgeSet> reachableFrom(graph.edges.size());
    for (pathType path : trainPaths[train.id]) {
        for (size_t startIndex = 0; startIndex < path.size() - 1; startIndex++) {
            size_t endIndex = startIndex+1;
            for (size_t pathIndex = startIndex; pathIndex <= endIndex; pathIndex++) {
                reachableFrom[path[startIndex]->id].insert(path[pathIndex]);
            }
        }
    }*/

    for (size_t time = 0; time < (TEG[train.id].size()-1);time++) {
        expr_vector mutexCurrVars(*c);
        for (int u = 0; u < TEG[train.id][time].size(); ++u){
            //if(TEG[train.id][time][u]== train.stops.back().stopEdge)
                //continue;
            if((EdgeTime[train.id][TEG[train.id][time][u]]+time)>(TEG[train.id].size()-1))
                continue;
            expr_vector mutexNextVars(*c);
            for (int v = 0; v < TEG[train.id][time+1].size(); ++v) {

                if((EdgeTime[train.id][TEG[train.id][time+1][v]]+time+1)>(TEG[train.id].size()-1))
                    continue;

                bool isNextEdge= false;
                for (const Edge *reachable : reachableFrom[train.id][TEG[train.id][time][u]->id]) {
                    if(reachable==TEG[train.id][time + 1][v]) {
                        isNextEdge = true;

                        mutexNextVars.push_back(occupiedVars[train.id][time + 1][v]);
                    }
                }
                if(!isNextEdge) continue;


            }
            mutexCurrVars.push_back(occupiedVars[train.id][time][u]) ;

            if (!mutexNextVars.empty()) {
                //constraints.push_back(implies(occupiedVars[train.id][time][u], mk_or(mutexNextVars)));
                //constraints.push_back(atmost(mutexNextVars, 1));

                //solver.add(atmost(mutexNextVars, 1));


                solver.add(implies(occupiedVars[train.id][time][u],mk_or(mutexNextVars)));


                //solver.add(implies(occupiedVars[train.id][time][u], (z3::atmost(mutexNextVars),1)));

            }

        }
        if (mutexCurrVars.empty()) {
            continue; // 跳过本次循环
        } else{
            //constraints.push_back(atmost(mutexCurrVars, 1));
            //solver.add(atmost(mutexCurrVars,1));
            solver.add(atmost(mutexCurrVars,1));

        }


    }

    expr_vector mutexFinalVars(*c);
    for (int u = 0; u < TEG[train.id][TEG[train.id].size()-1].size(); ++u){
        mutexFinalVars.push_back(occupiedVars[train.id][TEG[train.id].size()-1][u]);
    }
    if (!mutexFinalVars.empty()) {
       // constraints.push_back(atmost(mutexFinalVars, 1));
        //solver.add(atmost(mutexFinalVars, 1));
        solver.add(atmost(mutexFinalVars,1));
    }

}


bool Instance::solve() {



    //params p(*c);
    //p.set("unsat_core", true);
    //solver.set(p);

    addConstraintsToSolver();
    //z3::params p(*c);
    //.set(":timeout", static_cast<unsigned>(1000*60));
    //solver.set(p);

    bool result = solver.check() == sat;



    return result;
}


void Instance::addConstraintsToSolver() {
    //for (expr constraint : constraints)
        //solver.add(constraint);

    /*// 对起始位置的顶点进行约束设置
    for (const Train &train : trains) {
        for (int u = 0; u < MDD[train.id][0].size(); ++u)
            if (MDD[train.id][0][u] == train.start.stopEdge) {
                solver.add(occupiedVars[train.id][0][u]);
                std::cout << "第一个时间步数的布尔变量为occupiedVars[train.id][0]【" << u <<"]"<<std::endl;
            }

    }*/
    for (const Train &train : trains) {
        for (int u = 0; u < TEG[train.id][0].size(); ++u)
            if (TEG[train.id][0][u] == train.start.stopEdge) {
                solver.add(occupiedVars[train.id][0][u]);

            }

    }

    /*
    for (const Train &train : trains) {
        int maxTime= TEG[train.id].size() - 1;
        std::cout << "maxTime" << maxTime <<std::endl;
        for (int u = 0; u < TEG[train.id][maxTime].size(); ++u){
            if(TEG[train.id][maxTime][u] == train.stops.back().stopEdge){
                solver.add(occupiedVars[train.id][maxTime][u]);
                std::cout << "最后一个时间步数的布尔变量为occupiedVars[train.id][maxTime]【" << u <<"]"<<std::endl;
                std::cout <<TEG[train.id][maxTime][u]->from<<"-"<<TEG[train.id][maxTime][u]->to<<std::endl;
                std::cout  << "occupies_" << train.id << "_" << maxTime << "_" << TEG[train.id][maxTime][u]->from<<"->"<<TEG[train.id][maxTime][u]->to;
                std::cout << " = " << TEG[train.id][maxTime][u]<< " "<< std::endl;            }
        }
    }*/
    for (const Train &train : trains) {
        int stopTime= trainPaths[train.id][0].size() - 1;
        expr_vector stopVars(*c);
        for(int time=stopTime;time<TEG[train.id].size();time++){
            for (int u = 0; u < TEG[train.id][time].size(); ++u){
                if(TEG[train.id][time][u] == train.stops.back().stopEdge){
                    stopVars.push_back(occupiedVars[train.id][time][u]);
                }
            }
        }
        solver.add(mk_or(stopVars));
    }




}
/*
void Instance::computeEachCollisionConstraint(Train &train, int &makespan) {



    for (size_t time = train.start.arrivalTime; time <  (TEG[train.id].size()-1); time++) {

        //expr_vector atMostOneEdgeVars(*c);
        //std::stringstream Var123Name;
        for (int u = 0; u < TEG[train.id][time].size(); ++u) {
            if(TEG[train.id][time][u]== train.stops.back().stopEdge)
                continue;
            if((EdgeTime[train.id][TEG[train.id][time][u]]+time)>(TEG[train.id].size()-1))
                continue;

                    //atMostOneEdgeVars.push_back(occupiedVars[train.id][time][u]);
                    expr_vector otherPathVars(*c);
                    for (const Train &otherTrain: trains) {
                        if (train == otherTrain || otherTrain.start.arrivalTime >
                                                   time)
                            continue;
                        for (int m = 0; m < TEG[otherTrain.id][time].size(); ++m) {

                            if(TEG[otherTrain.id][time][m]== otherTrain.stops.back().stopEdge)
                                continue;
                            if((EdgeTime[otherTrain.id][TEG[otherTrain.id][time][m]]+time)>(TEG[otherTrain.id].size()-1))
                                continue;

                                if (TEG[train.id][time][u] == TEG[otherTrain.id][time][m] ) {
                                std::cout << "occupies_Train1:" << train.id << ",time:" << time
                                          << ",edgeID:" << TEG[train.id][time][u]->id << ":"
                                          << TEG[train.id][time][u]->from << "->"
                                          << TEG[train.id][time][u]->to << ",occupies_Train2:"
                                          << otherTrain.id << ",time:" << time << ",edgeID:"
                                          << TEG[otherTrain.id][time][m]->id << ":"
                                          << TEG[otherTrain.id][time][m]->from << "->"
                                          << TEG[otherTrain.id][time][m]->to<<std::endl;

                                    //atMostOneEdgeVars.push_back(occupiedVars[otherTrain.id][time][m]);

                                     solver.add(!occupiedVars[otherTrain.id][time][m] || !occupiedVars[train.id][time][u]);
                                    otherPathVars.push_back(!occupiedVars[otherTrain.id][time][m]);


                                    //otherPathVars.push_back(!occupiedVars[otherTrain.id][time+1][n]);
                                    //solver.add(mk_or(otherPathVars));


                                    //otherPathVars.push_back(!occupiedVars[train.id][time][u]||!occupiedVars[otherTrain.id][time][m]);
                                    //solver.add(mk_or(otherPathVars));
                                    //solver.add(atmost(atMostOneEdgeVars, 1),Var123Name.str().c_str());
                                    //break;
                                }

                        }

                    }
                //solver.add(implies(occupiedVars[train.id][time][u], mk_and(otherPathVars)));


                    // solver.add(implies(occupiedVars[train.id][time][u]&&occupiedVars[train.id][time+1][v],mk_and(otherPathVars)));



            //solver.add(implies(occupiedVars[train.id][time][u], mk_and(otherPathVars)));
        }
    }
}*/

void Instance::computePathNextEdge(){
    reachableFrom.resize(trains.size());
    for (Train &train : trains) {
        reachableFrom[train.id].resize(graph.edges.size());
        for (pathType path: trainPaths[train.id]) {

            for (size_t startIndex = 0; startIndex < path.size() - 1; startIndex++) {
                //reachableFrom[train.id][path[startIndex]->id].insert(path[startIndex+1]);
                //reachableFrom[train.id][path[startIndex]->id].insert(path[startIndex]);

                size_t endIndex = startIndex + 1;   // 计算可达的终止顶点索引
                for (size_t pathIndex = startIndex; pathIndex <=endIndex; pathIndex++) {
                    reachableFrom[train.id][path[startIndex]->id].insert(path[pathIndex]);
                }
            }
        }
    }
}

void Instance::computeEdgeTime(){
    EdgeTime.resize(trains.size());
    for (Train &train : trains) {
        //EdgeTime[train.id].resize(graph.edges.size());
        for (pathType path: trainPaths[train.id]) {

            for (int startIndex = 0; startIndex < path.size() ; startIndex++) {

                int needTime=path.size()-1-startIndex;
                EdgeTime[train.id][path[startIndex]]=needTime;

                /*for (size_t pathIndex = startIndex; pathIndex <=endIndex; pathIndex++) {
                    reachableFrom[train.id][path[startIndex]->id].insert(path[pathIndex]);
                }*/
            }
        }
    }
}


void Instance::computeCollisionConstraint(Train &train, int &makespan) {
    /*std::vector<EdgeSet> reachableFrom(graph.edges.size());
    for (pathType path : trainPaths[train.id]) {
        for (size_t startIndex = 0; startIndex < path.size() - 1; startIndex++) {
            size_t endIndex = startIndex+1;   // 计算可达的终止顶点索引
            for (size_t pathIndex = startIndex; pathIndex <= endIndex; pathIndex++) {
                reachableFrom[path[startIndex]->id].insert(path[pathIndex]);
            }
        }
    }*/
    for (size_t time = train.start.arrivalTime; time <  (TEG[train.id].size()-1); time++) {

        //expr_vector atMostOneEdgeVars(*c);
        //std::stringstream Var123Name;
        for (int u = 0; u < TEG[train.id][time].size(); ++u) {
            //if(TEG[train.id][time][u]== train.stops.back().stopEdge)
               // continue;
            if((EdgeTime[train.id][TEG[train.id][time][u]]+time)>(TEG[train.id].size()-1))
                continue;

            for (int v = 0; v < TEG[train.id][time + 1].size(); ++v) {
                if((EdgeTime[train.id][TEG[train.id][time+1][v]]+time+1)>(TEG[train.id].size()-1))
                    continue;

                bool isNextEdge= false;
                for (const Edge *reachable : reachableFrom[train.id][TEG[train.id][time][u]->id]) {
                    if(reachable==TEG[train.id][time + 1][v]) {
                        isNextEdge = true;
                    }
                }
                if(!isNextEdge) continue;


                    //atMostOneEdgeVars.push_back(occupiedVars[train.id][time][u]);
                    expr_vector otherPathVars(*c);
                    expr_vector otherPathVars1(*c);
                    expr_vector otherPathVars2(*c);
                for (const Train &otherTrain: trains) {
                        if (train == otherTrain || otherTrain.start.arrivalTime >
                                                   time)
                            continue;
                        for (int m = 0; m < TEG[otherTrain.id][time].size(); ++m) {
                            //if(TEG[otherTrain.id][time][m]== otherTrain.stops.back().stopEdge)
                               // continue;
                            if((EdgeTime[otherTrain.id][TEG[otherTrain.id][time][m]]+time)>(TEG[otherTrain.id].size()-1))
                                continue;
                            if ( TEG[otherTrain.id][time][m]==TEG[train.id][time][u]&&v==0)  {//代表互斥！！！！！！！！
                                /*
                                std::cout << "occupies_Train1:" << train.id << ",time:" << time
                                          << ",edgeID:" << TEG[train.id][time][u]->id << ":"
                                          << TEG[train.id][time][u]->from << "->"
                                          << TEG[train.id][time][u]->to << ",occupies_Train2:"
                                          << otherTrain.id << ",time:" << time << ",edgeID:"
                                          << TEG[otherTrain.id][time][m]->id << ":"
                                          << TEG[otherTrain.id][time][m]->from << "->"
                                          << TEG[otherTrain.id][time][m]->to<<std::endl;
                                solver.add(!occupiedVars[otherTrain.id][time][m] || !occupiedVars[train.id][time][u]);*/
                                //otherPathVars2.push_back(!occupiedVars[otherTrain.id][time][m]);

                            }



                            if ( TEG[otherTrain.id][time][m]==TEG[train.id][time+1][v])  {//代表互斥！！！！！！！！


                                //solver.add(!occupiedVars[otherTrain.id][time][m]||!occupiedVars[train.id][time+1][v]);

                            }

                            for (int n = 0; n < TEG[otherTrain.id][time + 1].size(); ++n) {

                                if((EdgeTime[otherTrain.id][TEG[otherTrain.id][time+1][n]]+time+1)>(TEG[otherTrain.id].size()-1))
                                    continue;
                                bool isNextEdge1= false;
                                for (const Edge *reachable1 : reachableFrom[otherTrain.id][TEG[otherTrain.id][time][m]->id]) {
                                    if(reachable1==TEG[otherTrain.id][time + 1][n]) {
                                        isNextEdge1 = true;
                                    }
                                }
                                if(!isNextEdge1) continue;


                                /*if ((TEG[train.id][time][u] == TEG[otherTrain.id][time + 1][n]) &&
                                    (TEG[train.id][time + 1][v] == TEG[otherTrain.id][time][m])&&(TEG[train.id][time][u]!=TEG[train.id][time + 1][v])
                                    &&(TEG[otherTrain.id][time + 1][n]!=TEG[otherTrain.id][time][m])) {


                                    std::cout << "occupies_Train1:" << train.id << ",time:" << time
                                              << ",edgeID:" << TEG[train.id][time][u]->id << ":"
                                              << TEG[train.id][time][u]->from << "->"
                                              << TEG[train.id][time][u]->to << ",occupies_Train2:"
                                              << otherTrain.id << ",time:" << time << ",edgeID:"
                                              << TEG[otherTrain.id][time][m]->id << ":"
                                              << TEG[otherTrain.id][time][m]->from << "->"
                                              << TEG[otherTrain.id][time][m]->to<<std::endl;

                                    std::cout << "occupies_Train1:" << train.id << ",time:" << time + 1
                                              << ",edgeID:" << TEG[train.id][time + 1][v]->id << ":"
                                              << TEG[train.id][time + 1][v]->from << "->"
                                              << TEG[train.id][time + 1][v]->to << ",occupies_Train2:"
                                              << otherTrain.id << ",time:" << time + 1 << ",edgeID:"
                                              << TEG[otherTrain.id][time + 1][n]->id << ":"
                                              << TEG[otherTrain.id][time + 1][n]->from << "->"
                                              << TEG[otherTrain.id][time + 1][n]->to<<std::endl;






                                     otherPathVars1.push_back(implies(occupiedVars[train.id][time][u] &&
                                                                        occupiedVars[train.id][time + 1][v],
                                                                        !occupiedVars[otherTrain.id][time][m] &&
                                                                        !occupiedVars[otherTrain.id][time +
                                                                                                     1][n]));
                                        //otherPathVars1.push_back(implies(occupiedVars[otherTrain.id][time][m] &&
                                                                        occupiedVars[otherTrain.id][time + 1][n],
                                                                        !occupiedVars[train.id][time][u] &&
                                                                        !occupiedVars[train.id][time +
                                                                                                1][v]));
                                        solver.add(mk_or(otherPathVars1));
                                    }*/
                                if ((TEG[train.id][time][u]!=TEG[train.id][time + 1][v]) &&
                                    (TEG[otherTrain.id][time][m]!=TEG[otherTrain.id][time + 1][n] ) ) {
                                    int shareId1 = TEG[train.id][time][u]->sharedVertex(TEG[train.id][time + 1][v]);
                                    int shareId2 = TEG[otherTrain.id][time][m]->sharedVertex(TEG[otherTrain.id][time + 1][n]);
                                    if (shareId1 == shareId2) {//共享一个顶点
                                        if ((TEG[train.id][time][u] != TEG[otherTrain.id][time + 1][n]) &&
                                            (TEG[train.id][time + 1][v] != TEG[otherTrain.id][time][m]) &&
                                            (TEG[otherTrain.id][time + 1][n] != TEG[train.id][time + 1][v])&&
                                            TEG[train.id][time][u] !=TEG[otherTrain.id][time][m] ) {


                                            /*std::cout << "occupies_Train1:" << train.id << ",time:" << time
                                                      << ",edgeID:" << TEG[train.id][time][u]->id << ":"
                                                      << TEG[train.id][time][u]->from << "->"
                                                      << TEG[train.id][time][u]->to << ",occupies_Train2:"
                                                      << otherTrain.id << ",time:" << time << ",edgeID:"
                                                      << TEG[otherTrain.id][time][m]->id << ":"
                                                      << TEG[otherTrain.id][time][m]->from << "->"
                                                      << TEG[otherTrain.id][time][m]->to<<std::endl;

                                            std::cout << "occupies_Train1:" << train.id << ",time:" << time + 1
                                                      << ",edgeID:" << TEG[train.id][time + 1][v]->id << ":"
                                                      << TEG[train.id][time + 1][v]->from << "->"
                                                      << TEG[train.id][time + 1][v]->to << ",occupies_Train2:"
                                                      << otherTrain.id << ",time:" << time + 1 << ",edgeID:"
                                                      << TEG[otherTrain.id][time + 1][n]->id << ":"
                                                      << TEG[otherTrain.id][time + 1][n]->from << "->"
                                                      << TEG[otherTrain.id][time + 1][n]->to<<std::endl;*/

                                            otherPathVars.push_back(implies(occupiedVars[train.id][time][u] && occupiedVars[train.id][time + 1][v],!occupiedVars[otherTrain.id][time + 1][n]));
                                            otherPathVars.push_back(implies(occupiedVars[otherTrain.id][time][m] && occupiedVars[otherTrain.id][time + 1][n],!occupiedVars[train.id][time + 1][v]));
                                            solver.add(mk_or(otherPathVars));

                                        }
                                    }
                                }


                                if ( TEG[otherTrain.id][time+1][n]==TEG[train.id][time][u]||TEG[otherTrain.id][time+1][n]==TEG[train.id][time+1][v])  {
                                    //std::cout << "进入普通碰撞" << std::endl;

                                    otherPathVars2.push_back(!occupiedVars[otherTrain.id][time+1][n]);

                                }
                                if ( TEG[otherTrain.id][time][m]==TEG[train.id][time+1][v]||TEG[otherTrain.id][time][m]==TEG[train.id][time][u])  {
                                    //std::cout << "进入普通碰撞" << std::endl;

                                    otherPathVars2.push_back(!occupiedVars[otherTrain.id][time][m]);

                                }







                            }
                        }

                    }
                    //if(TEG[train.id][time][u]!=TEG[train.id][time+1][v]) {
                        solver.add(implies(occupiedVars[train.id][time][u] && occupiedVars[train.id][time + 1][v],
                                       mk_and(otherPathVars2)));
                    //}
            }
            //solver.add(implies(occupiedVars[train.id][time][u], mk_and(otherPathVars)));
        }
    }
}













void Instance::printTrainRoute(int &makespan) {
    model m = solver.get_model();
    int currentSOc=0;
    int xiebian=0;
    for (Train train : trains) {
        bool reachedGoal = false;

        for (int time = 0; time < makespan; time++) {
            std::cout << time << ": ";
            if(time < train.start.arrivalTime || reachedGoal){
                std::cout << std::endl;
                continue;
            }

            for (size_t v = 0; v < TEG[train.id][time].size(); v++) {
                if (m.eval(occupiedVars[train.id][time][v], false)
                            .bool_value() == Z3_L_TRUE) {
                    reachedGoal = TEG[train.id][time][v] == train.stops.back().stopEdge;
                    std::cout << "(" << TEG[train.id][time][v]->from << " " << TEG[train.id][time][v]->to << ") ";
                    if(TEG[train.id][time][v]->from+1!=TEG[train.id][time][v]->to) xiebian++;
                    if(time!=0){
                        currentSOc++;
                    }
                }
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }


}




