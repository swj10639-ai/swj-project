#include <sstream>
#include "instance.h"
#include <z3++.h>
#include <z3_api.h>
#include <iostream>
#include <set>
#include <regex>
#include <queue>

Instance::Instance(Graph graph, std::vector<Train> trains, uint32_t maxTimeSteps,context *c)
        : graph(graph), trains(trains), maxTimeSteps(maxTimeSteps), trainPaths(trains.size()),c(c), solver(*c),     //trainPaths是 一个存储列车路径的向量，大小为trains.size()。
        occupiedVars(trains.size(), std::vector<expr_vector>(maxTimeSteps, *c))
        {
    //算法开始
    //（1）.A*计算初始makespan
    //std::cout<<"开始";
    std::clock_t c_start = std::clock();  //std::clock() 获取当前的时钟时间，这标记了开始计时的时间点。

    int start_Makespan=0;//初始的最大个体成本
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
            std::cout << "初始的sumOfCost=" << sumOfCost<<std::endl;
            std::cout << "初始的makespan为" << start_Makespan<<std::endl;

            //std::cout <<start_Makespan<< std::endl;
            // 输出 trainPaths
           /* for (size_t train = 0; train < trainPaths.size(); ++train) {

                std::cout << "Train " << train << " paths:" << std::endl;
                for (size_t path = 0; path < trainPaths[train].size(); ++path) {
                    std::cout << "Path " << path << ": ";
                    for (size_t edge = 0; edge < trainPaths[train][path].size(); ++edge) {
                        const Edge* currentEdge = trainPaths[train][path][edge];
                        std::cout << "(" << currentEdge->from << " -> " << currentEdge->to << ") ";
                    }
                    std::cout << std::endl;
                }
                std::cout << std::endl;
            }*/
            computePathNextEdge();//计算路径中每条边的下一条边
            computeEdgeTime();//计算路径中每条边的到终边需要多少时间
            for (Train &train : trains) { // 遍历每个列车对象
                trainMdd mdd(train.id, train.start.stopEdge, train.stops.back().stopEdge); // 创建 MDD 实例
                for (const auto& path : trainPaths[train.id]) {
                    mdd.add_path(path); // 将每条路径添加到 MDD 中
                }
                MDDs.push_back(mdd);


                /*std::cout << "新的MDD"<<std::endl;
                // 获取 MDD 中的所有状态
                auto all_paths = mdd.get_mdd();

                // 遍历每个状态
                for (const auto& entry : all_paths) {
                    const tup& current_state = entry.first; // 当前状态 (边指针, 深度)
                    const s_tup& successors = entry.second; // 当前状态的后继集合

                    const Edge* current_edge = std::get<0>(current_state); // 获取边指针
                    int depth = std::get<1>(current_state); // 获取深度

                    // 输出当前状态的信息
                    std::cout << "CurrentEdge: " << current_edge->from<< "-" << current_edge->to<< ", Depth: " << depth << std::endl;

                    // 遍历当前状态的后继状态
                    for (const auto& successor : successors) {
                        const Edge* successor_edge = std::get<0>(successor); // 获取后继边指针
                        int successor_depth = std::get<1>(successor); // 获取后继深度

                        // 输出后继状态的信息
                        std::cout << "    successor_edge: " << successor_edge->from<< "-" << successor_edge->to
                                  << ", Successor Depth: " << successor_depth << std::endl;
                    }
                }*/

            }
    for (int soc = sumOfCost; soc < maxTimeSteps*trains.size(); ++soc) {      //总迭代soc

        int extra_cost=soc-sumOfCost;
        int makespan = start_Makespan+1+extra_cost;

        bool innerLoopCompleted = false;  //是否结束makespan的循环
        bool nextMakespan = false;  //makespan是否++
       std::cout << "迭代makespan的当前makespan为" << makespan-1<<std::endl;


        while (true){

            //2.然后构建mdd



            for (Train &train : trains) { // 遍历每个列车对象
                mddSet.emplace_back(); // 确保每个列车第一个 unordered_set 已经被创建

                //std::cout << "newnewnew新的MDD" << std::endl;
                // 获取 MDD 中的所有状态
                auto all_paths = MDDs[train.id].get_mdd();

                // 遍历每个状态
                for (const auto &entry: all_paths) {
                    const tup &current_state = entry.first; // 当前状态 (边指针, 深度)
                    const s_tup &successors = entry.second; // 当前状态的后继集合

                    const Edge *current_edge = std::get<0>(current_state); // 获取边指针
                    mddSet[train.id].insert(current_edge);


                    int depth = std::get<1>(current_state); // 获取深度

                    // 输出当前状态的信息
                   // std::cout << "CurrentEdge: " << current_edge->from << "-" << current_edge->to << ", Depth: "<< depth << std::endl;


                }
            }



            /*
            std::cout <<"mddSet生成成功"<< std::endl;
            for (size_t train = 0; train < trains.size(); train++) {
                std::cout << "第" <<train<<"个列车的mddSet："<<std::endl;
                for (const auto &mddEgge: mddSet[train]) {
                    std::cout << mddEgge->from << "-" << mddEgge->to << std::endl; // 输出指针指向的值
                }
            }
*/

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
                    /*if(time==makespan-1){
                        TEG[train][time].push_back(trains[train].stops.back().stopEdge);
                        continue;
                    }*/
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

            /*std::cout <<"TEG生成成功"<< std::endl;
            // 输出 TEG
            for (size_t train = 0; train < trains.size(); train++) {
                std::cout << "TEG+" <<train<<std::endl;
                for (size_t time = 0; time < makespan; time++) {
                    for (int v=0;v<TEG[train][time].size();++v) {
                        // Output the edge or perform desired operations
                        // For example:
                        std::cout << "Train:" << train << ",Time:" << time << ",Edge ID: " << TEG[train][time][v]->id << ", From: " << TEG[train][time][v]->from << ", To: " << TEG[train][time][v]->to << std::endl;
                    }
                }
            }*/



            solver.reset();//清空所有约束


            for (size_t train = 0; train < trains.size(); train++) {
                //occupiedVars.clear();
                //occupiedVars(trains.size(), std::vector<expr_vector>(maxTimeSteps, *c));
                for (size_t time = trains[train].start.arrivalTime; time < TEG[train].size(); time++) {
                    //std::cout << "curr:"<<time<<std::endl;
                    occupiedVars[train][time] = expr_vector(*c);  // 初始化占用变量的表达式向量。expr_vector(*c) 表达式创建了一个空的表达式向量

                    /*if(time==0) {
                        std::stringstream oVarName1; // 创建占用变量的名称
                        oVarName1 << "occupies_" << train << "_" << time << "_" << trains[train].start.stopEdge;  ////要不要加->id
                        occupiedVars[train][0].push_back(
                                c->bool_const(oVarName1.str().c_str()));// 使用生成的名称创建布尔类型的常量表达式，表示一个占用变量
                        continue;
                    }*/
                    /*if(time==makespan-1) {
                        std::stringstream oVarName2; // 创建占用变量的名称
                        oVarName2 << "occupies_" << train << "_" << time << "_" << trains[train].stops.back().stopEdge;  ////要不要加->id
                        occupiedVars[train][time].push_back(
                                c->bool_const(oVarName2.str().c_str()));// 使用生成的名称创建布尔类型的常量表达式，表示一个占用变量
                        continue;
                    }*/
                    for (int v=0;v< TEG[train][time].size();++v) {
                        std::stringstream oVarName; // 创建占用变量的名称
                        oVarName << "occupies_" << train << "_" << time << "_" << TEG[train][time][v];  ////要不要加->id
                        occupiedVars[train][time].push_back(
                                c->bool_const(oVarName.str().c_str()));// 使用生成的名称创建布尔类型的常量表达式，表示一个占用变量


                    }




                        /*for (const auto& mddEgge : mddSet[train]) {

                      //在内部循环中，对于列车在时间步time中的第v条边，使用列车索引号、时间步和边索引号生成了占用变量的名称。

                        if((EdgeTime[train][mddEgge]+time)>(makespan-1))
                            continue;
                        bool isNextEdge;
                        int pretime=time-1;
                        for (int u = 0; u < TEG[train][pretime].size(); ++u) {// 对于当前MDD时间步中的每个边

                            for (const Edge *reachable: reachableFrom[train][TEG[train][pretime][u]->id]) {
                                if (reachable == mddEgge) {
                                    std::stringstream oVarName; // 创建占用变量的名称
                                    oVarName << "occupies_" << train << "_" << time << "_" << mddEgge;  ////要不要加->id
                                    occupiedVars[train][time].push_back(
                                            c->bool_const(oVarName.str().c_str()));// 使用生成的名称创建布尔类型的常量表达式，表示一个占用变量

                                    isNextEdge = true;
                                    break;
                                }
                            }
                            if (isNextEdge) break;
                        }





                        //std::stringstream Var123Name; // 创建占用变量的名称
                        //Var123Name << "occupies_Train:" << train << ",time:" << time << ",edgeID:" <<MDD[train][time][v]->id<<":"<< MDD[train][time][v]->from<< "->"<< MDD[train][time][v]->to;

                        //solver.add(occupiedVars[train][time][v],Var123Name.str().c_str());
                    }*/
                }
            }
            /*std::cout <<"布尔变量生成成功"<< std::endl;
            // 遍历 trains  //输出布尔变量

            for (size_t train = 0; train < trains.size(); train++) {
                // 遍历时间步
                for (size_t time = trains[train].start.arrivalTime; time < TEG[train].size(); time++) {
                    // 输出占用变量和 MDD 值
                    std::cout << "occupiedVars[" << train << "][" << time << "]: ";

                    // 遍历占用变量的表达式向量和 MDD
                    for (int v=0;v< occupiedVars[train][time].size();++v) {
                    //for (const auto& mddEgge : mddSet[train]) {
                        // 输出占用变量的名称和 MDD 值
                        std::stringstream oVarName;
                        oVarName << "occupies_" << train << "_" << time << "_" << TEG[train][time][v]->from<<"->"<<TEG[train][time][v]->to;
                        std::cout << oVarName.str() << " = " << TEG[train][time][v] << " "<< std::endl;
                    }

                    std::cout << std::endl;
                }
            }*/

            z3::expr sum = c->int_val(0);  // 初始化求和表达式为 0
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




            //4.计算约束

            for (Train &train : trains) { // 遍历每个列车对象
                computeReachableConstraint(train,makespan);   //4.1 计算与当前列车相关的可达性约束。它根据给定的列车对象，生成约束条件，以确保在每个时间步中，某条边被占用，则下一个时间步至少有一个相邻的边被列车占用。
                //computeEachCollisionConstraint(train, makespan);    //4.5
                computeCollisionConstraint(train,makespan);     //4.2有问题，再定计算与当前列车相关的碰撞约束条件。。它通过遍历给定列车的每条路径，生成约束条件以确保列车在运行过程中不会与其他列车发生碰撞。（某列车，在当前时间步占用的边，直到下一时间步占用的边，这条路径上的所有边；其他列车在这两个时间步都不可以占据）
                //computeCrossCollisionConstraint(train,makespan,_nextMakespan);    //4.4交叉碰撞
                //computeEachOtherCollisionConstraint(train,makespan);   //4.3正面碰撞
            }



            if(solve()){//求解，并且如果求解成功



                std::cout << "SAT" << std::endl;// 打印 "SAT" 表示可满足

                std::clock_t c_end = std::clock();    //std::clock() 获取当前的时钟时间，这标记了结束计时的时间点
                time_t = 1.0 * (c_end - c_start) / CLOCKS_PER_SEC;  // 计算执行约束条件所花费的时间，并累加到constraint_t变量中
                std::cout << std::endl;
                printTrainRoute(makespan);
                std::cout << "SOLVE TIME: " << time_t << std::endl; // 打印整体求解时间

                innerLoopCompleted = true;
                break ;// 使用标签和 break 跳出外层循环


            } else {
                nextMakespan= true;     //进入下一个makespan和soc
                //std::cout << "UNSAT" << std::endl; // 如果求解失败，则打印 "UNSAT" 表示不可满足



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





//4.1 计算与当前列车相关的可达性约束。它根据给定的列车对象，生成约束条件，以确保在每个时间步中，某条边被占用，则下一个时间步至少有一个相邻的边被列车占用。
//智能体只能在当前时间步占用一个布尔变量，下一时间步如果可以走几条边，只能限制一条
void Instance::computeReachableConstraint(Train &train, int &makespan) {
    /*std::vector<EdgeSet> reachableFrom(graph.edges.size());   // 用来存储从每个边可达的边集合
    for (pathType path : trainPaths[train.id]) {  // 遍历列车的每条路径
        for (size_t startIndex = 0; startIndex < path.size() - 1; startIndex++) {   // 遍历路径中的每个起始顶点索引
            size_t endIndex = startIndex+1;   // 计算可达的终止顶点索引
            for (size_t pathIndex = startIndex; pathIndex <= endIndex; pathIndex++) {
                reachableFrom[path[startIndex]->id].insert(path[pathIndex]);    // 将可达的边添加到reachableFrom中
            }
        }
    }*/

    for (size_t time = 0; time < (TEG[train.id].size()-1);time++) {// 对于每个时间步，生成约束条件
        expr_vector mutexCurrVars(*c);// 创建一个用于存储互斥变量的表达式向量
        for (int u = 0; u < TEG[train.id][time].size(); ++u){// 对于当前MDD时间步中的每个边
            //if(TEG[train.id][time][u]== train.stops.back().stopEdge)
                //continue;
            if((EdgeTime[train.id][TEG[train.id][time][u]]+time)>(TEG[train.id].size()-1))
                continue;
            expr_vector mutexNextVars(*c);// 创建一个用于存储互斥变量的表达式向量
            for (int v = 0; v < TEG[train.id][time+1].size(); ++v) {// 遍历MDD下一时间步中的每个边

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
            // 确保在当前时间步 `time` 占用边 `u` 时，下一时间步 `time + 1` 只能占用一个相邻的边
            if (!mutexNextVars.empty()) {
                //constraints.push_back(implies(occupiedVars[train.id][time][u], mk_or(mutexNextVars)));
                //constraints.push_back(atmost(mutexNextVars, 1)); // 确保在下一个时间步 `time + 1` 只能占用一个相邻的边

                //solver.add(atmost(mutexNextVars, 1));


                solver.add(implies(occupiedVars[train.id][time][u],mk_or(mutexNextVars)));


                //solver.add(implies(occupiedVars[train.id][time][u], (z3::atmost(mutexNextVars),1)));

            }

        }
        if (mutexCurrVars.empty()) {
            continue; // 跳过本次循环
        } else{
            //constraints.push_back(atmost(mutexCurrVars, 1)); // 确保在当前时间步 `time` 只能占用一个边
            //solver.add(atmost(mutexCurrVars,1));
            solver.add(atmost(mutexCurrVars,1));

        }


    }

    expr_vector mutexFinalVars(*c);// 创建一个用于存储互斥变量的表达式向量
    for (int u = 0; u < TEG[train.id][TEG[train.id].size()-1].size(); ++u){// 将最后一个时间步中的顶点的占用变量添加到顶点互斥标识符中
        mutexFinalVars.push_back(occupiedVars[train.id][TEG[train.id].size()-1][u]);
    }
    if (!mutexFinalVars.empty()) {
       // constraints.push_back(atmost(mutexFinalVars, 1)); // 确保在最后一个时间步 `makespan` 只能占用一个边
        //solver.add(atmost(mutexFinalVars, 1));
        solver.add(atmost(mutexFinalVars,1));
    }

    //// 要不要去。。对冲突的处理。这段代码的目的是根据冲突信息，在SAT求解器中设置约束，以确保冲突顶点A和冲突顶点B不会同时被占据，从而解决路径规划中的冲突问题。
}


bool Instance::solve() {



    //params p(*c);
    //p.set("unsat_core", true);
    //solver.set(p);

    addConstraintsToSolver();//加约束到求解器
    //z3::params p(*c);     // 创建Z3求解器的参数对象p
    //.set(":timeout", static_cast<unsigned>(1000*60)); // 设置求解器超时时间为1分钟
    //solver.set(p);

    bool result = solver.check() == sat;  // 检查求解器的结果是否为满足（satisfiable）



    return result;    //返回求解器的结果
}


void Instance::addConstraintsToSolver() {
    //for (expr constraint : constraints)   // 遍历constraints列表中的每个约束
        //solver.add(constraint); // 将约束添加到求解器中

    /*// 对起始位置的顶点进行约束设置
    for (const Train &train : trains) {   // 遍历trains列表中的每个列车
        for (int u = 0; u < MDD[train.id][0].size(); ++u)
            if (MDD[train.id][0][u] == train.start.stopEdge) {
                solver.add(occupiedVars[train.id][0][u]);  // 将起始停靠点的占用变量添加到求解器中
                std::cout << "第一个时间步数的布尔变量为occupiedVars[train.id][0]【" << u <<"]"<<std::endl;
            }

    }*/
    for (const Train &train : trains) {   // 遍历trains列表中的每个列车
        for (int u = 0; u < TEG[train.id][0].size(); ++u)
            if (TEG[train.id][0][u] == train.start.stopEdge) {
                solver.add(occupiedVars[train.id][0][u]);  // 将起始停靠点的占用变量添加到求解器中
                //std::cout << "第一个时间步数的布尔变量为occupiedVars[train.id][0]【" << u <<"]"<<std::endl;
               // std::cout <<TEG[train.id][0][u]->from<<"-"<<TEG[train.id][0][u]->to<<std::endl;
                //std::cout  << "occupies_" << train.id << "_" << 0 << "_" << TEG[train.id][0][u]->from<<"->"<<TEG[train.id][0][u]->to;
                //std::cout << " = " << TEG[train.id][0][u]<< " "<< std::endl;
            }

    }

    /*// 对终位置的顶点进行约束设置
    for (const Train &train : trains) {   // 遍历trains列表中的每个列车
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
        expr_vector stopVars(*c); // 创建存储停靠点占用变量的向量
        for(int time=stopTime;time<TEG[train.id].size();time++){
            for (int u = 0; u < TEG[train.id][time].size(); ++u){
                if(TEG[train.id][time][u] == train.stops.back().stopEdge){
                    stopVars.push_back(occupiedVars[train.id][time][u]);
                }
            }
        }
        solver.add(mk_or(stopVars));   // 即至少有一个时间点该终边被占用。
    }

       // std::cout<<"调用了addConstraintsToSolver方法"<<std::endl;


}
/*
//4.5计算碰撞约束。它通过遍历给定列车的每条路径，生成约束条件以确保列车在运行过程中不会与其他列车发生碰撞。（某列车，在当前时间步占用的边，直到下一时间步占用的边，这两边；其他列车在这两个时间步都不可以占据）
void Instance::computeEachCollisionConstraint(Train &train, int &makespan) {



    for (size_t time = train.start.arrivalTime; time <  (TEG[train.id].size()-1); time++) {// 对于每个时间步，生成约束条件

        //expr_vector atMostOneEdgeVars(*c);
        //std::stringstream Var123Name; // 创建占用变量的名称
        for (int u = 0; u < TEG[train.id][time].size(); ++u) {// 对于当前MDD时间步中的每个边
            if(TEG[train.id][time][u]== train.stops.back().stopEdge)
                continue;
            if((EdgeTime[train.id][TEG[train.id][time][u]]+time)>(TEG[train.id].size()-1))
                continue;

                    //atMostOneEdgeVars.push_back(occupiedVars[train.id][time][u]);
                    expr_vector otherPathVars(*c);    //创建一个 Z3 表达式向量 otherPathVars，用于存储其他列车路径上的约束条件。
                    for (const Train &otherTrain: trains) {  //对于列车集合中的每个其他列车 otherTrain
                        if (train == otherTrain || otherTrain.start.arrivalTime >
                                                   time) //如果当前列车与 otherTrain 相同，或者 otherTrain 的起始到达时间大于当前时间步 time，则跳过此次迭代。
                            continue;
                        for (int m = 0; m < TEG[otherTrain.id][time].size(); ++m) {

                            if(TEG[otherTrain.id][time][m]== otherTrain.stops.back().stopEdge)
                                continue;
                            if((EdgeTime[otherTrain.id][TEG[otherTrain.id][time][m]]+time)>(TEG[otherTrain.id].size()-1))
                                continue;

                                if (TEG[train.id][time][u] == TEG[otherTrain.id][time][m] ) {//代表互斥！！！！！！！！
                                    std::cout << "进入普通碰撞" << std::endl;
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
                                    //break; // 退出最内层循环
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
        for (pathType path: trainPaths[train.id]) {  // 遍历列车的每条路径

            for (size_t startIndex = 0; startIndex < path.size() - 1; startIndex++) {   // 遍历路径中的每个起始顶点索引
                //reachableFrom[train.id][path[startIndex]->id].insert(path[startIndex+1]);
                //reachableFrom[train.id][path[startIndex]->id].insert(path[startIndex]);    // 将可达的边添加到reachableFrom中

                size_t endIndex = startIndex + 1;   // 计算可达的终止顶点索引
                for (size_t pathIndex = startIndex; pathIndex <=endIndex; pathIndex++) {
                    reachableFrom[train.id][path[startIndex]->id].insert(path[pathIndex]);    // 将可达的边添加到reachableFrom中
                }
            }
        }
    }
}

void Instance::computeEdgeTime(){
    EdgeTime.resize(trains.size());
    for (Train &train : trains) {
        //EdgeTime[train.id].resize(graph.edges.size());
        for (pathType path: trainPaths[train.id]) {  // 遍历列车的每条路径

            for (int startIndex = 0; startIndex < path.size() ; startIndex++) {   // 遍历路径中的每个起始顶点索引

                int needTime=path.size()-1-startIndex;//当前边需要多久到终点
                EdgeTime[train.id][path[startIndex]]=needTime;

                /*for (size_t pathIndex = startIndex; pathIndex <=endIndex; pathIndex++) {
                    reachableFrom[train.id][path[startIndex]->id].insert(path[pathIndex]);    // 将可达的边添加到reachableFrom中
                }*/
            }
        }
    }
}


//4.2计算碰撞约束。它通过遍历给定列车的每条路径，生成约束条件以确保列车在运行过程中不会与其他列车发生碰撞。（某列车，在当前时间步占用的边，直到下一时间步占用的边，这两边；其他列车在这两个时间步都不可以占据）
void Instance::computeCollisionConstraint(Train &train, int &makespan) {
    /*std::vector<EdgeSet> reachableFrom(graph.edges.size());   // 用来存储从每个边可达的边集合
    for (pathType path : trainPaths[train.id]) {  // 遍历列车的每条路径
        for (size_t startIndex = 0; startIndex < path.size() - 1; startIndex++) {   // 遍历路径中的每个起始顶点索引
            size_t endIndex = startIndex+1;   // 计算可达的终止顶点索引
            for (size_t pathIndex = startIndex; pathIndex <= endIndex; pathIndex++) {
                reachableFrom[path[startIndex]->id].insert(path[pathIndex]);    // 将可达的边添加到reachableFrom中
            }
        }
    }*/
    for (size_t time = train.start.arrivalTime; time <  (TEG[train.id].size()-1); time++) {// 对于每个时间步，生成约束条件

        //expr_vector atMostOneEdgeVars(*c);
        //std::stringstream Var123Name; // 创建占用变量的名称
        for (int u = 0; u < TEG[train.id][time].size(); ++u) {// 对于当前MDD时间步中的每个边
            //if(TEG[train.id][time][u]== train.stops.back().stopEdge)
               // continue;
            if((EdgeTime[train.id][TEG[train.id][time][u]]+time)>(TEG[train.id].size()-1))
                continue;

            for (int v = 0; v < TEG[train.id][time + 1].size(); ++v) {// 遍历MDD下一时间步中的每个边
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
                    expr_vector otherPathVars(*c);    //创建一个 Z3 表达式向量 otherPathVars，用于存储其他列车路径上的约束条件。
                    expr_vector otherPathVars1(*c);    //创建一个 Z3 表达式向量 otherPathVars，用于存储其他列车路径上的约束条件。
                    expr_vector otherPathVars2(*c);    //创建一个 Z3 表达式向量 otherPathVars，用于存储其他列车路径上的约束条件。
                for (const Train &otherTrain: trains) {  //对于列车集合中的每个其他列车 otherTrain
                        if (train == otherTrain || otherTrain.start.arrivalTime >
                                                   time) //如果当前列车与 otherTrain 相同，或者 otherTrain 的起始到达时间大于当前时间步 time，则跳过此次迭代。
                            continue;
                        for (int m = 0; m < TEG[otherTrain.id][time].size(); ++m) {
                            //if(TEG[otherTrain.id][time][m]== otherTrain.stops.back().stopEdge)
                               // continue;
                            if((EdgeTime[otherTrain.id][TEG[otherTrain.id][time][m]]+time)>(TEG[otherTrain.id].size()-1))
                                continue;
                            if ( TEG[otherTrain.id][time][m]==TEG[train.id][time][u]&&v==0)  {//代表互斥！！！！！！！！
                                /*std::cout << "进入普通碰撞::" << std::endl;
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
                                //std::cout << "进入普通碰撞" << std::endl;

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
                                    &&(TEG[otherTrain.id][time + 1][n]!=TEG[otherTrain.id][time][m])) {//代表互斥！！！！！！！！
                                    std::cout << "进入正面碰撞！！！" << std::endl;

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
                                                                                                     1][n]));//表示在当前时间步 time，其他列车不占用路径上的顶点。
                                        //otherPathVars1.push_back(implies(occupiedVars[otherTrain.id][time][m] &&
                                                                        occupiedVars[otherTrain.id][time + 1][n],
                                                                        !occupiedVars[train.id][time][u] &&
                                                                        !occupiedVars[train.id][time +
                                                                                                1][v]));//表示在当前时间步 time，其他列车不占用路径上的顶点。
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
                                            TEG[train.id][time][u] !=TEG[otherTrain.id][time][m] ) {//非正面碰撞
                                            //std::cout << "进入交叉碰撞！！！！！" << std::endl;

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


                                if ( TEG[otherTrain.id][time+1][n]==TEG[train.id][time][u]||TEG[otherTrain.id][time+1][n]==TEG[train.id][time+1][v])  {//代表互斥！！！！！！！！
                                    //std::cout << "进入普通碰撞" << std::endl;

                                    otherPathVars2.push_back(!occupiedVars[otherTrain.id][time+1][n]);

                                }
                                if ( TEG[otherTrain.id][time][m]==TEG[train.id][time+1][v]||TEG[otherTrain.id][time][m]==TEG[train.id][time][u])  {//代表互斥！！！！！！！！
                                    //std::cout << "进入普通碰撞" << std::endl;

                                    otherPathVars2.push_back(!occupiedVars[otherTrain.id][time][m]);

                                }



                                /*if(TEG[train.id][time][u]!=TEG[train.id][time+1][v]) {

                                    if (TEG[otherTrain.id][time][m] == TEG[train.id][time + 1][v]) {//代表互斥！！！！！！！！
                                        //std::cout << "进入普通碰撞" << std::endl;
                                        //otherPathVars2.push_back(!occupiedVars[otherTrain.id][time][m]);

                                    }

                                    if (TEG[otherTrain.id][time + 1][n] == TEG[train.id][time + 1][v]) {//代表互斥！！！！！！！！
                                        //std::cout << "进入普通碰撞" << std::endl;
                                        //otherPathVars2.push_back(!occupiedVars[otherTrain.id][time + 1][n]);

                                    }
                                }*/



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







/*
//4.3计算相向而行直接相撞的约束
void Instance::computeEachOtherCollisionConstraint(Train &train, int &makespan) {
    //std::cout<<"进入了新约束！！！！！"<<std::endl;
    //bool exitEachLoops = false; //退出所有循环
    for (size_t time = train.start.arrivalTime; time < (makespan - 1) ;time++) {// 对于每个时间步，生成约束条件

        expr_vector atMostOneVars(*c);
        std::stringstream Var321Name; // 创建占用变量的名称
        for (int u = 0; u < TEG[train.id][time].size() ; ++u){// 对于当前MDD时间步中的每个边

            //atMostOneEdgeVars.push_back(occupiedVars[train.id][time][u]);
            for (int v = 0; v < TEG[train.id][time+1].size() ; ++v) {// 遍历MDD下一时间步中的每个边

                int timeT11=0;
                int timeT22=0;
                for (int timeT1 = 0; timeT1 < MDD[train.id].size(); timeT1++) {
                    for (const Edge* edge1 : MDD[train.id][timeT1]) {
                        if(TEG[train.id][time][u]->id==edge1->id){
                            timeT11=timeT1;
                            break;
                        }

                    }
                }
                for (int timeT2 = 0; timeT2 < MDD[train.id].size(); timeT2++) {
                    for (const Edge* edge2 : MDD[train.id][timeT2]) {
                        if(TEG[train.id][time+1][v]->id==edge2->id){
                            timeT22=timeT2;
                            break;
                        }

                    }
                }
                if(timeT11==MDD[train.id].size()||timeT22==MDD[train.id].size()) continue;


                if(TEG[train.id][time][u]->containsNeighbour(TEG[train.id][time + 1][v])&&timeT11+1==timeT22) {
                    expr_vector otherPathVars(*c);
                    for (const Train &otherTrain: trains) {  //对于列车集合中的每个其他列车 otherTrain
                        if (train == otherTrain || otherTrain.start.arrivalTime >
                                                   time) //如果当前列车与 otherTrain 相同，或者 otherTrain 的起始到达时间大于当前时间步 time，则跳过此次迭代。
                            continue;
                        for (int m = 0; m < TEG[otherTrain.id][time].size(); ++m) {
                            for (int n = 0; n < TEG[otherTrain.id][time + 1].size(); ++n) {
                                //std::cout<<"这里！！！！！"<<std::endl;
                                int timeT111=0;
                                int timeT222=0;
                                for (int timeT1 = 0; timeT1 < MDD[otherTrain.id].size(); timeT1++) {
                                    for (const Edge* edge1 : MDD[otherTrain.id][timeT1]) {
                                        if(TEG[otherTrain.id][time][m]->id==edge1->id){
                                            timeT111=timeT1;
                                            break;
                                        }

                                    }
                                }
                                for (int timeT2 = 0; timeT2 < MDD[otherTrain.id].size(); timeT2++) {
                                    for (const Edge* edge2 : MDD[otherTrain.id][timeT2]) {
                                        if(TEG[otherTrain.id][time+1][n]->id==edge2->id){
                                            timeT222=timeT2;
                                            break;
                                        }

                                    }
                                }

                                if(timeT111==MDD[otherTrain.id].size()||timeT222==MDD[otherTrain.id].size()) continue;



                                if ((TEG[train.id][time][u] == TEG[otherTrain.id][time + 1][n]) &&
                                    (TEG[train.id][time + 1][v] == TEG[otherTrain.id][time][m])&&timeT111+1==timeT222) {//代表互斥！！！！！！！！
                                    if ((TEG[train.id][time][u]->containsNeighbour(TEG[train.id][time + 1][v])) &&
                                        (TEG[otherTrain.id][time][m]->containsNeighbour(
                                                TEG[otherTrain.id][time + 1][n]))) {
                                        std::cout << "进入了正面碰撞！！！！！" << std::endl;
                                        atMostOneVars.push_back(
                                                occupiedVars[train.id][time][u] && occupiedVars[train.id][time + 1][v]);
                                        atMostOneVars.push_back(occupiedVars[otherTrain.id][time][m] &&
                                                                occupiedVars[otherTrain.id][time + 1][n]);
                                        Var321Name << "occupies_Train1:" << train.id << ",time:" << time << ",edgeID:"
                                                   << TEG[train.id][time][u]->id << ":" << TEG[train.id][time][u]->from
                                                   << "->" << TEG[train.id][time][u]->to << ",occupies_Train2:"
                                                   << otherTrain.id << ",time:" << time << ",edgeID:"
                                                   << TEG[otherTrain.id][time][m]->id << ":"
                                                   << TEG[otherTrain.id][time][m]->from << "->"
                                                   << TEG[otherTrain.id][time][m]->to;
                                        //solver.add(atmost(atMostOneVars, 1),Var321Name.str().c_str());
                                        //break; // 退出最内层循环
                                        //solver.add(implies(occupiedVars[train.id][time][u]&&occupiedVars[train.id][time + 1][v],!occupiedVars[otherTrain.id][time][m]&&!occupiedVars[otherTrain.id][time + 1][n]));//表示在当前时间步 time，其他列车不占用路径上的顶点。

                                        otherPathVars.push_back(implies(occupiedVars[train.id][time][u]&&occupiedVars[train.id][time + 1][v],!occupiedVars[otherTrain.id][time][m]&&!occupiedVars[otherTrain.id][time + 1][n]));//表示在当前时间步 time，其他列车不占用路径上的顶点。
                                        otherPathVars.push_back( implies(occupiedVars[otherTrain.id][time][m]&&occupiedVars[otherTrain.id][time + 1][n],!occupiedVars[train.id][time][u]&&!occupiedVars[train.id][time + 1][v]));//表示在当前时间步 time，其他列车不占用路径上的顶点。
                                        solver.add(mk_or(otherPathVars));
                                    }


                                    // atMostOneEdgeVars.push_back(occupiedVars[otherTrain.id][time][m]);
                                }
                            }
                        }

                    }
                    //solver.add(implies(occupiedVars[train.id][time][u]&&occupiedVars[train.id][time + 1][v],mk_and(otherPathVars)));  //表示如果当前列车在起始顶点和终止顶点同时占用，则其他列车在路径上的所有顶点都不应该被占用。这是一个逻辑蕴含式，意味着当 startVar 和 endVar 都为真时，otherPathVars 中的所有约束条件都必须为真。
                }
            }
        }
        //if(atMostOneVars.size()>=2){
            //constraints.push_back(atmost(atMostOneEdgeVars, 1)); // 确保在当前时间步 `time` ,该边只能被一个智能体占用
            //solver.add(atmost(atMostOneVars, 1));
        //}


    }



}
*/

/*
//4.4计算交叉碰撞
void Instance::computeCrossCollisionConstraint(Train &train, int &makespan,bool _nextMakespan) {
    //bool exitEachLoops = false; //退出所有循环
    for (size_t time = train.start.arrivalTime; time < (makespan - 1);time++) {// 对于每个时间步，生成约束条件

        expr_vector atMostOneVars(*c);
        std::stringstream Var132Name; // 创建占用变量的名称
        for (int u = 0; u < TEG[train.id][time].size() ; ++u){// 对于当前MDD时间步中的每个边

            //atMostOneEdgeVars.push_back(occupiedVars[train.id][time][u]);
            for (int v = 0; v < TEG[train.id][time+1].size() ; ++v) {// 遍历MDD下一时间步中的每个边

                int timeT11=0;
                int timeT22=0;
                for (int timeT1 = 0; timeT1 < MDD[train.id].size(); timeT1++) {
                    for (const Edge* edge1 : MDD[train.id][timeT1]) {
                        if(TEG[train.id][time][u]->id==edge1->id){
                            timeT11=timeT1;
                            break;
                        }

                    }
                }
                for (int timeT2 = 0; timeT2 < MDD[train.id].size(); timeT2++) {
                    for (const Edge* edge2 : MDD[train.id][timeT2]) {
                        if(TEG[train.id][time+1][v]->id==edge2->id){
                            timeT22=timeT2;
                            break;
                        }

                    }
                }

                if(timeT11==MDD[train.id].size()||timeT22==MDD[train.id].size()) continue;



                if(TEG[train.id][time][u]->containsNeighbour(TEG[train.id][time + 1][v])&&timeT11+1==timeT22) {
                    expr_vector otherPathVars(*c);
                    for (const Train &otherTrain: trains) {  //对于列车集合中的每个其他列车 otherTrain
                        if (train == otherTrain || otherTrain.start.arrivalTime >time) //如果当前列车与 otherTrain 相同，或者 otherTrain 的起始到达时间大于当前时间步 time，则跳过此次迭代。
                            continue;
                        for (int m = 0; m < TEG[otherTrain.id][time].size(); ++m) {
                            for (int n = 0; n < TEG[otherTrain.id][time + 1].size(); ++n) {
                                //std::cout<<"这里！！！！！"<<std::endl;

                                int timeT111=0;
                                int timeT222=0;
                                for (int timeT1 = 0; timeT1 < MDD[otherTrain.id].size(); timeT1++) {
                                    for (const Edge* edge1 : MDD[otherTrain.id][timeT1]) {
                                        if(TEG[otherTrain.id][time][m]->id==edge1->id){
                                            timeT111=timeT1;
                                            break;
                                        }

                                    }
                                }
                                for (int timeT2 = 0; timeT2 < MDD[otherTrain.id].size(); timeT2++) {
                                    for (const Edge* edge2 : MDD[otherTrain.id][timeT2]) {
                                        if(TEG[otherTrain.id][time+1][n]->id==edge2->id){
                                            timeT222=timeT2;
                                            break;
                                        }

                                    }
                                }

                                if(timeT111==MDD[otherTrain.id].size()||timeT222==MDD[otherTrain.id].size()) continue;





                                if ((TEG[train.id][time][u]->containsNeighbour(TEG[train.id][time + 1][v])) &&
                                    (TEG[otherTrain.id][time][m]->containsNeighbour(TEG[otherTrain.id][time + 1][n]))&&timeT111+1==timeT222   ) {
                                    int shareId1 = TEG[train.id][time][u]->sharedVertex(TEG[train.id][time + 1][v]);
                                    int shareId2 = TEG[otherTrain.id][time][m]->sharedVertex(
                                            TEG[otherTrain.id][time + 1][n]);
                                    if (shareId1 == shareId2) {//共享一个顶点
                                        if ((TEG[train.id][time][u] != TEG[otherTrain.id][time + 1][n]) &&
                                            (TEG[train.id][time + 1][v] != TEG[otherTrain.id][time][m]) &&
                                            (TEG[otherTrain.id][time + 1][n] != TEG[train.id][time + 1][v])) {//非正面碰撞


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
                                            std::cout << "进入交叉碰撞！！！！！" << std::endl;

                                            //otherPathVars.push_back(!occupiedVars[otherTrain.id][time][m]);

                                            //otherPathVars.push_back(!occupiedVars[otherTrain.id][time + 1][n]);
                                            //solver.add(atmost(atMostOneVars, 1),Var132Name.str().c_str());
                                            //break; // 退出最内层循环


                                            otherPathVars.push_back(implies(occupiedVars[train.id][time][u] && occupiedVars[train.id][time + 1][v],!occupiedVars[otherTrain.id][time + 1][n]));
                                            otherPathVars.push_back(implies(occupiedVars[otherTrain.id][time][m] && occupiedVars[otherTrain.id][time + 1][n],!occupiedVars[train.id][time + 1][v]));
                                            solver.add(mk_or(otherPathVars));
                                            //if(TEG[otherTrain.id][time + 1][n]->id==trains[otherTrain.id].stops.back().stopEdge->id){
                                               // _nextMakespan= true;
                                                //break;
                                           // }
                                        }
                                    }


                                }


                                //otherPathVars.push_back(!occupiedVars[otherTrain.id][time][m]);//表示在当前时间步 time，其他列车不占用路径上的顶点。
                                // atMostOneEdgeVars.push_back(occupiedVars[otherTrain.id][time][m]);

                            }
                        }

                    }

                    //solver.add(implies(occupiedVars[train.id][time][u] && occupiedVars[train.id][time + 1][v],
                                       //mk_and(otherPathVars)));
                }

                //constraints.push_back(
                //implies(occupiedVars[train.id][time][u] , mk_and(otherPathVars)));  //表示如果当前列车在起始顶点和终止顶点同时占用，则其他列车在路径上的所有顶点都不应该被占用。这是一个逻辑蕴含式，意味着当 startVar 和 endVar 都为真时，otherPathVars 中的所有约束条件都必须为真。

            }
        }
        //if(atMostOneVars.size()>=2){
            //constraints.push_back(atmost(atMostOneEdgeVars, 1)); // 确保在当前时间步 `time` ,该边只能被一个智能体占用
            //solver.add(atmost(atMostOneVars, 1));
       // }


    }



}
*/





void Instance::printTrainRoute(int &makespan) {
    model m = solver.get_model(); // 获取求解器的模型
    int currentSOc=0;
    int xiebian=0;
    for (Train train : trains) {  // 遍历每个列车
        bool reachedGoal = false;    // 标记列车是否到达目标点

        for (int time = 0; time < makespan; time++) {// 遍历每个时间步
            std::cout << time << ": "; // 输出当前时间步
            if(time < train.start.arrivalTime || reachedGoal){// 如果当前时间步小于列车的起始时间步或已经到达目标点，则换行并继续下一次循环
                std::cout << std::endl;
                continue;
            }

            for (size_t v = 0; v < TEG[train.id][time].size(); v++) {  // 遍历图中的每条边
                if (m.eval(occupiedVars[train.id][time][v], false)// 如果当前边在当前时间步被占用
                            .bool_value() == Z3_L_TRUE) {
                    reachedGoal = TEG[train.id][time][v] == train.stops.back().stopEdge;// 检查当前边是否为列车的最后一个停靠点
                    std::cout << "(" << TEG[train.id][time][v]->from << " " << TEG[train.id][time][v]->to << ") ";     // 输出边的起始点和终止点
                    if(TEG[train.id][time][v]->from+1!=TEG[train.id][time][v]->to) xiebian++;
                    if(time!=0){
                        currentSOc++;
                    }
                }
            }
            std::cout << std::endl;   // 换行
        }
        std::cout << std::endl; // 输出空行，用于分隔不同列车的路径
    }
    std::cout << "makespan" << makespan-1 <<std::endl;
    std::cout << "sum of cost:" << currentSOc <<std::endl;
    std::cout << "xiebianwei:" << xiebian <<std::endl;
    std::cout << "！！！！连续sum of cost:" << currentSOc+xiebian*0.5042135 <<std::endl;


}




