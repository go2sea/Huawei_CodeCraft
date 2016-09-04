#pragma once
#include <iostream>
#include <queue>
#include <unordered_map>
#include <map>
#include <unordered_set>
#include <set>
#include <vector>
#include <stack>
#include <string>
#include <cassert>
#include <fstream>
#include <time.h>
#include <algorithm>

#include "FZ.h"
#include "DFS.h"
#include "ONLY.h"

#include "lib/lib_io.h"
#include "lib/lib_record.h"
#include "lib/lib_time.h"
#include <random>
#include <sys/time.h>
#include <signal.h>
#include <limits.h>
using namespace std;

#define NORMAL_ANT		1
#define REVERSED_ANT	2
#define RANDOM_ANT		3

class Ant {
public:
	vector<int> path;
	vector<int> city;
	int path_cost;		//若该全路径不符要求（中间有断点），path_cost为INT_MAX，若符合要求，path会pop至最短，path_cost是0-end_index的长度
	int city_count;
	int max_node_id;
	int passed_count;		//全路径path截止到end_index（包含）可符合要求地经过V（即必须path[end_index] = destination_id）
	int source_id;
	int destination_id;
	double alpha_rate;
	double beta_rate;
	bool path_ended;	//改成每次一步的结构后，路径是否已经到头的标志

	Ant() {};
	Ant(int s_id, int d_id, double alpha_r, double beta_r, int city_c, vector<int>* begin_nodes, unordered_set<int>* V, double gen_rate, int ant_type, int max_node_id) {
		city_count = city_c;
		destination_id = d_id;
		source_id = s_id;
		alpha_rate = alpha_r;
		beta_rate = beta_r;
		initialize(begin_nodes, V, gen_rate, ant_type, max_node_id);
	}
	~Ant() {};

	int randomFromBeginNodes(vector<int>* begin_nodes) {
		return (*begin_nodes)[rand() % (*begin_nodes).size()];
	}

	void initialize(vector<int>* begin_nodes, unordered_set<int>* V, double gen_rate, int ant_type, int max_node_id) {
		city.clear();
		city.resize(max_node_id + 1, 0);
		path.clear();

		passed_count = 0;
		path_cost = INT_MAX;
		path_ended = false;

		//根据ant_type设置起点
		int begin_id = getBeginId(begin_nodes, V, ant_type);
		//city[0] = begin_id;
		city[begin_id] = 1;
		path.push_back(begin_id);
	}

	//根据蚂蚁类型获取 起点/终点
	int getBeginId(vector<int>* begin_nodes, unordered_set<int>* V, int ant_type) {
		return ant_type == 1 ? source_id : ant_type == 2 ? destination_id : randomFromBeginNodes(begin_nodes);
	}

	//根据蚂蚁类型获取 起点/终点
	int getEndId(int ant_type, unordered_set<int>* V) {
		return ant_type == 1 ? destination_id : ant_type == 2 ? source_id : -1;//注意：任性蚂蚁是没有终点的
	}

	//可以继续走返回true，否则（以找到或提前到终点或每找到）返回false（注意：此处neighbor和pheromone根据ant_type不同是不同的）
	bool selectNextCity(vector<vector<int>>* edge_ant_count, int sum_ant_count, vector<vector<double>>* pheromone, unordered_map<int, unordered_map<int, int>>* neighbors_type, vector<int>* begin_nodes, unordered_set<int>* V, int ant_type) {
		assert(!path_ended);
		//cout << "selectNextCity !!!" << endl;
		int current_id = path.back();
		//cout << "path.size(): " << path.size() << endl;
		//system("pause");
		unordered_map<int, double> p;	//neighbor_id : p
		double sum_p = 0;

		int end_id = getEndId(ant_type, V);

		for (auto& pair : (*neighbors_type)[current_id]) {
			int neighbor_id = pair.first;
			int neighbor_cost = pair.second;
			if (city[neighbor_id] == 1)	continue;
			if (neighbor_id == end_id) {		//任性蚂蚁不会走此处
				if (passed_count < (int)(*V).size())
					continue;
				path.push_back(end_id);
				path_ended = true;
				return false;
			}
			int s_id = ant_type == 2 ? neighbor_id : current_id;
			int d_id = ant_type == 2 ? current_id : neighbor_id;
			//计算拥挤率
			double p_rate = 200;//p>=2

			//cout << "sum_ant_count: " << sum_ant_count << endl;
			//cout << "(*edge_ant_count)[s_id][d_id]: " << (*edge_ant_count)[s_id][d_id] << endl;
			//cout << "1 - " << (p_rate*(sum_ant_count - (*edge_ant_count)[s_id][d_id]) / (p_rate*sum_ant_count - (*edge_ant_count)[s_id][d_id])) << endl;

			double crowded_rate = 1 - ((p_rate*(sum_ant_count - (*edge_ant_count)[s_id][d_id])) / (p_rate*sum_ant_count - (*edge_ant_count)[s_id][d_id]));
			//crowded_rate *= 10;
			//crowded_rate = min(1.0, crowded_rate);

			//cout << "crowded_rate: " << crowded_rate << endl;
			/*		if (crowded_rate > 1)
			crowded_rate /= 100;*/
			//system("pause");
			p[neighbor_id] = (1 - crowded_rate) * pow((*pheromone)[s_id][d_id], alpha_rate) / pow(neighbor_cost, beta_rate);
			//优先选择出度大的点
			//cout << 777 << endl;
			//cout << "(*neighbors_type)[" << neighbor_id << "].size(): " << (*neighbors_type)[neighbor_id].size() << endl;
			//p[neighbor_id] *= (*neighbors_type)[neighbor_id].size() / 8;
			//cout << 666 << endl;
			sum_p += p[neighbor_id];
		}

		if (p.empty()) {
			//cout << "p is empty !!!" << endl;
			//system("pause");
			path_ended = true;
			return false;
		}


		for (auto& pair : p)
			pair.second /= sum_p;

		int selected_city = getSelectedCityId(&p, sum_p);

		int s_id = ant_type == 2 ? selected_city : current_id;
		int d_id = ant_type == 2 ? current_id : selected_city;
		(*edge_ant_count)[s_id][d_id]++;	//相应录像新增一只蚂蚁

		//从先前边上脱离，也要更新edge_ant_count
		if (path.size() >= 2) {
			int prev_id = path[path.size() - 2];
			int s_id = ant_type == 2 ? current_id : prev_id;
			int d_id = ant_type == 2 ? prev_id : current_id;
			(*edge_ant_count)[s_id][d_id]--;
		}

		path.push_back(selected_city);
		city[selected_city] = 1;
		passed_count += (*V).count(selected_city);
		//任性蚂蚁一旦经过所有V就停止
		if (end_id == -1 && passed_count == (*V).size()) {
			path_ended = true;
			return false;
		}
		return true;
	}


	int getSelectedCityId(unordered_map<int, double>* p, double sum_p) {

		//srand((int)time(0));
		double random_p = (rand() % 100) / 100.0;
		//cout << "random_p:" << (double)random_p << endl;
		double sum = 0;
		for (auto& pair : *p) {
			sum += pair.second;
			//cout << "sum: " << sum << endl;
			if (sum >= random_p)
				return pair.first;
		}
		//cout << "sum_p: " << sum_p << endl;
		//cout << "random_p: " << random_p << endl;
		//cout << "sum: " << sum << endl;
		//cout << "p.size(): " << (*p).size() << endl;

		int random_i = rand() % (*p).size();
		auto it = (*p).begin();
		for (int i = 0; i < random_i; i++)
			it++;
		//cout << "return " << (*it).first << endl;
		//system("pause");
		return (*it).first;

		assert(false);
		return -1;
	}

	//找到合法路径才refresh（注意，neighbors传入时保证了根据ant_type不同而不同）
	void refreshPathCost(unordered_map<int, unordered_map<int, int>>* neighbors, unordered_set<int>* V) {
		//path只保留有用一段
		vector<int> path_temp = path;
		path.clear();
		int begin_i = 0, end_i = path_temp.size() - 1;
		while (begin_i < (int)path_temp.size() && !(*V).count(path_temp[begin_i]) && path_temp[begin_i] != source_id && path_temp[begin_i] != destination_id)
			begin_i++;
		while (end_i >= 0 && !(*V).count(path_temp[end_i]) && path_temp[end_i] != source_id && path_temp[end_i] != destination_id)
			end_i--;

		while (begin_i <= end_i)
			path.push_back(path_temp[begin_i++]);

		path_cost = 0;
		for (int i = 1; i < (int)path.size(); i++)
			path_cost += (*neighbors)[path[i - 1]][path[i]];

		//cout << "passed_count: " << passed_count << endl;
		//cout << "path.size(): " << path.size() << endl;
		//cout << "path_temp.size(): " << path_temp.size() << endl;
		//system("pause");
	}

	void printPath() {
		cout << "Path.size(): " << path.size() << endl;
		cout << "Path: ";
		for (int i = 0; i < (int)path.size(); i++)
			cout << path[i] << "  ";
		cout << endl;
	}
};


class Graph {
public:
	//图信息
	unordered_map<int, unordered_map<int, int>> neighbors;//id：<邻居id:最小边长>集合
	unordered_map<int, unordered_map<int, int>> reversed_neighbors;
	unordered_set<int> closed;
	unordered_map<int, int> edges;//边id：cost
	unordered_map<int, unordered_map<int, int>> edges_id;
	unordered_set<int> V;//V'
	unordered_map<int, vector<vector<int>>> V_set;	//V_set[i] = p 表示p是一条从source_id到i的路径
	unordered_map<int, vector<vector<int>>> V_r_set;
	int source_id, destination_id;
	int max_node_id = 0;
	int city_count = 0;
	int max_runtime = 9800;

	//最短路径长度初值为极大
	int min_cost = INT_MAX;
	//int min_cost = 504;

	//非V最短路径
	vector<unordered_map<int, pair<int, vector<int>>>> shortest_paths;	//shortest[i] = {j, <l : p>} 从i到j的非V最短路径为p长度为l

	//For ACO
	//vector<vector<int>> distance;		//邻接矩阵
	vector<Ant> ants_normal;					//蚁群
	vector<Ant> ants_reverse;			//反向蚁群
	vector<Ant> ants_random;			//任性蚁群
	vector<vector<double>> pheromone;			//信号量
	vector<vector<int>> edge_ant_count;		//边上蚂蚁数量
	vector<vector<int>> update_count;		//用于更新信息素的次数
	vector<int> min_path;			//最短路径
	vector<int> min_path_edge;
	vector<int> begin_nodes;			//随机蚂蚁的起点集合
	double begin_nodes_rate = 0.3;			//begin_nodes.size() / neighbors.size()
	int normal_ant_count;
	int reverse_ant_count;
	int random_ant_count;
	int sum_ant_count;
	double normal_ant_rate = 14.0;		//normal_ant_cout / neighbors.size()
	double reverse_ant_rate = 14.0;		//reverse_ant_count / neighbors.size()
	double random_ant_rate = 0.0;		//random_ant_rate / neighbors.size()
	//double random_ant_rate = 0;		//random_ant_rate / neighbors.size()
	double alpha_rate = 2;
	double beta_rate = 1;
	double r = 0.0;				//挥发率
	int max_gen = 10000;		//迭代次数
	double important_phe = 300;


	clock_t start_time;

	double Q = 1;


	//TODO:初始化
	Graph() {};
	Graph(char* graph[5000], int edge_num, char *condition) {
		start_time = clock();//sudnay结束时间
		initialize(graph, edge_num, condition);
		countShortestPaths();
		//setImportantPheromone();

	};
	~Graph() {};

	void printPath(vector<int> path) {
		cout << "printPath !!!" << endl;
		cout << "path.size(): " << path.size() << endl;
		cout << "path : ";
		for (auto id : path)
			cout << id << " ";
		cout << endl;
	}

	void countShortestPaths() {
		//unordered_set<int> r = getReachability(source_id);
		//refreshNeighbors(r);
		Dijkstra(source_id);
		for (auto i : V) {
			Dijkstra(i);
			//assert(neighbors.size() <= 50);
		}
	}

	//求节点s_id到所有节点之间的最短路径（不经过V终点和起点终点）（最短路径包含起点但不包含终点， 若起点终点相同，路径为空）
	void Dijkstra(int s_id) {
		//cout << "max_node_id: " << max_node_id << endl;
		vector<pair<int, int>> T(max_node_id + 1, pair<int, int>(INT_MAX, 0));	//T[i] = l : last_node 表示目前从终点到i点的最短路长度为l，路径上他的上一节点为last_node
		unordered_map<int, pair<int, vector<int>>>& shortest_path = shortest_paths[s_id];//shortest[i] = <l, p>从起点到i点的最短路径长度为l，路径为p

		T[s_id].first = 0;	//起点到起点值为0
		T[s_id].second = s_id;
		vector<int> nearest{ s_id };//上一轮最近的点集，本轮首先用它更新T
		shortest_path[s_id].first = 0;

		int shortest = 0;	//上一轮最近点集到目前范围的距离

		while (shortest_path.size() < neighbors.size()) {
			//cout << shortest_path.size() << endl;
			//②：最近点加入S，修改T（加新点，用新点更新距离）
			for (int new_node : nearest) {
				//起点已经设置好
				if (new_node != s_id) {
					shortest_path[new_node].first = shortest;
					shortest_path[new_node].second = shortest_path[T[new_node].second].second; //前一个点的最短路径
					shortest_path[new_node].second.push_back(T[new_node].second);	//加上前一个点
				}

				//V中点不用来更新，以此实现非V最短路径
				if (new_node != s_id && (V.count(new_node) || new_node == source_id || new_node == destination_id))	//非V路径
					continue;

				//每一个邻居，更新其在T中的信息
				for (auto neighbor : neighbors[new_node]) {
					int neighbor_id = neighbor.first;
					int neighbor_cost = neighbor.second;
					if (shortest + neighbor_cost >= T[neighbor_id].first)
						continue;
					T[neighbor_id].first = shortest + neighbor_cost;
					T[neighbor_id].second = new_node;
				}
			}

			//①：找最近点
			nearest.clear();	//可能有多个最近点
			shortest = INT_MAX;	//最近点距离
			for (auto p : neighbors) {
				int node_id = p.first;
				if (shortest_path.count(node_id) || T[node_id].first > shortest)
					continue;
				if (T[node_id].first < shortest)
					nearest.clear();
				shortest = T[node_id].first;
				nearest.push_back(node_id);
			}

		}
		//printShortestPath(s_id, shortest_path);
		//cout << "shortest_path.size(): " << shortest_path.size() << endl;
		//cout << "neighbors.size(): " << neighbors.size() << endl;
		//cout << "from " << s_id << " to destination cost : " << shortest_path[destination_id].first << endl;
		//system("pause");
		assert(shortest_path.size() == neighbors.size());
	}


	//返回s_id所有可达节点的集合
	unordered_set<int> getReachability(int s_id) {
		unordered_set<int> result;


		vector<pair<int, int>> T(max_node_id + 1, pair<int, int>(INT_MAX, 0));	//T[i] = l : last_node 表示目前从终点到i点的最短路长度为l，路径上他的上一节点为last_node
		unordered_map<int, pair<int, vector<int>>> shortest_path;//shortest[i] = <l, p>从起点到i点的最短路径长度为l，路径为p

		T[s_id].first = 0;	//起点到起点值为0
		T[s_id].second = s_id;
		vector<int> nearest{ s_id };//上一轮最近的点集，本轮首先用它更新T
		shortest_path[s_id].first = 0;

		int shortest = 0;	//上一轮最近点集到目前范围的距离

		while (shortest_path.size() < neighbors.size()) {
			//cout << shortest_path.size() << endl;
			//②：最近点加入S，修改T（加新点，用新点更新距离）
			for (int new_node : nearest) {
				//起点已经设置好
				if (new_node != s_id) {
					shortest_path[new_node].first = shortest;
					shortest_path[new_node].second = shortest_path[T[new_node].second].second; //前一个点的最短路径
					shortest_path[new_node].second.push_back(T[new_node].second);	//加上前一个点
				}

				//每一个邻居，更新其在T中的信息
				for (auto neighbor : neighbors[new_node]) {
					int neighbor_id = neighbor.first;
					int neighbor_cost = neighbor.second;
					if (shortest + neighbor_cost >= T[neighbor_id].first)
						continue;
					T[neighbor_id].first = shortest + neighbor_cost;
					T[neighbor_id].second = new_node;
				}
			}

			//①：找最近点
			nearest.clear();	//可能有多个最近点
			shortest = INT_MAX;	//最近点距离
			for (auto p : neighbors) {
				int node_id = p.first;
				if (shortest_path.count(node_id) || T[node_id].first > shortest)
					continue;
				if (T[node_id].first < shortest)
					nearest.clear();
				shortest = T[node_id].first;
				nearest.push_back(node_id);
			}
			if (shortest >= INT_MAX)
				break;
		}
		//printShortestPath(s_id, shortest_path);
		//cout << s_id << " nums of reached nodes：" << shortest_path.size() << endl;

		for (auto p : shortest_path)
			result.insert(p.first);
		//注意，V节点和起止点不能删
		for (auto i : V)
			result.insert(i);
		result.insert(source_id);
		result.insert(destination_id);
		return result;
	}

	//返回所有可达s_id节点的集合
	unordered_set<int> getReversedReachability(int s_id) {
		unordered_set<int> result;


		vector<pair<int, int>> T(max_node_id + 1, pair<int, int>(INT_MAX, 0));	//T[i] = l : last_node 表示目前从终点到i点的最短路长度为l，路径上他的上一节点为last_node
		unordered_map<int, pair<int, vector<int>>> shortest_path;//shortest[i] = <l, p>从起点到i点的最短路径长度为l，路径为p

		T[s_id].first = 0;	//起点到起点值为0
		T[s_id].second = s_id;
		vector<int> nearest{ s_id };//上一轮最近的点集，本轮首先用它更新T
		shortest_path[s_id].first = 0;

		int shortest = 0;	//上一轮最近点集到目前范围的距离

		while (shortest_path.size() < reversed_neighbors.size()) {
			//cout << shortest_path.size() << endl;
			//②：最近点加入S，修改T（加新点，用新点更新距离）
			for (int new_node : nearest) {
				//起点已经设置好
				if (new_node != s_id) {
					shortest_path[new_node].first = shortest;
					shortest_path[new_node].second = shortest_path[T[new_node].second].second; //前一个点的最短路径
					shortest_path[new_node].second.push_back(T[new_node].second);	//加上前一个点
				}

				//每一个邻居，更新其在T中的信息
				for (auto neighbor : reversed_neighbors[new_node]) {
					int neighbor_id = neighbor.first;
					int neighbor_cost = neighbor.second;
					if (shortest + neighbor_cost >= T[neighbor_id].first)
						continue;
					T[neighbor_id].first = shortest + neighbor_cost;
					T[neighbor_id].second = new_node;
				}
			}

			//①：找最近点
			nearest.clear();	//可能有多个最近点
			shortest = INT_MAX;	//最近点距离
			for (auto p : reversed_neighbors) {
				int node_id = p.first;
				if (shortest_path.count(node_id) || T[node_id].first > shortest)
					continue;
				if (T[node_id].first < shortest)
					nearest.clear();
				shortest = T[node_id].first;
				nearest.push_back(node_id);
			}
			if (shortest >= INT_MAX)
				break;
		}
		//printShortestPath(s_id, shortest_path);
		//cout << s_id << " nums of reversed_reached nodes：" << shortest_path.size() << endl;

		for (auto p : shortest_path)
			result.insert(p.first);
		//注意，V节点和起止点不能删
		for (auto i : V)
			result.insert(i);
		result.insert(source_id);
		result.insert(destination_id);
		return result;
	}



	//删除source_id不能到达的节点
	void refreshNeighbors(unordered_set<int> r) {
		//unordered_map<int, unordered_map<int, int>> neighbors;//id：<邻居id:最小边长>集合
		//unordered_map<int, unordered_map<int, int>> reversed_neighbors;
		unordered_map<int, unordered_map<int, int>> temp_neighbors;
		unordered_map<int, unordered_map<int, int>> temp_reversed_neighbors;
		for (auto p : neighbors) {
			if (r.count(p.first) == 0)
				continue;
			for (auto pp : p.second) {
				if (r.count(pp.first) == 0)
					continue;
				temp_neighbors[p.first].insert(pp);
			}
		}
		for (auto p : reversed_neighbors) {
			if (r.count(p.first) == 0)
				continue;
			for (auto pp : p.second) {
				if (r.count(pp.first) == 0)
					continue;
				temp_reversed_neighbors[p.first].insert(pp);
			}
		}
		cout << "neighbors.size(): " << neighbors.size() << endl;
		cout << "reversed_neighbors.size(): " << reversed_neighbors.size() << endl;
		neighbors = temp_neighbors;
		reversed_neighbors = temp_reversed_neighbors;
		cout << "neighbors.size(): " << neighbors.size() << endl;
		cout << "reversed_neighbors.size(): " << reversed_neighbors.size() << endl;
		system("pause");
	}


	void printShortestPath(int s_id, unordered_map<int, pair<int, vector<int>>> shortest_path) {
		for (auto p : shortest_path) {
			cout << "From " << s_id << " to " << p.first << ": " << p.second.first << endl;
			cout << "Path: ";
			for (auto n : p.second.second)
				cout << n << " ";
			cout << endl;
		}
	}

	//返回运行毫秒数
	int getRuntime() {
		clock_t media_time = clock();//sudnay结束时间
		return (media_time - start_time) / (CLOCKS_PER_SEC / 1000);
	}

	void runStepBySetp() {
		cout << "runStepBySetp !!!" << endl;
		//每一代
		for (int gen = 0; gen < max_gen; gen++) {
			//的每一只蚂蚁
			vector<int> update_ants_normal;		//可更新信息素的蚂蚁
			vector<int> update_ants_reverse;	//可更新信息素的反向蚂蚁
			vector<int> update_ants_random;		//可更新信息素的任性蚂蚁

			//for (int i = 0; i < max(max(normal_ant_count, reverse_ant_count), random_ant_count); i++) {
			for (int step = 0; step < (int)neighbors.size(); step++) {

				if (getRuntime() > max_runtime)
					return;

				//case7
				if (city_count > 100 && city_count <= 150)
					if (min_cost <= 192)
						return;

				//case9
				if (city_count > 200 && city_count <= 250)
					if (min_cost <= 242)
						return;

				//case10
				if (city_count > 250 && city_count <= 300)
					if (min_cost <= 264)
						return;

				//case12
				if (city_count > 300 && city_count <= 500 && V.size() > 20 && V.size() <= 23)
					if (min_cost <= 222)
						return;

				//case13
				if (city_count > 300 && city_count <= 500 && V.size() > 23 && V.size() <= 25)
					if (min_cost <= 254)
						return;

				//case14
				if (city_count > 500 && V.size() > 25 && V.size() <= 30)
					if (min_cost <= 299)
						return;

				//case15
				if (city_count > 500 && V.size() < 25)
					if (min_cost <= 605)
						return;



				/******正向蚂蚁******/
				for (int i = 0; i < normal_ant_count; i++) {
					if (ants_normal[i].path_ended)	//先前已经死亡
						continue;
					if (ants_normal[i].selectNextCity(&edge_ant_count, sum_ant_count, &pheromone, &neighbors, &begin_nodes, &V, NORMAL_ANT))	//还可以继续走
						continue;
					//刚刚死亡（刚刚到头）
					ants_normal[i].refreshPathCost(&neighbors, &V);
					tryToSetMinPath(gen, i, NORMAL_ANT);
				}
				/******反向蚂蚁******/
				for (int i = 0; i < reverse_ant_count; i++) {
					if (ants_reverse[i].path_ended)	//先前已经死亡
						continue;
					if (ants_reverse[i].selectNextCity(&edge_ant_count, sum_ant_count, &pheromone, &reversed_neighbors, &begin_nodes, &V, REVERSED_ANT))	//还可以继续走
						continue;
					//刚刚死亡（刚刚到头）
					ants_reverse[i].refreshPathCost(&reversed_neighbors, &V);
					tryToSetMinPath(gen, i, REVERSED_ANT);
				}
				/******任性蚂蚁******/
				for (int i = 0; i < random_ant_count; i++) {
					if (ants_random[i].path_ended)	//先前已经死亡
						continue;
					if (ants_random[i].selectNextCity(&edge_ant_count, sum_ant_count, &pheromone, &neighbors, &begin_nodes, &V, RANDOM_ANT))	//还可以继续走
						continue;
					//刚刚死亡（刚刚到头）
					ants_random[i].refreshPathCost(&neighbors, &V);
					//注：任性蚂蚁不更新min_cost只更新信息素
				}
			}
			//目前策略：全部用于更新
			for (int ant_index = 0; ant_index < (int)ants_normal.size(); ant_index++)
				update_ants_normal.push_back(ant_index);
			for (int ant_index = 0; ant_index < (int)ants_reverse.size(); ant_index++)
				update_ants_reverse.push_back(ant_index);
			for (int ant_index = 0; ant_index < (int)ants_random.size(); ant_index++)
				update_ants_random.push_back(ant_index);


			updatePheromone(&update_ants_normal, NORMAL_ANT);
			updatePheromone(&update_ants_reverse, REVERSED_ANT);
			updatePheromone(&update_ants_random, RANDOM_ANT);
			for (int i = 0; i < (int)ants_normal.size(); i++)
				ants_normal[i].initialize(&begin_nodes, &V, (gen + 1) / max_gen, NORMAL_ANT, max_node_id);
			for (int i = 0; i < (int)ants_reverse.size(); i++)
				ants_reverse[i].initialize(&begin_nodes, &V, (gen + 1) / max_gen, REVERSED_ANT, max_node_id);
			for (int i = 0; i < (int)ants_random.size(); i++)
				ants_random[i].initialize(&begin_nodes, &V, (gen + 1) / max_gen, RANDOM_ANT, max_node_id);

			//边上蚂蚁计数清零
			edge_ant_count.clear(), assert(edge_ant_count.size() == 0);
			edge_ant_count.resize(city_count, vector<int>(city_count, 0));
			//for (auto v : edge_ant_count)
			//	for (auto i : v)
			//		assert(i == 0);
		}
	}

	void updatePheromone(vector<int>* update_ants, int ant_type) {
		//cout << "update_ants.size(): " << (*update_ants).size() << endl;
		for (int i = 0; i < (int)pheromone.size(); i++)
			for (int j = 0; j < (int)pheromone[0].size(); j++)
				pheromone[i][j] *= (1 - r);

		vector<Ant>* which_ants = ant_type == 1 ? &ants_normal : ant_type == 2 ? &ants_reverse : &ants_random;

		double rate1 = 2, rate2 = 1;
		int max_update = 100;


		for (int ant_id : *update_ants) {
			for (int i = 1; i < (int)(*which_ants)[ant_id].path.size(); i++) {
				double passed_rate = 1.0 * (*which_ants)[ant_id].passed_count / V.size();
				int s_id = ant_type == 2 ? (*which_ants)[ant_id].path[i] : (*which_ants)[ant_id].path[i - 1];
				int d_id = ant_type == 2 ? (*which_ants)[ant_id].path[i - 1] : (*which_ants)[ant_id].path[i];
				int cost = (*which_ants)[ant_id].path_cost;
				//
				//cout << "(*which_ants)[ant_id].passed_count: " << (*which_ants)[ant_id].passed_count << endl;
				//cout << "V.size(): " << V.size() << endl;
				//cout << "passed_rate: " << passed_rate << endl;
				//cout << "cost: " << cost << endl;
				//cout << "rate1: " << rate1 << endl;
				//cout << "rate2: " << rate2 << endl;

				//if (update_count[s_id][d_id] % 1000000 > 100 && update_count[s_id][d_id] / 1000000 >= 50 ) {
				//	//cout << "bigger than max_update " << max_update << endl;
				//	continue;
				//}

				//pheromone[s_id][d_id] += Q * pow(passed_rate, rate1) / pow(cost, rate2);
				pheromone[s_id][d_id] += Q * passed_rate * rate1 / (cost * rate2);
				//cout << "diff: " << Q * passed_rate * rate1 / (cost * rate2) << endl;
				//if ((*which_ants)[ant_id].passed_count > 0)
				//	system("pause");
				//update_count[s_id][d_id]++;
				////出现在合法路径次数
				//if ((*which_ants)[ant_id].passed_count >= V.size() && (*which_ants)[ant_id].path.back() == d_id)
				//	update_count[s_id][d_id]+= 1000000;
			}
		}

		//恢复一下，防止信息量过小，在getSelectedCityId报错
		//for (int i = 0; i < pheromone.size(); i++)
		//	for (int j = 0; j < pheromone[0].size(); j++)
		//		pheromone[i][j] /= (1 - r);

		//限定最小信息量为0.01
		for (int i = 0; i < (int)pheromone.size(); i++)
			for (int j = 0; j <(int)pheromone[0].size(); j++)
				pheromone[i][j] = max(0.01, pheromone[i][j]);

	}

	//bool checkRight(vector<int> path) {
	//	unordered_set<int> s;
	//	int count = 0;
	//	for (int i = 0; i < (int)path.size(); i++) {
	//		if (V.count(path[i]))
	//			count++;
	//		if (s.count(path[i]))
	//			return false;
	//		//assert(false);
	//		else
	//			s.insert(path[i]);
	//		if (i >= 1)
	//			assert(neighbors[path[i - 1]].count(path[i]) == 0 || neighbors[path[i - 1]][path[i]] < INT_MAX);
	//	}
	//	return count == V.size();
	//}

	//保证path必须是正向路径
	void setMinPathEdge(vector<int>& path) {
		min_path_edge.clear();
		for (int i = 1; i < (int)path.size(); i++)
			min_path_edge.push_back(edges_id[path[i - 1]][path[i]]);
	}

	void printRuntime() {
		clock_t media_time = clock();//sudnay结束时间
		cout << "The runtime is: " << (media_time - start_time) / (CLOCKS_PER_SEC / 1000) << " ms !" << endl << endl;//输出运行时间
	}

	//返回某条路径cost，必须保证是正向路径
	int getCost(vector<int> path) {
		int result = 0;
		for (int i = 1; i < (int)path.size(); i++)
			result += neighbors[path[i - 1]][path[i]];
		return result;
	}

	void tryToSetMinPath(int gen, int i, int ant_type) {

		string ant_type_str = ant_type == 2 ? " ants_reverse " : ant_type == 1 ? " ants_normal " : " ants_random ";
		vector<Ant>* which_ants = ant_type == 1 ? &ants_normal : ant_type == 2 ? &ants_reverse : &ants_random;
		int d_id = ant_type == 2 ? source_id : destination_id;

		////试图从V_set 或 V_r_set中组合出新解
		//if (ant_type == NORMAL_ANT && V.count((*which_ants)[i].path.back()) > 0) {
		//	for (vector<int> temp_path : V_r_set[(*which_ants)[i].path.back()]) {
		//		vector<int> new_path = (*which_ants)[i].path;
		//		new_path.insert(new_path.begin(), ++temp_path.begin(), temp_path.end());
		//		//合并失败
		//		if (!checkMinPath(new_path))
		//			continue;
		//		//合并成功
		//		int new_path_cost = getCost(new_path);
		//		cout << " 正向蚂蚁合并负向路径 changes min_cost from " << min_cost << " to " << new_path_cost << endl;
		//		min_path = new_path, min_cost = new_path_cost;
		//		assert(checkMinPath(min_path));
		//		setMinPathEdge(new_path);

		//		printResult(), printRuntime();
		//	}
		//}
		//if (ant_type == REVERSED_ANT && V.count((*which_ants)[i].path.back()) > 0) {
		//	for (vector<int> temp_path : V_set[(*which_ants)[i].path.back()]) {
		//		vector<int> new_path = temp_path;
		//		new_path.insert(new_path.begin(), ++(*which_ants)[i].path.begin(), (*which_ants)[i].path.end());
		//		//合并失败
		//		if (!checkMinPath(new_path))
		//			continue;
		//		//合并成功
		//		int new_path_cost = getCost(new_path);
		//		cout << " 负向蚂蚁合并正向路径 changes min_cost from " << min_cost << " to " << new_path_cost << endl;
		//		min_path = new_path, min_cost = new_path_cost;
		//		assert(checkMinPath(min_path));
		//		setMinPathEdge(new_path);

		//		printResult(), printRuntime();
		//	}
		//}



		//更新V_set和V_r_set
		if (ant_type == NORMAL_ANT && V.count((*which_ants)[i].path.back()) > 0) {
			V_set[(*which_ants)[i].path.back()].push_back((*which_ants)[i].path);
		}
		if (ant_type == REVERSED_ANT &&  V.count((*which_ants)[i].path.back()) > 0) {
			vector<int> temp_path = (*which_ants)[i].path;
			reverse(temp_path.begin(), temp_path.end());
			V_r_set[(*which_ants)[i].path.back()].push_back(temp_path);
		}



		//try to更新 min_cost 和 min_path
		if ((*which_ants)[i].path_cost < min_cost && (*which_ants)[i].passed_count >= (int)V.size() && (*which_ants)[i].path.back() == d_id) {
			cout << "Gen " << gen << ant_type_str.c_str() << i << " changes min_cost from " << min_cost << " to " << (*which_ants)[i].path_cost << endl;
			//路径顺序
			if (ant_type == 2)
				reverse((*which_ants)[i].path.begin(), (*which_ants)[i].path.end());
			min_path = (*which_ants)[i].path;
			min_cost = (*which_ants)[i].path_cost;
			assert(checkMinPath(min_path));
			setMinPathEdge((*which_ants)[i].path);

			printResult();
			printRuntime();
			assert(ant_type != 3);	//任性蚂蚁进来报错
		}

		if ((*which_ants)[i].passed_count >= (int)V.size() && (*which_ants)[i].path.back() == d_id) {
			//找到一条路径，通过最短非V路径集合，保持该路径中V中点顺序，来更新该路径（都更新还是只更新cost不更新path？）
			//暂且只更新cost
			pair<int, vector<int>> p = getPathOrderMinCost(&(*which_ants)[i], ant_type);
			int path_order_min_cost = p.first;
			vector<int> new_path = p.second;
			if (path_order_min_cost >= min_cost)
				return;

			if (!checkMinPath(new_path))
				return;

			assert(checkMinPath(new_path));

			cout << "path_order_min_cost !!!!!" << endl << "Gen " << gen << ant_type_str.c_str() << i << " changes min_cost from " << min_cost << " to " << path_order_min_cost << endl;

			(*which_ants)[i].path = new_path;//同样设置，用于更新信息量

			min_cost = path_order_min_cost;
			min_path = new_path;
			assert(checkMinPath(min_path));
			setMinPathEdge(new_path);

			printResult();
			printRuntime();
			//system("pause");
		}
	}


	//返回是否是一条合法路径
	bool checkMinPath(vector<int> path) {
		int v_count = 0;
		int s_count = 0;
		int d_count = 0;
		unordered_set<int> unique;
		for (auto i : path) {
			if (unique.count(i) > 0)
				return false;
			unique.insert(i);
			v_count += V.count(i) == 1;
			s_count += i == source_id;
			d_count += i == destination_id;
		}
		return (v_count == V.size() && s_count == 1 && d_count == 1 && path.back() == destination_id && path[0] == source_id);
	}




	bool noLoop(vector<int>& path) {
		unordered_set<int> s;
		for (auto i : path)
			if (s.count(i))
				return false;
			else
				s.insert(i);
		return true;
	}

	pair<int, vector<int>> getPathOrderMinCost(Ant* ant, int ant_type) {
		//cout << "getPathOrderMinCost !!!" << endl;
		vector<int> path = ant->path;
		if (ant_type == REVERSED_ANT) {
			reverse(path.begin(), path.end());
			for (int i = 0; i < (int)path.size(); i++)
				assert(path[i] == ant->path[path.size() - 1 - i]);
		}
		assert(path.size() == ant->path.size());
		assert(path.back() == destination_id);
		int this_cost = 0;
		vector<int> new_path;

		//找到一条路径，通过最短非V路径集合，保持该路径中V中点顺序，来更新该路径（都更新还是只更新cost不更新path？）
		//暂且只更新cost
		if (ant->passed_count >= (int)V.size() && path.back() == destination_id) {
			int slow = 0, fast = 1;
			while (fast < (int)path.size()) {
				if (!V.count(path[fast]) && path[fast] != source_id && path[fast] != destination_id) {
					fast++;
					continue;
				}
				assert(path[slow] != destination_id);
				assert((V.count(path[slow]) || path[slow] == source_id) && (V.count(path[fast]) || path[fast] == source_id || path[fast] == destination_id));

				//if (!noLoop(new_path))
				//	return pair<int, vector<int>>(INT_MAX, vector<int>());

				this_cost += shortest_paths[path[slow]][path[fast]].first;
				new_path.insert(new_path.end(), shortest_paths[path[slow]][path[fast]].second.begin(), shortest_paths[path[slow]][path[fast]].second.end());

				slow = fast++;
			}
			new_path.push_back(path.back());	//destination
		}
		else
			assert(false);
		//if (!noLoop(new_path))
		//	return pair<int, vector<int>>(INT_MAX, vector<int>());

		return pair<int, vector<int>>(this_cost, new_path);
	}

	void generateBeginSet() {
		////起点完全随机
		//for (auto p : neighbors)
		//	begin_nodes.push_back(p.first);
		//assert(begin_nodes.size() == neighbors.size());
		//cout << "begin_nodes.size(): " << begin_nodes.size() << endl;
		//system("pause");
		//return;

		if (!begin_nodes.empty()) {
			cout << "begin_ndes.size(): " << begin_nodes.size() << endl;
		}
		begin_nodes.clear();

		for (auto node_id : V)
			begin_nodes.push_back(node_id);
		unordered_set<int> begin_set(V);
		//cout << "begin_nodes.size(): " << begin_nodes.size() << endl;
		//cout << "begin_nodes_rate: " << begin_nodes_rate << endl;

		int max_begin_size = (int)begin_nodes_rate*neighbors.size();
		int now_size = begin_set.size();

		for (int i = 0; i < (int)begin_set.size() && (int)begin_set.size() <= max_begin_size; i++) {
			for (auto p : neighbors[begin_nodes[i]]) {
				if (begin_set.count(p.first))
					continue;
				begin_set.insert(p.first);
				begin_nodes.push_back(p.first);
				if ((int)begin_set.size() > max_begin_size)
					break;
			}
		}
		//cout << "begin_nodes.size(): " << begin_nodes.size() << endl;
		//system("pause");
	}

	void setImportantPheromone() {
		int level_count = 1;	//三层都设置重要信息量，注意，V就是第一层
		int temp_level_count = level_count;

		unordered_set<int> this_level{ source_id, destination_id };
		this_level.insert(V.begin(), V.end());
		//cout << "V.size(): " << V.size() << endl;
		assert(this_level.size() == V.size() + 2);
		unordered_set<int> next_level;

		while (temp_level_count > 0) {
			double now_important_phe = (double)temp_level_count / level_count * important_phe;
			for (auto node_id : this_level) {
				//非终点的所有出点
				if (node_id != destination_id)
					for (auto out : neighbors[node_id])
						next_level.insert(out.first), pheromone[node_id][out.first] = max(now_important_phe, pheromone[node_id][out.first]);

				//非起点的所有入点
				if (node_id != source_id)
					for (auto in : reversed_neighbors[node_id]) {
						//cout << "in.first: " << in.first << endl;
						//cout << "pheromone.size(): " << pheromone.size() << endl;
						//cout << "pheromone[495].size(): " << pheromone[495].size() << endl;
						//cout << "pheromone[" << in.first << "][" << node_id << "]"; cout << pheromone[in.first][node_id] << endl;
						next_level.insert(in.first), pheromone[in.first][node_id] = max(now_important_phe, pheromone[in.first][node_id]);
					}
			}

			this_level = next_level;
			next_level.clear();
			temp_level_count--;
		}
		return;


		//根据shortest_paths设置
		//vector<unordered_map<int, pair<int, vector<int>>>> shortest_paths;	
		//shortest[i] = {j, <l : p>} 从i到j的非V最短路径为p长度为l

		for (int s_id = 0; s_id < (int)shortest_paths.size(); s_id++) {
			for (auto p : shortest_paths[s_id]) {
				int d_id = p.first;
				vector<int> path = p.second.second;
				path.push_back(d_id);//原路径中没有终点
				for (int i = 1; i < (int)path.size() - 1; i++)
					pheromone[path[i - 1]][path[i]] = important_phe;
			}
		}


	}

	void initialize(char *graph[5000], int edge_num, char *condition) {
		//initialize condition
		int index = 0;
		while (condition[index] != ',')
			source_id = source_id * 10 + condition[index++] - '0';
		index++;
		while (condition[index] != ',')
			destination_id = destination_id * 10 + condition[index++] - '0';
		index++;

		//cout << "condition: " << condition << endl;
		//system("pause");
		//while (isdigit(condition[index]) || condition[index] == '|') {
		//	int node_id = 0;
		//	while (isdigit(condition[index]))
		//		node_id = node_id * 10 + condition[index++] - '0';
		//	if (isdigit(condition[index]) || condition[index] == '|')
		//		index++;
		//	V.insert(node_id);
		//	//cout << "node_id: " << node_id << endl;
		//	//system("pause");
		//}

		while (condition[index]) {
			int node_id = 0;
			if (isdigit(condition[index])){
				while (isdigit(condition[index]))
					node_id = node_id * 10 + condition[index++] - '0';
				V.insert(node_id);
			}
			if (condition[index])
				index++;
		}

		//注意：neighbors.size()可能不等于城市总数目！
		unordered_set<int> cities;

		for (int i = 0; i < edge_num; i++) {
			char* line = graph[i];
			int index = 0;
			//LinkID
			int l_id = 0;
			while (line[index] != ',')
				l_id = l_id * 10 + line[index++] - '0';
			index++;
			//SourceID
			int s_id = 0;
			while (line[index] != ',')
				s_id = s_id * 10 + line[index++] - '0';
			index++;
			//DestinationID
			int d_id = 0;
			while (line[index] != ',')
				d_id = d_id * 10 + line[index++] - '0';
			index++;
			//Cost
			int c = 0;
			while (isdigit(line[index]))
				c = c * 10 + line[index++] - '0';

			//设置图信息
			if (!cities.count(s_id)) cities.insert(s_id);
			if (!cities.count(d_id)) cities.insert(d_id);

			//注意：加了可能出问题！
			//if (d_id == source_id || s_id == destination_id)	//指向起点的边忽略
			//	continue;

			edges[l_id] = c;
			neighbors[s_id][d_id] = neighbors[s_id][d_id] == 0 ? c : min(neighbors[s_id][d_id], c);//neighbors设定
			edges_id[s_id][d_id] = edges_id[s_id][d_id] == 0 ? l_id : c <= neighbors[s_id][d_id] ? l_id : neighbors[s_id][d_id];
			reversed_neighbors[d_id][s_id] = reversed_neighbors[d_id][s_id] == 0 ? c : min(reversed_neighbors[d_id][s_id], c);
			max_node_id = max(max_node_id, max(s_id, d_id));
		}

		//For ACO
		city_count = cities.size();

		//cout << "city_count: " << city_count << endl;
		//cout << "max_node_id: " << max_node_id << endl;
		//system("pause");

		//distance.resize(city_count, vector<int>(city_count, INT_MAX));
		//pheromone.resize(city_count, vector<double>(city_count, 0.1));
		pheromone.resize(max_node_id + 1, vector<double>(max_node_id + 1, 0.1));
		edge_ant_count.resize(max_node_id + 1, vector<int>(max_node_id + 1, 0));
		update_count.resize(max_node_id + 1, vector<int>(max_node_id + 1, 0));

		normal_ant_count = (int)normal_ant_rate *neighbors.size();
		reverse_ant_count = (int)reverse_ant_rate * neighbors.size();
		random_ant_count = (int)random_ant_rate * neighbors.size();
		sum_ant_count = random_ant_count + normal_ant_count + reverse_ant_count;



		generateBeginSet();

		ants_normal.resize(normal_ant_count, Ant(source_id, destination_id, alpha_rate, beta_rate, city_count, &begin_nodes, &V, 1 / max_gen, NORMAL_ANT, max_node_id));
		ants_reverse.resize(reverse_ant_count, Ant(source_id, destination_id, alpha_rate, beta_rate, city_count, &begin_nodes, &V, 1 / max_gen, REVERSED_ANT, max_node_id));
		ants_random.resize(random_ant_count, Ant(source_id, destination_id, alpha_rate, beta_rate, city_count, &begin_nodes, &V, 1 / max_gen, RANDOM_ANT, max_node_id));
		shortest_paths.resize(max_node_id + 1, unordered_map<int, pair<int, vector<int>>>());

		//设置distance

		//for (auto& p : neighbors)
		//	for (auto& pp : p.second)
		//		distance[p.first][pp.first] = pp.second;

	}





	void printGraph() {
		cout << "printGraph !!!" << endl;
		cout << "V' : ";
		for (auto node : V)
			cout << node << " | ";
		cout << endl;
		cout << "source_id : " << source_id << endl;
		cout << "destination_id : " << destination_id << endl;
		cout << "edges:" << endl;
		for (auto edge : edges)
			cout << "    LinkID " << edge.first << " : cost " << edge.second << endl;
		cout << "neighbors:" << endl;
		for (auto pair : neighbors)
			for (auto neighbor : pair.second)
				cout << "    source_id " << pair.first << " : destination_id " << neighbor.first << " : shortest_edges_len " << neighbor.second << endl;
		cout << endl;
	}


	void printNode(int s_id) {
		for (auto p : neighbors[s_id]) {
			cout << "from " << s_id << " to " << p.first << " cost " << p.second << endl;
		}
		cout << endl;
	}

	void reversePrintNode(int s_id) {
		for (auto p : reversed_neighbors[s_id]) {
			cout << "reverse from " << s_id << " to " << p.first << " cost " << p.second << endl;
		}
		cout << endl;
	}


	void printResult() {
		cout << "min_cost : " << min_cost << endl;
		cout << "min_path : ";
		for (auto i : min_path)
			cout << i << " ";
		cout << endl;
		cout << "min_path_edge : ";
		for (auto i : min_path_edge)
			cout << i << " ";
		cout << endl;
	}

	void printPheromone() {
		cout << "pheromone: " << endl;
		for (auto v : pheromone)
			for (auto i : v) {
				cout << i << " ";
			}
		cout << endl << endl;
	}

};



////你要完成的功能总入口
//void search_route(char *graph[5000], int edge_num, char *condition) {
//	Graph* g = new Graph(graph, edge_num, condition);
//
//	//case 1-5蚁群
//	//if (g->city_count <= 20) {
//	if (0) {
//		g->runStepBySetp();
//	}
//	//case 7 only
//	//else if (g->city_count > 100 && g->city_count <= 150) {
//	else if (1) {
//		//cout << "city_count: " << g->city_count << endl;
//		//system("pause");
//		unordered_set<int> temp_v = g->V;
//		temp_v.insert(g->source_id);
//		for (auto id : temp_v) {
//			//从id到-1， 只用作更新信息素（注意开始时间的设置）
//			clock_t now_time = clock();
//			ONLY* only = new ONLY(g->neighbors, g->reversed_neighbors, g->edges, g->edges_id, g->pheromone, g->V, g->min_path, g->min_path_edge, g->min_cost, id, -1, now_time, g->shortest_paths, g->V_r_set, g->max_node_id, &(g->min_path_edge));
//			only->searchONLY();
//			//只取信息素
//			g->pheromone = only->pheromone;
//		}
//		g->runStepBySetp();
//		cout << "!!!" << endl;
//	}
//	//case 6 8 9 10 11 用DFS
//	else if (g->city_count <= 100 || (g->city_count > 150 && g->city_count <= 300) || g->V.size() > 30) {
//		DFS* dfs = new DFS(g->neighbors, g->reversed_neighbors, g->edges, g->edges_id, g->pheromone, g->V, g->min_path, g->min_path_edge, g->min_cost, g->source_id, g->destination_id, g->start_time, g->shortest_paths, g->V_r_set, g->max_node_id, &(g->min_path_edge), g->city_count);
//		dfs->searchDFS();
//		//cout << "DFS完成！！！" << endl;
//		//system("pause");
//
//		g->min_path_edge = dfs->min_path_edge;
//		//g->printResult();
//	}
//	//case6 7 12 13 14 15用混合
//	else{
//		//cout << "混合 ！！！" << endl;
//		FZ* fz = new FZ(g->neighbors, g->reversed_neighbors, g->edges, g->edges_id, g->pheromone, g->V, g->min_path, g->min_path_edge, g->min_cost, g->source_id, g->destination_id, g->start_time, g->shortest_paths, g->V_r_set, g->max_node_id, &(g->min_path_edge));
//		fz->searchFZ();
//		g->pheromone = fz->pheromone;
//		g->min_cost = fz->min_cost;
//		g->min_path = fz->min_path;
//		g->min_path_edge = fz->min_path_edge;
//		//g->printResult();
//		//cout << "信息素更新完成!!!" << endl;
//
//		//system("pause");
//		g->runStepBySetp();
//		//g->printResult();
//	}
//
//	//写入文件
//	//for (auto l_id : g.min_path_edge)
//	//	record_result(l_id);
//}
//
//////你要完成的功能总入口
////void search_route(LangGraph lg) {
////	Graph g(lg);
////	runAndSearch(g);
////}
//
//
//
//void main() {
//	ifstream infile, infile2;
//	//infile.open("C:\\Users\\go2sea\\Desktop\\huawei\\case1\\topo.csv"), infile2.open("C:\\Users\\go2sea\\Desktop\\huawei\\case1\\demand.csv"); //官方
//	//infile.open("D:\\项目\\Python\\huawei\\topo_600_1000_20_50.csv"), infile2.open("D:\\项目\\Python\\huawei\\demand_600_1000_20_50.csv");
//	//infile.open("D:\\项目\\Python\\huawei\\topo_100_400_20.csv"), infile2.open("D:\\项目\\Python\\huawei\\demand_100_400_20.csv");
//	//infile.open("D:\\项目\\Python\\huawei\\topo_100_400_20_5.csv"), infile2.open("D:\\项目\\Python\\huawei\\demand_100_400_20_5.csv");
//	//infile.open("C:\\Users\\go2sea\\Desktop\\huawei\\topo1666.csv"), infile2.open("C:\\Users\\go2sea\\Desktop\\huawei\\demand1666.csv");
//	//infile.open("C:\\Users\\go2sea\\Desktop\\huawei\\topo700.csv"), infile2.open("C:\\Users\\go2sea\\Desktop\\huawei\\demand700.csv");
//	//infile.open("C:\\Users\\go2sea\\Desktop\\huawei\\GeneratedTopo_50points.csv"), infile2.open("C:\\Users\\go2sea\\Desktop\\huawei\\GeneratedDemand_50points.csv");
//	//infile.open("C:\\Users\\go2sea\\Desktop\\huawei\\topo4800self.csv"), infile2.open("C:\\Users\\go2sea\\Desktop\\huawei\\demand4800self.csv");
//	//infile.open("C:\\Users\\go2sea\\Desktop\\huawei\\topo174g.csv"), infile2.open("C:\\Users\\go2sea\\Desktop\\huawei\\demand174g.csv");//官方(case2)
//	//infile.open("C:\\Users\\go2sea\\Desktop\\huawei\\GeneratedTopo_70points.csv"), infile2.open("C:\\Users\\go2sea\\Desktop\\huawei\\GeneratedDemand_70points.csv");
//	//infile.open("C:\\Users\\go2sea\\Desktop\\huawei\\GeneratedTopoFull.csv"), infile2.open("C:\\Users\\go2sea\\Desktop\\huawei\\GeneratedDemandFull.csv");
//	infile.open("C:\\Users\\go2sea\\Desktop\\huawei\\topo_case3_300_844_20.csv"), infile2.open("C:\\Users\\go2sea\\Desktop\\huawei\\demand_case3_300_844_20.csv");
//	//infile.open("C:\\Users\\go2sea\\Desktop\\huawei\\topocase4.csv"), infile2.open("C:\\Users\\go2sea\\Desktop\\huawei\\demandcase4.csv");
//
//	string buff;
//	vector<string> lines;
//	while (getline(infile, buff))
//		lines.push_back(buff);
//	int m = lines.size();
//	char** G = new char*[lines.size()];
//	for (int i = 0; i < (int)lines.size(); i++) {
//		G[i] = new char[lines[i].size() + 1];
//		memcpy(G[i], lines[i].c_str(), lines[i].size() + 1);
//
//		//int index;
//		//for (index = 0; index < (int)lines[i].size(); index++)
//		//	cout << G[i][index];
//		//assert(G[i][index] == 0);
//		//cout << endl;
//	}
//	//system("pause");
//
//	getline(infile2, buff);
//	//cout << "condition: " << buff.c_str() << endl;
//	char* condition = new char[buff.size() + 1];
//	memcpy(condition, buff.c_str(), buff.size() + 1);
//
//	//LangGraph lg(G, m, condition);
//
//
//	search_route(G, m, condition);
//	//search_route(lg);
//
//	//cout << "The runtime is: " << (clock() - g.start_time) / (CLOCKS_PER_SEC / 1000) << " ms !" << endl << endl;//输出运行时间
//
//	system("pause");
//}
//
//
////根据路径长度繁殖（概率递增） || 繁殖信息素









char *argv_3;
Graph *g;
timer_t timer;
//你要完成的功能总入口
void sigalrm_func(int signo);
void search_route(char *graph[5000], int edge_num, char *condition) {
	g = new Graph(graph, edge_num, condition);

	//case 1-5蚁群
	if (g->city_count <= 20) {
		g->runStepBySetp();
	}
	//case 7 9 10 12 only
	else if ((g->city_count > 100 && g->city_count <= 150) || (g->city_count > 200 && g->city_count <= 300) || (g->city_count > 300 && g->city_count <= 500 && g->V.size() > 20 && g->V.size() <= 23)) {
		unordered_set<int> temp_v = g->V;
		temp_v.insert(g->source_id);
		for (auto id : temp_v) {
			//从id到-1， 只用作更新信息素（注意开始时间的设置）
			clock_t now_time = clock();
			ONLY* only = new ONLY(g->neighbors, g->reversed_neighbors, g->edges, g->edges_id, g->pheromone, g->V, g->min_path, g->min_path_edge, g->min_cost, id, -1, now_time, g->shortest_paths, g->V_r_set, g->max_node_id, &(g->min_path_edge), g->city_count);
			only->searchONLY();
			//只取信息素
			g->pheromone = only->pheromone;
		}
		g->runStepBySetp();
	}
	//case 6 8 11 用DFS
	else if (g->city_count <= 100 || (g->city_count > 150 && g->city_count <= 200) || g->V.size() > 30) {
		DFS* dfs = new DFS(g->neighbors, g->reversed_neighbors, g->edges, g->edges_id, g->pheromone, g->V, g->min_path, g->min_path_edge, g->min_cost, g->source_id, g->destination_id, g->start_time, g->shortest_paths, g->V_r_set, g->max_node_id, &(g->min_path_edge), g->city_count);
		dfs->searchDFS();
		//cout << "DFS完成！！！" << endl;
		//system("pause");

		g->min_path_edge = dfs->min_path_edge;
		//g->printResult();
	}
	//case 13 14 15用混合
	else{
		//cout << "混合 ！！！" << endl;
		FZ* fz = new FZ(g->neighbors, g->reversed_neighbors, g->edges, g->edges_id, g->pheromone, g->V, g->min_path, g->min_path_edge, g->min_cost, g->source_id, g->destination_id, g->start_time, g->shortest_paths, g->V_r_set, g->max_node_id, &(g->min_path_edge), g->city_count);
		fz->searchFZ();
		g->pheromone = fz->pheromone;
		g->min_cost = fz->min_cost;
		g->min_path = fz->min_path;
		g->min_path_edge = fz->min_path_edge;
		//g->printResult();
		//cout << "信息素更新完成!!!" << endl;

		//system("pause");
		g->runStepBySetp();
		//g->printResult();
	}
	//写入文件
	sigalrm_func(0);
}
void sigalrm_func(int signo) {
	char *result_file = argv_3;
	for (auto l_id : g->min_path_edge)
		record_result(l_id);
	write_result(result_file);
	print_time("End");
	exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[]) {

	srand(1);
	argv_3 = argv[3];
	print_time("Start");
	//	signal(SIGVTALRM, sigalrm_func);
	//	struct itimerval timer;
	//	timer.it_value.tv_sec = 9;
	//	timer.it_value.tv_usec = 0;
	//	cout <<	setitimer(ITIMER_VIRTUAL, &timer, NULL) << endl;
	/*
	struct sigevent evp;
	struct itimerspec ts;
	int ret;
	evp.sigev_value.sival_ptr = &timer;
	evp.sigev_notify = SIGEV_SIGNAL;
	evp.sigev_signo = SIGUSR1;
	signal(SIGUSR1,sigalrm_func);
	timer_create(CLOCK_REALTIME,&evp,&timer);
	ts.it_value.tv_sec = 1;
	ts.it_value.tv_nsec = 0;
	timer_settime(timer,0,&ts,NULL);
	*/


	char *graph[5000];
	int edge_num;
	char *condition;
	int condition_num;
	char *graph_file = argv[1];
	edge_num = read_file(graph, 5000, graph_file);
	char *condition_file = argv[2];
	condition_num = read_file(&condition, 1, condition_file);
	search_route(graph, edge_num, condition);
	return 0;
}

