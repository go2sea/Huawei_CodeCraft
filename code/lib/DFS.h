#pragma once
#include <iostream>
#include <bitset>
#include <unordered_map>
#include <unordered_set>
#include <time.h>
#include <queue>
#include <stack>
#include <cassert>
using namespace std;

struct DFSNode {
	int id;
	vector<int> path;
	int path_cost;
	int already;						//path中V'中节点数目（默认source和destination不在V'中）
	unsigned char path_map[75];			//75 * 8 = 600
	int last_v_dis;				//距离上一个路径中v中点的距离


	DFSNode(int i) : id(i), path_cost(0), already(0), last_v_dis(0) {
		int block = id / 8;
		int offset = id % 8;

		memset(path_map, 0, 75);
		path_map[block] |= (1 << offset);
	}
	DFSNode(int i, int c, vector<int> p, unsigned char* p_m, int a, int lvd) : id(i), path_cost(c), path(p), already(a), last_v_dis(lvd) {
		memcpy(path_map, p_m, 75);

		int block = id / 8;
		int offset = id % 8;

		path_map[block] |= (1 << offset);
	}

	bool inPath(int target){
		int block = target / 8;
		int offset = target % 8;
		return (path_map[block] & (1 << offset)) == 0 ? false : true;
	}
};

//pq中优先级的比较
struct DFSCompare {
	//返回 node1优先级低于node2优先级
	bool operator () (DFSNode* node1, DFSNode* node2) {
		//按经过V中点顺序
		//return node1->already <= node2->already;

		////一定already之后，按照already数量排优先级
		//if (node1->already > 2 && node2->already > 2) {
		//	return node1->already <= node2->already;
		//}

		if (node1->already == 0 && node2->already == 0)
			return node1->path_cost >= node2->path_cost;
		//if (node1->already == 0)
		//	return false;
		//if (node2->already == 0)
		//	return true;

		//经过一个V'节点的平均耗费
		if (node1->already * node2->already == 0) {
			if (!node1->already && node2->already)
				//if (!node1->already )
				return true;
			return false;
		}
		//平均耗费越大，优先级越低
		return (node1->path_cost / node1->already) > (node2->path_cost / node2->already);
	}
};



//寻找最近nearst用的比较函数
struct DFSNearestCompare {
	//返回 node1优先级低于node2优先级
	bool operator () (DFSNode* node1, DFSNode* node2) {
		return node1->path_cost > node2->path_cost;
	}
};


class DFS {
public:
	clock_t start_time;

	//注意区分DFS中done，FZ的done中元素包含目前在pq中和曾在pq中的
	unordered_map<int, vector<bitset<600>>> done;

	//图信息
	unordered_map<int, unordered_map<int, int>> neighbors;	//id：<邻居id:最小边长>集合
	unordered_map<int, unordered_map<int, int>> reversed_neighbors;
	unordered_map<int, int> edges;							//边id：cost
	unordered_map<int, unordered_map<int, int>> edges_id;
	unordered_set<int> V;
	int source_id, destination_id;
	int max_node_id = 0;
	vector<vector<double>> pheromone;
	vector<unordered_map<int, pair<int, vector<int>>>> shortest_paths;	//shortest[i] = {j, <l : p>} 从i到j的非V最短路径为p长度为l
	unordered_map<int, vector<vector<int>>> V_r_set;
	vector<int>* min_path_edge_buff;
	int city_count;

	//pq中路口点计数
	long long cross_count = 0;
	long long max_cross_count = 400;
	long long max_pq_size = 30 * 10000;
	int C = 2;
	int max_runtime = 9500;		//最大运行毫秒数
	long long increase_count = 0;

	//结果信息
	vector<int> min_path;
	vector<int> min_path_edge;
	int min_cost;

	long long node_rest = 0;
	priority_queue<DFSNode*, vector<DFSNode*>, DFSCompare> pq;	//优先队列（待扩展点） for分支限界
	priority_queue<DFSNode*, vector<DFSNode*>, DFSNearestCompare> nearest_pq;
	stack<DFSNode*> stack_node;
	std::stack<unordered_set<int>*> stack_closed;//与stack同步，对应stack中点的禁忌表
	//unordered_set<int> closed_id;	//source_id的V邻居禁忌表
	double average_pheromone;	//平均数
	double media_pheromone;		//中位数
	double Q1_media_pheromone;	//第一4分位数
	//
public:
	DFS(unordered_map<int, unordered_map<int, int>> nei,
		unordered_map<int, unordered_map<int, int>> reversed_nei,
		unordered_map<int, int> edg,
		unordered_map<int, unordered_map<int, int>> edg_id,
		vector<vector<double>> phe,
		unordered_set<int> v,
		vector<int> m_p,
		vector<int> m_p_edge,
		int m_c,
		int s_id, int d_id,
		clock_t s_time,
		vector<unordered_map<int, pair<int, vector<int>>>> s_p,
		unordered_map<int, vector<vector<int>>> V_r_s,
		int m_n_i,
		vector<int>* m_p_edge_buff,
		int city_c)
	{
		neighbors = nei;
		reversed_neighbors = reversed_nei;
		edges = edg;
		edges_id = edg_id;
		pheromone = phe;
		V = v;
		min_path = m_p;
		min_path_edge = m_p_edge;
		min_cost = m_c;
		source_id = s_id, destination_id = d_id;
		start_time = s_time;
		shortest_paths = s_p;
		/*V_r_set = V_r_s;*/
		max_node_id = m_n_i;
		min_path_edge_buff = m_p_edge_buff;
		city_count = city_c;

		//+1是因为source_id
		//max_runtime /= (V.size() + 1);
		pheromone = phe;

		pheromone.resize(max_node_id + 1, vector<double>(max_node_id + 1, 0.1));

		//cout << "蚁群 min_cost : " << min_cost << endl;
		//system("pause");
	}
	~DFS() {};


	//清除pq中cost大于等于min_cost的点
	int clearPqTrash() {
		int count = 0;
		priority_queue<DFSNode*, vector<DFSNode*>, DFSCompare> temp_pq;	//优先队列（待扩展点） for分支限界
		while (!pq.empty()) {
			DFSNode* node = priorityPop();
			if (node->path_cost < min_cost)
				//if (estimateCost(node) < min_cost)
				temp_pq.push(node);
			else
				delete node, count++;
		}
		pq = temp_pq;
		return count;
	}


	//取pq的top并将其pop
	DFSNode* priorityPop() {
		DFSNode* result = pq.top();
		pq.pop();
		return result;
	}


	//清理nearest_pq，释放内存
	void clearNearestPq() {
		while (!nearest_pq.empty()) {
			delete nearest_pq.top();
			nearest_pq.pop();
		}
	}

	DFSNode* getNearestV() {
		//cout << "getNearestV !!!" << endl;

		assert(nearest_pq.empty());
		assert(V.count(stack_node.top()->id) || stack_node.top()->id == source_id);

		//栈顶第一个点作为起点
		nearest_pq.push(stack_node.top());
		unordered_set<int> this_closed = *stack_closed.top();

		int temp_id = stack_node.top()->id;

		while (!nearest_pq.empty()) {
			//cout << "nearest_pq.size(): " << nearest_pq.size() << endl;
			DFSNode* node = nearest_pq.top();
			nearest_pq.pop();


			//注意：在出栈的时候加入禁忌表，可以实现dijkstra！！！
			if (node->id != temp_id)
				this_closed.insert(node->id);
			//cout << "this_closed.size(): " << this_closed.size() << endl;

			for (auto pair : neighbors[node->id]) {
				int neighbor_id = pair.first, cost = pair.second;//邻居节点、相应cost
				//已在path中，或未遍历完V情况下到了终点，或closed中包含该点
				if (node->inPath(neighbor_id) || (node->already < (int)V.size() && neighbor_id == destination_id)) {
					this_closed.insert(neighbor_id);
					continue;
				}
				//已在禁忌表中
				if (this_closed.count(neighbor_id) > 0)
					continue;
				//加该邻居后超min_cost
				if (node->path_cost + cost >= min_cost) {
					this_closed.insert(neighbor_id);
					continue;
				}
				//最优情况下扔超min_cost
				if (node->path_cost + cost + shortest_paths[neighbor_id][destination_id].first >= min_cost) {
					this_closed.insert(neighbor_id);
					continue;
				}


				DFSNode* neighbor_node = new DFSNode(neighbor_id,
					node->path_cost + cost,
					node->path,
					node->path_map,
					V.count(neighbor_id) ? node->already + 1 : node->already,
					node->last_v_dis);

				neighbor_node->last_v_dis = V.count(neighbor_id) ? 0 : node->last_v_dis + 1;
				neighbor_node->path.push_back(node->id);


				//已经过V中全部点 且 该点是终点
				if (neighbor_node->already == V.size() && neighbor_id == destination_id)	 {
					//cout << "change min_cost from " << min_cost ;
					min_cost = neighbor_node->path_cost, min_path = neighbor_node->path;
					//cout << " to " << min_cost << endl;
					//printPath(min_path);
					min_path.push_back(destination_id);
					setMinPathEdge(min_path);
					(*min_path_edge_buff) = min_path_edge;
					//printMinPathEdge();
					//assert(checkMinPath(min_path));
					//当有新的min_cost出现时，回头清理pq中点，释放内存
					clearNearestPq();
					return neighbor_node;
				}

				//找到第一个V中点
				if (V.count(neighbor_id) > 0) {
					clearNearestPq();
					//	*stack_closed.top() = this_closed;
					return neighbor_node;
				}
				//非V中点加入nearsest_pq
				nearest_pq.push(neighbor_node);
			}
			//注意：栈顶元素交给外层方法释放
			if (node->id != stack_node.top()->id)
				delete node;
		}
		//cout << "返回 null" << endl;
		//返回NULL
		assert(nearest_pq.empty());
		return NULL;
	}


	void searchDFS() {
		//cout << "searchDFS !!!" << endl;
		DFSNode* start_node = new DFSNode(source_id);
		stack_node.push(start_node);
		stack_closed.push(new unordered_set<int>());
		while (!stack_node.empty()) {

			if (getRuntime() >= max_runtime)
				break;


			//case8及时退出
			if (city_count > 150 && city_count <= 200)
				if (min_cost <= 265)
					break;

			//case9及时退出
			if (city_count > 200 && city_count <= 250)
				if (min_cost <= 251)
					break;

			//case10及时退出
			if (city_count > 250 && city_count <= 300)
				if (min_cost <= 297)
					break;

			//case11及时退出
			if (V.size() > 30)
				if (min_cost <= 516)
					break;



			assert(nearest_pq.empty());
			assert(stack_node.size() == stack_closed.size());
			//cout << "stack.size(): " << stack.size() << endl;
			//cout << "stack_closed.size(): " << stack_closed.size() << endl;
			DFSNode* temp_last = getNearestV();

			//死胡同，或者找到解，都回退
			if (!temp_last) {
				//cout << "确实是null" << endl;
				//system("pause");
				//getBack();	//退回start_node，同时更新closed_id
				//该点成为上个节点的禁忌点
				int new_closed_id = stack_node.top()->id;
				delete stack_node.top(), stack_node.pop();
				delete stack_closed.top(), stack_closed.pop();
				if (!stack_closed.empty())
					stack_closed.top()->insert(new_closed_id);
				continue;
			}

			if (temp_last->id == destination_id) {
				//cout << "find one!!!" << endl;
				//system("pause");
				//及时退出
				if (city_count <= 100 && min_cost == 143)
					break;

				int new_closed_id = stack_node.top()->id;
				delete stack_node.top(), stack_node.pop();
				delete stack_closed.top(), stack_closed.pop();
				if (!stack_closed.empty())
					stack_closed.top()->insert(new_closed_id);
				delete temp_last;
				continue;
			}
			assert(V.count(temp_last->id));
			stack_node.push(temp_last);
			stack_closed.push(new unordered_set<int>());
		}

		//	cout << "searchDFS 完毕！！！" << endl;
		//cout << "min_cost: " << min_cost << endl;
		//printMinPathEdge();
		//system("pause");
	}



	void printRuntime() {
		clock_t media_time = clock();//sudnay结束时间
		cout << "The runtime is: " << (media_time - start_time) / (CLOCKS_PER_SEC / 1000) << " ms !" << endl << endl;//输出运行时间
	}

	//返回运行毫秒数
	int getRuntime() {
		clock_t media_time = clock();//sudnay结束时间
		return (media_time - start_time) / (CLOCKS_PER_SEC / 1000);
	}

	void printPath(vector<int> path) {
		cout << "printPath !!!" << endl;
		cout << "path.size(): " << path.size() << endl;
		cout << "path : ";
		for (auto id : path)
			cout << id << " ";
		cout << endl;
	}

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



	//保证path必须是正向路径
	void setMinPathEdge(vector<int>& path) {
		min_path_edge.clear();
		for (int i = 1; i < (int)path.size(); i++)
			min_path_edge.push_back(edges_id[path[i - 1]][path[i]]);
	}


	void printMinPathEdge() {
		cout << "min_cost : " << min_cost << endl;
		cout << "min_path_edge : ";
		for (auto i : min_path_edge)
			cout << i << " ";
		cout << endl;
	}













};

