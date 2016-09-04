#pragma once
#include <iostream>
#include <bitset>
#include <unordered_map>
#include <unordered_set>
#include <time.h>
#include <queue>
#include <stack>
#include <cassert>
#include <string.h>
#include <limits.h>
using namespace std;

struct FZNode {
	int id;
	vector<int> path;
	int path_cost;
	int already;						//path中V'中节点数目（默认source和destination不在V'中）
	unsigned char path_map[75];			//75 * 8 = 600
	int last_v_dis;				//距离上一个路径中v中点的距离


	FZNode(int i) : id(i), path_cost(0), already(0), last_v_dis(0) {
		int block = id / 8;
		int offset = id % 8;

		memset(path_map, 0, 75);
		path_map[block] |= (1 << offset);
	}
	FZNode(int i, int c, vector<int> p, unsigned char* p_m, int a, int lvd) : id(i), path_cost(c), path(p), already(a), last_v_dis(lvd) {
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
struct FZCompare {
	//返回 node1优先级低于node2优先级
	bool operator () (FZNode* node1, FZNode* node2) {
		//按经过V中点顺序
		//return node1->already <= node2->already;

		if (node1->already == 0 && node2->already == 0)
			return node1->path_cost >= node2->path_cost;
		if (node1->already == 0)
			return false;
		if (node2->already == 0)
			return true;

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


class FZ {
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
	int max_runtime = 5000;		//最大运行毫秒数

	long long increase_count = 0;
	long long max_increase_count = 207125000;

	//结果信息
	vector<int> min_path;
	vector<int> min_path_edge;
	int min_cost;

	long long node_rest = 0;
	priority_queue<FZNode*, vector<FZNode*>, FZCompare> pq;	//优先队列（待扩展点） for分支限界
	double average_pheromone;	//平均数
	double media_pheromone;		//中位数
	double Q1_media_pheromone;	//第一4分位数

public:
	FZ(unordered_map<int, unordered_map<int, int>> nei,
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
		V_r_set = V_r_s;
		max_node_id = m_n_i;
		min_path_edge_buff = m_p_edge_buff;
		city_count = city_c;

		pheromone.resize(max_node_id + 1, vector<double>(max_node_id + 1, 0.1));

		//13
		if (city_count>300 && city_count <= 500 && V.size() > 23 && V.size() <= 25)
			max_increase_count = 145000000;
		//12
		else if (city_count>300 && city_count <= 500 && V.size() > 20 && V.size() <= 23)
			max_increase_count = 160000000;
		//10
		else if (city_count > 250 && city_count <= 300)
			max_increase_count = 170000000;
		//15
		else if (city_count > 500 && V.size() <= 25)
			max_increase_count = 190000000;
		//cout << "蚁群 min_cost : " << min_cost << endl;
		//system("pause");
	}
	~FZ() {};


	//第一四分位数
	void countQ1MediaPheromone() {
		vector<double> total;
		for (auto v : pheromone)
			total.insert(total.begin(), v.begin(), v.end());
		sort(total.begin(), total.end());
		Q1_media_pheromone = total[total.size() / 4 * 3.985];
		////返回第一个非默认值
		for (int i = 1; i < (int)total.size(); i++)
			if (total[i]>total[i - 1]) {
				Q1_media_pheromone = total[i];
				break;
			}

		cout << "bigest: " << total.back() << endl;
		cout << "smallest: " << total.front() << endl;
		cout << "Q1_media_pheromone: " << Q1_media_pheromone << endl;
	}



	//增加节点路径上所有边的信息素
	void increasePathPhe(FZNode* node) {
		for (int i = 1; i < (int)node->path.size(); i++) {
			pheromone[node->path[i - 1]][node->path[i]] += 1;
			increase_count++;
		}
	}


	bool reachAll(FZNode* root) {
		stack<FZNode*> stack;
		unordered_set<int> visited(root->path.begin(), root->path.end());
		stack.push(root);
		while (!stack.empty()) {
			FZNode* node = stack.top();
			stack.pop();
			visited.insert(node->id);	//注意：出栈再insert可能更合理

			for (auto pair : neighbors[node->id]) {
				if (visited.count(pair.first) > 0)		//已走过
					continue;
				visited.insert(pair.first);
				FZNode* neighbor_node = new FZNode(pair.first,
					node->path_cost + neighbors[node->id][pair.first],
					node->path,
					node->path_map,
					V.count(pair.first) ? node->already + 1 : node->already,
					V.count(pair.first) ? 0 : node->last_v_dis + 1);

				neighbor_node->path.push_back(node->id);
				stack.push(neighbor_node);
			}
			//注意：root不能释放！！！
			if (node != root)
				delete node;
		}

		//检查是否能到达所有V中节点以及destination
		for (auto i : V)
			if (visited.count(i) == 0) {
				cout << "false" << endl;
				return false;
			}
		if (visited.count(destination_id) == 0)
			return false;
		cout << "           true" << endl;
		return true;
	}



	//将当前扩展节点node的 不在node->path中的neighbor节点add到pq中
	void addNeighborsToPq(FZNode* node) {
		//node的每一条出边
		for (auto pair : neighbors[node->id]) {
			int neighbor_id = pair.first, cost = pair.second;//邻居节点、相应cost
			//已在path中，或未遍历完V情况下到了终点，或closed中包含该点
			if (node->inPath(neighbor_id) || (node->already < (int)V.size() && neighbor_id == destination_id))
				continue;
			//低于信息素阈值
			//if (pheromone[node->id][neighbor_id] < Q1_media_pheromone)
			//	continue;
			//加该邻居后超min_cost
			if (node->path_cost + cost >= min_cost)
				continue;
			//最优情况下扔超min_cost
			if (node->path_cost + cost + shortest_paths[neighbor_id][destination_id].first >= min_cost) {
				//cout << "continue 55555555" << endl;
				continue;
			}
			//估计耗费剪枝
			//if (min_cost < INT_MAX && node->already > 10 && estimateCost(node) >= min_cost)
			//	continue;
			//已有包含路径被遍历过
			vector<int> neighbor_path = node->path;
			neighbor_path.push_back(node->id);
			//很久没有V中点了
			int neighbor_lvd = V.count(neighbor_id) ? 0 : node->last_v_dis + 1;
			if (neighbor_lvd > 20)
				continue;


			FZNode* neighbor_node = new FZNode(neighbor_id,
				node->path_cost + cost,
				neighbor_path,
				node->path_map,
				V.count(neighbor_id) ? node->already + 1 : node->already,
				neighbor_lvd);

			node_rest++;

			////近似判断是否可达所有V中点
			//if (!reachAll(neighbor_node)) {
			//	delete neighbor_node;
			//	continue;
			//}


			//走到V中点，试试组合
			//if (V.count(neighbor_id))
			//	tryVrset(neighbor_node->path, neighbor_id);

			//路口点，路径上所有边的信息素增加
			//if (V.count(neighbor_id))
			increasePathPhe(neighbor_node);

			//已经过V中全部点 且 该点是终点
			if (neighbor_node->already == V.size() && neighbor_id == destination_id)	 {
				//cout << "change min_cost from " << min_cost;
				min_cost = neighbor_node->path_cost, min_path = neighbor_node->path;
				//cout << " to " << min_cost << endl;
				//printPath(min_path);
				min_path.push_back(destination_id);
				setMinPathEdge(min_path);
				(*min_path_edge_buff) = min_path_edge;
				//printMinPathEdge();
				//assert(checkMinPath(min_path));
				//当有新的min_cost出现时，回头清理pq中点，释放内存
				//cout << "从pq中清理" << clearPqTrash() << "个垃圾点" << endl;
				clearPqTrash();
				delete neighbor_node;
				node_rest--;
			}
			else {
				//shrinkDone(path_bitset, neighbor_id);
				//shrinkPq(path_bitset, neighbor_id);		//FZ特有：因为done和pq有重复
				//done[neighbor_id].push_back(path_bitset);
				pq.push(neighbor_node);
				if (V.count(neighbor_id))
					cross_count++;
			}
		}
	}

	//返回某条路径cost，必须保证是正向路径
	int getCost(vector<int> path) {
		int result = 0;
		for (int i = 1; i < (int)path.size(); i++)
			result += neighbors[path[i - 1]][path[i]];
		return result;
	}

	void tryVrset(vector<int> path, int id) {
		//cout << "tryVrset !!!" << endl;
		//试图从V_set 或 V_r_set中组合出新解
		for (vector<int> temp_path : V_r_set[id]) {
			vector<int> new_path = path;
			//for (auto i : temp_path)
			//	cout << i << " ";
			//cout << endl;
			//system("pause");
			new_path.insert(new_path.begin(), temp_path.begin(), temp_path.end());
			//合并失败
			if (!checkMinPath(new_path))
				continue;
			//合并成功
			int new_path_cost = getCost(new_path);
			cout << " 正向蚂蚁合并负向路径 changes min_cost from " << min_cost << " to " << new_path_cost << endl;
			min_path = new_path, min_cost = new_path_cost;
			setMinPathEdge(new_path);
			printMinPathEdge();
		}
	}

	//估计合法路径的cost
	int estimateCost(FZNode* node) {
		int already = node->already;
		int cost = node->path_cost;
		if (already == 0)	//没有经过V中点的一定清理
			return INT_MAX;
		//return 0;
		return cost / already * V.size();
	}

	//清除pq中cost大于等于min_cost的点
	int clearPqTrash() {
		int count = 0;
		priority_queue<FZNode*, vector<FZNode*>, FZCompare> temp_pq;	//优先队列（待扩展点） for分支限界
		while (!pq.empty()) {
			FZNode* node = priorityPop();
			if (node->path_cost < min_cost)
				//if (estimateCost(node) < min_cost)
				temp_pq.push(node);
			else
				delete node, count++;
		}
		pq = temp_pq;
		return count;
	}

	//返回较高信息素的分位数
	double getMediaPhe() {
		vector<int> phe;
		for (auto v : pheromone)
			for (auto i : v)
				if (i > 0)
					phe.push_back(i);
		sort(phe.begin(), phe.end());
		return phe[phe.size() / 4 * 3.99999];
	}

	//清除pq中非路口点
	void clearToOnlyCross() {
		int count = 0;
		priority_queue<FZNode*, vector<FZNode*>, FZCompare> temp_pq;	//优先队列（待扩展点） for分支限界
		//int media_phe = getMediaPhe();		//较高信息素的分位数
		while (!pq.empty()) {
			FZNode* node = priorityPop();
			//路口点，或者高信息素
			if (V.count(node->id) > 0/* || pheromone[node->path[node->path.size()-2]][node->path.back()] > media_phe*/)
				temp_pq.push(node);
			else
				delete node, count++;
		}
		pq = temp_pq;
		//cout << "清除了 " << count << " 个非路口点" << endl;
		//cout << "max_cross_count: " << max_cross_count << endl;
		//system("pause");
	}

	//取pq的top并将其pop
	FZNode* priorityPop() {
		FZNode* result = pq.top();
		pq.pop();
		return result;
	}

	bitset<600> getPathBitSet(vector<int>& path) {
		//cout << "getPathBitSet !!!" << endl;
		//cout << "path.size(): " << path.size() << endl;
		bitset<600> result;
		//注意：由于path中不包含end节点，因此path中所有节点都要处理
		for (int i = 0; i < (int)path.size(); i++)
			result[path[i]] = 1;
		return result;
	}

	//返回a包含b
	bool contains(bitset<600> a, bitset<600> b) {
		//cout << "contains !!!" << endl;
		bitset<600> temp_b = b;
		temp_b ^= a;
		temp_b &= b;
		return temp_b.count() == 0;
	}

	//判断是否包含至少一个
	bool containsOne(int end_id, bitset<600> set) {
		//cout << "containsOne !!!" << endl;
		for (int i = 0; i < (int)done[end_id].size(); i++)
			if (contains(set, done[end_id][i])) {
				//cout << "yes" << endl;
				return true;
			}
		//cout << "				false" << endl;
		return false;
	}


	//将done[id]中包含set的都删掉
	void shrinkDone(bitset<600> set, int id) {
		int before_size = done[id].size();
		vector<bitset<600>> temp;
		for (auto s : done[id])
			if (!contains(s, set))
				temp.push_back(s);
		done[id] = temp;
		if (done[id].size() < before_size) {
			cout << "before shrink done[id].sie(): " << before_size << endl;
			cout << "after shrink done[id].sie(): " << done[id].size() << endl;
		}
	}

	//将done[id]中包含set的都删掉
	void shrinkPq(bitset<600> set, int id) {
		int count = 0;
		priority_queue<FZNode*, vector<FZNode*>, FZCompare> temp_pq;	//优先队列（待扩展点） for分支限界
		int before_size = pq.size();
		while (!pq.empty()) {
			FZNode* node = priorityPop();
			if (node->id == id && contains(getPathBitSet(node->path), set))
				delete node, count++;
			else
				temp_pq.push(node);
		}
		pq = temp_pq;
		//if (count > 0)
		//	cout << "从pq中shrink了" << count << "个不必搜索的分支点" << endl;
		//assert(before_size - pq.size() == count);
	}



	void searchFZ() {
		//countQ1MediaPheromone();
		FZNode* source_node = new FZNode(source_id);
		pq.push(source_node);

		//分支限界
		//cout << "search 分支限界 !!!" << endl;
		while (!pq.empty()) {

			//及时退出
			//printRuntime();
			//if (getRuntime() > max_runtime)
			//	break;
			if (increase_count >= max_increase_count)
				break;


			//cout << "node_rest: " << node_rest << endl;
			//cout << "pq.size(): " << pq.size() << endl;
			//cout << "diff: " << node_rest - pq.size() << endl;
			FZNode* node = priorityPop();
			if (V.count(node->id))
				cross_count--;
			if (node->path_cost >= min_cost)  {
				delete node;
				continue;
			}
			addNeighborsToPq(node);
			delete node;
			node_rest--;
			if (cross_count > max_cross_count || pq.size() > max_pq_size) {
				clearToOnlyCross();				//删除pq中非路口点
				max_cross_count *= C;			//上限提升
			}
		}

		//cout << endl << endl;
		//cout << "FZ 完毕 ！！！" << endl;
		//cout << "min_cost : " << min_cost << endl;
		//cout << "min_path : ";
		//printMinPathEdge();
		//printPath(min_path);

		//cout << endl << "increase_count: " << increase_count << endl;

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

