#ifndef GRAPH_H
#define GRAPH_H

#include "geometry.h"

// グラフ関連の定数
#define MAX_POINTS 200001
#define MAX_ROADS 100001
#define MAX_N 200001
#define MAX_M 100001
#define MAX_INTERSECTIONS 100001
#define MAX_NODES 300001
#define INF 1e12

// グラフエッジ構造体
typedef struct GraphEdge {
    int to;
    double weight;
    struct GraphEdge* next;
} GraphEdge;

// 橋検出用エッジ構造体
typedef struct Edge {
    int to;
    struct Edge* next;
} Edge;

// 橋構造体
typedef struct {
    int u, v;
} Bridge;

// K最短路のための経路構造体
typedef struct {
    double distance;
    int path[1000]; // 大規模データでは経路長を制限
    int path_length;
} PathInfo;

// 優先度キューのノード
typedef struct PQNode {
    int vertex;
    double distance;
} PQNode;

// 簡易優先度キュー（最小ヒープ）
typedef struct {
    PQNode nodes[MAX_NODES];
    int size;
} PriorityQueue;

// 関数プロトタイプ宣言

// メモリ管理
void allocate_memory();

// グラフ構築
void add_graph_edge(int u, int v, double weight);
void add_edge(int u, int v);
void build_graph_optimized();

// 優先度キュー操作
void pq_init(PriorityQueue* pq);
void pq_push(PriorityQueue* pq, int vertex, double distance);
PQNode pq_pop(PriorityQueue* pq);
int pq_empty(PriorityQueue* pq);

// 経路探索
void dijkstra_path_optimized(int start_node, double* dist, int* pred);
int find_k_shortest_paths_optimized(int start, int end, int k, PathInfo results[]);
void get_path(int v, int pred[], int path[], int *len);

// 橋検出
void bridge_dfs(int u);
void find_bridges();
int compare_bridges(const void *a, const void *b);

// ユーティリティ
void getToken(int v, char *buf);

// 外部変数宣言
extern Point* g_all_nodes;
extern int g_total_nodes;
extern GraphEdge** g_graph;
extern Edge** adj;
extern int* visited;
extern int* disc;
extern int* low;
extern int* parent;
extern int timer;
extern Bridge* bridges;
extern int bridge_count;

#endif // GRAPH_H
