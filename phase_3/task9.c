#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// グラフの最大サイズ
#define MAX_N 2001
#define MAX_M 2001

typedef struct {
    double x, y;
} Point;

typedef struct Edge {
    int to;
    struct Edge* next;
} Edge;

typedef struct {
    int u, v;
} Bridge;

int N, M, P, Q;
Point g_points[MAX_N];
Edge* adj[MAX_N];
int visited[MAX_N];
int disc[MAX_N];
int low[MAX_N];
int parent[MAX_N];
int timer;
Bridge bridges[MAX_M];
int bridge_count;

void add_edge(int u, int v) {
    Edge* new_edge = (Edge*)malloc(sizeof(Edge));
    new_edge->to = v;
    new_edge->next = adj[u];
    adj[u] = new_edge;
}

void bridge_dfs(int u) {
    visited[u] = 1;
    disc[u] = low[u] = ++timer;
    
    for (Edge* e = adj[u]; e != NULL; e = e->next) {
        int v = e->to;
        if (!visited[v]) {
            parent[v] = u;
            bridge_dfs(v);
            
            low[u] = (low[u] < low[v]) ? low[u] : low[v];
            
            // 橋の条件：low[v] > disc[u]
            if (low[v] > disc[u]) {
                bridges[bridge_count].u = u;
                bridges[bridge_count].v = v;
                bridge_count++;
            }
        } else if (v != parent[u]) {
            low[u] = (low[u] < disc[v]) ? low[u] : disc[v];
        }
    }
}

void find_bridges() {
    timer = 0;
    bridge_count = 0;
    
    // 初期化（隣接リストはクリアしない）
    for (int i = 0; i < N; i++) {
        visited[i] = 0;
        parent[i] = -1;
    }
    
    for (int i = 0; i < N; i++) {
        if (!visited[i]) {
            bridge_dfs(i);
        }
    }
}

int compare_bridges(const void *a, const void *b) {
    Bridge *bridge_a = (Bridge*)a;
    Bridge *bridge_b = (Bridge*)b;
    
    int min_a = (bridge_a->u < bridge_a->v) ? bridge_a->u : bridge_a->v;
    int max_a = (bridge_a->u < bridge_a->v) ? bridge_a->v : bridge_a->u;
    int min_b = (bridge_b->u < bridge_b->v) ? bridge_b->u : bridge_b->v;
    int max_b = (bridge_b->u < bridge_b->v) ? bridge_b->v : bridge_b->u;
    
    if (min_a != min_b) {
        return min_a - min_b;
    }
    return max_a - max_b;
}

int main() {
    int i, u, v;
    
    scanf("%d %d %d %d", &N, &M, &P, &Q);
    
    // 隣接リストを初期化
    for (i = 0; i < N; i++) {
        adj[i] = NULL;
    }
    
    // 地点の座標を読み込む
    for (i = 0; i < N; i++) {
        scanf("%lf %lf", &g_points[i].x, &g_points[i].y);
    }
    
    // 道路の接続情報を読み込み、隣接リストを構築
    for (i = 0; i < M; i++) {
        scanf("%d %d", &u, &v);
        u--; v--; // 0-indexed に変換
        
        // 双方向の辺を追加
        add_edge(u, v);
        add_edge(v, u);
    }
    
    // 橋を検出
    find_bridges();
    
    // 橋をソートして出力
    qsort(bridges, bridge_count, sizeof(Bridge), compare_bridges);
    
    for (i = 0; i < bridge_count; i++) {
        int min_node = (bridges[i].u < bridges[i].v) ? bridges[i].u : bridges[i].v;
        int max_node = (bridges[i].u < bridges[i].v) ? bridges[i].v : bridges[i].u;
        printf("%d %d\n", min_node + 1, max_node + 1); // 1-indexed に戻す
    }
    
    return 0;
}