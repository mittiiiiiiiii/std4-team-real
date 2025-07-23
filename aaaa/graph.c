#include "graph.h"
#include <string.h>

// メモリを動的に割り当てる関数
void allocate_memory() {
    g_points = (Point*)malloc(MAX_POINTS * sizeof(Point));
    g_segments = (Segment*)malloc(MAX_ROADS * sizeof(Segment));
    g_intersections = (Point*)malloc(MAX_INTERSECTIONS * sizeof(Point));
    g_all_nodes = (Point*)malloc(MAX_NODES * sizeof(Point));
    g_graph = (GraphEdge**)malloc(MAX_NODES * sizeof(GraphEdge*));
    
    adj = (Edge**)malloc(MAX_N * sizeof(Edge*));
    visited = (int*)malloc(MAX_N * sizeof(int));
    disc = (int*)malloc(MAX_N * sizeof(int));
    low = (int*)malloc(MAX_N * sizeof(int));
    parent = (int*)malloc(MAX_N * sizeof(int));
    bridges = (Bridge*)malloc(MAX_M * sizeof(Bridge));
    
    // 初期化
    for (int i = 0; i < MAX_NODES; i++) {
        g_graph[i] = NULL;
    }
    for (int i = 0; i < MAX_N; i++) {
        adj[i] = NULL;
    }
}

// グラフにエッジを追加
void add_graph_edge(int u, int v, double weight) {
    GraphEdge* new_edge = (GraphEdge*)malloc(sizeof(GraphEdge));
    new_edge->to = v;
    new_edge->weight = weight;
    new_edge->next = g_graph[u];
    g_graph[u] = new_edge;
}

// 橋検出用エッジを追加
void add_edge(int u, int v) {
    Edge* new_edge = (Edge*)malloc(sizeof(Edge));
    new_edge->to = v;
    new_edge->next = adj[u];
    adj[u] = new_edge;
}

// 優先度キュー操作
void pq_init(PriorityQueue* pq) {
    pq->size = 0;
}

void pq_push(PriorityQueue* pq, int vertex, double distance) {
    int i = pq->size++;
    pq->nodes[i].vertex = vertex;
    pq->nodes[i].distance = distance;
    
    // ヒープアップ
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (pq->nodes[parent].distance <= pq->nodes[i].distance) break;
        
        PQNode temp = pq->nodes[parent];
        pq->nodes[parent] = pq->nodes[i];
        pq->nodes[i] = temp;
        i = parent;
    }
}

PQNode pq_pop(PriorityQueue* pq) {
    PQNode result = pq->nodes[0];
    pq->nodes[0] = pq->nodes[--pq->size];
    
    // ヒープダウン
    int i = 0;
    while (i * 2 + 1 < pq->size) {
        int child = i * 2 + 1;
        if (child + 1 < pq->size && pq->nodes[child + 1].distance < pq->nodes[child].distance) {
            child++;
        }
        if (pq->nodes[i].distance <= pq->nodes[child].distance) break;
        
        PQNode temp = pq->nodes[i];
        pq->nodes[i] = pq->nodes[child];
        pq->nodes[child] = temp;
        i = child;
    }
    
    return result;
}

int pq_empty(PriorityQueue* pq) {
    return pq->size == 0;
}

// ノードIDからトークン文字列を生成
void getToken(int v, char *buf) {
    if (v < N)
        sprintf(buf, "%d", v + 1);
    else
        sprintf(buf, "C%d", v - N + 1);
}

// 経路を再構成
void get_path(int v, int pred[], int path[], int *len) {
    int temp[1000], cnt = 0, i;
    while (v != -1 && cnt < 1000) {
        temp[cnt++] = v;
        v = pred[v];
    }
    *len = cnt;
    for (i = 0; i < cnt; i++) {
        path[i] = temp[cnt - i - 1];
    }
}

// 最適化されたダイクストラ法
void dijkstra_path_optimized(int start_node, double* dist, int* pred) {
    PriorityQueue pq;
    pq_init(&pq);
    
    for (int i = 0; i < g_total_nodes; i++) {
        dist[i] = INF;
        pred[i] = -1;
    }
    dist[start_node] = 0;
    pq_push(&pq, start_node, 0);
    
    while (!pq_empty(&pq)) {
        PQNode current = pq_pop(&pq);
        int u = current.vertex;
        double d = current.distance;
        
        if (d > dist[u]) continue;
        
        for (GraphEdge* edge = g_graph[u]; edge != NULL; edge = edge->next) {
            int v = edge->to;
            double new_dist = dist[u] + edge->weight;
            
            if (new_dist < dist[v]) {
                dist[v] = new_dist;
                pred[v] = u;
                pq_push(&pq, v, new_dist);
            }
        }
    }
}

// 大規模データ用簡略化K最短路
int find_k_shortest_paths_optimized(int start, int end, int k, PathInfo results[]) {
    if (start == end) {
        results[0].distance = 0;
        results[0].path_length = 1;
        results[0].path[0] = start;
        return 1;
    }
    
    // 基本の最短路のみ計算（大規模データではK=1に限定）
    double* dist = (double*)malloc(MAX_NODES * sizeof(double));
    int* pred = (int*)malloc(MAX_NODES * sizeof(int));
    dijkstra_path_optimized(start, dist, pred);
    
    if (dist[end] >= INF) {
        free(dist);
        free(pred);
        return 0;
    }
    
    get_path(end, pred, results[0].path, &results[0].path_length);
    results[0].distance = dist[end];
    int result_count = 1;
    
    // 大規模データでは計算量を制限してK最短路を簡略化
    if (k > 1 && g_total_nodes < 1000) { // 小規模の場合のみK最短路を実行
        // 候補経路を格納する配列
        PathInfo candidates[50];
        int candidate_count = 0;
        
        // K-1回の反復でK最短路を求める（制限付き）
        for (int iter = 1; iter < k && iter < 3; iter++) { // 最大3経路まで
            candidate_count = 0;
            
            // 一部のエッジを削除して試す（制限付き）
            for (int u = 0; u < g_total_nodes && candidate_count < 20; u++) {
                for (GraphEdge* edge = g_graph[u]; edge != NULL && candidate_count < 20; edge = edge->next) {
                    int v = edge->to;
                    double temp_weight = edge->weight;
                    
                    // エッジを一時的に削除
                    edge->weight = INF;
                    
                    // 新しい最短路を計算
                    dijkstra_path_optimized(start, dist, pred);
                    
                    if (dist[end] < INF) {
                        candidates[candidate_count].distance = dist[end];
                        get_path(end, pred, candidates[candidate_count].path, &candidates[candidate_count].path_length);
                        candidate_count++;
                    }
                    
                    // エッジを復元
                    edge->weight = temp_weight;
                }
            }
            
            // 候補をソート
            for (int i = 0; i < candidate_count - 1; i++) {
                for (int j = i + 1; j < candidate_count; j++) {
                    if (candidates[i].distance > candidates[j].distance) {
                        PathInfo temp = candidates[i];
                        candidates[i] = candidates[j];
                        candidates[j] = temp;
                    }
                }
            }
            
            // 重複していない最短の候補を結果に追加
            for (int i = 0; i < candidate_count; i++) {
                int is_duplicate = 0;
                for (int j = 0; j < result_count; j++) {
                    if (fabs(candidates[i].distance - results[j].distance) < EPS &&
                        candidates[i].path_length == results[j].path_length) {
                        int same = 1;
                        for (int l = 0; l < candidates[i].path_length; l++) {
                            if (candidates[i].path[l] != results[j].path[l]) {
                                same = 0;
                                break;
                            }
                        }
                        if (same) {
                            is_duplicate = 1;
                            break;
                        }
                    }
                }
                if (!is_duplicate) {
                    results[result_count] = candidates[i];
                    result_count++;
                    break;
                }
            }
        }
    }
    
    free(dist);
    free(pred);
    return result_count;
}

// 橋検出のDFS
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

// 橋を検出
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

// 橋の比較関数（ソート用）
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

// 大規模データ用最適化グラフ構築
void build_graph_optimized() {
    // グラフを初期化
    for (int i = 0; i < MAX_NODES; i++) {
        g_graph[i] = NULL;
    }
    
    // 交差点を検出（一般的な線分交差も含む）
    g_intersection_count = 0;
    for (int i = 0; i < M && g_intersection_count < MAX_INTERSECTIONS - 1; i++) {
        for (int j = i + 1; j < M && g_intersection_count < MAX_INTERSECTIONS - 1; j++) {
            Point temp_intersection;
            // まず最適化版を試行、失敗したら一般版を使用
            if (findIntersectionOptimized(g_segments[i], g_segments[j], &temp_intersection) ||
                findIntersection(g_segments[i], g_segments[j], &temp_intersection)) {
                g_intersections[g_intersection_count++] = temp_intersection;
            }
        }
    }
    
    // 見つけた交差点をソートして、重複を削除
    qsort(g_intersections, g_intersection_count, sizeof(Point), comparePoints);
    int unique_count = 0;
    if (g_intersection_count > 0) {
        unique_count = 1;
        for (int i = 1; i < g_intersection_count; i++) {
            if (comparePoints(&g_intersections[i-1], &g_intersections[i]) != 0) {
                g_intersections[unique_count++] = g_intersections[i];
            }
        }
        g_intersection_count = unique_count;
    }

    // 全ノードを設定
    for (int i = 0; i < N; i++) {
        g_all_nodes[i] = g_points[i];
    }
    for (int i = 0; i < g_intersection_count; i++) {
        g_all_nodes[N + i] = g_intersections[i];
    }
    g_total_nodes = N + g_intersection_count;

    // 道（セグメント）を効率的に分解してグラフに辺を追加
    for (int i = 0; i < M; i++) {
        // 簡略化：各セグメントごとに関連するノードを効率的に処理
        Point nodes_on_seg[1000]; // 制限
        int node_indices[1000];
        double dists_from_start[1000];
        int local_count = 0;
        
        // 道の両端を追加
        nodes_on_seg[local_count] = g_segments[i].p1;
        node_indices[local_count] = g_segments[i].start_idx;
        dists_from_start[local_count] = 0.0;
        local_count++;

        nodes_on_seg[local_count] = g_segments[i].p2;
        node_indices[local_count] = g_segments[i].end_idx;
        dists_from_start[local_count] = distance(g_segments[i].p1, g_segments[i].p2);
        local_count++;

        // この道の上にある交差点を効率的に検索
        for (int j = 0; j < g_intersection_count && local_count < 999; j++) {
            // 垂直・水平線分に最適化された判定
            int on_segment = 0;
            double d1 = distance(g_segments[i].p1, g_intersections[j]);
            double d2 = distance(g_intersections[j], g_segments[i].p2);
            double d_total = distance(g_segments[i].p1, g_segments[i].p2);
            
            if (fabs(d1 + d2 - d_total) < EPS) {
                on_segment = 1;
            }
            
            if (on_segment) {
                nodes_on_seg[local_count] = g_intersections[j];
                node_indices[local_count] = N + j;
                dists_from_start[local_count] = d1;
                local_count++;
            }
        }

        // 効率的ソート（挿入ソート）
        for (int j = 1; j < local_count; j++) {
            double key_dist = dists_from_start[j];
            Point key_point = nodes_on_seg[j];
            int key_index = node_indices[j];
            int k = j - 1;
            
            while (k >= 0 && dists_from_start[k] > key_dist) {
                dists_from_start[k + 1] = dists_from_start[k];
                nodes_on_seg[k + 1] = nodes_on_seg[k];
                node_indices[k + 1] = node_indices[k];
                k--;
            }
            dists_from_start[k + 1] = key_dist;
            nodes_on_seg[k + 1] = key_point;
            node_indices[k + 1] = key_index;
        }
        
        // ソートされた順に隣り合うノード間に辺を張る
        for (int j = 0; j < local_count - 1; j++) {
            int u = node_indices[j];
            int v = node_indices[j + 1];
            double dist = distance(nodes_on_seg[j], nodes_on_seg[j + 1]);
            
            // 隣接リストに追加
            add_graph_edge(u, v, dist);
            add_graph_edge(v, u, dist);
        }
    }
}
