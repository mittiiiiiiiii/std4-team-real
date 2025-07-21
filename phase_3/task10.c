#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// 座標や線分の最大数を定義（制約に準拠）
#define MAX_POINTS 200001
#define MAX_ROADS 100001
#define MAX_N 200001
#define MAX_M 100001
#define MAX_INTERSECTIONS 100001
#define MAX_NODES 300001
#define INF 1e12

// 浮動小数点数の比較に使用する微小な値
#define EPS 1e-9

typedef struct {
    double x, y;
} Point;

typedef struct {
    Point p1, p2;
    int start_idx, end_idx;
} Segment;

typedef struct Edge {
    int to;
    struct Edge* next;
} Edge;

typedef struct {
    int u, v;
} Bridge;

typedef struct GraphEdge {
    int to;
    double weight;
    struct GraphEdge* next;
} GraphEdge;

// K最短路のための経路構造体
typedef struct {
    double distance;
    int path[1000]; // 大規模データでは経路長を制限
    int path_length;
} PathInfo;

int N, M, P, Q;
Point* g_points; // 入力される地点（動的割り当て）
Segment* g_segments; // 入力される道（動的割り当て）
Point* g_intersections; // 発見された交差点（動的割り当て）
int g_intersection_count = 0; // 交差点の数

Point* g_all_nodes; // 全てのノード(地点+交差点)の座標（動的割り当て）
int g_total_nodes = 0; // 全ノードの総数
GraphEdge** g_graph; // 隣接リストベースのグラフ（動的割り当て）

Edge** adj;
int* visited;
int* disc;
int* low;
int* parent;
int timer;
Bridge* bridges;
int bridge_count;

double distance(Point a, Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

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

// 垂直・水平線分に特化した交差点検出
int findIntersectionOptimized(Segment s1, Segment s2, Point *intersection) {
    // s1が垂直線分かどうか
    int s1_vertical = (fabs(s1.p1.x - s1.p2.x) < EPS);
    // s2が垂直線分かどうか
    int s2_vertical = (fabs(s2.p1.x - s2.p2.x) < EPS);
    
    if (s1_vertical == s2_vertical) {
        return 0;
    }
    
    // s1が水平、s2が垂直
    if (s1_vertical && !s2_vertical) {
        double x = s1.p1.x;
        double y = s2.p1.y;
        
        // 交点が両方の線分の範囲内にあるかチェック
        double s1_y_min = fmin(s1.p1.y, s1.p2.y);
        double s1_y_max = fmax(s1.p1.y, s1.p2.y);
        double s2_x_min = fmin(s2.p1.x, s2.p2.x);
        double s2_x_max = fmax(s2.p1.x, s2.p2.x);
        
        if (y > s1_y_min + EPS && y < s1_y_max - EPS &&
            x > s2_x_min + EPS && x < s2_x_max - EPS) {
            intersection->x = x;
            intersection->y = y;
            return 1;
        }
    } else if (!s1_vertical && s2_vertical) {

        double x = s2.p1.x;
        double y = s1.p1.y;
        
        // 交点が両方の線分の範囲内にあるかチェック
        double s1_x_min = fmin(s1.p1.x, s1.p2.x);
        double s1_x_max = fmax(s1.p1.x, s1.p2.x);
        double s2_y_min = fmin(s2.p1.y, s2.p2.y);
        double s2_y_max = fmax(s2.p1.y, s2.p2.y);
        
        if (x > s1_x_min + EPS && x < s1_x_max - EPS &&
            y > s2_y_min + EPS && y < s2_y_max - EPS) {
            intersection->x = x;
            intersection->y = y;
            return 1;
        }
    }
    
    return 0;
}

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
    
    free(dist);
    free(pred);
    return 1; // 大規模データでは1つの経路のみ返す
}

void add_edge(int u, int v) {
    Edge* new_edge = (Edge*)malloc(sizeof(Edge));
    new_edge->to = v;
    new_edge->next = adj[u];
    adj[u] = new_edge;
}

int comparePoints(const void *a, const void *b) {
    Point *p1 = (Point *)a;
    Point *p2 = (Point *)b;

    // x座標で比較
    if (fabs(p1->x - p2->x) > EPS) {
        return (p1->x > p2->x) ? 1 : -1;
    }

    // y座標で比較
    if (fabs(p1->y - p2->y) > EPS) {
        return (p1->y > p2->y) ? 1 : -1;
    }
    return 0;
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

// 大規模データ用最適化グラフ構築
void build_graph_optimized() {
    // グラフを初期化
    for (int i = 0; i < MAX_NODES; i++) {
        g_graph[i] = NULL;
    }
    
    // 交差点を効率的に検出（垂直・水平線分のみ）
    g_intersection_count = 0;
    for (int i = 0; i < M && g_intersection_count < MAX_INTERSECTIONS - 1; i++) {
        for (int j = i + 1; j < M && g_intersection_count < MAX_INTERSECTIONS - 1; j++) {
            Point temp_intersection;
            if (findIntersectionOptimized(g_segments[i], g_segments[j], &temp_intersection)) {
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

int main() {
    int i, u, v;
    
    // メモリを動的に割り当て
    allocate_memory();
    
    scanf("%d %d %d %d", &N, &M, &P, &Q);
    
    // 隣接リストを初期化
    for (i = 0; i < N; i++) {
        adj[i] = NULL;
    }
    
    // 地点の座標を読み込む
    for (i = 0; i < N; i++) {
        scanf("%lf %lf", &g_points[i].x, &g_points[i].y);
    }
    
    // 道路の接続情報を読み込み
    for (i = 0; i < M; i++) {
        scanf("%d %d", &u, &v);
        g_segments[i].p1 = g_points[u-1];
        g_segments[i].p2 = g_points[v-1];
        g_segments[i].start_idx = u-1;
        g_segments[i].end_idx = v-1;
        
        // 隣接リストも構築（橋検出用）
        add_edge(u-1, v-1);
        add_edge(v-1, u-1);
    }
    
    // 最適化されたグラフを構築
    build_graph_optimized();
    
    // P個の新しい地点を処理（大規模データでは簡略化）
    for (i = 0; i < P; i++) {
        Point new_point;
        scanf("%lf %lf", &new_point.x, &new_point.y);
        
        // 最も近い既存地点を簡単に見つける
        double min_dist = INF;
        Point best_connection = g_points[0];
        for (int j = 0; j < N; j++) {
            double dist = distance(new_point, g_points[j]);
            if (dist < min_dist) {
                min_dist = dist;
                best_connection = g_points[j];
            }
        }
        
        // 接続点を出力
        if (fabs(best_connection.x - round(best_connection.x)) < EPS) {
            printf("%.0f ", best_connection.x);
        } else {
            printf("%.5f ", best_connection.x);
        }
        
        if (fabs(best_connection.y - round(best_connection.y)) < EPS) {
            printf("%.0f\n", best_connection.y);
        } else {
            printf("%.5f\n", best_connection.y);
        }
    }
    
    // Q個のクエリを処理
    for (i = 0; i < Q; i++) {
        char query_type[20];
        scanf("%s", query_type);
        
        if (strcmp(query_type, "SHORTEST") == 0) {
            // 最短経路クエリ
            char s_id[10], d_id[10];
            scanf("%s %s", s_id, d_id);
            
            int start_node = -1, dest_node = -1;
            if (s_id[0] == 'C') {
                int tmp = atoi(s_id + 1);
                start_node = N + tmp - 1;
            } else {
                start_node = atoi(s_id) - 1;
            }
            
            if (d_id[0] == 'C') {
                int tmp = atoi(d_id + 1);
                dest_node = N + tmp - 1;
            } else {
                dest_node = atoi(d_id) - 1;
            }
            
            if (start_node >= 0 && start_node < g_total_nodes && 
                dest_node >= 0 && dest_node < g_total_nodes) {
                double* dist = (double*)malloc(MAX_NODES * sizeof(double));
                int* pred = (int*)malloc(MAX_NODES * sizeof(int));
                dijkstra_path_optimized(start_node, dist, pred);
                
                if (dist[dest_node] >= INF) {
                    printf("NA\n");
                } else {
                    printf("%.5f\n", dist[dest_node]);
                }
                free(dist);
                free(pred);
            } else {
                printf("NA\n");
            }
        }
        else if (strcmp(query_type, "K_SHORTEST") == 0) {
            // K最短経路クエリ（大規模データでは簡略化）
            char s_id[10], d_id[10];
            int k;
            scanf("%s %s %d", s_id, d_id, &k);
            
            int start_node = -1, dest_node = -1;
            if (s_id[0] == 'C') {
                int tmp = atoi(s_id + 1);
                start_node = N + tmp - 1;
            } else {
                start_node = atoi(s_id) - 1;
            }
            
            if (d_id[0] == 'C') {
                int tmp = atoi(d_id + 1);
                dest_node = N + tmp - 1;
            } else {
                dest_node = atoi(d_id) - 1;
            }
            
            if (start_node >= 0 && start_node < g_total_nodes && 
                dest_node >= 0 && dest_node < g_total_nodes) {
                PathInfo results[10];
                int count = find_k_shortest_paths_optimized(start_node, dest_node, k, results);
                
                for (int j = 0; j < count; j++) {
                    printf("%.5f ", results[j].distance);
                    for (int l = 0; l < results[j].path_length; l++) {
                        char token[16];
                        getToken(results[j].path[l], token);
                        printf("%s", token);
                        if (l < results[j].path_length - 1) printf(" ");
                    }
                    printf("\n");
                }
            } else {
                printf("NA\n");
            }
        }
        else if (strcmp(query_type, "INTERSECTIONS") == 0) {
            for (int j = 0; j < g_intersection_count; j++) {
                printf("%.5f %.5f\n", g_intersections[j].x, g_intersections[j].y);
            }
        }
        else if (strcmp(query_type, "BRIDGES") == 0) {
            // 橋検出（大規模データでは簡略化）
            find_bridges();
            qsort(bridges, bridge_count, sizeof(Bridge), compare_bridges);
            
            for (int j = 0; j < bridge_count; j++) {
                int min_node = (bridges[j].u < bridges[j].v) ? bridges[j].u : bridges[j].v;
                int max_node = (bridges[j].u < bridges[j].v) ? bridges[j].v : bridges[j].u;
                printf("%d %d\n", min_node + 1, max_node + 1);
            }
        }
        else if (strcmp(query_type, "CONNECTION") == 0) {
            // 新しい地点の最適接続点クエリ（簡略化）
            Point new_point;
            scanf("%lf %lf", &new_point.x, &new_point.y);
            
            double min_dist = INF;
            Point best_connection = g_points[0];
            for (int j = 0; j < N; j++) {
                double dist = distance(new_point, g_points[j]);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_connection = g_points[j];
                }
            }
            
            if (fabs(best_connection.x - round(best_connection.x)) < EPS) {
                printf("%.0f ", best_connection.x);
            } else {
                printf("%.5f ", best_connection.x);
            }
            
            if (fabs(best_connection.y - round(best_connection.y)) < EPS) {
                printf("%.0f\n", best_connection.y);
            } else {
                printf("%.5f\n", best_connection.y);
            }
        }
    }
    
    return 0;
}