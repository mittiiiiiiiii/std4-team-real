#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// 座標や線分の最大数を定義
#define MAX_POINTS 2001
#define MAX_ROADS 2001
#define MAX_N 2001
#define MAX_M 2001
#define MAX_INTERSECTIONS 10000
#define MAX_NODES (MAX_POINTS+MAX_INTERSECTIONS)
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

// K最短路のための経路構造体
typedef struct {
    double distance;
    int path[MAX_NODES];
    int path_length;
} PathInfo;

int N, M, P, Q;
Point g_points[MAX_POINTS]; // 入力される地点
Segment g_segments[MAX_ROADS]; // 入力される道
Point g_intersections[MAX_INTERSECTIONS]; // 発見された交差点
int g_intersection_count = 0; // 交差点の数

Point g_all_nodes[MAX_NODES]; // 全てのノード(地点+交差点)の座標
int g_total_nodes = 0; // 全ノードの総数
double g_graph[MAX_NODES][MAX_NODES]; // ノード間の距離を格納する隣接行列グラフ

Edge* adj[MAX_N];
int visited[MAX_N];
int disc[MAX_N];
int low[MAX_N];
int parent[MAX_N];
int timer;
Bridge bridges[MAX_M];
int bridge_count;

double distance(Point a, Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

void getToken(int v, char *buf) {
    if (v < N)
        sprintf(buf, "%d", v + 1);
    else
        sprintf(buf, "C%d", v - N + 1);
}

// 経路を再構成
void get_path(int v, int pred[], int path[], int *len) {
    int temp[MAX_NODES], cnt = 0, i;
    while (v != -1) {
        temp[cnt++] = v;
        v = pred[v];
    }
    *len = cnt;
    for (i = 0; i < cnt; i++) {
        path[i] = temp[cnt - i - 1];
    }
}

// ダイクストラ法
void dijkstra_path(int start_node, double dist[], int pred[]) {
    int visited_d[MAX_NODES] = {0};
    int i, j;
    for (i = 0; i < g_total_nodes; i++) {
        dist[i] = INF;
        pred[i] = -1;
    }
    dist[start_node] = 0;
    
    for (i = 0; i < g_total_nodes; i++) {
        int u = -1;
        double min_d = INF;
        for (j = 0; j < g_total_nodes; j++) {
            if (!visited_d[j] && dist[j] < min_d) {
                min_d = dist[j];
                u = j;
            }
        }
        if (u == -1) break;
        visited_d[u] = 1;
        for (j = 0; j < g_total_nodes; j++) {
            if (g_graph[u][j] != INF && dist[u] + g_graph[u][j] < dist[j]) {
                dist[j] = dist[u] + g_graph[u][j];
                pred[j] = u;
            }
        }
    }
}

// K最短路を求める
int find_k_shortest_paths(int start, int end, int k, PathInfo results[]) {
    if (start == end) {
        results[0].distance = 0;
        results[0].path_length = 1;
        results[0].path[0] = start;
        return 1;
    }
    
    // 最初の最短路を求める
    double dist[MAX_NODES];
    int pred[MAX_NODES];
    dijkstra_path(start, dist, pred);
    
    if (dist[end] >= INF) {
        return 0;
    }
    
    // 最初の経路を保存
    get_path(end, pred, results[0].path, &results[0].path_length);
    results[0].distance = dist[end];
    int result_count = 1;
    
    // 候補経路を格納する配列
    PathInfo candidates[100];
    int candidate_count = 0;
    
    // K-1回の反復でK最短路を求める
    for (int iter = 1; iter < k; iter++) {
        candidate_count = 0;
        
        // 全てのエッジを個別に削除して試す
        for (int u = 0; u < g_total_nodes && candidate_count < 50; u++) {
            for (int v = 0; v < g_total_nodes && candidate_count < 50; v++) {
                if (g_graph[u][v] != INF) {
                    double temp = g_graph[u][v];
                    g_graph[u][v] = INF;
                    
                    // 新しい最短路を計算
                    dijkstra_path(start, dist, pred);
                    
                    if (dist[end] < INF) {
                        candidates[candidate_count].distance = dist[end];
                        get_path(end, pred, candidates[candidate_count].path, &candidates[candidate_count].path_length);
                        candidate_count++;
                    }
                    
                    // エッジを復元
                    g_graph[u][v] = temp;
                }
            }
        }
        
        // 候補の中から最短のものを選択
        if (candidate_count == 0) {
            break;
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
    
    return result_count;
}

// 点から線分への最短距離と接続点を求める
double point_to_segment_distance(Point p, Segment seg, Point *closest) {
    double dx = seg.p2.x - seg.p1.x;
    double dy = seg.p2.y - seg.p1.y;
    
    if (fabs(dx) < EPS && fabs(dy) < EPS) {
        // 線分が点の場合
        *closest = seg.p1;
        return distance(p, seg.p1);
    }
    
    double t = ((p.x - seg.p1.x) * dx + (p.y - seg.p1.y) * dy) / (dx * dx + dy * dy);
    
    if (t < 0) {
        *closest = seg.p1;
        return distance(p, seg.p1);
    } else if (t > 1) {
        *closest = seg.p2;
        return distance(p, seg.p2);
    } else {
        closest->x = seg.p1.x + t * dx;
        closest->y = seg.p1.y + t * dy;
        return distance(p, *closest);
    }
}

// 新しい地点を道路網に接続する最適な点を求める
Point find_optimal_connection(Point new_point) {
    double min_distance = INF;
    Point best_connection;
    
    for (int i = 0; i < N; i++) {
        double dist = distance(new_point, g_points[i]);
        if (dist < min_distance) {
            min_distance = dist;
            best_connection = g_points[i];
        }
    }
    
    for (int i = 0; i < M; i++) {
        Point closest;
        double dist = point_to_segment_distance(new_point, g_segments[i], &closest);
        
        if (dist < min_distance) {
            min_distance = dist;
            best_connection = closest;
        }
    }
    
    return best_connection;
}

void add_edge(int u, int v) {
    Edge* new_edge = (Edge*)malloc(sizeof(Edge));
    new_edge->to = v;
    new_edge->next = adj[u];
    adj[u] = new_edge;
}

int findIntersection(Segment s1, Segment s2, Point *intersection) {
    double dx1 = s1.p2.x - s1.p1.x;  // s1のx方向の差分
    double dy1 = s1.p2.y - s1.p1.y;  // s1のy方向の差分
    double dx2 = s2.p2.x - s2.p1.x;  // s2のx方向の差分
    double dy2 = s2.p2.y - s2.p1.y;  // s2のy方向の差分

    double determinant = dx1 * dy2 - dy1 * dx2; // 交点計算に必要な行列式

    if (fabs(determinant) < 1e-9) {
        return 0;
    }

    double s = ((s2.p1.x - s1.p1.x) * dy2 - (s2.p1.y - s1.p1.y) * dx2) / determinant;
    double t = ((s2.p1.x - s1.p1.x) * dy1 - (s2.p1.y - s1.p1.y) * dx1) / determinant;

    if (s > EPS && s < 1.0 - EPS && t > EPS && t < 1.0 - EPS) {
        intersection->x = s1.p1.x + s * dx1;
        intersection->y = s1.p1.y + s * dy1;
        return 1;
    }

    return 0;
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

// グラフを構築する関数
void build_graph() {
    // 交差点を検出
    g_intersection_count = 0;
    for (int i = 0; i < M; i++) {
        for (int j = i + 1; j < M; j++) {
            Point temp_intersection;
            if (findIntersection(g_segments[i], g_segments[j], &temp_intersection)) {
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

    // グラフ初期化
    for (int i = 0; i < g_total_nodes; i++) {
        for (int j = 0; j < g_total_nodes; j++) {
            g_graph[i][j] = INF;
        }
    }

    // 道（セグメント）を分解してグラフに辺（エッジ）を追加
    for (int i = 0; i < M; i++) {
        int local_count = 0;
        Point nodes_on_seg[MAX_NODES];
        int node_indices[MAX_NODES];
        double dists_from_start[MAX_NODES];
        
        nodes_on_seg[local_count] = g_segments[i].p1;
        node_indices[local_count] = g_segments[i].start_idx;
        dists_from_start[local_count] = 0.0;
        local_count++;

        nodes_on_seg[local_count] = g_segments[i].p2;
        node_indices[local_count] = g_segments[i].end_idx;
        dists_from_start[local_count] = distance(g_segments[i].p1, g_segments[i].p2);
        local_count++;

        for (int j = 0; j < g_intersection_count; j++) {
            double d1 = distance(g_segments[i].p1, g_intersections[j]);
            double d2 = distance(g_intersections[j], g_segments[i].p2);
            double d_total = distance(g_segments[i].p1, g_segments[i].p2);
            
            if (fabs(d1 + d2 - d_total) < EPS) {
                nodes_on_seg[local_count] = g_intersections[j];
                node_indices[local_count] = N + j;
                dists_from_start[local_count] = d1;
                local_count++;
            }
        }

        // 道の上にあるノードを始点からの距離でソート（バブルソート）
        for (int j = 0; j < local_count - 1; j++) {
            for (int k = j + 1; k < local_count; k++) {
                if (dists_from_start[j] > dists_from_start[k]) {
                    double temp_d = dists_from_start[j];
                    dists_from_start[j] = dists_from_start[k];
                    dists_from_start[k] = temp_d;
                    
                    Point temp_p = nodes_on_seg[j];
                    nodes_on_seg[j] = nodes_on_seg[k];
                    nodes_on_seg[k] = temp_p;
                    
                    int temp_i = node_indices[j];
                    node_indices[j] = node_indices[k];
                    node_indices[k] = temp_i;
                }
            }
        }
        
        // ソートされた順に隣り合うノード間に辺を張る
        for (int j = 0; j < local_count - 1; j++) {
            int u = node_indices[j];
            int v = node_indices[j + 1];
            double dist = distance(nodes_on_seg[j], nodes_on_seg[j + 1]);
            g_graph[u][v] = dist;
            g_graph[v][u] = dist;
        }
    }
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
    
    // グラフを構築
    build_graph();
    
    // P個の新しい地点を処理
    for (i = 0; i < P; i++) {
        Point new_point;
        scanf("%lf %lf", &new_point.x, &new_point.y);
        
        Point connection = find_optimal_connection(new_point); // 最適な接続点を求める
        
        // 接続点を出力
        if (fabs(connection.x - round(connection.x)) < EPS) {
            printf("%.0f ", connection.x);
        } else {
            printf("%.5f ", connection.x);
        }
        
        if (fabs(connection.y - round(connection.y)) < EPS) {
            printf("%.0f\n", connection.y);
        } else {
            printf("%.5f\n", connection.y);
        }
        
        // 新しい道路セグメントを追加
        g_segments[M+i].p1 = new_point;
        g_segments[M+i].p2 = connection;
        g_segments[M+i].start_idx = -1;
        g_segments[M+i].end_idx = -1;
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
                double dist[MAX_NODES];
                int pred[MAX_NODES];
                dijkstra_path(start_node, dist, pred);
                
                if (dist[dest_node] >= INF) {
                    printf("NA\n");
                } else {
                    printf("%.5f\n", dist[dest_node]);
                }
            } else {
                printf("NA\n");
            }
        }
        else if (strcmp(query_type, "K_SHORTEST") == 0) {
            // K最短経路クエリ
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
                int count = find_k_shortest_paths(start_node, dest_node, k, results);
                
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
            // 橋検出
            find_bridges();
            qsort(bridges, bridge_count, sizeof(Bridge), compare_bridges);
            
            for (int j = 0; j < bridge_count; j++) {
                int min_node = (bridges[j].u < bridges[j].v) ? bridges[j].u : bridges[j].v;
                int max_node = (bridges[j].u < bridges[j].v) ? bridges[j].v : bridges[j].u;
                printf("%d %d\n", min_node + 1, max_node + 1);
            }
        }
        else if (strcmp(query_type, "CONNECTION") == 0) {
            // 新しい地点の最適接続点クエリ
            Point new_point;
            scanf("%lf %lf", &new_point.x, &new_point.y);
            
            Point connection = find_optimal_connection(new_point);
            
            if (fabs(connection.x - round(connection.x)) < EPS) {
                printf("%.0f ", connection.x);
            } else {
                printf("%.5f ", connection.x);
            }
            
            if (fabs(connection.y - round(connection.y)) < EPS) {
                printf("%.0f\n", connection.y);
            } else {
                printf("%.5f\n", connection.y);
            }
        }
    }
    
    return 0;
}