#include <stdio.h>
#include <string.h>
#include "geometry.h"
#include "graph.h"

// グローバル変数定義
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
    
    // P個の新しい地点を処理（最適接続点検出）
    for (i = 0; i < P; i++) {
        Point new_point;
        scanf("%lf %lf", &new_point.x, &new_point.y);
        
        // 最適な接続点を求める（task9の機能）
        Point connection = find_optimal_connection(new_point);
        
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
            // 新しい地点の最適接続点クエリ（task9の機能）
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