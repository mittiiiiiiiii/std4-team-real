#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// 座標や線分の最大数を定義
#define MAX_POINTS 201
#define MAX_ROADS 101
#define MAX_N 201
#define MAX_M 101
#define MAX_INTERSECTIONS 5000
#define MAX_NODES (MAX_POINTS+MAX_INTERSECTIONS)
#define INF 1e12

// 浮動小数点数の比較に使用する微小な値
#define EPS 1e-9

typedef struct {
    double x,y;
}Point;

typedef struct {
    Point p1,p2;
    int start_idx,end_idx;
}Segment;

int N, M, P, Q;
Point g_points[MAX_POINTS]; // 入力される地点
Segment g_segments[MAX_ROADS]; // 入力される道
Point g_intersections[MAX_INTERSECTIONS]; // 発見された交差点
int g_intersection_count = 0; // 交差点の数

Point g_all_nodes[MAX_NODES]; // 全てのノード(地点+交差点)の座標
int g_total_nodes = 0; // 全ノードの総数
double g_graph[MAX_NODES][MAX_NODES]; // ノード間の距離を格納する隣接行列グラフ

double distance(Point a,Point b) {
    return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
}

void dijkstra(int start_node, double dist[]) {
    int visited[MAX_NODES]={0}; // 訪問済みフラグの配列
    int i, j;

    // 全ノードへの距離を無限大で初期化
    for (i=0;i<g_total_nodes;i++){
        dist[i] = INF;
    }
    dist[start_node] = 0;

    for (i=0;i<g_total_nodes;i++){
        int min_v=-1;
        double min_dist=INF;

        // 未訪問ノードの中から一番近いノードを探す
        for (j=0;j<g_total_nodes;j++) {
            if (!visited[j] && dist[j] < min_dist) {
                min_dist=dist[j];
                min_v=j;
            }
        }

        if (min_v == -1) break;

        // 選んだノードを訪問済み
        visited[min_v]=1;

        // 選んだノードから行ける場所への距離を更新
        for (j=0;j<g_total_nodes;j++){
            if (g_graph[min_v][j] != INF) {
                if (dist[min_v] + g_graph[min_v][j] < dist[j]) {
                    dist[j] = dist[min_v] + g_graph[min_v][j];
                }
            }
        }
    }
}


int findIntersection(Segment s1,Segment s2,Point *intersection) {
    double dx1=s1.p2.x-s1.p1.x;  // s1のx方向の差分
    double dy1=s1.p2.y-s1.p1.y;  // s1のy方向の差分
    double dx2=s2.p2.x-s2.p1.x;  // s2のx方向の差分
    double dy2=s2.p2.y-s2.p1.y;  // s2のy方向の差分

    // 交点計算に必要な行列式
    double determinant=dx1*dy2-dy1*dx2;

    if(fabs(determinant)<1e-9){
        return 0;
    }

    double s=((s2.p1.x-s1.p1.x)*dy2-(s2.p1.y-s1.p1.y)*dx2)/determinant;
    double t=((s2.p1.x-s1.p1.x)*dy1-(s2.p1.y-s1.p1.y)*dx1)/determinant;

    if (s>EPS && s<1.0-EPS && t>EPS && t<1.0-EPS){
        intersection->x=s1.p1.x+s*dx1;
        intersection->y=s1.p1.y+s*dy1;
        return 1;
    }

    return 0;
}

int comparePoints(const void *a, const void *b) {
    Point *p1=(Point *)a;
    Point *p2=(Point *)b;

    // x座標で比較
    if(fabs(p1->x-p2->x) > EPS){
        return (p1->x > p2->x) ? 1 : -1;
    }

    // xが同じならy座標で比較
    if (fabs(p1->y-p2->y) > EPS){
        return (p1->y > p2->y) ? 1 : -1;
    }
    return 0;
}

int main(){
    int N,M,P,Q,i,j,a,b=0,u,v,unique_count,count=0,k_dummy;;
    int intersection_count=0;

    scanf("%d %d %d %d",&N,&M,&P,&Q);

    Point points[N];
    for(i=0;i<N;i++){
        scanf("%lf %lf",&points[i].x,&points[i].y);
    }
    
    // 追加: 読み込んだ地点情報をグローバル配列にコピー
    for(i=0;i<N;i++){
        g_points[i] = points[i];
    }
    
    for (i=0;i<M;i++){
        scanf("%d %d",&u,&v);
        g_segments[i].p1=g_points[u-1];
        g_segments[i].p2=g_points[v-1];
        g_segments[i].start_idx=u-1;
        g_segments[i].end_idx=v-1;
    }

    for(i=0;i<M;i++){
        for(j=i+1;j<M;j++){
            Point temp_intersection;
            if(findIntersection(g_segments[i],g_segments[j],&temp_intersection)){
                g_intersections[g_intersection_count++] = temp_intersection;
            }
        }
    }
    
    // 見つけた交差点をソートして、重複を削除
    qsort(g_intersections,g_intersection_count,sizeof(Point),comparePoints);
    if(g_intersection_count>0){
        unique_count = 1;
        for (i=1;i<g_intersection_count;i++){
            if (comparePoints(&g_intersections[i-1],&g_intersections[i])!=0){
                g_intersections[unique_count++]=g_intersections[i];
            }
        }
    }
    g_intersection_count=unique_count;

    for(i=0;i<N;i++){
        g_all_nodes[i]=g_points[i];
    }
    for(i=0;i<g_intersection_count;i++){
        g_all_nodes[N+i]=g_intersections[i];
    }
    g_total_nodes=N+g_intersection_count;

    // グラフを初期化
    for(i=0;i<g_total_nodes;i++){
        for(j=0;j<g_total_nodes;j++){
            g_graph[i][j] = INF;
        }
    }

    // 道（セグメント）を分解してグラフに辺（エッジ）を追加
    for(i=0;i<M;i++){
        int local_count = 0;
        Point nodes_on_seg[MAX_NODES];
        int node_indices[MAX_NODES];
        double dists_from_start[MAX_NODES];
        
        // 道の両端を追加
        nodes_on_seg[local_count] = g_segments[i].p1;
        node_indices[local_count] = g_segments[i].start_idx;
        dists_from_start[local_count] = 0.0;
        local_count++;

        nodes_on_seg[local_count] = g_segments[i].p2;
        node_indices[local_count] = g_segments[i].end_idx;
        dists_from_start[local_count] = distance(g_segments[i].p1, g_segments[i].p2);
        local_count++;

        // この道の上にある交差点を追加
        for(j=0;j<g_intersection_count;j++){
            double d1 = distance(g_segments[i].p1,g_intersections[j]);
            double d2 = distance(g_intersections[j],g_segments[i].p2);
            double d_total = distance(g_segments[i].p1,g_segments[i].p2);
            if(fabs(d1+d2-d_total) < EPS){ // 線分上にあるかチェック
                nodes_on_seg[local_count] = g_intersections[j];
                node_indices[local_count] = N + j;
                dists_from_start[local_count] = d1;
                local_count++;
            }
        }

        // 道の上にあるノードを始点からの距離でソート（バブルソート）
        int k;
        for(j=0;j<local_count-1;j++){
            for (k=0; k<local_count-j-1;k++){
                if(dists_from_start[k]>dists_from_start[k+1]){
                    double temp_d=dists_from_start[k];
                    dists_from_start[k] = dists_from_start[k + 1];
                    dists_from_start[k + 1] = temp_d;
                    int temp_i = node_indices[k];
                    node_indices[k] = node_indices[k + 1];
                    node_indices[k + 1] = temp_i;
                }
            }
        }
        
        // ソートされた順に隣り合うノード間に辺を張る
        for(j=0;j<local_count-1;j++){
            int u = node_indices[j];
            int v = node_indices[j + 1];
            double cost = dists_from_start[j + 1] - dists_from_start[j];
            if(cost > EPS){
                g_graph[u][v] = g_graph[v][u] = cost;
            }
        }
    }
    
    // --- 問い合わせ処理フェーズ ---
    for (i=0;i<Q;i++){
        char s_id_str[10],d_id_str[10];

        scanf("%s %s %d",s_id_str,d_id_str,&k_dummy);
        
        int start_node = -1,dest_node = -1;
        if (s_id_str[0]=='C'){
            sscanf(&s_id_str[1],"%d",&start_node);
            start_node+=N-1;
        }else{
            sscanf(s_id_str,"%d",&start_node);
            start_node--;
        }
        
        if (d_id_str[0] == 'C'){
            sscanf(&d_id_str[1],"%d",&dest_node);
            dest_node += N - 1;
        }else{
            sscanf(d_id_str,"%d",&dest_node);
            dest_node--;
        }
        
        if (start_node<0 || start_node>=g_total_nodes || dest_node<0 || dest_node>=g_total_nodes){
            printf("NA\n");
            continue;
        }
        
        double dist[MAX_NODES];
        dijkstra(start_node,dist);
        
        double result=dist[dest_node];
        if (result>=INF) {
            printf("NA\n");
        } else {
            printf("%.5f\n",result);
        }
    }

    return 0;
}