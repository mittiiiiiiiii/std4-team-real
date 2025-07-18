#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// 座標や線分の最大数を定義
#define MAX_POINTS 1001
#define MAX_ROADS 1001
#define MAX_N 1001
#define MAX_M 1001
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
        // 線分の開始点が最も近い
        *closest = seg.p1;
        return distance(p, seg.p1);
    } else if (t > 1) {
        // 線分の終了点が最も近い
        *closest = seg.p2;
        return distance(p, seg.p2);
    } else {
        // 線分上の点が最も近い
        closest->x = seg.p1.x + t * dx;
        closest->y = seg.p1.y + t * dy;
        return distance(p, *closest);
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

int comparePoints(const void *a,const void *b) {
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

// 新しい地点を道路網に接続する最適な点を求める
Point find_optimal_connection(Point new_point) {
    double min_distance = INF;
    Point best_connection;
    int best_segment_idx = -1;
    
    // 全ての既存の地点をチェック
    for (int i = 0; i < N; i++) {
        double dist = distance(new_point, g_points[i]);
        if (dist < min_distance) {
            min_distance = dist;
            best_connection = g_points[i];
            best_segment_idx = -1;
        }
    }
    
    // 全ての既存の道路セグメントをチェック
    for (int i = 0; i < M; i++) {
        Point closest;
        double dist = point_to_segment_distance(new_point, g_segments[i], &closest);
        
        if (dist < min_distance) {
            min_distance = dist;
            best_connection = closest;
            best_segment_idx = i;
        }
    }
    
    // 新しく追加された道路セグメントもチェック
    for (int i = M; i < M + P; i++) {
        if (i >= M + P) break;
        Point closest;
        double dist = point_to_segment_distance(new_point, g_segments[i], &closest);
        
        if (dist < min_distance) {
            min_distance = dist;
            best_connection = closest;
            best_segment_idx = i;
        }
    }
    
    return best_connection;
}

int main(){
    int i,j,u,v,unique_count;
    
    scanf("%d %d %d %d",&N,&M,&P,&Q);

    Point points[N];
    for(i=0;i<N;i++){
        scanf("%lf %lf",&points[i].x,&points[i].y);
    }
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

    // 最初に交差点を求める
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
    unique_count = 0;
    if(g_intersection_count>0){
        unique_count = 1;
        for (i=1;i<g_intersection_count;i++){
            if (comparePoints(&g_intersections[i-1],&g_intersections[i])!=0){
                g_intersections[unique_count++]=g_intersections[i];
            }
        }
        g_intersection_count=unique_count;
    }

    // P個の新しい地点を処理
    for(i=0;i<P;i++){
        Point new_point;
        scanf("%lf %lf",&new_point.x,&new_point.y);
        
        // 最適な接続点を求める
        Point connection = find_optimal_connection(new_point);
        
        // 接続点を出力
        // 整数の場合は小数点なしで出力
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
        g_segments[M+i].start_idx = -1; // 新しい地点なので無効インデックス
        g_segments[M+i].end_idx = -1;
    }
    
    return 0;
}