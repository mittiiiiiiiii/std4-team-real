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

void getToken(int v,char *buf) {
    if(v < N)
        sprintf(buf,"%d",v+1);
    else
        sprintf(buf,"C%d",v-N+1);
}

// 経路（vertex列）を再構成（先頭から順）する．
void get_path(int v,int pred[],int path[],int *len) {
    int temp[MAX_NODES],cnt = 0,i;
    while(v!=-1){
        temp[cnt++]=v;
        v=pred[v];
    }
    *len=cnt;
    for(i=0;i<cnt;i++){
        path[i]=temp[cnt-i-1];
    }
}

int lex_compare_paths(int path1[],int len1,int path2[],int len2){
    int i=0;
    for(i=0;i<len1 && i < len2;i++){
        char tok1[16],tok2[16];
        getToken(path1[i],tok1);
        getToken(path2[i],tok2);
        int cmp=strcmp(tok1,tok2);
        if(cmp != 0){
            return cmp;
        }
    }
    if(len1 == len2) return 0;
    return (len1 < len2) ? -1 : 1;
}

// ダイクストラ法
// dist[] と pred[] を更新する．
void dijkstra_path(int start_node,double dist[],int pred[]) {
    int visited[MAX_NODES]={0};
    int i,j;
    for(i=0; i<g_total_nodes;i++){
        dist[i]=INF;
        pred[i]=-1;
    }
    dist[start_node] = 0;
    
    for(i=0;i<g_total_nodes;i++){
        int u=-1;
        double min_d = INF;
        for(j = 0; j < g_total_nodes; j++){
            if(!visited[j] && dist[j] < min_d){
                min_d = dist[j];
                u = j;
            }
        }
        if(u == -1) break;
        visited[u] = 1;
        for(j = 0; j < g_total_nodes; j++){
            if(g_graph[u][j] < INF){
                double cand = dist[u] + g_graph[u][j];
                if(cand < dist[j] - EPS){
                    dist[j] = cand;
                    pred[j] = u;
                } else if(fabs(cand - dist[j]) < EPS) {
                    int currPath[MAX_NODES],candPath[MAX_NODES];
                    int len_curr,len_cand;
                    get_path(j,pred,currPath,&len_curr);
                    get_path(u,pred,candPath,&len_cand);
                    candPath[len_cand] = j;
                    len_cand++;
                    if(lex_compare_paths(candPath, len_cand, currPath, len_curr) < 0){
                        pred[j] = u;
                    }
                }
            }
        }
    }
}

// ダイクストラ法（経路文字列付き）
void dijkstra_multi(int start, double dist[], char best[][256]) {
    int visited[MAX_NODES] = {0};
    int v, i;
    for(v = 0; v < g_total_nodes; v++){
        dist[v] = INF;
        best[v][0] = '\0';
    }
    dist[start] = 0;
    char token[16];
    getToken(start, token);
    strcpy(best[start], token);
    
    for(i=0;i<g_total_nodes;i++){
        int u=-1;
        double min_d=INF;
        for(v=0;v<g_total_nodes;v++){
            if(!visited[v] && dist[v] < min_d){
                min_d=dist[v];
                u=v;
            }
        }
        if(u == -1) break;
        visited[u] = 1;
        for(v=0;v<g_total_nodes;v++){
            if(g_graph[u][v] < INF){
                double nd = dist[u]+g_graph[u][v];
                char candidate[256],t[16];
                getToken(v, t);
                sprintf(candidate,"%s %s",best[u],t);
                if(nd < dist[v] - EPS){
                    dist[v] = nd;
                    strcpy(best[v], candidate);
                } else if(fabs(nd - dist[v]) < EPS){
                    if(strcmp(candidate, best[v]) < 0){
                        strcpy(best[v], candidate);
                    }
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

int main(){
    int N,M,P,Q,i,j,a,b=0,u,v,unique_count,count=0,k_dummy;;
    int intersection_count=0;

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

        // 道の上にあるノードを始点からの距離でソート
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
        
        for(j=0;j<local_count-1;j++){
            int u = node_indices[j];
            int v = node_indices[j + 1];
            double cost = dists_from_start[j + 1] - dists_from_start[j];
            if(cost > EPS){
                g_graph[u][v] = g_graph[v][u] = cost;
            }
        }
    }
    
    for (i=0;i<Q;i++){
        char s_id_str[10],d_id_str[10];
        scanf("%s %s %d",s_id_str,d_id_str,&k_dummy);
        
        int start_node=-1,dest_node = -1;
        if (s_id_str[0]=='C'){
            int tmp;
            sscanf(&s_id_str[1],"%d",&tmp);
            start_node = N + tmp - 1;
        } else {
            sscanf(s_id_str,"%d",&start_node);
            start_node--;
        }
        
        if (d_id_str[0]=='C'){
            int tmp;
            sscanf(&d_id_str[1],"%d",&tmp);
            dest_node=N+tmp-1;
        } else {
            sscanf(d_id_str,"%d",&dest_node);
            dest_node--;
        }
        
        if (start_node < 0 || start_node >= g_total_nodes || dest_node < 0 || dest_node >= g_total_nodes){
            printf("NA\n");
            continue;
        }
        
        double dist[MAX_NODES];
        char best[MAX_NODES][256];
        dijkstra_multi(start_node, dist, best);
        
        if(dist[dest_node] >= INF){
            printf("NA\n");
        } else {
            printf("%.5f\n", dist[dest_node]);
            printf("%s\n", best[dest_node]);
        }
    }
    
    return 0;
}