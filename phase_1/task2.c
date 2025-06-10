#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// 座標や線分の最大数を定義
#define MAX_N 201
#define MAX_M 101
#define MAX_INTERSECTIONS 5000

// 浮動小数点数の比較に使用する微小な値
#define EPS 1e-9

typedef struct {
    double x, y;
}Point;

typedef struct {
    Point p1, p2;
}Segment;

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
    Point *p1 = (Point *)a;
    Point *p2 = (Point *)b;

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
    int N,M,P,Q,i,j;
    int intersection_count=0;

    scanf("%d %d %d %d",&N,&M,&P,&Q);

    Point points[N];
    for(i=0;i<N;i++){
        scanf("%lf %lf",&points[i].x,&points[i].y);
    }

    int a,b=0;
    Segment segments[M];
    for(i=0;i<M;i++){
        scanf("%d %d",&a,&b);
        segments[i].p1=points[a-1];
        segments[i].p2=points[b-1];
    }

    Point intersections[MAX_INTERSECTIONS];

    for(i=0;i<M;i++){
        for(j=i+1;j<M;j++){
            Point temp_intersection;
            if(findIntersection(segments[i],segments[j],&temp_intersection)){
                //重複チェックは一旦省略
                intersections[intersection_count++] = temp_intersection;
            }
        }
    }

    // 見つかった交差点をソート
    qsort(intersections, intersection_count,sizeof(Point),comparePoints);

    // ソートされた交差点を出力
    for(i=0;i<intersection_count;i++){
        printf("%.5f %.5f\n",intersections[i].x,intersections[i].y);
    }

    return 0;
}