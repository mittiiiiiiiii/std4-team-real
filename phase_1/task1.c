#include <stdio.h>

typedef struct {
    double x, y;
}Point;

typedef struct {
    Point p1, p2;
}Segment;

int main(){
    int N,M,P,Q,i;

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

    return 0;
}