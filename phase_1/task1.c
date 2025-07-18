#include <stdio.h>
#include <math.h>

typedef struct {
    double x, y;
}Point;

typedef struct {
    Point p1, p2;
}Segment;

int findIntersection(Segment s1,Segment s2,Point *intersection) {
    double a1=s1.p2.y-s1.p1.y;
    double b1=s1.p1.x-s1.p2.x;
    double c1=a1*s1.p1.x+b1*s1.p1.y;

    double a2=s2.p2.y-s2.p1.y;
    double b2=s2.p1.x-s2.p2.x;
    double c2=a2*s2.p1.x+b2*s2.p1.y;

    double determinant=a1*b2-a2*b1;

    if(fabs(determinant)<1e-9){
        return 0;
    }

    intersection->x=(b2*c1-b1*c2)/determinant;
    intersection->y=(a1*c2-a2*c1)/determinant;

    if(intersection->x < fmin(s1.p1.x, s1.p2.x)||intersection->x > fmax(s1.p1.x,s1.p2.x)||
        intersection->x < fmin(s2.p1.x, s2.p2.x)||intersection->x > fmax(s2.p1.x,s2.p2.x)||
        intersection->y < fmin(s1.p1.y, s1.p2.y)||intersection->y > fmax(s1.p1.y,s1.p2.y)||
        intersection->y < fmin(s2.p1.y, s2.p2.y)||intersection->y > fmax(s2.p1.y,s2.p2.y)){
        return 0;
    }

    return 1;
}

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

    Point addPoints[P];
    for(i=0;i<P;i++){
        scanf("%lf %lf",&addPoints[i].x,&addPoints[i].y);
    }

    Point intersection;
    if(findIntersection(segments[0],segments[1],&intersection)){
        printf("%.5lf %.5lf",intersection.x,intersection.y);
    } else {
        printf("No intersection\n");
    }

    return 0;
}