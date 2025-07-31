#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// 浮動小数点数の比較に使用する微小な値
#define EPS 1e-9

// 座標構造体
typedef struct {
    double x, y;
} Point;

// 線分構造体
typedef struct {
    Point p1, p2;
    int start_idx, end_idx;
} Segment;

// 関数プロトタイプ宣言

// 基本的な幾何学計算
double distance(Point a, Point b);
int comparePoints(const void *a, const void *b);

// 線分交差判定
int findIntersection(Segment s1, Segment s2, Point *intersection);
int findIntersectionOptimized(Segment s1, Segment s2, Point *intersection);

// 点と線分の関係
double point_to_segment_distance(Point p, Segment seg, Point *closest);
Point find_optimal_connection(Point new_point);

// 外部変数宣言（main.cで定義される）
extern int N, M;
extern Point* g_points;
extern Segment* g_segments;
extern Point* g_intersections;
extern int g_intersection_count;

#endif // GEOMETRY_H
