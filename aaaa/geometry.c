#include "geometry.h"

// 2点間の距離を計算
double distance(Point a, Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// 点の比較関数（ソート用）
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

// 一般的な線分交差判定
int findIntersection(Segment s1, Segment s2, Point *intersection) {
    double dx1 = s1.p2.x - s1.p1.x;
    double dy1 = s1.p2.y - s1.p1.y;
    double dx2 = s2.p2.x - s2.p1.x;
    double dy2 = s2.p2.y - s2.p1.y;

    double determinant = dx1 * dy2 - dy1 * dx2;

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

// 垂直・水平線分に特化した交差点検出
int findIntersectionOptimized(Segment s1, Segment s2, Point *intersection) {
    // s1が垂直線分かどうか
    int s1_vertical = (fabs(s1.p1.x - s1.p2.x) < EPS);
    // s2が垂直線分かどうか
    int s2_vertical = (fabs(s2.p1.x - s2.p2.x) < EPS);
    
    if (s1_vertical == s2_vertical) {
        return 0;
    }
    
    // s1が垂直、s2が水平
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
        // s1が水平、s2が垂直
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

// 点から線分への最短距離と接続点を求める
double point_to_segment_distance(Point p, Segment seg, Point *closest) {
    double dx = seg.p2.x - seg.p1.x;
    double dy = seg.p2.y - seg.p1.y;
    
    // 線分が点の場合
    if (fabs(dx) < EPS && fabs(dy) < EPS) {
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
    double min_distance = 1e12; // INF
    Point best_connection;
    
    // 既存の地点をチェック
    for (int i = 0; i < N; i++) {
        double dist = distance(new_point, g_points[i]);
        if (dist < min_distance) {
            min_distance = dist;
            best_connection = g_points[i];
        }
    }
    
    // 道路セグメントをチェック
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
