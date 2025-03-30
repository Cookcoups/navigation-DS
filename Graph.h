#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <windows.h>
#include <queue>
#include <random>
#include "BasicStruct.h"
namespace Solution{
    bool KDflg;
    void KDTree(std::vector<Point> &p, int l, int r)
    {
        if (l == r)
            return;
        int mid = l + (r - l) / 2;
        std::nth_element(p.begin() + l, p.begin() + mid, p.begin() + r, [](Point a, Point b) {
            if (KDflg) return a.x < b.x;
            else return a.y < b.y; });
        bool nowflg = KDflg;
        KDflg = nowflg ^ 1;
        KDTree(p, l, mid);
        KDflg = nowflg ^ 1;
        KDTree(p, mid + 1, r);
    }
}
