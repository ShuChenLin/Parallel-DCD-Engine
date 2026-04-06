#pragma once
#include <vector>
#include <algorithm>

struct CollisionPair {
    int a, b;
    bool operator==(const CollisionPair& o) const { return a == o.a && b == o.b; }
    bool operator<(const CollisionPair& o) const {
        return a < o.a || (a == o.a && b < o.b);
    }
};

inline CollisionPair make_pair_ordered(int i, int j) {
    return {std::min(i, j), std::max(i, j)};
}

inline void deduplicate(std::vector<CollisionPair>& pairs) {
    std::sort(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
}
