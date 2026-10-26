#include <iostream>
#include <vector>
#include <queue>

using namespace std;

class Solution {
public:
    vector<vector<int>> floodFill(vector<vector<int>>& image, int sr, int sc, int color) {
        int original = image[sr][sc];
        if (original == color) return image;

        int m = image.size();
        int n = image[0].size();

        queue<pair<int,int>> q;
        q.push({sr, sc});
        image[sr][sc] = color;

        const int dr[4] = {-1, 1, 0, 0};
        const int dc[4] = {0, 0, -1, 1};

        while (!q.empty()) {
            auto [r, c] = q.front();
            q.pop();
            for (int k = 0; k < 4; ++k) {
                int nr = r + dr[k];
                int nc = c + dc[k];
                if (nr >= 0 && nr < m && nc >= 0 && nc < n && image[nr][nc] == original) {
                    image[nr][nc] = color;
                    q.push({nr, nc});
                }
            }
        }
        return image;
    }
};

static void printImage(const vector<vector<int>>& img) {
    for (const auto& row : img) {
        for (size_t j = 0; j < row.size(); ++j) {
            cout << row[j] << (j + 1 < row.size() ? ' ' : '\n');
        }
    }
}

int main() {
    // Example: image = [[1,1,1],[1,1,0],[1,0,1]], sr=1, sc=1, color=2
    vector<vector<int>> image = {
        {1, 1, 1},
        {1, 1, 0},
        {1, 0, 1}
    };
    int sr = 1, sc = 1, color = 2;

    Solution s;
    auto result = s.floodFill(image, sr, sc, color);

    cout << "Result:\n";
    printImage(result);
    return 0;
}
