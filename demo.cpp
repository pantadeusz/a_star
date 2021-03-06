/*
 A* Algorithm example

 Tadeusz Puźniakowski
 2016, 2017, 2018, 2019, 2020
 MIT License
 */
#include "thirdparty/lodepng.cpp"
#include "thirdparty/lodepng.h"
#include "astar.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <vector>

using namespace std;

/**
 * class representing node in graph. In this particular case it is point_t.
 * */

class point_t : public array<int, 2>
{
public:
    point_t(int x = 0, int y = 0) : std::array<int, 2>()
    {
        (*this)[0] = x;
        (*this)[1] = y;
    };
};
namespace std {
/**
 * @brief hash function for point_t
 */
template <>
struct hash<point_t> {
    size_t operator()(const point_t& k) const
    {
        return ((k[0] << sizeof(k[0]) * 4) | (k[0] >> sizeof(k[0]) * 4)) ^ k[1];
    }
};
} // namespace std

/**
 * class representing path
 * */
class path_t : public list<point_t>
{
};

/**
 * we could use some different tile type. Here is just a char that represents
 * tile.
 * */
typedef unsigned char tile_t;
/**
 * helper class for image support. This just wraps lodepng library in more OOP
 * way.
 * */
class image_t : public vector<tile_t>
{
public:
    unsigned int width;
    unsigned int height;
    tile_t& operator()(const point_t& p) { return (*this)[p[1] * width + p[0]]; }
    tile_t operator()(const point_t& p) const
    {
        return (*this)[p[1] * width + p[0]];
    }

    void save(const string& fname)
    {
        image_t image = *this;
        unsigned error =
            lodepng::encode(fname, (unsigned char*)image.data(), image.width,
                image.height, LodePNGColorType::LCT_GREY, 8);
        if (error)
            throw std::runtime_error(lodepng_error_text(error));
    }

    static image_t load(const string& fname)
    {
        image_t image;
        unsigned error = lodepng::decode(image, image.width, image.height, fname,
            LodePNGColorType::LCT_GREY, 8);
        if (error)
            throw std::runtime_error(lodepng_error_text(error));
        return image;
    }
};

int main(int argc, char** argv)
{
    auto img = image_t::load("sample.png");

    point_t start(21, 17);
    point_t goal(211, 132);

    /**
   * function that returns the list of accessible vertices from the current.
   * It takes current coordinates and returns coordinate shifted by the
   * direction. It also checks if we are in the image or outside.
   * */
    const function<list<point_t>(const point_t&)> accessible_verts =
        [&](const point_t& best) -> list<point_t> {
        list<point_t> ret = {{+0, -1}, {-1, -1}, {-1, +0}, {-1, +1},
            {+0, +1}, {+1, +1}, {+1, +0}, {+1, -1}};
        for (auto& e : ret)
            e = {best[0] + e[0], best[1] + e[1]};
        ret.erase(remove_if(ret.begin(), ret.end(),
                      [&img](const point_t& candidate_node) {
                          // remove everything that is incorrect
                          return (candidate_node[0] < 0) ||
                                 (candidate_node[1] < 0) ||
                                 (candidate_node[0] >= img.width) ||
                                 (candidate_node[1] >= img.height) ||
                                 (img(candidate_node) >= 128);
                      }),
            ret.end());
        return ret;
    };
    auto heuristic_f = [&](const point_t& a, const point_t& b) -> double {
        return ::sqrt((a[0] - b[0]) * (a[0] - b[0]) +
                      (a[1] - b[1]) * (a[1] - b[1]));
    };
    auto dist_f = heuristic_f;

    for (auto p : a_star::searchPath<point_t>(start, goal, dist_f, heuristic_f,
             accessible_verts)) {
        static int color = 0;
        color = (color + 16) % 128;
        img(p) = 128 + color; //; ///< draw path
    }
    img.save("result.png");
    return 0;
}
