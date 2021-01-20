# a_star
A* universal implementation

This is a header only implementation. You can use it just by:

```c++
#include "astar.hpp"
```

The implementation requires that the type for vertice (node) of the graph have accompanying hash function. You can see it in the demo:

```c++
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
```

When using the library, you should implement the function that generates the list of adjacent nodes. See (assuming the node is ```point_t```):

```c++
std::list<point_t> generate_adjacent_nodes(const point_t&);
```

You should also define heuristic function and distance function between nodes. They can be the same (most cases) or have some differences. For example, if we have 2D point, then the distance function can look like this:

```c++
auto heuristic_f = [&](const point_t& a, const point_t& b) -> double {
    return ::sqrt((a[0] - b[0]) * (a[0] - b[0]) +
                  (a[1] - b[1]) * (a[1] - b[1]));
};
auto dist_f = heuristic_f;
```

If you have defined everything, then just use:

```c++
auto shortest_path = 
 a_star::searchPath<point_t>(start, goal, dist_f, heuristic_f, accessible_verts);
```

and the shortest path would be in ```shortest_path```.

## Contribution

Feel free to send pull requests or discuss. All contributions that will end up in this repo will be credited.
