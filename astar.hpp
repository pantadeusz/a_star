/*
 A* Algorithm example

 Tadeusz Puźniakowski
 2016, 2017, 2018, 2019, 2020, 2021
 MIT License
 */
#ifndef __A_STAR_PUZNIAKOWSKI_HPP___
#define __A_STAR_PUZNIAKOWSKI_HPP___
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <list>
#include <unordered_map>
#include <numeric>
#include <queue>
#include <set>
#include <vector>

namespace a_star
{

  using std::function;
  using std::list;
  using std::max_element;
  using std::priority_queue;
  using std::remove_if;
  using std::reverse;
  using std::set;
  using std::string;
  using std::unordered_map;
  using std::vector;
/**
 * @brief generate result path
 *
 * @param came_from the dictionary that represents the move directions. key is
 * destination, and value is the source node
 * @param goal the node where we should finish
 * @return Container
 */
  template <class Node, class Container>
  Container reconstructPath(unordered_map<Node, Node> &came_from, Node goal)
  {
    Container reconstructed_path;
    Node current = goal;
    reconstructed_path.push_back(current);
    while (came_from.count(current) > 0)
    {
      current = came_from[current];
      reconstructed_path.push_back(current);
    }
    reverse(reconstructed_path.begin(),
            reconstructed_path.end());
    return reconstructed_path;
  }

  /**
 * @brief calculates shortest path using A* algorithm
 * 
 * @tparam Node the node type
 * @tparam Path the path type. can be for example std::list<Node>
 * @param start the start node 
 * @param goal the goal to reach
 * @param dist distance function
 * @param h heuristic function
 * @param accessible_verts the function that returns accesible nodes from the given node
 * @return Path the generated path
 */
  template <class Node, class Path = std::list<Node>>
  Path searchPath(
      const Node &start, ///< start point
      const Node &goal,  ///< goal point
      const function<double(const Node &, const Node &)>
          dist,                                             ///< actual distance between adjacent points
      const function<double(const Node &, const Node &)> h, ///< heuristic function
      const function<list<Node>(const Node &)>
          accessible_verts ///< returns accessible vertices
  )
  {
    set<Node> closedSet;
    unordered_map<Node, Node> came_from;
    unordered_map<Node, double> g_score;
    unordered_map<Node, double> f_score;
    set<Node> openSet;
    openSet.insert(start);
    g_score[start] = 0;                  ///< distance from start
    f_score[start] = 0 + h(start, goal); ///< estimate distancd to goal

    int t = 0;
    while (openSet.size() > 0)
    {
      /// searching openSet element with lowest f_score and saving it to "best"
      const Node best = *std::max_element(openSet.begin(), openSet.end(), [&f_score](Node best, Node b) { return f_score[best] > f_score[b]; });
      openSet.erase(best); ///< we took the best, so it is no longer in open set

      if (best == goal)
        return reconstructPath<Node, Path>(came_from, goal);

      closedSet.insert(best);
      /// check every possible direction
      for (const Node &neighbor : accessible_verts(best))
      {
        /// not in closed set?
        if (closedSet.count(neighbor) == 0)
        {
          /// calculate temporary t_g_score that is the sum of g_score of current
          /// node (best) and actual distance between (best-neighbour)
          double t_g_score = g_score[best] + dist(neighbor, best);
          /// if the neighbor does not exist, we assume that it is with inf value
          if ((g_score.count(neighbor) == 0) || (t_g_score < g_score[neighbor]))
          {
            came_from[neighbor] = best;
            g_score[neighbor] = t_g_score;
            f_score[neighbor] = g_score[neighbor] + h(neighbor, goal);
          }
          /// we should put neighbor to current openSet - it can be evaluated later
          openSet.insert(neighbor);
        }
      }
      t++;
    }
    return {};
    /// no path found
  }

} // namespace a_star

#endif
