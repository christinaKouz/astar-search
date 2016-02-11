Astar**uses a best-first search and finds the least-cost path from a given initial node to one goal node (out of one or more possible goals).**

It uses a distance-plus-cost heuristic function (usually denoted f(x)) to determine the order in which the search visits nodes in the tree. The distance-plus-cost heuristic is a sum of two functions: the path-cost function, which is the cost from the starting node to the current node (usually denoted g(x))and an admissible "heuristic estimate" of the distance to the goal (usually denoted h(x)).

The h(x) part of the f(x) function must be an admissible heuristic; that is, it must not overestimate the distance to the goal. Thus, for an application like routing, h(x) might represent the straight-line distance to the goal, since that is physically the smallest possible distance between any two points or nodes.

If the heuristic h satisfies the additional condition {h(x) <= d(x,y) + h(y)} for every edge x, y of the graph (where d denotes the length of that edge), then h is called monotone, or consistent. In such a case, Astar**can be implemented more efficiently—roughly speaking, no node needs to be processed more than once (see closed set below)—and Astar** is equivalent to running Dijkstra's algorithm with the reduced cost d'(x,y): = d(x,y) − h(x) + h(y).

Note that Astar**has been generalized into a bidirectional heuristic search algorithm; see bidirectional search. There can also be a bidirectional Astar** search, which is beneficial for problems where it is unknown how much the heuristic helps.

Source: Wikipedia