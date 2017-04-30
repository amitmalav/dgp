#include "PointKDTree.hpp"

PointKDTree::Node::~Node()
{
  delete lo;
  delete hi;
}

PointKDTree::PointKDTree(std::vector<Point> const & points)
: root(NULL)
{
  build(points);
}

PointKDTree::PointKDTree(std::vector<Point *> const & points)
: root(NULL)
{
  build(points);
}

PointKDTree::PointKDTree(PointCloud const & pcloud)
: root(NULL)
{
  build(pcloud.getPoints());
}

void
PointKDTree::build(std::vector<Point> const & points)
{
  std::vector<Point *> pp(points.size());
  for (size_t i = 0; i < pp.size(); ++i)
    pp[i] = const_cast<Point *>(&points[i]);  // removing the const is not the greatest thing to do, be careful...

  build(pp);
}

void
PointKDTree::build(std::vector<Point *> const & points)
{
  // TODO

  static size_t const MAX_POINTS_PER_LEAF = 5;

  // A kd-tree is just a binary search tree, and is constructed in a near-identical way.
  //
  // - Initially assign (pointers to) all points to the root node.
  // - Recursively subdivide the points, splitting the parent box in half along the longest axis and creating two child nodes
  //   for each split. Stop when number of points in node <= MAX_POINTS_PER_LEAF.
  // - Don't forget to save space by clearing the arrays of points in internal nodes. Only the leaves need to store references
  //   to points.
}
