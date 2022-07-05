#include <iostream>
#include "RotateNMS.hpp"


int main(int, char**)
{
  std::cout << "Hello, world!\n";
  RBox box1{64, 150, 20, 36, PI / 6}, box2{62, 151, 19, 34, PI / 7};
  std::cout << box1.x << "," << box1.y << "," << box1.w << "," << box1.h << "," << box1.a << "\n";
  std::cout << RotatedIOU(box1, box2) << "\n";
  std::cout << atan2(1.0, 0.0) << "\n";
  Polygon2D poly1 = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
  Polygon2D poly2 = {{-1, 2}, {-1, -2}, {1, -2}, {1, 2}};
  Polygon2D intersectPoly;
  GetIntersectionPolygon(poly1, poly2, intersectPoly);
  ReorderPolygon(intersectPoly);
  double intersection = GetAreaOfPolygon(intersectPoly);
  std::cout << intersection << "\n";
}
