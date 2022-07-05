#ifndef RotateNMS_hpp
#include "Geometry.hpp"
struct RBox
{
  double x, y, w, h, a;
};

inline void ConvertRBoxToPolygon(const RBox &box, Polygon2D &poly)
{
  double theta = box.a;
  double cosTheta = cos(theta) * 0.5;
  double sinTheta = sin(theta) * 0.5;

  poly[0][0] = box.x - sinTheta * box.h - cosTheta * box.w;
  poly[0][1] = box.y + cosTheta * box.h - sinTheta * box.w;
  poly[1][0] = box.x + sinTheta * box.h - cosTheta * box.w;
  poly[1][1] = box.y - cosTheta * box.h - sinTheta * box.w;
  poly[2][0] = 2 * box.x - poly[0][0];
  poly[2][1] = 2 * box.y - poly[0][1];
  poly[3][0] = 2 * box.x - poly[1][0];
  poly[3][1] = 2 * box.y - poly[1][1];
}

double RotatedIOU(const RBox &box1, const RBox &box2)
{
  Polygon2D poly1(4), poly2(4);
  ConvertRBoxToPolygon(box1, poly1);
  ConvertRBoxToPolygon(box2, poly2);

  Polygon2D intersectPoly;
  GetIntersectionPolygon(poly1, poly2, intersectPoly);
  ReorderPolygon(intersectPoly);
  double intersection = GetAreaOfPolygon(intersectPoly);
  double area1 = box1.w * box1.h;
  double area2 = box2.w * box2.h;
  return intersection / (area1 + area2 - intersection);
}
#endif