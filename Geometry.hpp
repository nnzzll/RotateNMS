#ifndef Geometry_hpp
#include <array>
#include <vector>
#include <algorithm>
#define PI 3.141592653589
#define TOLERENCE 1e-9

typedef std::vector<std::array<double, 2>> Polygon2D;
typedef std::array<double, 2> Point2D;

inline bool CheckOnSegment(double x, double y, double x1, double y1, double x2, double y2)
{
  bool checkX = (x - x1) * (x2 - x) >= 0;
  bool checkY = (y - y1) * (y2 - y) >= 0;
  bool checkLine = fabs((x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)) <= TOLERENCE;
  return checkX && checkY && checkLine;
}

inline void GetIntersectionPoint(Point2D pt11, Point2D pt12, Point2D pt21, Point2D pt22,
                                 Polygon2D &poly)
{
  /*Ax + By + C = 0  =>  A=y2-y1, B=x1-x2 C=x2y1-x1y2*/
  double A1 = pt12[1] - pt11[1];
  double B1 = pt11[0] - pt12[0];
  double C1 = pt12[0] * pt11[1] - pt11[0] * pt12[1];

  double A2 = pt22[1] - pt21[1];
  double B2 = pt21[0] - pt22[0];
  double C2 = pt22[0] * pt21[1] - pt21[0] * pt22[1];

  double Det = A1 * B2 - A2 * B1;
  if (fabs(Det) <= TOLERENCE)
  {
    /*平行*/
    return;
  }
  else
  {
    /*计算交点*/
    double x = (B1 * C2 - B2 * C1) / Det;
    double y = (A2 * C1 - A1 * C2) / Det;
    if (CheckOnSegment(x, y, pt11[0], pt11[1], pt12[0], pt12[1]) &&
        CheckOnSegment(x, y, pt21[0], pt21[1], pt22[0], pt22[1]))
    {
      poly.push_back({x, y});
    }
  }
}

inline bool IsPointInsidePoly(double x, double y, const Polygon2D &poly)
{
  bool isInside = false;
  int n = poly.size();
  // 多边形的点要有序
  for (int i = 0, j = n - 1; i < n; j = i++)
  {
    auto p1 = poly[i];
    auto p2 = poly[j];
    // 点在多边形的边上
    if (CheckOnSegment(x, y, p1[0], p1[1], p2[0], p2[1])) return true;
    /*
        射线法判断点是否在多边形内部,从被测点出发水平向右发射射线
        ①多边形的边的两个端点在水平射线的上下两端,则说明有交点
        ②交点视为水平射线的上端
        ③被测点要在交点的左侧(单向射线)
    */
    if (((p1[1] > y) ^ (p2[1] > y)) &&
        (x < (p2[0] - p1[0]) * (y - p1[1]) / (p2[1] - p1[1]) + p1[0]))
      isInside = !isInside;
  }
  return isInside;
}

void GetIntersectionPolygon(Polygon2D &poly1, Polygon2D &poly2, Polygon2D &intersectPoly)
{
  for (int i = 0; i < poly1.size(); i++)
    for (int j = 0; j < poly2.size(); j++)
    {
      Point2D p11 = poly1[i];
      Point2D p12 = poly1[(i + 1) % poly1.size()];
      Point2D p21 = poly2[j];
      Point2D p22 = poly2[(j + 1) % poly2.size()];
      GetIntersectionPoint(p11, p12, p21, p22, intersectPoly);
    }

  /*检查poly1中的顶点是否在poly2内部*/
  for (int i = 0; i < poly1.size(); i++)
  {
    if (IsPointInsidePoly(poly1[i][0], poly1[i][1], poly2))
      intersectPoly.push_back({poly1[i][0], poly1[i][1]});
  }

  /*检查poly2中的顶点是否在poly1内部*/
  for (int j = 0; j < poly2.size(); j++)
  {
    if (IsPointInsidePoly(poly2[j][0], poly2[j][1], poly1))
      intersectPoly.push_back({poly2[j][0], poly2[j][1]});
  }
}

inline void ReorderPolygon(Polygon2D &poly)
{
  int count = poly.size();
  if (count == 0)
  {
    return;
  }
  double center_x = 0, center_y = 0;
  for (int i = 0; i < count; i++)
  {
    center_x += poly[i][0];
    center_y += poly[i][1];
  }
  center_x /= count;
  center_y /= count;
  std::sort(poly.begin(), poly.end(),
            [=](Point2D pt1, Point2D pt2) -> bool
            {
              double angle1 = atan2(pt1[1] - center_y, pt1[0] - center_x);
              angle1 = angle1 >= 0 ? angle1 : 2 * PI - fabs(angle1);
              double angle2 = atan2(pt2[1] - center_y, pt2[0] - center_x);
              angle2 = angle2 >= 0 ? angle2 : 2 * PI - fabs(angle2);
              return angle1 < angle2;
            });

  // 删除重复元素
  Polygon2D::iterator iter = std::unique(poly.begin(), poly.end());
  poly.erase(iter, poly.end());
}

/*
    使用Shoelace Formula 计算任意2D多边形面积
    $S = \frac{1}{2} |\sum_{i=1}^n(x_{i}y_{i+1} - x_{i+1}y_{i})| $
    其中$x_{n+1} = x_{1}$
*/
inline double GetAreaOfPolygon(const Polygon2D &poly)
{
  double area = 0, x1, y1, x2, y2;
  int n = poly.size();
  if (n <= 2) return 0;
  for (int i = 0; i < n; i++)
  {
    x1 = poly[i][0];
    y1 = poly[i][1];
    x2 = poly[(i + 1) % n][0];
    y2 = poly[(i + 1) % n][1];
    area += x1 * y2 - x2 * y1;
  }
  return fabs(area / 2);
}

#endif