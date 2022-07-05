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

template <typename T>
inline void argsort(const std::vector<T> &arr, std::vector<int> &order, bool reverse = false)
{
  size_t n = arr.size();
  order.resize(n);
  for (int i = 0; i < n; i++) order[i] = i;
  std::sort(order.begin(), order.end(),
            [reverse, &arr](int i, int j)
            {
              if (reverse)
                return arr[i] > arr[j];
              else
                return arr[i] < arr[j];
            });
}

void RotatedNMS(std::vector<RBox> &boxes, std::vector<double> &scores, std::vector<RBox> &out_boxes,
                std::vector<double> &out_scores, double threshold = 0.5)
{
  size_t n_boxes = boxes.size();
  std::vector<int> order;
  argsort<double>(scores, order, true);
  std::vector<int> suppressed(n_boxes);
  std::vector<int> keep(n_boxes);
  int num_to_keep = 0;
  for (int i = 0; i < n_boxes; i++)
  {
    int idx_i = order[i];
    if (suppressed[idx_i]) continue;
    keep[num_to_keep++] = idx_i;

    for (int j = i + 1; j < n_boxes; j++)
    {
      int idx_j = order[j];
      if (suppressed[idx_j]) continue;
      double iou = RotatedIOU(boxes[idx_i], boxes[idx_j]);
      if (iou > threshold) suppressed[idx_j] = 1;
    }
  }

  for (int i = 0; i < num_to_keep; i++)
  {
    out_boxes.push_back(boxes[keep[i]]);
    out_scores.push_back(scores[keep[i]]);
    std::cout << keep[i] << "\n";
  }
}
#endif