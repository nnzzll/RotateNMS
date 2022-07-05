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

  std::vector<RBox> boxes = {{5.7577e+01, 3.7292e+01, 2.2060e+01, 3.8184e+01, 3.8675e-02},
                             {5.7394e+01, 3.9794e+01, 2.0384e+01, 3.9700e+01, 8.2425e-02},
                             {5.5799e+01, 3.8566e+01, 2.1895e+01, 4.1546e+01, 1.0161e-01},
                             {5.9808e+01, 3.8911e+01, 1.9453e+01, 3.8958e+01, 9.1494e-02},
                             {5.9902e+01, 3.8489e+01, 2.1932e+01, 4.0749e+01, 1.4022e-01},
                             {5.3985e+01, 4.2908e+01, 2.1121e+01, 3.7793e+01, 3.1780e-02},
                             {5.4886e+01, 4.1313e+01, 2.3230e+01, 4.0198e+01, 6.4324e-02},
                             {5.7362e+01, 3.9988e+01, 3.9429e+01, 2.1398e+01, -3.1622e-01},
                             {5.8534e+01, 4.3232e+01, 2.2281e+01, 3.8025e+01, 6.9410e-02},
                             {5.8096e+01, 4.1503e+01, 2.3518e+01, 3.9718e+01, 1.0204e-01},
                             {5.8414e+01, 4.3853e+01, 2.1490e+01, 3.9735e+01, 7.6759e-02},
                             {5.7020e+01, 4.4112e+01, 2.1894e+01, 4.0607e+01, 1.0524e-01},
                             {6.3445e+01, 3.9990e+01, 4.0786e+01, 2.1402e+01, -2.9464e-01},
                             {5.9113e+01, 4.3122e+01, 2.1914e+01, 3.7624e+01, 8.2327e-02},
                             {5.9230e+01, 4.1366e+01, 2.3421e+01, 4.0709e+01, 1.0817e-01},
                             {5.9411e+01, 4.3718e+01, 2.0905e+01, 3.8770e+01, 8.9742e-02},
                             {5.9931e+01, 4.3920e+01, 2.2368e+01, 3.9398e+01, 1.3451e-01},
                             {5.0351e+01, 4.8937e+01, 3.9585e+01, 2.0575e+01, -3.2408e-01},
                             {5.3306e+01, 4.5435e+01, 2.1169e+01, 3.8291e+01, 1.7222e-02},
                             {5.4411e+01, 4.6556e+01, 2.2814e+01, 3.9215e+01, 7.5899e-02},
                             {5.7475e+01, 4.8124e+01, 4.0057e+01, 2.1352e+01, -3.1917e-01},
                             {5.8827e+01, 4.4945e+01, 2.2437e+01, 3.7775e+01, 8.7484e-02},
                             {5.9162e+01, 4.6300e+01, 2.3043e+01, 3.7828e+01, 1.2625e-01},
                             {5.8233e+01, 4.4684e+01, 2.1450e+01, 3.9390e+01, 4.8678e-02},
                             {5.7013e+01, 4.4741e+01, 2.1999e+01, 4.1172e+01, 1.0985e-01},
                             {6.3838e+01, 4.7595e+01, 4.0704e+01, 2.1547e+01, -2.7639e-01},
                             {6.0106e+01, 4.5480e+01, 2.1975e+01, 3.6785e+01, 1.2213e-01},
                             {6.0300e+01, 4.5841e+01, 2.2975e+01, 3.8405e+01, 1.2851e-01},
                             {5.9608e+01, 4.4921e+01, 2.0620e+01, 3.8877e+01, 6.7662e-02},
                             {6.0008e+01, 4.5556e+01, 2.2034e+01, 3.9772e+01, 1.3134e-01},
                             {6.5745e+01, 4.6350e+01, 2.0963e+01, 3.5859e+01, 1.1343e-01},
                             {6.6402e+01, 4.6660e+01, 2.3448e+01, 3.9591e+01, 8.6243e-02},
                             {5.7555e+01, 5.0223e+01, 2.2413e+01, 3.7654e+01, 2.2941e-02},
                             {5.7112e+01, 4.9052e+01, 2.0468e+01, 3.7101e+01, 4.9880e-02},
                             {5.6013e+01, 4.9518e+01, 2.2115e+01, 4.0992e+01, 1.0664e-01},
                             {6.1777e+01, 5.0563e+01, 2.1937e+01, 3.6763e+01, 6.4936e-02},
                             {6.0477e+01, 4.8445e+01, 2.0399e+01, 3.7555e+01, 8.4775e-02},
                             {6.0542e+01, 4.8431e+01, 2.2236e+01, 3.9440e+01, 1.2480e-01},
                             {2.9210e+01, 8.5022e+01, 2.1860e+01, 3.4889e+01, -1.6429e-01}};
  std::vector<double> scores = {0.0656, 0.4463, 0.2342, 0.1594, 0.1305, 0.1908, 0.1223, 0.0729,
                                0.9505, 0.4845, 0.9770, 0.7614, 0.0788, 0.8764, 0.3719, 0.9707,
                                0.7215, 0.0579, 0.3371, 0.1519, 0.0711, 0.9902, 0.6203, 0.9687,
                                0.7300, 0.0577, 0.9897, 0.5861, 0.9962, 0.8065, 0.1585, 0.0634,
                                0.0897, 0.0892, 0.1957, 0.1876, 0.7752, 0.3008, 0.0911};
  std::vector<RBox> out_b;
  std::vector<double> out_s;
  RotatedNMS(boxes, scores, out_b, out_s);
  std::cout << out_b.size() << "\n";
  for (auto box : out_b)
    std::cout << box.x << "," << box.y << "," << box.w << "," << box.h << "," << box.a << "\n";
}
