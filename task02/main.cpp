#include <deque>
#include <filesystem>
// #include <experimental/filesystem> // uncomment here if the <filesystem> cannot be included above
//
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "Eigen/Core"
//
#include "parse_svg.h"

/***
 * signed area of a triangle connecting points (p0, p1, p2) in counter-clockwise order.
 * @param p0 1st point xy-coordinate
 * @param p1 2nd point xy-coordinate
 * @param p2 3rd point xy-coordinate
 * @return signed area (float)
 */
float area(
    const Eigen::Vector2f &p0,
    const Eigen::Vector2f &p1,
    const Eigen::Vector2f &p2) {
  const auto v01 = p1 - p0;
  const auto v02 = p2 - p0;
  // return 0.5f * (v01[0] * v02[1] - v01[1] * v02[0]); // right handed coordinate
  return 0.5f * (v01[1] * v02[0] - v01[0] * v02[1]); // left-handed coordinate (because pixel y-coordinate is going down)
}


/***
 * compute number of intersection of a ray against a line segment
 * @param org ray origin
 * @param dir ray direction (unit normal)
 * @param ps one of the two end points
 * @param pe the other end point
 * @return number of intersection
 */
int number_of_intersection_ray_against_edge(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pe) {
  auto a = area(org, org + dir, ps);
  auto b = area(org, pe, org + dir);
  auto c = area(org, ps, pe);
  auto d = area(dir+ps, ps, pe);
  if (a * b > 0.f && d * c < 0.f) { return 1; }
  return 0;
  // the following code was a bug
  //auto d = area(org + dir, ps, pe);
  //if (a * b > 0.f && d * c > 0.f && fabs(d) > fabs(c)) { return 1; }
}

/***
 *
 * @param org ray origin
 * @param dir ray direction (unit vector)
 * @param ps one of the two end points
 * @param pc control point
 * @param pe the other end point
 * @return the number of intersections
 */
int number_of_intersection_ray_against_quadratic_bezier(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pc,
    const Eigen::Vector2f &pe,
    int iter_count) {
  // Sorry, but I met a difficulty in implementing this function with the desired method.
  // I hope I can update this later.
  // Currently, it's a simple iterative method.
  auto make_triangle = [](Eigen::Vector2f &ps, Eigen::Vector2f &pc, Eigen::Vector2f &pe) {
    Eigen::Matrix<float, 3, 2> triangle;
    triangle.row(0) = ps;
    triangle.row(1) = pc;
    triangle.row(2) = pe;
    return triangle;
  };
  std::deque<Eigen::Matrix<float, 3, 2>> triangles;
  std::deque<int> iter_indices;
  
  triangles.push_back(make_triangle(ps.const_cast_derived(), pc.const_cast_derived(), pe.const_cast_derived()));
  iter_indices.push_back(0);
  
  int last_iter_index = -1;
  int cross_count = 0;
  while (!triangles.empty()) {
    Eigen::Matrix<float, 3, 2> triangle = triangles.front();
    int iter_index = iter_indices.front();

    triangles.pop_front();
    iter_indices.pop_front();

    Eigen::Vector2f p0, p1, p2;
    p0 = triangle.row(0);
    p1 = triangle.row(1);
    p2 = triangle.row(2);
    
    if (iter_index > last_iter_index) {
      if (iter_index > iter_count) { break; }
      last_iter_index = iter_index;
      cross_count = 0;
    }

    int c0 = number_of_intersection_ray_against_edge(org, dir, p0, p1);
    int c1 = number_of_intersection_ray_against_edge(org, dir, p1, p2);
    int c2 = number_of_intersection_ray_against_edge(org, dir, p0, p2);
    
    int intersect_count = c0 + c1 + c2;
    cross_count += intersect_count > 1 && c2 == 1;

    if (intersect_count > 0) {
      Eigen::Vector2f p3, p4, p5;
      p3 = (p0 + p1) / 2;
      p4 = (p1 + p2) / 2;
      p5 = (p3 + p4) / 2;

      triangles.push_back(make_triangle(p0, p3, p5));
      triangles.push_back(make_triangle(p5, p4, p2));

      iter_indices.push_back(iter_index + 1);
      iter_indices.push_back(iter_index + 1);
    }
  }
    
  return cross_count;
}

int main(int argc, char *argv[]) {
  int iter_count = 5;
  if(argc >= 2) {
    iter_count = std::stoi(argv[1]);
  };
  
  const auto input_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / ".." / "asset" / "r.svg";
  const auto [width, height, shape] = acg::svg_get_image_size_and_shape(input_file_path);
  if (width == 0) { // something went wrong in loading the function
    std::cout << "file open failure" << std::endl;
    abort();
  }
  const std::vector<std::string> outline_path = acg::svg_outline_path_from_shape(shape);
  const std::vector<std::vector<acg::Edge>> loops = acg::svg_loops_from_outline_path(outline_path);
  //
  std::vector<unsigned char> img_data(width * height, 255); // grayscale image initialized white
  for (unsigned int ih = 0; ih < height; ++ih) {
    for (unsigned int iw = 0; iw < width; ++iw) {
      const auto org = Eigen::Vector2f(iw + 0.5, ih + 0.5); // pixel center
      const auto dir = Eigen::Vector2f(60., 20.); // search direction
      int count_cross = 0;
      for (const auto &loop: loops) { // loop over loop (letter R have internal/external loops)
        for (const auto &edge: loop) { // loop over edge in the loop
          if (edge.is_bezier) { // in case the edge is a quadratic Bézier
            count_cross += number_of_intersection_ray_against_quadratic_bezier(
                org, dir,
                edge.ps, edge.pc, edge.pe,
                iter_count);
          } else { // in case the edge is a line segment
            count_cross += number_of_intersection_ray_against_edge(
                org, dir,
                edge.ps, edge.pe);
          }
        }
      }
      if (count_cross % 2 == 1) { // Jordan's curve theory
        img_data[ih * width + iw] = 0; // paint black if it is inside
      }
    }
  }
  const auto output_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / "output.png";
  stbi_write_png(output_file_path.string().c_str(), width, height, 1, img_data.data(), width);
}
