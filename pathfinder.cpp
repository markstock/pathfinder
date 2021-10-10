//
// pathfinder - find paths through terrain
//
// (c)2021 Mark J. Stock <markjstock@gmail.com>
//

#include "inout.h"
#include "CLI11.hpp"

#include <Eigen/Core>

#include <cassert>
#include <vector>
#include <limits>
#include <cmath>
#include <queue>
#include <iostream>


// a commonly-used data item

struct element {
  int32_t i;
  int32_t j;
  float dist;

  element(int32_t i=0, int32_t j=0, float d=0.0) 
        : i(i), j(j), dist(d)
    { }

  bool operator==(const element& other) const {
    return (i == other.i && j == other.j);
  }

  bool operator<(const element& other) const {
    return (dist < other.dist);
  }

  bool operator>(const element& other) const {
    return (dist > other.dist);
  }
};


// several cost functions

float cost_uniform (const float _zthis, const float _zneib, const float _xydist) {
  return _xydist * 0.001;
}

float cost_symmetric (const float _zthis, const float _zneib, const float _xydist) {
  return _xydist * (0.001 + std::pow((_zneib-_zthis)/_xydist, 2));
}

float cost_asymmetric (const float _zthis, const float _zneib, const float _xydist) {
  return _xydist * (0.001 + std::pow(0.04 + (_zneib-_zthis)/_xydist, 2));
}


// dijkstra's algorithm to find the distance (cost) field for travel from the given pixel

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> generate_distance_field (
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& elev,
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& hard,
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& easy,
    const size_t sx, const size_t sy) {

  const bool use_hard = (hard.rows() == elev.rows() and hard.cols() == elev.cols());
  const bool use_easy = (easy.rows() == elev.rows() and easy.cols() == elev.cols());

  const size_t nx = elev.rows();
  const size_t ny = elev.cols();
  std::cerr << "incoming dem is " << nx << " " << ny << " pixels and start point is " << sx << " " << (ny-sy-1) << std::endl;

  // ensure that the given cell is within the bounds
  assert(sx < nx and "Start cell is not inside field bounds");
  assert(sy < ny and "Start cell is not inside field bounds");

  // now, some matrices used to compute distances/costs
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> unvisited;
  unvisited.resize(nx,ny);
  for (size_t i=0; i<nx; ++i) for (size_t j=0; j<ny; ++j) unvisited(i,j) = true;

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> distance;
  distance.resize(nx,ny);
  for (size_t i=0; i<nx; ++i) for (size_t j=0; j<ny; ++j) distance(i,j) = std::numeric_limits<float>::max();

  // a cached matrix to save math
  Eigen::Matrix<float, 3, 3> euclid;
  euclid(0,0) = std::sqrt(2.0);
  euclid(0,1) = 1.0;
  euclid(0,2) = std::sqrt(2.0);
  euclid(1,0) = 1.0;
  euclid(1,1) = 0.0;
  euclid(1,2) = 1.0;
  euclid(2,0) = std::sqrt(2.0);
  euclid(2,1) = 1.0;
  euclid(2,2) = std::sqrt(2.0);

  // set start point as zero
  distance(sx,sy) = 0.0;

  // create the priority queue of active unvisited nodes
  // using std::greater so that the element with the smallest distance appears at the top
  std::priority_queue<element, std::vector<element>, std::greater<element>> active;

  active.emplace(element(sx,sy,distance(sx,sy)));

  std::cerr << std::endl << "Computing..." << std::endl;

  // iterate until we can stop
  while (active.size() > 0) {
    //std::cerr << "active queue has " << active.size() << " elements" << std::endl;

    // completion test (needed only if we want to stop when a target cell is reached)
    //if (not unvisited(nx-1, ny-1)) break;

    // remove the member with the lowest distance (copy the full data, NOT a reference!)
    element const current = active.top();
    active.pop();

    // if it's already visited (i.e. it was in the queue more than once), discard and continue
    if (not unvisited(current.i, current.j)) continue;

    //std::cerr << "  current cell " << current.i << " " << current.j << std::endl;

    // mark the current one "visited" so we don't check it again
    unvisited(current.i, current.j) = false;

    // cache this elevation
    const float thiselev = elev(current.i, current.j);

    // find all unvisited cells/elements
    const size_t ilo = std::max((int32_t)0,current.i-1);
    const size_t ihi = std::min((int32_t)nx,current.i+2);
    const size_t jlo = std::max((int32_t)0,current.j-1);
    const size_t jhi = std::min((int32_t)ny,current.j+2);

    for (size_t i=ilo; i<ihi; ++i) {
    for (size_t j=jlo; j<jhi; ++j) {
      if (unvisited(i,j)) {
        //std::cerr << "  testing cell " << i << " " << j << std::endl;

        // compute distance (cost) from current to target
        //float dist = cost_uniform(thiselev, elev(i,j), euclid(current.i-i+1, current.j-j+1));
        float dist = cost_symmetric(thiselev, elev(i,j), euclid(current.i-i+1, current.j-j+1));
        //float dist = cost_asymmetric(thiselev, elev(i,j), euclid(current.i-i+1, current.j-j+1));
        //std::cerr << "    unvisited, distance is " << euclid(current.i-i+1, current.j-j+1) << " " << dist << std::endl;

        // add a cost if hard(i,j) > 0.0 (like open water, a river, woods, etc.)
        if (use_hard) { dist += hard(i,j); }

        // subtract cost if easy(i,j) and easy(current.i, current.j) are > 0.0
        if (use_easy) { dist *= 1.0 - 0.5*easy(i,j)*easy(current.i, current.j); }

        // reset distance on target
        const float testdist = distance(current.i, current.j) + dist;
        //std::cerr << "    other distances " << distance(current.i, current.j) << " " << testdist << std::endl;
        if (testdist < distance(i,j)) {
          distance(i,j) = testdist;

          // add to the priority queue (even if it's already in there)
          active.emplace(element(i,j,testdist));
        }
      }
    }
    }
  }

  return distance;
}


// given a distance field, find a path from the original source point to the given target point

std::vector<element> generate_path_from(
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& distance,
    const size_t tx, const size_t ty) {

  const size_t nx = distance.rows();
  const size_t ny = distance.cols();

  // ensure that the given cell is within the bounds
  assert(tx < nx and "Target cell is not inside field bounds");
  assert(ty < ny and "Target cell is not inside field bounds");

  // use the distance matrix to generate the vectorial path
  std::vector<element> path;
  path.emplace_back(element(tx,ty,distance(tx,ty)));
  // starting at the end point, march along the direction of the lowest distance
  while (true) {

    // get a *reference* to the current cell
    element const& current = path.back();
    const float thisdist = distance(current.i, current.j);

    // check the neighbors of the tail of the vector for a smaller distance
    element candidate = element(current.i, current.j, thisdist);

    const size_t ilo = std::max((int32_t)0,current.i-1);
    const size_t ihi = std::min((int32_t)nx,current.i+2);
    const size_t jlo = std::max((int32_t)0,current.j-1);
    const size_t jhi = std::min((int32_t)ny,current.j+2);

    for (size_t i=ilo; i<ihi; ++i) {
    for (size_t j=jlo; j<jhi; ++j) {
      if (distance(i,j) < candidate.dist) {
        // overwrite candidate
        candidate = element(i, j, distance(i,j));
      }
    }
    }

    // break out if there are no smaller distances (this means that we've reached the end)
    if (current.i == candidate.i and current.j == candidate.j) break;

    // otherwise, add this to the vector
    //std::cerr << "path goes through " << candidate.i << " " << candidate.j << " with dist " << candidate.dist << std::endl;
    path.push_back(candidate);
  }

  return path;
}


// begin execution here

int main(int argc, char const *argv[]) {

  std::cerr << std::endl << "pathfinder" << std::endl << std::endl;

  // process command line args
  CLI::App app{"Find optimal paths in DEMs with modifier fields"};

  // load a dem from a png file - check command line for file name
  std::string demfile;
  app.add_option("-d,--dem", demfile, "digital elevation model as png");

  // the vertical scale of black-to-white, scaled by the x size of the terrain
  float vscale = 1.0;
  app.add_option("-v,--vert", vscale, "vertical scale of dem w.r.t. horizontal scale");

  // optional png images for hard or easy cells/traversals
  std::string hardfile;
  app.add_option("--hard", hardfile, "png with white areas harder to access");
  std::string easyfile;
  app.add_option("-e,--easy", easyfile, "png with white areas easier to access");

  // start point
  std::array<int32_t,2> startp{0,0};
  app.add_option("-s,--start", startp, "start pixel, from top left");

  // end point (optional)
  std::array<int32_t,2> finishp{-1,-1};
  app.add_option("-f,--finish", finishp, "finish pixel, from top left");
  bool finish_south = false;
  app.add_flag("--fs", finish_south, "find optimal finish pixel on the south edge");
  bool finish_north = false;
  app.add_flag("--fn", finish_north, "find optimal finish pixel on the north edge");
  bool finish_east = false;
  app.add_flag("--fe", finish_east, "find optimal finish pixel on the east edge");
  bool finish_west = false;
  app.add_flag("--fw", finish_west, "find optimal finish pixel on the west edge");

  // write out certain arrays
  std::string outdist;
  app.add_option("--od", outdist, "write distance matrix to png");
  std::string outpath;
  app.add_option("--op", outpath, "write path matrix to png");
  std::string outmap;
  app.add_option("--om", outmap, "write dem and path to a png");

  // finally parse
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    return app.exit(e);
  }

  // input data
  // a floating point heightfield (16b grey)
  size_t nx, ny;
  // start and end points
  size_t xs,ys,xf,yf;

  // load the data
  nx = 100;
  ny = 100;

  // array of the ground elevations (DTM)
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> elev;

  if (demfile.empty()) {
    // simply random
    elev.resize(nx,ny);
    elev.setRandom(nx,ny);	// sets to [-1..1]
    elev = elev.array().pow(2);		// change to all positive

  } else {
    // read a png to get the elevation
    // check the resolution first
    int hgt, wdt;
    (void) read_png_res (demfile.c_str(), &hgt, &wdt);
    if (wdt > 0) nx = wdt;
    if (hgt > 0) ny = hgt;

    // allocate the space
    elev.resize(nx,ny);
    float** data = allocate_2d_array_f((int)nx, (int)ny);

    // read the first channel into the elevation array, scaled as 0..vscale
    (void) read_png (demfile.c_str(), (int)nx, (int)ny, 0, 0, 0.0, 0,
                     data, 0.0, vscale*(float)nx, nullptr, 0.0, 1.0, nullptr, 0.0, 1.0);

    for (size_t i=0; i<nx; ++i) for (size_t j=0; j<ny; ++j) elev(i,j) = data[i][j];

    free_2d_array_f(data);
  }

  // write out corners of matrix
  //std::cerr << "Top left corner of elevation matrix:" << std::endl;
  //std::cerr << elev.block(0,0,6,6) << std::endl;
  //std::cerr << std::endl;
  //std::cerr << "Bottom right corner of elevation matrix:" << std::endl;
  //std::cerr << elev.block(nx-6,ny-6,6,6) << std::endl;


  // another matrix which would increase the cost of traversal
  // this could be used for rivers, woods, etc.
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> hard;

  if (not hardfile.empty()) {
    // check the resolution first
    int hgt, wdt;
    (void) read_png_res (hardfile.c_str(), &hgt, &wdt);
    if (wdt != nx) exit(1);
    if (hgt != ny) exit(1);

    // allocate the space
    hard.resize(nx,ny);
    float** data = allocate_2d_array_f((int)nx, (int)ny);

    // read the first channel into the elevation array, scaled as 0..vscale
    (void) read_png (hardfile.c_str(), (int)nx, (int)ny, 0, 0, 0.0, 0,
                     data, 0.0, 1.0, nullptr, 0.0, 1.0, nullptr, 0.0, 1.0);

    for (size_t i=0; i<nx; ++i) for (size_t j=0; j<ny; ++j) hard(i,j) = data[i][j];

    free_2d_array_f(data);
  }


  // finally, a matrix for existing paths and their weights
  // this would be used to prefer existing paths when pathfinding
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> easy;

  if (not easyfile.empty()) {
    // check the resolution first
    int hgt, wdt;
    (void) read_png_res (easyfile.c_str(), &hgt, &wdt);
    if (wdt != nx) exit(1);
    if (hgt != ny) exit(1);

    // allocate the space
    easy.resize(nx,ny);
    float** data = allocate_2d_array_f((int)nx, (int)ny);

    // read the first channel into the elevation array, scaled as 0..vscale
    (void) read_png (easyfile.c_str(), (int)nx, (int)ny, 0, 0, 0.0, 0,
                     data, 0.0, 1.0, nullptr, 0.0, 1.0, nullptr, 0.0, 1.0);

    for (size_t i=0; i<nx; ++i) for (size_t j=0; j<ny; ++j) easy(i,j) = data[i][j];

    free_2d_array_f(data);
  }


  // set sample start point
  xs = startp[0];
  ys = ny - startp[1] - 1;


  // one function to generate distances to a given point/cell
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> distance = generate_distance_field(elev, hard, easy, xs, ys);

  // write out corners of matrix
  //std::cerr << std::endl;
  //std::cerr << "Top left corner of distance matrix:" << std::endl;
  //std::cerr << distance.block(0,0,6,6) << std::endl;
  //std::cerr << std::endl;
  //std::cerr << "Bottom right corner of distance matrix:" << std::endl;
  //std::cerr << distance.block(nx-6,ny-6,6,6) << std::endl;

  // write out the distance matrix as a png
  if (not outdist.empty()) {
    float** data = allocate_2d_array_f((int)nx, (int)ny);
    for (size_t i=0; i<nx; ++i) for (size_t j=0; j<ny; ++j) data[i][j] = distance(i,j);

    // find mins and maxs
    const float scale = distance.maxCoeff() - distance.minCoeff();

    (void) write_png (outdist.c_str(), (int)nx, (int)ny, FALSE, TRUE,
                      data, 0.0, scale, nullptr, 0.0, 1.0, nullptr, 0.0, 1.0);

    free_2d_array_f(data);
  }

  // now generate paths based on desired end points

  // array to store paths in
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> pathimg;
  pathimg.resize(nx,ny);
  pathimg.setZero(nx,ny);

  // add all paths defined
  if (finish_north) {
    std::cerr << "Finding best path to northern border" << std::endl;

    // find the shortest distance on the southern border
    float mindist = std::numeric_limits<float>::max();
    size_t minidx = -1;	// this is huge, size_t is unsigned
    yf = ny-1;
    for (size_t i=0; i<nx; ++i) {
      //std::cout << "dist at " << i << " " << yf << " is " << distance(i,yf);
      if (distance(i,yf) < mindist) {
        mindist = distance(i,yf);
        minidx = i;
        //std::cout << " NEW SMALLEST";
      }
      //std::cout << std::endl;
    }
    xf = minidx;
    std::cerr << "  finish point is at " << xf << " " << (ny-yf-1) << std::endl;

    // generate paths to target and render to pathimg field
    std::vector<element> path = generate_path_from(distance, xf, yf);
    for (element& cell : path) { pathimg(cell.i,cell.j) += 1.0; }
  }

  if (finish_south) {
    std::cerr << "Finding best path to southern border" << std::endl;

    // find the shortest distance on the southern border
    float mindist = std::numeric_limits<float>::max();
    size_t minidx = -1;	// this is huge, size_t is unsigned
    yf = 0;
    for (size_t i=0; i<nx; ++i) {
      if (distance(i,yf) < mindist) {
        mindist = distance(i,yf);
        minidx = i;
      }
    }
    xf = minidx;
    std::cerr << "  finish point is at " << xf << " " << (ny-yf-1) << std::endl;

    // generate paths to target and render to pathimg field
    std::vector<element> path = generate_path_from(distance, xf, yf);
    for (element& cell : path) { pathimg(cell.i,cell.j) += 1.0; }
  }

  if (finish_east) {
    std::cerr << "Finding best path to eastern border" << std::endl;

    // find the shortest distance on the eastern border
    float mindist = std::numeric_limits<float>::max();
    size_t minidx = -1;	// this is huge, size_t is unsigned
    xf = 0;
    for (size_t j=0; j<ny; ++j){
      if (distance(xf,j) < mindist) {
        mindist = distance(xf,j);
        minidx = j;
      }
    }
    yf = minidx;
    std::cerr << "  finish point is at " << xf << " " << (ny-yf-1) << std::endl;

    // generate paths to target and render to pathimg field
    std::vector<element> path = generate_path_from(distance, xf, yf);
    for (element& cell : path) { pathimg(cell.i,cell.j) += 1.0; }
  }

  if (finish_west) {
    std::cerr << "Finding best path to western border" << std::endl;

    // find the shortest distance on the western border
    float mindist = std::numeric_limits<float>::max();
    size_t minidx = -1;	// this is huge, size_t is unsigned
    xf = nx-1;
    for (size_t j=0; j<ny; ++j){
      if (distance(xf,j) < mindist) {
        mindist = distance(xf,j);
        minidx = j;
      }
    }
    yf = minidx;
    std::cerr << "  finish point is at " << xf << " " << (ny-yf-1) << std::endl;

    // generate paths to target and render to pathimg field
    std::vector<element> path = generate_path_from(distance, xf, yf);
    for (element& cell : path) { pathimg(cell.i,cell.j) += 1.0; }
  }

  if (finishp[0] == -1 and finishp[1] == -1) {
    std::cerr << "No finish pixel given, creating no paths" << std::endl;
    //std::cerr << "No finish pixel given, finding best path to bottom right corner" << std::endl;

    //xf = nx-1;
    //yf = 0;
    //std::cerr << "  finish point is at " << xf << " " << (ny-yf-1) << std::endl;

    // generate paths to target and render to pathimg field
    //std::vector<element> path = generate_path_from(distance, xf, yf);
    //for (element& cell : path) { pathimg(cell.i,cell.j) += 1.0; }

  } else {
    std::cerr << "Finding best path to point " << finishp[0] << " " << finishp[1] << std::endl;
    xf = finishp[0];
    yf = ny - finishp[1] - 1;

    // generate paths to target and render to pathimg field
    std::vector<element> path = generate_path_from(distance, xf, yf);
    for (element& cell : path) { pathimg(cell.i,cell.j) += 1.0; }
  }



  // write out the path matrix as a png
  if (not outpath.empty()) {
    float** data = allocate_2d_array_f((int)nx, (int)ny);
    for (size_t i=0; i<nx; ++i) for (size_t j=0; j<ny; ++j) data[i][j] = pathimg(i,j);

    (void) write_png (outpath.c_str(), (int)nx, (int)ny, FALSE, TRUE,
                      data, 0.0, 1.0, nullptr, 0.0, 1.0, nullptr, 0.0, 1.0);

    free_2d_array_f(data);
  }

  // write out a combination of the dem and path matrix as a png
  if (not outmap.empty()) {
    float** data = allocate_2d_array_f((int)nx, (int)ny);
    for (size_t i=0; i<nx; ++i) for (size_t j=0; j<ny; ++j) data[i][j] = elev(i,j);

    // make the line darker or lighter (to see it)
    const float factor = 1.0/(vscale*(float)nx);
    for (size_t i=0; i<nx; ++i) for (size_t j=0; j<ny; ++j) {
      if (data[i][j]*factor < 0.5) {
        data[i][j] += 0.5*pathimg(i,j)/factor;
      } else {
        data[i][j] -= 0.5*pathimg(i,j)/factor;
      }
    }

    (void) write_png (outmap.c_str(), (int)nx, (int)ny, FALSE, TRUE,
                      data, 0.0, vscale*(float)nx, nullptr, 0.0, 1.0, nullptr, 0.0, 1.0);

    free_2d_array_f(data);
  }
}
