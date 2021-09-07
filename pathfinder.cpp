//
// pathfinder - find paths through terrain
//
// (c)2021 Mark J. Stock <markjstock@gmail.com>
//

#include "inout.h"

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
  return _xydist * 0.01;
}

float cost_symmetric (const float _zthis, const float _zneib, const float _xydist) {
  return _xydist * (0.01 + std::pow((_zneib-_zthis)/_xydist, 2));
}

float cost_asymmetric (const float _zthis, const float _zneib, const float _xydist) {
  return _xydist * (0.01 + std::pow(0.04 + (_zneib-_zthis)/_xydist, 2));
}


// dijkstra's algorithm to find the distance (cost) field for travel from the given pixel

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> generate_distance_field (
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& elev,
    const size_t sx, const size_t sy) {

  const size_t nx = elev.rows();
  const size_t ny = elev.cols();
  std::cerr << "incoming elev is " << nx << "," << ny << " and start pt is " << sx << "," << sy << std::endl;

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

    // completion test
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
        //const float dist = cost_uniform(thiselev, elev(i,j), euclid(current.i-i+1, current.j-j+1));
        const float dist = cost_symmetric(thiselev, elev(i,j), euclid(current.i-i+1, current.j-j+1));
        //const float dist = cost_asymmetric(thiselev, elev(i,j), euclid(current.i-i+1, current.j-j+1));
        //std::cerr << "    unvisited, distance is " << euclid(current.i-i+1, current.j-j+1) << " " << dist << std::endl;

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

  // load a dem from a png file - check command line for file name
  std::string infile;
  float vscale = 1.0;
  if (argc == 3) {
    infile = argv[1];
    vscale = std::atof(argv[2]);
  } else {
    std::cerr << std::endl << "Usage:" << std::endl;
    std::cerr << "  " << argv[0] << " dem.png relhgt" << std::endl << std::endl;
    return -1;
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

  if (false) {
    // simply random
    elev.resize(nx,ny);
    elev.setRandom(nx,ny);	// sets to [-1..1]
    elev = elev.array().pow(2);		// change to all positive

  } else {
    // read a png to get the elevation
    // check the resolution first
    int hgt, wdt;
    (void) read_png_res (infile.c_str(), &hgt, &wdt);
    if (wdt > 0) nx = wdt;
    if (hgt > 0) ny = hgt;

    // allocate the space
    elev.resize(nx,ny);
    float** data = allocate_2d_array_f((int)nx, (int)ny);

    // read the first channel into the elevation array, scaled as 0..vscale
    (void) read_png (infile.c_str(), (int)nx, (int)ny, 0, 0, 0.0, 0,
                     data, 0.0, vscale*(float)nx, nullptr, 0.0, 1.0, nullptr, 0.0, 1.0);

    for (size_t i=0; i<nx; ++i) for (size_t j=0; j<ny; ++j) elev(i,j) = data[i][j];

    free_2d_array_f(data);
  }

  // write out corners of matrix
  std::cerr << "Top left corner of elevation matrix:" << std::endl;
  std::cerr << elev.block(0,0,6,6) << std::endl;
  std::cerr << std::endl;
  std::cerr << "Bottom right corner of elevation matrix:" << std::endl;
  std::cerr << elev.block(nx-6,ny-6,6,6) << std::endl;


  // another matrix which would increase the cost of traversal
  // this could be used for rivers, woods, etc.
  //Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> extracost;

  // finally, a matrix for existing paths and their weights
  // this would be used to prefer existing paths when pathfinding
  //Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> paths;


  xs = nx/5;
  ys = ny/5;
  xf = (4*nx)/5;
  yf = (4*ny)/5;


  // one function to generate distances to a given point/cell
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> distance = generate_distance_field(elev, xs, ys);

  // write out corners of matrix
  std::cerr << std::endl;
  std::cerr << "Top left corner of distance matrix:" << std::endl;
  std::cerr << distance.block(0,0,6,6) << std::endl;
  std::cerr << std::endl;
  std::cerr << "Bottom right corner of distance matrix:" << std::endl;
  std::cerr << distance.block(nx-6,ny-6,6,6) << std::endl;

  // write out the distance matrix as a png
  if (true) {
    std::string outroot = "dist";
    float** data = allocate_2d_array_f((int)nx, (int)ny);
    for (size_t i=0; i<nx; ++i) for (size_t j=0; j<ny; ++j) data[i][j] = distance(i,j);

    // find mins and maxs
    const float scale = distance.maxCoeff() - distance.minCoeff();

    (void) write_png (outroot.c_str(), (int)nx, (int)ny, FALSE, TRUE,
                      data, 0.0, scale, nullptr, 0.0, 1.0, nullptr, 0.0, 1.0);

    free_2d_array_f(data);
  }

  // another function to generate paths to target points using those distances
  std::vector<element> path = generate_path_from(distance, xf, yf);


  // render the path to an array
  // and write out a modified version of the elevation with the path in black
  if (true) {
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> pathimg;
    pathimg.resize(nx,ny);
    pathimg.setZero(nx,ny);

    for (element& cell : path) {
      pathimg(cell.i,cell.j) = 1.0;
    }

    std::string outroot = "out";
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

    (void) write_png (outroot.c_str(), (int)nx, (int)ny, FALSE, TRUE,
                      data, 0.0, vscale*(float)nx, nullptr, 0.0, 1.0, nullptr, 0.0, 1.0);

    free_2d_array_f(data);
  }
}
