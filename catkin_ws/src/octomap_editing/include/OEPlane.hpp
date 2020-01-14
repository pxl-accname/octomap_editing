#ifndef OEPLANE_HPP
#define OEPLANE_HPP

#include <tf/tf.h>
#include <octomap/octomap.h>

namespace octomap_editing
{
  class OEPlane
  {
  public:
    OEPlane(octomap::point3d n, double d, int sign);
    bool checkDistancePointPlane(octomap::point3d point);
    double pointPlaneDistance(octomap::point3d point);
    int getSign();
    void setSign(int s);

  private:
    octomap::point3d _n;
    double _d;
    int _sign;
  };
}

#endif // OEPLANE_HPP
