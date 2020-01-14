#include <OEPlane.hpp>

namespace octomap_editing
{
  OEPlane::OEPlane(octomap::point3d n, double d, int sign)
    : _n(n),
      _d(d),
      _sign(sign)
  {}

  bool
  OEPlane::checkDistancePointPlane(octomap::point3d point)
  {
    double distance = (_n.x() * point.x() + _n.y() * point.y() + _n.z() * point.z() - _d) / (sqrt(pow(_n.x(), 2) + pow(_n.y(), 2) + pow(_n.z(), 2)));
    if (_sign == 1 && distance < 0)
    {
      return false;
    }
    else if (_sign == -1 && distance >= 0)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  double
  OEPlane::pointPlaneDistance(octomap::point3d point)
  {
    return (_n.x() * point.x() + _n.y() * point.y() + _n.z() * point.z() - _d) / (sqrt(pow(_n.x(), 2) + pow(_n.y(), 2) + pow(_n.z(), 2)));
  }

  int OEPlane::getSign()  { return _sign; }
  void OEPlane::setSign(int s) { _sign = s; }
}
