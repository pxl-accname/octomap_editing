#include <OEMarker.hpp>

namespace octomap_editing
{
  OEMarker::OEMarker(octomap::ColorOcTreeNode::Color color, octomap::point3d coords, octomap::OcTreeKey key)
  {
    _color = color;
    _coordinates = coords;
    _key = key;
  }
}
