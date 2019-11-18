#ifndef OEMARKER_HPP
#define OEMARKER_HPP

#include <octomap/ColorOcTree.h>

namespace octomap_editing
{
  struct OEMarker
  {
    OEMarker(octomap::ColorOcTreeNode::Color color, octomap::point3d coords, octomap::OcTreeKey key);
    octomap::ColorOcTreeNode::Color _color;
    octomap::point3d _coordinates;
    octomap::OcTreeKey _key;
  };
}
#endif // OEMARKER_HPP
