#ifndef OECUBEMARKER_HPP
#define OECUBEMARKER_HPP

#include <visualization_msgs/InteractiveMarker.h>
#include <string>

namespace octomap_editing
{
  class OECubeMarker
  {
  public:
    OECubeMarker(std::string id, visualization_msgs::InteractiveMarker marker);
    void addNeighbour(OECubeMarker neighbour);
    std::string getId();
    bool isNeighbour(OECubeMarker neighbour);
    visualization_msgs::InteractiveMarker getMarker();
    std::vector<std::shared_ptr<OECubeMarker>> getNeighbours();

  private:
    std::vector<std::shared_ptr<OECubeMarker>> _neighbours;
    std::string _id;
    std::shared_ptr<visualization_msgs::InteractiveMarker> _marker;
  };
}

#endif // OECUBEMARKER_HPP
