#ifndef OECUBEMARKER_HPP
#define OECUBEMARKER_HPP

#include <visualization_msgs/InteractiveMarker.h>
#include <string>

namespace octomap_editing
{
  class OECubeMarker
  {
  public:
    OECubeMarker(std::string id, std::shared_ptr<visualization_msgs::InteractiveMarker> marker, uint seq);
    void addNeighbour(OECubeMarker neighbour);
    std::string getId();
    bool isNeighbour(OECubeMarker neighbour);
    std::shared_ptr<visualization_msgs::InteractiveMarker> getMarker();
    std::vector<std::shared_ptr<OECubeMarker>> getNeighbours();
    bool checkCoordsConstraints(geometry_msgs::Point new_own_position);

  private:
    bool checkCoords(int c1, int c2, double v1, double v2);


    std::vector<std::shared_ptr<OECubeMarker>> _neighbours;
    std::string _id;
    std::shared_ptr<visualization_msgs::InteractiveMarker> _marker;
    uint _seq;
  };
}

#endif // OECUBEMARKER_HPP
