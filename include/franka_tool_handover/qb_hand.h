#include <ros/ros.h>

namespace franka_tool_handover {

class QbHand {
  protected:
    ros::NodeHandle node_handle_;
    std::string hand_name;
  
  public:
    QbHand(std::string name);
    ~QbHand();

    void publisher(const std::string &action);

};

} // namespace franka_tool_handover