#ifndef RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_
#define RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_

#include "rviz_common/message_filter_display.hpp"
#include "rviz_plugin_tutorial_msgs/msg/point2_d.hpp"
#include "rviz_rendering/objects/shape.hpp"

namespace rviz_plugin_tutorial {
    
class PointDisplay : public
    rviz_common::MessageFilterDisplay<rviz_plugin_tutorial_msgs::msg::Point2D>
{
    Q_OBJECT

protected:
    std::unique_ptr<rviz_rendering::Shape> point_shape_;

    void processMessage(
        const rviz_plugin_tutorial_msgs::msg::Point2D::ConstSharedPtr msg
    ) override;
    void onInitialize() override;
};

} // namespace rviz_plugin_tutorial

#endif // RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_