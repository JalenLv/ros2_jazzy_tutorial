#include "rviz_plugin_tutorial/point_display.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/parse_color.hpp"

namespace rviz_plugin_tutorial {
    
void PointDisplay::processMessage(
    const rviz_plugin_tutorial_msgs::msg::Point2D::ConstSharedPtr msg
) {
    RVIZ_COMMON_LOG_INFO_STREAM(
        "We got a message with frame " << msg->header.frame_id
    );

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
        RVIZ_COMMON_LOG_DEBUG_STREAM(
            "Error transforming from frame '" << msg->header.frame_id <<
            "' to frame '" << qPrintable(fixed_frame_) << "'";
        );
    }

    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);

    Ogre::Vector3 point_pos;
    point_pos.x = msg->x;
    point_pos.y = msg->y;
    this->point_shape_->setPosition(point_pos);
}

void PointDisplay::onInitialize() {
    MFDClass::onInitialize();
    this->point_shape_ = std::make_unique<rviz_rendering::Shape>(
        rviz_rendering::Shape::Type::Cube,
        scene_manager_, scene_node_
    );

    this->color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
        "Point Color", QColor(36, 64, 142),
        "Color to draw the point.",
        this, SLOT(updateStyle())
    );
    this->updateStyle();
}

void PointDisplay::updateStyle() {
    Ogre::ColourValue color = rviz_common::properties::qtToOgre(
        this->color_property_->getColor()
    );
    this->point_shape_->setColor(color);
}
    
} // namespace rviz_plugin_tutorial

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorial::PointDisplay, rviz_common::Display);
