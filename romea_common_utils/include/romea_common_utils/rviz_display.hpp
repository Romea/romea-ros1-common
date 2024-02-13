#ifndef _romea_Pose2DRvizDisplay_hpp_
#define _romea_Pose2DRvizDisplay_hpp_

//ros
#include <rviz_visual_tools/rviz_visual_tools.h>

//romea
#include <romea_core_common/geometry/Pose2D.hpp>
#include <romea_core_common/geometry/Position2D.hpp>

namespace romea
{

void publish(rviz_visual_tools::RvizVisualTools & rvizVisualTool,
             const romea::core::Pose2D & bodyPose2D,
             const rviz_visual_tools::colors & color,
             double positionAlongZBodyAxis=0,
             double scaleAlongZBodyAxis=0.1,
             double sigma=3);

void publish(rviz_visual_tools::RvizVisualTools & rvizVisualTool,
             const romea::core::Position2D & bodyPosition2D,
             const rviz_visual_tools::colors & color,
             double positionAlongZBodyAxis=0,
             double scaleAlongZBodyAxis=0.1,
             double sigma=3);

}

#endif
