#ifndef DATA_TYPE_H
#define DATA_TYPE_H


#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "color_type.h"

///mros Common DataType
namespace mros {

//--------------------base info-------------------------
////机器人位态以及运动速度
typedef struct PoseT{
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d velocity;
}PoseT;

//--------------------visualization_marker_publisher-------------------------
////marker可选颜色
typedef enum VisualizationMarkerColorsL{
  VISUALIZATION_MARKER_COLOR_WHITE = 0x00,
  VISUALIZATION_MARKER_COLOR_BLACK,
  VISUALIZATION_MARKER_COLOR_RED,
  VISUALIZATION_MARKER_COLOR_ORANGE,
  VISUALIZATION_MARKER_COLOR_YELLOW,
  VISUALIZATION_MARKER_COLOR_GREEN,
  VISUALIZATION_MARKER_COLOR_BLUE,
  VISUALIZATION_MARKER_COLOR_CYAN,
  VISUALIZATION_MARKER_COLOR_VIOLET,
}VisualizationMarkerColorsL;

///marker大小
typedef enum VisualizationMarkerScaleL{
  VISUALIZATION_MARKER_SCALE_01 = 0x00,
  VISUALIZATION_MARKER_SCALE_02,
  VISUALIZATION_MARKER_SCALE_03,
  VISUALIZATION_MARKER_SCALE_04,
  VISUALIZATION_MARKER_SCALE_05,
}VisualizationMarkerScaleL;

typedef struct VisualizationMarkerPubParametersT{
  std::string topic;        //要发布到的主题
  int fq;                   //发布定时器设置的回调频率
  std::string frame_id;     //参考ID
  int id;                   //当前marker id, id不能重复，否则会覆盖
  VisualizationMarkerColorsL color;
  VisualizationMarkerScaleL scale;
  int points_max;           ///点数最大值,-1则不做限制
}VisualizationMarkerPubParametersT;

static VisualizationMarkerPubParametersT VisualizationMarkerPubParameters(
    const std::string& topic, const int fq,
    const std::string& frame_id, const int id,
    const VisualizationMarkerColorsL& color,
    const VisualizationMarkerScaleL& scale,
    const int points_max  ){

  VisualizationMarkerPubParametersT params;
  params.topic = topic;
  params.fq = fq;
  params.frame_id = frame_id;
  params.id = id;
  params.scale = scale;
  params.color = color;
  params.points_max = points_max;
  return  params;

}
//-------------------------------------------------------------------------------



}  //end of namespace mros

#endif // DATA_TYPE_HPP


