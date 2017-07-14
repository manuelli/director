#ifndef __ddPointCloudLCM_h
#define __ddPointCloudLCM_h

#include <QObject>

#include "ddLCMThread.h"
#include "ddLCMSubscriber.h"
#include "ddAppConfigure.h"

#include <string>
#include <sstream>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkUnsignedIntArray.h>
#include <vtkFloatArray.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/pointcloud2_t.hpp>
#include <lcmtypes/bot_core/pointcloud_t.hpp>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

// ros includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"

//class RosPointcloudCallback{
//public:
//  RosPointcloudCallback(std::shared_ptr<ddPointCloudLCM> ptr, int idx);
//  void callback(const sensor_msgs::PointCloud2& msg);
//
//  int idx_;
//  std::shared_ptr<ddPointCloudLCM> ptr;
//};


class DD_APP_EXPORT ddPointCloudLCM : public QObject
{
  Q_OBJECT

public:

  ddPointCloudLCM(QObject* parent=NULL);
  
  void init(ddLCMThread* lcmThread, const QString& botConfigFile);
  qint64 getPointCloudFromPointCloud(vtkPolyData* polyDataRender);
  void getRosPointCloud(vtkPolyData* polyDataRender);
  void getRosPointCloud(vtkPolyData* polyDataRender, int cameraNumber);

  QStringList getLidarNames() const;
  QString getLidarFriendlyName(const QString& lidarName);
  int getLidarFrequency(const QString& lidarName);
  bool displayLidar(const QString& lidarName);
  QList<int> getLidarIntensity(const QString& lidarName);
  QString getLidarChannelName(const QString& lidarName);
  QString getLidarCoordinateFrame(const QString& lidarName);
  void onRosMessage(const std_msgs::String::ConstPtr& msg);
  void onRosMessage2(const std_msgs::String::ConstPtr& msg, int idx);
  void onPointCloud2RosMessage(const sensor_msgs::PointCloud2& msg, int cameraNumber);
  void onPointCloud2RosMessage1(const sensor_msgs::PointCloud2& msg);
  void onPointCloud2RosMessage2(const sensor_msgs::PointCloud2& msg);
  void onPointCloud2RosMessage3(const sensor_msgs::PointCloud2& msg);
  void onPointCloud2RosMessage4(const sensor_msgs::PointCloud2& msg);
  void onPointCloud2RosMessageTest(const sensor_msgs::PointCloud2& msg);

  void setupRosSubscribers();

protected slots:

  void onPointCloudFrame(const QByteArray& data, const QString& channel);
  void onPointCloud2Frame(const QByteArray& data, const QString& channel);


protected:

  BotParam* mBotParam;

  ddLCMThread* mLCM;

  vtkSmartPointer<vtkPolyData> mPolyData;
  vtkSmartPointer<vtkPolyData> rosPolyData;
  std::map<int, vtkSmartPointer<vtkPolyData>> rosPolyDataMap;
  int64_t mUtime;
  QMutex mPolyDataMutex;

  std::shared_ptr<ros::NodeHandle> nodeHandle;
  std::vector<ros::Subscriber> rosSubscribers;
  ros::Subscriber rosSubscriber;
  std::shared_ptr<ros::AsyncSpinner> asyncSpinner;
//  std::vector<std::shared_ptr<RosPointcloudCallback>> callbackObjects;
};



#endif
