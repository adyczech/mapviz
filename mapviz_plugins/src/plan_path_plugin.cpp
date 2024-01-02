// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <mapviz_plugins/plan_path_plugin.h>

// QT libraries
#include <QDateTime>
#include <QDialog>
#include <QGLWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPalette>
#include <QStaticText>

// ROS libraries
#include <ros/ros.h>

// #include <swri_route_util/util.h>
#include <swri_transform_util/frames.h>

// #include <marti_nav_msgs/srv/plan_route.hpp>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

// C++ standard libraries
#include <chrono>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::PlanPathPlugin, mapviz::MapvizPlugin)

using namespace std::chrono_literals;

// namespace mnm = marti_nav_msgs;
// namespace sru = swri_route_util;
namespace stu = swri_transform_util;

namespace mapviz_plugins
{
  PlanPathPlugin::PlanPathPlugin()
  : MapvizPlugin()
  , ui_()
  , config_widget_(new QWidget())
  , map_canvas_(nullptr)
  // , failed_service_(false)
  , selected_point_(-1)
  , is_mouse_down_(false)
  , mouse_down_time_(0)
  , max_ms_(Q_INT64_C(500))
  , max_distance_(2.0)
  {
    ui_.setupUi(config_widget_);

    // ui_.color->setColor(Qt::green);
    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Window, Qt::white);
    config_widget_->setPalette(p);
    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.publish, SIGNAL(clicked()), this,
                     SLOT(PublishPath()));
    QObject::connect(ui_.clear, SIGNAL(clicked()), this,
                     SLOT(Clear()));
    QObject::connect(this,
                     SIGNAL(VisibleChanged(bool)),
                     this,
                     SLOT(VisibilityChanged(bool)));
    frame_timer_.start(1000);
    QObject::connect(&frame_timer_, SIGNAL(timeout()), this, SLOT(updateFrames()));
  }

  PlanPathPlugin::~PlanPathPlugin()
  {
    if (map_canvas_)
    {
      map_canvas_->removeEventFilter(this);
    }
  }

  void PlanPathPlugin::VisibilityChanged(bool visible)
  {
    if (visible)
    {
      map_canvas_->installEventFilter(this);
    }
    else
    {
      map_canvas_->removeEventFilter(this);
    }
  }

  void PlanPathPlugin::PublishPath()
  {   
    if (waypoints_.size() < 1 || !Visible())
    {
      PrintError("Path must have at least one waypoint.");
      return;
    } 

    std::string output_frame = ui_.outputframe->currentText().toStdString();
    if (output_frame.empty())
    {
      PrintError("Output frame is not set.");
      return;
    }

    std::vector<geometry_msgs::PoseStamped> transformed_waypoints;

    TransformFrame(transformed_waypoints, output_frame);

    auto path = nav_msgs::Path();
    // path.header.frame_id = swri_transform_util::_wgs84_frame;
    path.header.frame_id = output_frame;
    path.header.stamp = ros::Time::now();
    path.poses = transformed_waypoints;

    if (path_topic_ != ui_.topic->text().toStdString())
      {
        path_topic_ = ui_.topic->text().toStdString();
        path_pub_.shutdown();
        path_pub_ = node_.advertise<nav_msgs::Path>(
          path_topic_,
          1);
      }

      path_pub_.publish(path);
    
  }

  void PlanPathPlugin::TransformFrame(std::vector<geometry_msgs::PoseStamped>& transformed_waypoints, const std::string& output_frame)
  {
    stu::Transform transform;
    if (tf_manager_->GetTransform(output_frame, stu::_wgs84_frame, transform))
    {
      for (auto & waypoint : waypoints_)
      {
        tf::Vector3 point(waypoint.pose.position.x, waypoint.pose.position.y, 0);
        point = transform * point;
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        transformed_waypoints.push_back(pose);
      }
    }
    else
    {
      PrintError("Failed to transform.");
    }

  }


  void PlanPathPlugin::Clear()
  {
    waypoints_.clear();
    // route_preview_ = sru::RoutePtr();
  }

  void PlanPathPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message, 1.0);
  }

  void PlanPathPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message, 1.0);
  }

  void PlanPathPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message, 1.0);
  }

  QWidget* PlanPathPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool PlanPathPlugin::Initialize(QGLWidget* canvas)
  {
    map_canvas_ = dynamic_cast<mapviz::MapCanvas*>(canvas);
    map_canvas_->installEventFilter(this);

    // retry_timer_ = node_->create_wall_timer(1000ms, [this](){Retry();});

    initialized_ = true;
    return true;
  }

  void PlanPathPlugin::updateFrames()
  {
    std::vector<std::string> frames;
    tf_->getFrameStrings(frames);

    bool supports_wgs84 = tf_manager_->SupportsTransform(
        swri_transform_util::_local_xy_frame,
        swri_transform_util::_wgs84_frame);

    if (supports_wgs84)
    {
      frames.push_back(swri_transform_util::_wgs84_frame);
    }

    if (ui_.outputframe->count() >= 0 &&
        static_cast<size_t>(ui_.outputframe->count()) == frames.size())
    {
      bool changed = false;
      for (size_t i = 0; i < frames.size(); i++)
      {
        if (frames[i] != ui_.outputframe->itemText(static_cast<int>(i)).toStdString())
        {
          changed = true;
        }
      }

      if (!changed)
        return;
    }

    std::string current_output = ui_.outputframe->currentText().toStdString();

    ui_.outputframe->clear();
    for (size_t i = 0; i < frames.size(); i++)
    {
      ui_.outputframe->addItem(frames[i].c_str());
    }

    if (current_output != "")
    {
      int index = ui_.outputframe->findText(current_output.c_str());
      if (index < 0)
      {
        ui_.outputframe->addItem(current_output.c_str());
      }

      index = ui_.outputframe->findText(current_output.c_str());
      ui_.outputframe->setCurrentIndex(index);
    }
  }

  bool PlanPathPlugin::eventFilter(QObject *object, QEvent* event)
  {
    switch (event->type())
    {
      case QEvent::MouseButtonPress:
        return handleMousePress(dynamic_cast<QMouseEvent*>(event));
      case QEvent::MouseButtonRelease:
        return handleMouseRelease(dynamic_cast<QMouseEvent*>(event));
      case QEvent::MouseMove:
        return handleMouseMove(dynamic_cast<QMouseEvent*>(event));
      default:
        return false;
    }
  }

  bool PlanPathPlugin::handleMousePress(QMouseEvent* event)
  {
    selected_point_ = -1;
    int closest_point = 0;
    double closest_distance = std::numeric_limits<double>::max();

    QPointF point = event->localPos();
    stu::Transform transform;
    if (tf_manager_->GetTransform(target_frame_, stu::_wgs84_frame, transform))
    {
      for (size_t i = 0; i < waypoints_.size(); i++)
      {
        tf::Vector3 waypoint(
            waypoints_[i].pose.position.x,
            waypoints_[i].pose.position.y,
            0.0);
        waypoint = transform * waypoint;

        QPointF transformed =
        map_canvas_->FixedFrameToMapGlCoord(QPointF(waypoint.x(), waypoint.y()));

        double distance = QLineF(transformed, point).length();

        if (distance < closest_distance)
        {
          closest_distance = distance;
          closest_point = static_cast<int>(i);
        }
      }
    }

    if (event->button() == Qt::LeftButton)
    {
      if (closest_distance < 15)
      {
        selected_point_ = closest_point;
        return true;
      } else {
        is_mouse_down_ = true;
        mouse_down_pos_ = event->localPos();
        mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();
        return false;
      }
    } else if (event->button() == Qt::RightButton) {
      if (closest_distance < 15)
      {
        waypoints_.erase(waypoints_.begin() + closest_point);
        // PlanPath();
        return true;
      }
    }

    return false;
  }

  bool PlanPathPlugin::handleMouseRelease(QMouseEvent* event)
  {
    QPointF point = event->localPos();
    if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < waypoints_.size())
    {
      stu::Transform transform;
      if (tf_manager_->GetTransform(stu::_wgs84_frame, target_frame_, transform))
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;
        waypoints_[selected_point_].pose.position.x = position.x();
        waypoints_[selected_point_].pose.position.y = position.y();
        // PlanPath();
      }

      selected_point_ = -1;
      return true;
    } else if (is_mouse_down_) {
      qreal distance = QLineF(mouse_down_pos_, point).length();
      qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

      // Only fire the event if the mouse has moved less than the maximum distance
      // and was held for shorter than the maximum time..  This prevents click
      // events from being fired if the user is dragging the mouse across the map
      // or just holding the cursor in place.
      if (msecsDiff < max_ms_ && distance <= max_distance_)
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);

        stu::Transform transform;
        tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
        if (tf_manager_->GetTransform(stu::_wgs84_frame, target_frame_, transform))
        {
          position = transform * position;

          geometry_msgs::PoseStamped pose;
          pose.pose.position.x = position.x();
          pose.pose.position.y = position.y();
          waypoints_.push_back(pose);
          // PlanPath();
        }
      }
    }
    is_mouse_down_ = false;

    return false;
  }

  bool PlanPathPlugin::handleMouseMove(QMouseEvent* event)
  {
    if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < waypoints_.size())
    {
      QPointF point = event->localPos();
      stu::Transform transform;
      if (tf_manager_->GetTransform(stu::_wgs84_frame, target_frame_, transform))
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;
        waypoints_[selected_point_].pose.position.y = position.y();
        waypoints_[selected_point_].pose.position.x = position.x();
        // PlanPath();
      }

      return true;
    }
    return false;
  }

  void PlanPathPlugin::Draw(double x, double y, double scale)
  {
    stu::Transform transform;
    if (tf_manager_->GetTransform(target_frame_, stu::_wgs84_frame, transform))
    {
      // if (!failed_service_)
      // {
      //   if (route_preview_)
      //   {
      //     sru::Route route = *route_preview_;
      //     sru::transform(route, transform, target_frame_);

      //     glLineWidth(2);
      //     const QColor color = ui_.color->color();
      //     glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);
      //     glBegin(GL_LINE_STRIP);

      //     for (auto & point : route.points)
      //     {
      //       glVertex2d(point.position().x(), point.position().y());
      //     }

      //     glEnd();
      //   }
      // }

      // Draw waypoints

      glPointSize(20);
      glColor4f(0.0, 1.0, 1.0, 1.0);
      glBegin(GL_POINTS);

      for (auto & waypoint : waypoints_)
      {
        tf::Vector3 point(waypoint.pose.position.x, waypoint.pose.position.y, 0);
        point = transform * point;
        glVertex2d(point.x(), point.y());
      }
      glEnd();
    } else {
      PrintError("Failed to transform.");
    }
  }

  void PlanPathPlugin::Paint(QPainter* painter, double x, double y, double scale)
  {
    painter->save();
    painter->resetTransform();

    QPen pen(QBrush(QColor(Qt::darkCyan).darker()), 1);
    painter->setPen(pen);
    painter->setFont(QFont("DejaVu Sans Mono", 7));

    stu::Transform transform;
    if (tf_manager_->GetTransform(target_frame_, stu::_wgs84_frame, transform))
    {
      for (size_t i = 0; i < waypoints_.size(); i++)
      {
        tf::Vector3 point(waypoints_[i].pose.position.x, waypoints_[i].pose.position.y, 0);
        point = transform * point;
        QPointF gl_point = map_canvas_->FixedFrameToMapGlCoord(QPointF(point.x(), point.y()));
        QPointF corner(gl_point.x() - 20, gl_point.y() - 20);
        QRectF rect(corner, QSizeF(40, 40));
        painter->drawText(
          rect,
          Qt::AlignHCenter | Qt::AlignVCenter,
          QString::fromStdString(std::to_string(i + 1)));
      }
    }

    painter->restore();
  }

  void PlanPathPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["path_topic"])
    {
      std::string path_topic = node["path_topic"].as<std::string>();
      ui_.topic->setText(path_topic.c_str());
    }
    // if (node["color"])
    // {
    //   std::string color = node["color"].as<std::string>();
    //   ui_.color->setColor(QColor(color.c_str()));
    // }
    // if (node["start_from_vehicle"])
    // {
      // bool start_from_vehicle = node["start_from_vehicle"].as<bool>();
      // ui_.start_from_vehicle->setChecked(start_from_vehicle);
    // }

    // PlanPath();
  }

  void PlanPathPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string path_topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "path_topic" << YAML::Value << path_topic;

    // std::string color = ui_.color->color().name().toStdString();
    // emitter << YAML::Key << "color" << YAML::Value << color;   

    // bool start_from_vehicle = ui_.start_from_vehicle->isChecked();
    // emitter << YAML::Key << "start_from_vehicle" << YAML::Value << start_from_vehicle;
  }
}   // namespace mapviz_plugins
