//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <QGraphicsItem>


Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    Doors doors;
    std::vector<RoboCompLidar3D::TPoint> peaks;

    // 1 Detectar picos usando iter::sliding_window(2)
    for (auto &&window : iter::sliding_window(points, 2))
    {
        const auto &p1 = window[0];
        const auto &p2 = window[1];

        float diff = std::abs(p2.distance2d - p1.distance2d);
        if (diff > 1000)
        {
            peaks.push_back(p1.distance2d < p2.distance2d ? p1 : p2);
        }
    }

    // 2 Dibujar los picos detectados (debug visual)
    static std::vector<QGraphicsItem*> draw_points;
    const QColor color("LightGreen");
    const QPen pen(color, 10);
    if (scene)
    {
        for (const auto &p : peaks)
        {
            const auto dp = scene->addRect(-25, -25, 50, 50, pen);
            dp->setPos(p.x, p.y);
        }
    }

    // 3 Non-maximum suppression (eliminar picos demasiado cercanos)
    std::vector<RoboCompLidar3D::TPoint> filtered_peaks;
    const float min_dist = 200.0f; // distancia mínima entre picos

    for (const auto &p : peaks)
    {
        bool too_close = false;
        for (const auto &fp : filtered_peaks)
        {
            float dist = std::hypot(p.x - fp.x, p.y - fp.y);
            if (dist < min_dist)
            {
                too_close = true;
                break;
            }
        }
        if (!too_close)
            filtered_peaks.push_back(p);
    }

    // 4 Buscar pares de picos que representen puertas
    for (auto &&pair : iter::combinations(filtered_peaks, 2))
    {
        const auto &a = pair[0];
        const auto &b = pair[1];

        float dist = std::hypot(a.x - b.x, a.y - b.y);
        if (dist >= 800 && dist <= 1200)
        {
            Door door(
            Eigen::Vector2f(a.x, a.y), a.phi,
            Eigen::Vector2f(b.x, b.y), b.phi
        );
            doors.push_back(door);
        }
    }
   return doors;
}

// Method to use the Doors vector to filter out the LiDAR points that como from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);
    if(doors.empty()) return points;

    // for each door, check if the distance from the robot to each lidar point is smaller than the distance from the robot to the door
    RoboCompLidar3D::TPoints filtered;
    for(const auto &d : doors)
    {
        const float dist_to_door = d.center().norm();
        // Check if the angular range wraps around the -π/+π boundary
        const bool angle_wraps = d.p2_angle < d.p1_angle;
        for(const auto &p : points)
        {
            // Determine if point is within the door's angular range
            bool point_in_angular_range;
            if (angle_wraps)
            {
                // If the range wraps around, point is in range if it's > p1_angle OR < p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) or (p.phi < d.p2_angle);
            }
            else
            {
                // Normal case: point is in range if it's between p1_angle and p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) and (p.phi < d.p2_angle);
            }

            // Filter out points that are through the door (in angular range and farther than door)
            if(point_in_angular_range and p.distance2d >= dist_to_door)
                continue;

            //qInfo() << __FUNCTION__ << "Point angle: " << p.phi << " Door angles: " << d.p1_angle << ", " << d.p2_angle << " Point distance: " << p.distance2d << " Door distance: " << dist_to_door;
            filtered.emplace_back(p);
        }
    }
    return filtered;
}
