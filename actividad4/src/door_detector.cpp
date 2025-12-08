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
    //std::vector<RoboCompLidar3D::TPoint> peaks;
    Peaks peaks;

    // 1 Detectar picos usando iter::sliding_window(2)
    for (auto &&window : iter::sliding_window(points, 2))
    {
        const auto &p1 = window[0];
        const auto &p2 = window[1];

        float diff = std::abs(p2.distance2d - p1.distance2d);
        if (diff > 1000)
        {
            const auto &p = (p1.distance2d < p2.distance2d ? p1 : p2);


            Eigen::Vector2f pos(p.x, p.y);    // posición en 2D
            float angle = p.phi;            // campo del ángulo

            peaks.emplace_back(pos, angle);

        }
    }

    // 2 Dibujar los picos detectados (debug visual)
    // static std::vector<QGraphicsItem*> draw_points;
    // const QColor color("LightGreen");
    // const QPen pen(color, 10);
    // if (scene)
    // {
    //      for (const auto &p : peaks)
    //      {
    //         const auto &pos = std::get<0>(p);
    //         const auto dp = scene->addRect(-25, -25, 50, 50, pen);
    //         dp->setPos(pos.x(), pos.y());
    //     }
    // }



    // non-maximum suppression of peaks: remove peaks closer than 500mm
    Peaks nms_peaks;
    for (const auto &[p, a] : peaks)
        if (const bool too_close = std::ranges::any_of(nms_peaks, [&p](const auto &p2) { return (p - std::get<0>(p2)).norm() < 500.f; }); not too_close)
            nms_peaks.emplace_back(p, a);
    peaks = nms_peaks;


    // 4 Buscar pares de picos que representen puertas
    for (auto &&pair : iter::combinations(nms_peaks, 2))
    {
        const auto &[pa, angA] = pair[0];
        const auto &[pb, angB] = pair[1];

        float dist = (pa - pb).norm();   // esto reemplaza hypot(x-x, y-y)

        if (dist >= 800 && dist <= 1200)
        {
            Door door(
                pa, angA,
                pb, angB
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
    for (const auto &p : points)
    {
        bool discard_point = false;

        for (const auto &d : doors)
        {
            const float dist_to_door = d.center().norm();
            const bool angle_wraps = d.p2_angle < d.p1_angle;
            float offset = 0.35;

            // Determinar si el punto está dentro del rango angular de la puerta
            bool point_in_angular_range;
            if (angle_wraps)
            {
                // Si el rango envuelve ±π → dentro si es > p1_angle O < p2_angle
                point_in_angular_range =
                    (p.phi > d.p1_angle - offset) ||
                    (p.phi < d.p2_angle + offset);
            }
            else
            {
                // Caso normal: dentro si está entre p1_angle y p2_angle
                point_in_angular_range =
                    (p.phi > d.p1_angle - offset) &&
                    (p.phi < d.p2_angle + offset);
            }

            // Comprobar si el punto está a través de esta puerta
            if (point_in_angular_range && p.distance2d >= dist_to_door)
            {
                discard_point = true;
                break;   // no hace falta revisar más puertas
            }
        }

        // Si no fue descartado por ninguna puerta, se añade
        if (!discard_point)
            filtered.emplace_back(p);
    }

    return filtered;
}
