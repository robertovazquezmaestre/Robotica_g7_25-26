#pragma once
#include <QPointF>
#include <QRectF>
#include <Eigen/Dense>
#include <vector>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/sliding_window.hpp>

#include "src/common_types.h"

  struct NominalRoom
        {
            float width; //  mm
            float length;
            explicit NominalRoom(const float width_=10000.f, const float length_=5000.f, Corners  corners_ = {}) :
                width(width_), length(length_)
            {};
            [[nodiscard]] Corners corners() const
            {
                // compute corners from width and length
                return {
                    {QPointF{-width/2.f, -length/2.f}, 0.f, 0.f},
                    {QPointF{width/2.f, -length/2.f}, 0.f, 0.f},
                    {QPointF{width/2.f, length/2.f}, 0.f, 0.f},
                    {QPointF{-width/2.f, length/2.f}, 0.f, 0.f}
                };
            }
            [[nodiscard]] QRectF rect() const
            {
                return QRectF{-width/2.f, -length/2.f, width, length};
            }
            [[nodiscard]] Corners transform_corners_to(const Eigen::Affine2d &transform) const  // for room to robot pass the inverse of robot_pose
            {
                Corners transformed_corners;
                for(const auto &[p, _, __] : corners())
                {
                    auto ep = Eigen::Vector2d{p.x(), p.y()};
                    Eigen::Vector2d tp = transform * ep;
                    transformed_corners.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, 0.f, 0.f);
                }
                return transformed_corners;
            }

            Doors doors;

            [[nodiscard]] Walls get_walls()
            {
                Walls walls;
                auto cs =corners();
                cs.push_back(cs[0]);
                for (const auto& [i,c] : cs | iter::sliding_window(2) | iter::enumerate)
                {
                    //esto coge el Qpoint de c[i]
                  const auto qp1 =std::get<0>(c[0]);
                    const auto qp2 =std::get<0>(c[1]);

                    //transformacion del Qpoint a Eigen
                    Eigen::Vector2f c1(qp1.x(), qp1.y());
                    Eigen::Vector2f c2(qp2.x(), qp2.y());
                    const auto r=Eigen::ParametrizedLine<float,2>::Through(c1,c2);

                    // c[0] y c[1] se pasan directamente porque son corners
                    walls.emplace_back(r,i,c[0],c[1]);
                }
                return walls;
            }

            [[nodiscard]] Wall get_closest_wall_to_point(const Eigen::Vector2f &p)
            {
                const auto walls = get_walls();
                auto m = std::ranges::min_element(walls, [p](const auto &w1,  const auto &w2)
                    {
                    const auto &[r1,r2,r3,r4]=w1;
                    const auto &[r5,r6,r7,r8]=w2;
                        return r1.distance(p) < r5.distance(p);
                    });
                return *m;
            }

            [[nodiscard]] Eigen::Vector2f get_projection_of_point_on_closest_wall(const Eigen::Vector2f &p)
            {
                const auto w = get_closest_wall_to_point(p);
                const auto &r = std::get<0>(w);
                auto pp = r.projection(p);
                return pp;
            }

        };