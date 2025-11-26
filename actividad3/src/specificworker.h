/*
 *    Copyright (C) 2025 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \brief Clase SpecificWorker implementa la funcionalidad principal del componente.
 * @author authorname
 */

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// Descomentar para reducir automáticamente el período por inactividad
//#define HIBERNATION_ENABLED

#include <expected>
#include <genericworker.h>
#include <tuple>
#include <vector>
#include <optional>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <random>
#include <doublebuffer/DoubleBuffer.h>
#include "time_series_plotter.h"
#ifdef emit
#undef emit
#endif
#include "common_types.h"
#include "hungarian.h"
#include "ransac_line_detector.h"
#include "munkres.hpp"
#include "room_detector.h"
#include "nominal_room.h"
#include "door_detector.h"
#include "image_processor.h"
#include "pointcloud_center_estimator.h"

class SpecificWorker : public GenericWorker
{
Q_OBJECT

    using Point = std::tuple<float,float,float>;
    using Points = std::vector<Point>;

public:
    // -----------------------
    // Constructor y destructor
    // -----------------------
    SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
    ~SpecificWorker();
    std::mt19937 rng;                       // Motor de números aleatorios
    std::uniform_int_distribution<int> dist; // Distribución 0-9

public slots:
    // -----------------------
    // Métodos principales
    // -----------------------
    void initialize();   ///< Inicializa el worker una vez
    void compute();      ///< Bucle principal de cálculo
    void emergency();    ///< Maneja el estado de emergencia
    void restore();      ///< Restaura el componente después de una emergencia
    int startup_check(); ///< Realiza las comprobaciones de inicio
    void new_target_slot(QPointF p); ///< Slot para recibir coordenadas de destino
    void update_ropot_position();    ///< Actualiza la posición del robot en la GUI
    void chocachoca(std::optional<RoboCompLidar3D::TPoints> filter_data, float &advance_speed, float &rotation_speed);

private:
    // -----------------------
    // Flags y atributos
    // -----------------------
    bool startup_check_flag; ///< Indica si se deben hacer comprobaciones de inicio

    // -----------------------
    // Atributos gráficos
    // -----------------------
    QRectF dimensions;                     ///< Dimensiones del área del visor
    AbstractGraphicViewer *viewer = nullptr; ///< Visor gráfico
    AbstractGraphicViewer *viewer2 = nullptr; ///< Visor gráfico
    const int ROBOT_LENGTH = 400;         ///< Longitud del robot
    QGraphicsPolygonItem *robot_polygon = nullptr; ///< Representación gráfica del robot
    QGraphicsPolygonItem *robot_polygon2 = nullptr; ///< Representación gráfica del robot

    //Variables respecto al robot
    float safe_distance = 400; // mm
    static constexpr float FRONT_LEFT_ANGLE  = M_PI / 2;
    static constexpr float FRONT_RIGHT_ANGLE = -M_PI / 2;
    // Ángulo frontal y trasero del robot
    static constexpr float FRONT_ANGLE = 0.0f;   // frente del robot
    static constexpr float BACK_ANGLE  = M_PI;   // parte trasera del robot


    // -----------------------
    // Estado de movimiento del robot
    // -----------------------
    enum class RobotMode { IDLE, FORWARD, TURN, FOLLOW_WALL, SPIRAL }; ///< Modos posibles de movimiento
    RobotMode current_mode = RobotMode::FORWARD; ///< Modo actual del robot

    // -----------------------
    // Métodos para cada modo (vacíos por ahora)
    // -----------------------
    std::tuple<RobotMode, float, float> FORWARD_method(const std::optional<RoboCompLidar3D::TPoints>& filter_data);
    std::tuple<RobotMode, float, float> FOLLOW_WALL_method(const std::optional<RoboCompLidar3D::TPoints>& filter_data);
    std::tuple<RobotMode, float, float> SPIRAL_method(const std::optional<RoboCompLidar3D::TPoints>& filter_data);
    std::tuple<RobotMode, float, float> TURN_method(const std::optional<RoboCompLidar3D::TPoints>& filter_data);

    // -----------------------
    // Métodos auxiliares
    // -----------------------
    void draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene* scene); ///< Dibuja los puntos del LIDAR en la escena
    std::optional<RoboCompLidar3D::TPoints> filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points); ///< Filtra la nube de puntos
    float calcularDistanciaMinima(const std::optional<RoboCompLidar3D::TPoints>& lidar_data, bool bandera);///< Calcula la dostancia mínima entre FRONT_LEFT_ANGLE y FRONT_RIGHT_ANGLE, si bandera  traue se evalua para 360
    float distanciaAnguloPromedio (const std::optional<RoboCompLidar3D::TPoints>& lidar_data, float ang);
    float calcularDistanciaMinima(const std::optional<RoboCompLidar3D::TPoints>& lidar_data, float rangoA, float rangoB);
    RoboCompLidar3D::TPoints filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d = 200.0f); ///< Filtra puntos aislados de la nube


    AbstractGraphicViewer *viewer_room;  // new frame to show the room
    Eigen::Affine2d robot_pose;  // Eigen type to represent a rotation+translation
    rc::Room_Detector room_detector;  // object to compute the corners
    rc::Hungarian hungarian; // object to match the two sets of corners
    QGraphicsPolygonItem *room_draw_robot; // to draw the robot inside the room

    NominalRoom room{10000.f, 5000.f,
            {{QPointF{-5000.f, -2500.f}, 0.f, 0.f},
                   {QPointF{5000.f, -2500.f}, 0.f, 0.f},
                   {QPointF{5000.f, 2500.f}, 0.f, 0.f},
                   {QPointF{-5000.f, 2500.f}, 0.f, 0.f}}};
    struct Params
        {
            float ROBOT_WIDTH = 460;  // mm
            float ROBOT_LENGTH = 480;  // mm
            float MAX_ADV_SPEED = 1000; // mm/s
            float MAX_ROT_SPEED = 1; // rad/s
            float MAX_SIDE_SPEED = 50; // mm/s
            float MAX_TRANSLATION = 500; // mm/s
            float MAX_ROTATION = 0.2;
            float STOP_THRESHOLD = 700; // mm
            float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3; // mm
            float LIDAR_FRONT_SECTION = 0.2; // rads, aprox 12 degrees
            // wall
            float LIDAR_RIGHT_SIDE_SECTION = M_PI/3; // rads, 90 degrees
            float LIDAR_LEFT_SIDE_SECTION = -M_PI/3; // rads, 90 degrees
            float WALL_MIN_DISTANCE = ROBOT_WIDTH*1.2;
            // match error correction
            float MATCH_ERROR_SIGMA = 150.f; // mm
            float DOOR_REACHED_DIST = 300.f;
            std::string LIDAR_NAME_LOW = "bpearl";
            std::string LIDAR_NAME_HIGH = "helios";
            QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};

            // relocalization
            float RELOCAL_CENTER_EPS = 300.f;    // mm: stop when |mean| < eps
            float RELOCAL_KP = 0.002f;           // gain to convert mean (mm) -> speed (magnitude)
            float RELOCAL_MAX_ADV = 300.f;       // mm/s cap while re-centering
            float RELOCAL_MAX_SIDE = 300.f;      // mm/s cap while re-centering
            float RELOCAL_ROT_SPEED = 0.3f;     // rad/s while aligning
            float RELOCAL_DELTA = 5.0f * M_PI/180.f; // small probe angle in radians
            float RELOCAL_MATCH_MAX_DIST = 2000.f;   // mm for Hungarian gating
            float RELOCAL_DONE_COST = 500.f;
            float RELOCAL_DONE_MATCH_MAX_ERROR = 1000.f;
        };
        Params params;

        // viewer
        QGraphicsPolygonItem *robot_draw, *robot_room_draw;

        // rooms
        std::vector<NominalRoom> nominal_rooms{ NominalRoom{5500.f, 4000.f}, NominalRoom{8000.f, 4000.f}};

        // state machine
        enum class STATE {GOTO_DOOR, ORIENT_TO_DOOR, LOCALISE, GOTO_ROOM_CENTER, TURN, IDLE, CROSS_DOOR};
        inline const char* to_string(const STATE s) const
        {
            switch(s) {
                case STATE::IDLE:               return "IDLE";
                case STATE::LOCALISE:           return "LOCALISE";
                case STATE::GOTO_DOOR:          return "GOTO_DOOR";
                case STATE::TURN:               return "TURN";
                case STATE::ORIENT_TO_DOOR:     return "ORIENT_TO_DOOR";
                case STATE::GOTO_ROOM_CENTER:   return "GOTO_ROOM_CENTER";
                case STATE::CROSS_DOOR:         return "CROSS_DOOR";
                default:                        return "UNKNOWN";
            }
        }
        STATE state = STATE::LOCALISE;
        using RetVal = std::tuple<STATE, float, float>;
        RetVal goto_door(const RoboCompLidar3D::TPoints &points);
        RetVal orient_to_door(const RoboCompLidar3D::TPoints &points);
        RetVal cross_door(const RoboCompLidar3D::TPoints &points);
        RetVal localise(const Match &match);
        RetVal goto_room_center(const RoboCompLidar3D::TPoints &points);
        RetVal update_pose(const Corners &corners, const Match &match);
        RetVal turn(const Corners &corners);
        RetVal process_state(const RoboCompLidar3D::TPoints &data, const Corners &corners, const Match &match, AbstractGraphicViewer *viewer);

        // draw
        void draw_lidar(const RoboCompLidar3D::TPoints &filtered_points, std::optional<Eigen::Vector2d> center, QGraphicsScene *scene);

        // aux
        RoboCompLidar3D::TPoints read_data();
        std::expected<int, std::string> closest_lidar_index_to_given_angle(const auto &points, float angle);
        RoboCompLidar3D::TPoints filter_same_phi(const RoboCompLidar3D::TPoints &points);
        void print_match(const Match &match, const float error =1.f) const;

        // random number generator
        std::random_device rd;

        // DoubleBuffer for velocity commands
        DoubleBuffer<std::tuple<float, float, float, long>, std::tuple<float, float, float, long>> commands_buffer;
        std::tuple<float, float, float, long> last_velocities{0.f, 0.f, 0.f, 0.f};

        // plotter
        std::unique_ptr<TimeSeriesPlotter> time_series_plotter;
        int match_error_graph; // To store the index of the speed graph

        // doors
        DoorDetector door_detector;

        // image processor
        rc::ImageProcessor image_processor;

        // timing
        std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();

        // relocalization
        bool relocal_centered = false;
        bool localised = false;

        bool update_robot_pose(const Corners &corners, const Match &match);
        void move_robot(float adv, float rot, float max_match_error);
        Eigen::Vector3d solve_pose(const Corners &corners, const Match &match);
        void predict_robot_pose();
        std::tuple<float, float> robot_controller(const Eigen::Vector2f &target);

        // Center
        rc::PointcloudCenterEstimator center_estimator;

signals:
    // void customSignal(); ///< Señal personalizada (si se necesita)
};

#endif
