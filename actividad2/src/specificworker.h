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

#include <genericworker.h>
#include <tuple>
#include <vector>
#include <optional>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>


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
    void draw_lidar(const auto &points, QGraphicsScene* scene); ///< Dibuja los puntos del LIDAR en la escena
    std::optional<RoboCompLidar3D::TPoints> filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points); ///< Filtra la nube de puntos
    float calcularDistanciaMinima(const std::optional<RoboCompLidar3D::TPoints>& lidar_data, bool bandera);///< Calcula la dostancia mínima entre FRONT_LEFT_ANGLE y FRONT_RIGHT_ANGLE, si bandera  traue se evalua para 360
    float distanciaAnguloPromedio (const std::optional<RoboCompLidar3D::TPoints>& lidar_data, float ang);
    float calcularDistanciaMinima(const std::optional<RoboCompLidar3D::TPoints>& lidar_data, float rangoA, float rangoB);
signals:
    // void customSignal(); ///< Señal personalizada (si se necesita)
};

#endif
