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
#include "specificworker.h"
#include <cppitertools/itertools.hpp>
#include <cppitertools/enumerate.hpp>

#ifdef emit
#undef emit
#endif

#include <execution>
#include <numeric> // para std::iota
#include <algorithm>


//std::chrono::steady_clock::time_point start_time;



SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif
		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

    //initializeCODE

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name")
	srand(static_cast<unsigned int>(time(nullptr)));  // Semilla del generador aleatorio
	// Inicializar generador aleatorio moderno
	rng = std::mt19937(std::random_device{}());
	dist = std::uniform_int_distribution<int>(0, 9);
	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	//viewer2 = new AbstractGraphicViewer(this->frame_room, this->dimensions);
	this->resize(900,450);
	viewer->show();
	//viewer2->show();
	const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);
	//const auto rob2 = viewer2->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	//robot_polygon2 = std::get<0>(rob);

	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
	//connect(viewer2, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
	viewer_room = new AbstractGraphicViewer(this->frame_room, this->dimensions);
	auto [rr, re] = viewer_room->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 100, QColor("Blue"));
	room_draw_robot = rr;

	// draw room in viewer_room
	viewer_room->scene.addRect(this->dimensions, QPen(Qt::black, 30));
	viewer_room->show();

	// initialise robot pose
	robot_pose.setIdentity();
	robot_pose.translate(Eigen::Vector2d(0.0,0.0));

}



void SpecificWorker::compute()
{
    std::optional<RoboCompLidar3D::TPoints> filter_data;

    // -----------------------
    // Leer LIDAR y filtrar
    // -----------------------
    try
    {
        auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 5000, 1);
        if(data.points.empty()) return;

        filter_data = filter_min_distance_cppitertools(data.points);
    	// Nuevo: filtrar puntos aislados
    	filter_data = filter_isolated_points(filter_data.value(), 200.0f);
        draw_lidar(filter_data.value(), &viewer->scene);
    }
    catch(const Ice::Exception &e)
    {
        std::cout << "Error reading Lidar: " << e << std::endl;
        return;
    }

    // -----------------------
    // Localización usando esquinas
    // -----------------------
    if (!filter_data.has_value() || filter_data->empty())
        return;

    // 1 Extraer esquinas del LIDAR
    auto measured_corners = room_detector.compute_corners(filter_data.value());

    // 2 Transformar esquinas nominales al marco del robot
    auto nominal_corners_robot = room.transform_corners_to(robot_pose.inverse());

    // 3 Emparejar esquinas usando Hungarian
    auto match = hungarian.match(measured_corners, nominal_corners_robot);

    if (match.empty())
        return;

    // 4 Calcular corrección de pose con pseudoinversa
    Eigen::MatrixXd W(match.size() * 2, 3);
    Eigen::VectorXd b(match.size() * 2);

    for (size_t i = 0; i < match.size(); ++i)
    {
        const auto& [meas_c, nom_c, _] = match[i];
        const auto& [p_meas, __, ___] = meas_c;
        const auto& [p_nom, ____, _____] = nom_c;

        b(2*i)     = p_nom.x() - p_meas.x();
        b(2*i+1)   = p_nom.y() - p_meas.y();

        W.block<1,3>(2*i,0)   << 1.0, 0.0, -p_meas.y();
        W.block<1,3>(2*i+1,0) << 0.0, 1.0,  p_meas.x();
    }

    Eigen::Vector3d r = (W.transpose() * W).inverse() * W.transpose() * b;

    if (r.array().isNaN().any())
        return;

    // 5 Actualizar pose del robot
    robot_pose.translate(Eigen::Vector2d(r(0), r(1)));
    robot_pose.rotate(r(2));

    // 6 Actualizar GUI
    room_draw_robot->setPos(robot_pose.translation().x(), robot_pose.translation().y());
    double angle = std::atan2(robot_pose.rotation()(1,0), robot_pose.rotation()(0,0));
    room_draw_robot->setRotation(qRadiansToDegrees(angle));
}




float SpecificWorker::calcularDistanciaMinima(const std::optional<RoboCompLidar3D::TPoints>& lidar_data, bool bandera)
{
	// Comprobamos si hay datos válidos
	if (!lidar_data.has_value() || lidar_data->empty())
	{
		qWarning() << "[calcularDistanciaMinima] No hay datos de LIDAR disponibles";
		return 1e6f;  // Valor grande por defecto
	}

	float distancia_minima = 1e6f;

	for (const auto& p : lidar_data.value())
	{   //Si bandera = true si hace modo 360, si no, restringido
		if (bandera || (p.phi >= FRONT_RIGHT_ANGLE && p.phi <= FRONT_LEFT_ANGLE)) {
			float dist = std::hypot(p.x, p.y);
			if (dist < distancia_minima)
				distancia_minima = dist;
		}
	}


	qInfo() <<"Distancia minima: " << distancia_minima;
	return distancia_minima;
}





static constexpr int iter_seguras = 2;
static int contador = 0;
static float vector_media[iter_seguras];

std::tuple<SpecificWorker::RobotMode, float, float>
SpecificWorker::FORWARD_method(const std::optional<RoboCompLidar3D::TPoints>& filter_data)
{
	float advance_speed = 1000.0f;  // mm/s

	// Guardamos la distancia mínima en el vector circular
	vector_media[contador] = calcularDistanciaMinima(filter_data, 0);
	contador = (contador + 1) % iter_seguras;  // avanza el índice y vuelve a 0

	// Si no hemos llenado suficientes iteraciones, seguimos FORWARD
	if (contador != 0)
		return {RobotMode::FORWARD, advance_speed, 0.0f};

	// Calculamos la media de las últimas 'iter_seguras' lecturas
	float suma = 0.0f;
	for (int i = 0; i < iter_seguras; ++i)
		suma += vector_media[i];

	float min_dist = suma / iter_seguras;

	if (min_dist < safe_distance)
	{
		advance_speed = 0.0f;
		return {RobotMode::TURN, advance_speed, 0.0f};
	}

	int probalidad = dist(rng);

	if (probalidad > 8  && calcularDistanciaMinima(filter_data, 1) >= ROBOT_LENGTH+1000) {
		qInfo() << "MOOOODO ESPIRAL";
		return {RobotMode::SPIRAL, 0.0f, 0.0f};
	}

	return {RobotMode::FORWARD, advance_speed, 0.0f};
}


#include <Eigen/Dense> // para Eigen::Vector2f
#include <numeric>     // para std::accumulate
#include <cmath>       // para std::atan2

std::tuple<SpecificWorker::RobotMode, float, float>
SpecificWorker::FOLLOW_WALL_method(const std::optional<RoboCompLidar3D::TPoints>& filter_data)
{
    if (!filter_data.has_value() || filter_data->empty())
        return {RobotMode::FOLLOW_WALL, 0.0f, 0.0f};

    // --- Parámetros ---
    const float REFERENCE_DISTANCE = 520.0f;  // mm, distancia ideal a la pared
    const float DELTA = 100.0f;               // margen aceptable de variación
    const float SAFE_DISTANCE = 400.0f;       // mm, distancia mínima frontal antes de girar
    const float MAX_ADV_SPEED = 900.0f;       // mm/s
    const float ROTATION_GAIN = 0.002f;       // ganancia de rotación proporcional

    float advance_speed = MAX_ADV_SPEED;
    float rot_speed = 0.0f;

    // --- Paso 1: detectar distancia frontal mínima ---
    float front_min = calcularDistanciaMinima(filter_data, false);  // usa la versión frontal
    if (front_min < SAFE_DISTANCE)
    {
        qInfo() << "FOLLOW_WALL -> obstáculo frontal detectado, cambiando a TURN";
        return {RobotMode::TURN, 0.0f, 0.5f};
    }

    // --- Paso 2: calcular distancia lateral a la pared (por la derecha, por ejemplo) ---
    // Vamos a tomar un promedio de los puntos que estén hacia la derecha del robot
    float suma = 0.0f;
    int cont = 0;

    for (const auto& p : filter_data.value())
    {
        // Ángulo negativo: derecha del robot
        if (p.phi < -M_PI_4 && p.phi > -M_PI_2)  // entre -45° y -90°
        {
            float dist = std::hypot(p.x, p.y);
            suma += dist;
            cont++;
        }
    }


    float lateral_dist = suma / cont;

    // --- Paso 3: control de rotación proporcional según la desviación lateral ---
    float error = REFERENCE_DISTANCE - lateral_dist;
    rot_speed = ROTATION_GAIN * error;

    // Limitar la rotación para evitar giros bruscos
    rot_speed = std::clamp(rot_speed, -0.5f, 0.5f);

    // --- Paso 4: control de avance (reducir si hay que girar mucho) ---
    float brake = exp(-3 * std::abs(rot_speed));  // tipo función gaussiana
    advance_speed *= brake;


    // --- Resultado ---
    qInfo() << "FOLLOW_WALL -> lateral:" << lateral_dist
            << " rot:" << rot_speed << " adv:" << advance_speed;

    return {RobotMode::FOLLOW_WALL, advance_speed, rot_speed};
}








// Variable estática que conserva su valor entre llamadas
// Simula el paso del tiempo para que la espiral se expanda progresivamente
static float t = 0;

std::tuple<SpecificWorker::RobotMode, float, float>
SpecificWorker::SPIRAL_method(const std::optional<RoboCompLidar3D::TPoints>& filter_data)
{
	// --- Parámetros base de la espiral ---
	float const v0 = 800.0f;   // mm/s -> arranque más potente
	float const w0 = 1.2f;     // rad/s -> giro inicial más cerrado (más espiral)
	float const kv = 80.0f;    // incremento más progresivo de velocidad lineal
	float const kw = 0.05f;    // decremento más suave de velocidad angular
	float const r  = 1000.0f; // Radio de seguridad (mm). Si algo está más cerca, se cambia de modo

	// Calculamos la distancia mínima medida por el LIDAR (360º en este caso)
	float min_dist = calcularDistanciaMinima(filter_data, 1);

	float vt; // Velocidad lineal resultante
	float wt; // Velocidad angular resultante

	// --- Condición principal del modo espiral ---
	// Mientras no haya obstáculos cerca (min_dist > r), el robot sigue expandiendo su espiral
	if (min_dist > safe_distance)
	{
		// Aumentamos progresivamente la velocidad lineal y reducimos la angular
		// Clamp limita los valores para evitar saturaciones o comportamientos bruscos
		vt = std::clamp(v0 + kv * t, 0.0f, 4000.0f);  // límite alto algo más bajo: evita saltos
		wt = std::clamp(w0 - kw * t, 0.0f, 4.0f);     // tope coherente con w0


		// Incrementamos el tiempo para la siguiente iteración
		t += 0.05f;

		// Continuamos en el modo espiral con las nuevas velocidades
		return {RobotMode::SPIRAL, vt, wt};
	}

	// --- Si se detecta un obstáculo cercano ---
	// Reiniciamos el tiempo para la próxima espiral
	t = 0;

	// Cambiamos a modo FORWARD para seguir con el comportamiento general
	// Se pueden ajustar las velocidades iniciales para suavizar la transición
	return {RobotMode::TURN, 200.0f, 0.3f};
}





std::tuple<SpecificWorker::RobotMode, float, float>
SpecificWorker::TURN_method(const std::optional<RoboCompLidar3D::TPoints>& filter_data)
{
	float rotation_speed = 1.0f;;   // gira en el sitio
	float advance_speed = 0.0f;    // no avanza
	float min_dist = calcularDistanciaMinima(filter_data, 0);

	// Mientras haya obstáculos cerca, sigue girando
	if (min_dist < safe_distance)
		return {RobotMode::TURN, advance_speed, rotation_speed};


	int probalidad = dist(rng);
	// Si ya hay espacio libre, decide siguiente modo
	if ( probalidad > 4) {
		qInfo() << "Foward";
		return {RobotMode::FORWARD, advance_speed, 0};
	}
	else {
		qInfo() << "Seguir pared";
		return {RobotMode::FOLLOW_WALL, advance_speed, 0};
	}
}


std:: optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points)
{
	if (points.empty())
		return {};

	RoboCompLidar3D::TPoints result; result.reserve(points.size());
	for (auto&& [angle, group ]: iter::groupby(points, [](const auto& p)
		{float multiplier = std::pow(10.0f, 2); return std::floor(p.phi*multiplier)/multiplier;}))
	{
		auto min_it = std:: min_element(std::begin(group), std::end(group),
			[](const auto&a, const auto&b){return a.r < b.r;});
		result.emplace_back(RoboCompLidar3D::TPoint{.x=min_it->x, .y=min_it->y, .phi=min_it->phi});
	}
	return result;
}

// Devuelve la distancia promedio de los puntos LIDAR alrededor de un ángulo dado (en radianes)
float SpecificWorker::distanciaAnguloPromedio(const std::optional<RoboCompLidar3D::TPoints>& lidar_data, float ang_rad)
{
	if (!lidar_data.has_value() || lidar_data->empty())
		return 1e6f;  // No hay datos disponibles

	float distanciaAcum = 0.0f;
	float margen_rad = 0.1f;
	int cont = 0;

	for (const auto& p : lidar_data.value())
	{
		// Verificar si el punto está dentro del rango angular deseado
		if (p.phi >= ang_rad - margen_rad && p.phi <= ang_rad + margen_rad)
		{
			float dist = std::hypot(p.x, p.y);
			// Filtrar distancias inválidas o fuera de rango
			if (dist > 0.1f && dist < 10.0f)  // Ajustar según tu sensor
			{
				distanciaAcum += dist;
				cont++;
			}
		}
	}

	if (cont == 0)
		return 1e6f;  // No se encontraron puntos válidos

	return distanciaAcum / static_cast<float>(cont);
}

float SpecificWorker::calcularDistanciaMinima(const std::optional<RoboCompLidar3D::TPoints> &lidar_data,
	float rangoA, float rangoB) {

	// Comprobamos si hay datos válidos
	if (!lidar_data.has_value() || lidar_data->empty())
	{
		qWarning() << "[calcularDistanciaMinima] No hay datos de LIDAR disponibles";
		return 1e6f;  // Valor grande por defecto
	}

	float distancia_minima = 1e6f;

	for (const auto& p : lidar_data.value())
	{   //Si bandera = true si hace modo 360, si no, restringido
		if ((p.phi >= rangoA && p.phi <= rangoB)) {
			float dist = std::hypot(p.x, p.y);
			if (dist < distancia_minima)
				distancia_minima = dist;
		}
	}


	qInfo() <<"Distancia minima: " << distancia_minima;
	return distancia_minima;




}

RoboCompLidar3D::TPoints SpecificWorker::filter_isolated_points(
	const RoboCompLidar3D::TPoints &points, float d) // distancia de 200 mm
{
	if(points.empty()) return {};

	const float d_squared = d * d;
	std::vector<bool> hasNeighbor(points.size(), false);

	std::vector<size_t> indices(points.size());
	std::iota(indices.begin(), indices.end(), size_t{0});

	std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
	{
		const auto& p1 = points[i];
		for(auto &&[j,p2] : iter::enumerate(points))
		{
			if(i == j) continue;
			const float dx = p1.x - p2.x;
			const float dy = p1.y - p2.y;
			if(dx*dx + dy*dy <= d_squared)
			{
				hasNeighbor[i] = true;
				break;
			}
		}
	});

	RoboCompLidar3D::TPoints result;
	result.reserve(points.size());
	for(auto &&[i, p] : iter::enumerate(points))
		if(hasNeighbor[i])
			result.push_back(points[i]);

	return result;
}


void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}



//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

void SpecificWorker::draw_lidar(const auto &points, QGraphicsScene* scene)
{
	static std::vector<QGraphicsItem*> draw_points;
	for (const auto &p : draw_points)
	{
		scene->removeItem(p);
		delete p;
	}
	draw_points.clear();

	const QColor color("LightGreen");
	const QPen pen(color, 10);
	//const QBrush brush(color, Qt::SolidPattern);
	for (const auto &p : points)
	{
		const auto dp = scene->addRect(-25, -25, 50, 50, pen);
		dp->setPos(p.x, p.y);
		draw_points.push_back(dp);   // add to the list of points to be deleted next time
	}
}



void SpecificWorker::update_ropot_position()
{
	try
	{
		RoboCompGenericBase:: TBaseState bState;
		omnirobot_proxy->getBaseState(bState);
		robot_polygon->setRotation(bState.alpha*180/M_PI);
		robot_polygon->setPos(bState.x, bState.z);
		std::cout << bState.alpha <<" " << bState.x << " " << bState.z << std::endl;
	}
	catch(const Ice::Exception &e){std::cout << e.what() << std::endl;}
}

void SpecificWorker::new_target_slot(QPointF p)
{
	qInfo() << "World coordinates" << p;
}

void SpecificWorker:: chocachoca(std::optional<RoboCompLidar3D::TPoints> filter_data,float &advance_speed,float &rotation_speed )
{
	const float FRONT_LEFT_ANGLE  = M_PI / 2;
	const float FRONT_RIGHT_ANGLE = -M_PI / 2;

	float min_dist = 1e6;

	for (const auto& p : filter_data.value())
	{
		if (p.phi >= FRONT_RIGHT_ANGLE && p.phi <= FRONT_LEFT_ANGLE)
		{
			float dist =std::hypot(p.x,p.y);
			if (dist < min_dist) min_dist = dist;
		}
	}

	float safe_distance = 450; // mm
	advance_speed = 600;  // mm/s
	rotation_speed = 0;   // rad/s

	if (min_dist < safe_distance)
	{
		advance_speed = 0;
		rotation_speed = 1;
	}
}

// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

