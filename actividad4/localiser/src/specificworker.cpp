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
#include <IceUtil/StringUtil.h>

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

	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		///////////// Your code ////////
		// Viewer
		viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
		auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_draw = r;
		//viewer->show();


		viewer_room = new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);
		auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_room_draw = rr;
		// draw room in viewer_room

		room_rect_item = viewer_room->scene.addRect(nominal_rooms[0].rect(), QPen(Qt::black, 30));
		//viewer_room->show();
		show();


		// initialise robot pose
		robot_pose.setIdentity();
		robot_pose.translate(Eigen::Vector2d(0.0,0.0));

		//show();
		//while (frame_plot_error == nullptr){}
		qInfo() << frame_plot_error;
		TimeSeriesPlotter::Config plotConfig;
		plotConfig.title = "Maximum Match Error Over Time";
		plotConfig.yAxisLabel = "Error (mm)";
		plotConfig.timeWindowSeconds = 15.0;
		plotConfig.autoScaleY = false;
		plotConfig.yMin = 0;
		plotConfig.yMax = 1000;

		time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, plotConfig);
		match_error_graph = time_series_plotter->addGraph("", Qt::blue);




		// stop robot
		move_robot(0, 0, 0);

	}


}

//Hemos detectado un problema en la inicialización del robot en las habitaciones que hace que el error en la segunda se dispare,  entre otras cosas. Hay que diferenciar entre la primera vez que se hace el match, cuando detecta el color, y las siguientes. La primera vez no se transforman las esquinas nominales al robot porque precisamente estamos asumiendo que el robot está bien colocado en el cuadrante y las coordenadas directas medidas van a encajar con las nominales. Una vez estimada la pose en esta situación, ésta se usa para las correcciones siguientes.

//El compute quedaría así:

void SpecificWorker::compute()
{
    RoboCompLidar3D::TPoints data = read_data();
    data = door_detector.filter_points(data);
    draw_local_doors(&viewer->scene);

    // compute corners
    const auto &[corners, lines] = room_detector.compute_corners(data, &viewer->scene);
    const auto center_opt = center_estimator.estimate(data);
    draw_lidar(data, center_opt, &viewer->scene);

    // update robot pose
    float max_match_error = -1;
    //Match match;
	if (localised)
	{
		auto r_poseFloat = robot_pose.cast<float>();
		if (const auto res = update_robot_pose(current_room, corners, r_poseFloat, true); res.has_value())
		{
			robot_pose = res.value().first.cast<double>();
			max_match_error = res.value().second;
			time_series_plotter->addDataPoint(match_error_graph,max_match_error);
		}
	}


    // Process state machine
	RetVal ret_val = process_state(data, corners, &viewer->scene, &viewer_room->scene);
	auto [st, adv, rot] = ret_val;
	state = st;

	// Send movements commands to the robot constrained by the match_error
	 if (not pushButton_stop->isChecked())
		move_robot(adv, rot, max_match_error);

	if (localised && change_rect)
		change_viewer_room();

	robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
	const double angle = qRadiansToDegrees(std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0)));
	robot_room_draw->setRotation(angle);
    // update GUI
    update_GUI(adv,rot,angle);
}


void SpecificWorker:: update_GUI(float adv, float rot, double angle)
{
	time_series_plotter->update();
	lcdNumber_adv->display(adv);
	lcdNumber_rot->display(rot);
	lcdNumber_x->display(robot_pose.translation().x());
	lcdNumber_y->display(robot_pose.translation().y());
	lcdNumber_room->display(current_room);
	lcdNumber_angle->display(angle);
	label_state->setText(to_string(state));
	last_time = std::chrono::high_resolution_clock::now();
}
void SpecificWorker:: draw_local_doors(QGraphicsScene* scene)
{
	// 1. Obtenemos la referencia a la habitación actual para mayor claridad
	for (auto *item : door_items)
	{
		if(item)
		{
			scene->removeItem(item);
			delete item;
		}
	}
	door_items.clear();
	const auto& doors = door_detector.doors();

	// 2. Iteramos sobre cada puerta en el vector de la habitación
	for (const auto& door : doors) {

		// --- DIBUJAR LA LÍNEA DE LA PUERTA (AZUL) ---
		QPen pen(QColor("blue"), 20);
		QLineF qdoor(door.p1.x(), door.p1.y(), door.p2.x(), door.p2.y());
		auto *line_item = scene->addLine(qdoor, pen);
		door_items.push_back(line_item);

		// --- DIBUJAR BOLAS MAGENTA EN EXTREMOS p1 y p2 ---
		const QColor color("darkMagenta");
		const int radius = 200; // diámetro 200

		// Punto p1
		auto *p1_item = scene->addEllipse(-radius/2.0, -radius/2.0, radius, radius,
										  QPen(color), QBrush(color));
		p1_item->setPos(door.p1.x(), door.p1.y());
		door_items.push_back(p1_item);

		// Punto p2
		auto *p2_item = scene->addEllipse(-radius/2.0, -radius/2.0, radius, radius,
										  QPen(color), QBrush(color));
		p2_item->setPos(door.p2.x(), door.p2.y());
		door_items.push_back(p2_item);
	}
}
void SpecificWorker:: draw_global_doors(QGraphicsScene* scene)
{
	// 1. Obtenemos la referencia a la habitación actual para mayor claridad
	for (auto *item : door_items_global)
	{
		if(item)
		{
			scene->removeItem(item);
			delete item;
		}
	}
	door_items_global.clear();
	const auto& doors = nominal_rooms[current_room].doors;

	// 2. Iteramos sobre cada puerta en el vector de la habitación
	for (const auto& door : doors) {

		// --- DIBUJAR LA LÍNEA DE LA PUERTA (AZUL) ---
		QPen pen(QColor("blue"), 20);
		QLineF qdoor(door.p1_global.x(), door.p1_global.y(), door.p2_global.x(), door.p2_global.y());
		auto *line_item = scene->addLine(qdoor, pen);
		door_items_global.push_back(line_item);

		// --- DIBUJAR BOLAS MAGENTA EN EXTREMOS p1 y p2 ---
		const QColor color("darkMagenta");
		const int radius = 200; // diámetro 200

		// Punto p1
		auto *p1_item = scene->addEllipse(-radius/2.0, -radius/2.0, radius, radius,
										  QPen(color), QBrush(color));
		p1_item->setPos(door.p1_global.x(), door.p1_global.y());
		door_items_global.push_back(p1_item);

		// Punto p2
		auto *p2_item = scene->addEllipse(-radius/2.0, -radius/2.0, radius, radius,
										  QPen(color), QBrush(color));
		p2_item->setPos(door.p2_global.x(), door.p2_global.y());
		door_items_global.push_back(p2_item);
	}
}

void  SpecificWorker::move_robot(float adv, float rot, float max_match_error)
{
 omnirobot_proxy->setSpeedBase(0,adv,rot);
}

std::optional<std::pair<Eigen::Affine2f, float>> SpecificWorker::update_robot_pose(int room_index,
																				  const Corners &corners,
																				  const Eigen::Affine2f &r_pose,
																				  bool transform_corners)
{
	// match corners  transforming first nominal corners to robot's frame
	Match match;
	if (transform_corners)
		match = hungarian.match(corners, nominal_rooms[room_index].transform_corners_to(r_pose.inverse().cast<float>()));
	else
		match = hungarian.match(corners, nominal_rooms[room_index].corners());


	if (match.empty() or match.size() < 3)
		return {};


	const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
	  { return std::get<2>(a) < std::get<2>(b); });


	const auto max_match_error = std::get<2>(*max_error_iter);


	// create matrices W and b for pose estimation
	auto r = solve_pose(corners, match);
	if (r.array().isNaN().any())
	{
		qWarning() << __FUNCTION__ << "NaN values in r ";
		return {};
	}


	auto r_pose_copy = r_pose;
	r_pose_copy.translate(Eigen::Vector2f(r(0), r(1)));
	r_pose_copy.rotate(r[2]);
	return {{r_pose_copy, max_match_error}};
}


Eigen::Vector3d SpecificWorker::solve_pose(const Corners &corners, const Match &match)
{
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
	return r;
}

void SpecificWorker::predict_robot_pose()
{

}



void SpecificWorker::change_viewer_room()
{
	if (nominal_rooms.empty() || !viewer_room || current_room < 0)
		return;
	if (room_rect_item != nullptr)
	{
		// Verificamos que el item todavía pertenece a la escena para evitar el error de "different scene"
		if (room_rect_item->scene() == &viewer_room->scene)
		{
			viewer_room->scene.removeItem(room_rect_item);
		}
		delete room_rect_item;
		room_rect_item = nullptr;
	}
	// draw room in viewer_room
	room_rect_item = viewer_room->scene.addRect(nominal_rooms[current_room].rect(), QPen(Qt::black, 30));
	change_rect = false;
}
std::tuple<float, float> SpecificWorker::robot_controller(const Eigen::Vector2f &target)
{
	constexpr float Kp = 0.3f;
	constexpr float vmax = 1000; // mm/s
	constexpr float sigma = M_PI / 6;

	// error angle
	double theta_e = std::atan2(target.x(), target.y());
	const float vrot = Kp * theta_e;
	// Gaussian angle brake
	const float angle_brake = std::exp(-(theta_e * theta_e) / (2 * sigma * sigma));
	// adv vel
	const float adv = vmax * angle_brake;
	// check exit condition
	if (target.norm() < 300.f)
		return {0.f, 0.f};

	return {adv, vrot};
}

RoboCompLidar3D::TPoints SpecificWorker::read_data()
{
	RoboCompLidar3D::TData data;
	RoboCompLidar3D::TPoints filter_data;
	try
	{
		data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 15000, 3);
		if(data.points.empty()) return RoboCompLidar3D::TPoints{};

		//filter_data = filter_min_distance_cppitertools(data.points);
		// Nuevo: filtrar puntos aislados
		filter_data = filter_isolated_points(data.points, 200.0f);
		//draw_lidar(filter_data.value(),center_opt, &viewer->scene);
		return filter_data;
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading Lidar: " << e << std::endl;
		return RoboCompLidar3D::TPoints{};
	}

}

SpecificWorker::RetVal SpecificWorker::goto_door(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
	if (!scene)
		return {STATE::GOTO_DOOR, 0.f, 0.f};

	if (current_room < 0 || current_room >= static_cast<int>(nominal_rooms.size()))
		return {STATE::GOTO_DOOR, 0.f, 0.f};
	//qInfo() << "Room: " << current_room << ", door: " << current_room << " de " << static_cast<int>(nominal_rooms[current_room].doors.size());
   Doors doors;
   // Exit conditions
   if ( doors = door_detector.doors(); doors.empty())
   {
       qInfo() << __FUNCTION__ << "No doors detected, switching to UPDATE_POSE";
       return {STATE::GOTO_DOOR, 0.f, 0.f};  // TODO: keep moving for a while?
   }
   // select from doors, the one closest to the nominal door
   Door target_door;
   if (localised)
   {
      //qInfo() << __FUNCTION__ << "Localised, selecting door closest to nominal door";
       const auto dn = nominal_rooms[current_room].doors[current_door];
       const auto sd = std::ranges::min_element(doors, [dn, this](const auto &a, const auto &b)
              {  return (a.center() - robot_pose.inverse().cast<float>() * dn.center_global()).norm() <
                        (b.center() - robot_pose.inverse().cast<float>() * dn.center_global()).norm(); });
       target_door = *sd;
   }
   else  // select the one closest to the robot's heading direction
   {
       //qInfo() << __FUNCTION__ << "Not localised, selecting door closest to robot heading";
       const auto sd = std::ranges::min_element(doors, [](const auto &a, const auto &b)
              {  return abs(a.p1_angle) < abs(b.p1_angle); });
       target_door = *sd;
   }

   // distance to target is less than threshold, stop and switch to ORIENT_TO_DOOR
   const auto target = target_door.center_before(robot_pose.translation().cast<float>(), params.RELOCAL_MIN_DISTANCE_TO_DOOR);
   const auto dist_to_door = target.norm();


   // draw target
   static QGraphicsItem *door_target_draw = nullptr;
   if (door_target_draw != nullptr)
       scene->removeItem(door_target_draw);
   door_target_draw = scene->addEllipse(-50, -50, 100, 100, QPen(Qt::magenta), QBrush(Qt::magenta));
   door_target_draw->setPos(target.x(), target.y());
	draw_global_doors(scene);
  // Exit condition
	//qInfo() << "distancia a la puerta" << stop_target.norm();
   if (dist_to_door < params.DOOR_REACHED_DIST)
   {
       //qInfo() << __FUNCTION__ << "Door reached at distance " << dist_to_door << ", switching to ORIENT_TO_DOOR";
       return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};
   }


   //qInfo() << __FUNCTION__ << "moving to door at " << target.x() << "," << target.y() << " dist: " << dist_to_door;
   const auto &[adv, rot] = robot_controller(target); // go to first detected door
	//qInfo() << "Voy hacia la puerta " << current_door << " de la habitacion " << current_room << " que esta " << nominal_rooms[current_room].doors[current_door].visited;
   return {STATE::GOTO_DOOR, adv, rot};
}



SpecificWorker::RetVal SpecificWorker::orient_to_door(const RoboCompLidar3D::TPoints &points)
{
	// data
	const auto doors = door_detector.doors();
	if (localised)
	{
		const auto dn = nominal_rooms[current_room].doors[current_door];
		const auto sd = std::ranges::min_element(doors, [dn, this](const auto &a, const auto &b)
			{  return (a.center() - robot_pose.inverse().cast<float>() * dn.center_global()).norm() <
					  (b.center() - robot_pose.inverse().cast<float>() * dn.center_global()).norm(); });
		//qInfo() << __FUNCTION__ << "Localised, selecting door closest to nominal door" << sd->center_angle() << params.RELOCAL_MAX_ORIENTED_ERROR << doors.size();
		if ( abs(sd->center_angle()) < params.RELOCAL_MAX_ORIENTED_ERROR)
			return {STATE::CROSS_DOOR, 0.1, 0.f};
		else
			return {STATE::ORIENT_TO_DOOR, 0.f, std::get<1>(robot_controller(sd->center()))};
	}
	else  // select the one closest to the robot's heading direction
	{
		qInfo() << __FUNCTION__ << "Not localised, selecting door closest to robot heading";
		const auto sd = std::ranges::min_element(doors, [](const auto &a, const auto &b)
			   {  return std::fabs(a.center_angle()) < std::fabs(b.center_angle());} );
		if (abs(sd->center_angle()) < params.RELOCAL_MAX_ORIENTED_ERROR)
			return {STATE::CROSS_DOOR, 0.5f, 0.f};
		else
			return {STATE::ORIENT_TO_DOOR, 0.f, std::get<1>(robot_controller(sd->center()))};
	}
}



SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points)
{
	static bool first_time = true;
   static std::chrono::time_point<std::chrono::system_clock> start;

   // Exit condition: the robot has advanced 1000 or equivalently 2 seconds at 500 mm/s
   if (first_time)
   {
       first_time = false;
       start = std::chrono::high_resolution_clock::now();
       return {STATE::CROSS_DOOR, 500.0f, 0.0f};
   }
   else
   {
       const auto elapsed = std::chrono::high_resolution_clock::now() - start;
       //qInfo() << __FUNCTION__ << "Elapsed time crossing door: "
       //         << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms";
       if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > 4000)
       {
           first_time = true;
           const auto &leaving_door = nominal_rooms[current_room].doors[current_door];
           int next_room_idx = leaving_door.connects_to_room;
           // if entering known room, relocalise the robot
           if (next_room_idx >= 0 and nominal_rooms[next_room_idx].visited)
           {
               //qInfo() << __FUNCTION__ << "Entering known room " << next_room_idx << ". Skipping LOCALISE.";


               // Update indices to the new room
               int next_door_idx = leaving_door.connects_to_door;
               current_room = next_room_idx;
               current_door = next_door_idx;


               // Compute robot pose based on the door in the new room frame.
               const auto &entering_door = nominal_rooms[current_room].doors[current_door]; // door we are entering now
               Eigen::Vector2f door_center = entering_door.center_global(); //
               // Vector from door to origin (0,0) is -door_center
               const float angle = std::atan2(-door_center.x(), -door_center.y());


               // robot_pose now must be translated so it is drawn in the new room correctly
               robot_pose.setIdentity();
               door_center.y() -= 500; // place robot 500 mm inside the room
               robot_pose.translate(door_center.cast<double>());
               robot_pose.rotate(0);
               //qInfo() << __FUNCTION__ << "Robot localised in NEW room " << current_room << " at door " << current_door;
               std::cout << door_center.x() << " " << door_center.y() << " " << angle << std::endl;




               localised = true;
           		change_rect = true;
               // Continue navigation in the new room

               return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
           }
           else // Unknown room. I need to store the door index of the current door and start tracking the just crossed door,
           {
               door_crossing = DoorCrossing{current_room, current_door};
               nominal_rooms[current_room].doors[current_door].visited = true; // exiting door
               // from here it must be updated until localisation is achieved again

               return {STATE::LOCALISE, 0.f, 0.f};
           }
       }
       else // keep crossing
           return {STATE::CROSS_DOOR, 500.f, 0.f};
   }
}


SpecificWorker::RetVal SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints &points)
{

	const auto center_opt = center_estimator.estimate(points);
    if (not center_opt)
    { qWarning() << __FUNCTION__ << "No room center"; return RetVal{STATE::GOTO_ROOM_CENTER, 0.f, 0.f};}
	// obtain target
	const Eigen::Vector2f target_f = center_opt.value().cast<float>();
	// exit condition
	if (target_f.norm() < 300.f && localised)
	 {
		current_door=choose_next_door(current_room);
	 	return RetVal{ STATE::GOTO_DOOR, 0.f, 0.f };
	 }
	if (target_f.norm() < 300.f && !localised)
		return RetVal{ STATE::TURN, 0.0f, 0.0f };

	// do my thing
	auto [v, w] = robot_controller(target_f);
	door_crossing.track_entering_door(door_detector.doors());
    return RetVal{STATE::GOTO_ROOM_CENTER, v, w};
}

SpecificWorker::RetVal SpecificWorker::update_pose(const Corners &corners, const Match &match)
{
	// Código de la función
	return RetVal{ STATE::IDLE, 0.0f, 0.0f };

}

int SpecificWorker:: choose_next_door(int currentRoom)
{
	static std::mt19937 gen(rd());
	auto &doors = nominal_rooms[currentRoom].doors;
	std::uniform_int_distribution<> dis(0, doors.size()-1);
	if (first_time == 1)
	{
		first_time = 0;
		const int num_rand = dis(gen);
		qInfo() << "Se ha generado el nuemro " << num_rand;
		return num_rand;

	}
	for (std::size_t i = 0; i < doors.size(); ++i)
	{

		if (doors[i].visited) {return static_cast<int>(i);}
	}
	return 1 - current_door;   // no quedan puertas en esta habitación
}

SpecificWorker::RetVal SpecificWorker::turn(const Corners &corners){
//////////////////////////////////////////////////////////////////
   // check for colour patch in image
   /////////////////////////////////////////////////////////////////
	const auto &[success, room_index, left_right] = image_processor.check_colour_patch_in_image(camera360rgb_proxy, this->label_img);
	//const auto &[success, room_index, left_right] = image_processor.check_for_number(mnist_proxy);

   if (success)
   {
       current_room = room_index;
   		auto r_poseFloat = robot_pose.cast<float>();
       // update robot pose to have a fresh value
       if (const auto res = update_robot_pose(current_room, corners, r_poseFloat, false); res.has_value())
           robot_pose = res.value().first.cast<double>();
       else return{STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED/2};


       ///////////////////////////////////////////////////////////////////////
		/// save doors to nominal_room if not previously visited
       ///////////////////////////////////////////////////////////////////////////
if (not nominal_rooms[current_room].visited)
{
           nominal_rooms[current_room].name = image_processor.room_name_from_index(current_room);
           auto doors = door_detector.doors();
           if (doors.empty()) { qWarning() << __FUNCTION__ << "empty doors"; return{STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};}
           for (auto &d : doors)
           {
               d.p1_global = nominal_rooms[current_room].get_projection_of_point_on_closest_wall(robot_pose.cast<float>() * d.p1);
               d.p2_global = nominal_rooms[current_room].get_projection_of_point_on_closest_wall(robot_pose.cast<float>() * d.p2);
           }
           nominal_rooms[current_room].doors = doors;
           // choose door to go
           current_door = choose_next_door(current_room);
           // we need to match the current selected nominal door to the successive local doors detected during the approach
           // select the local door closest to the selected nominal door
           const auto dn = nominal_rooms[current_room].doors[current_door];
           const auto ds = door_detector.doors();
           const auto sd = std::ranges::min_element(ds, [dn, this](const auto &a, const auto &b)
                   {  return (a.center() - robot_pose.inverse().cast<float>() * dn.center_global()).norm() <
                             (b.center() - robot_pose.inverse().cast<float>() * dn.center_global()).norm(); });
           // sd is the closest local door to the selected nominal door. Update nominal door with local values
           nominal_rooms[current_room].doors[current_door].p1 = sd->p1;
           nominal_rooms[current_room].doors[current_door].p2 = sd->p2;
           nominal_rooms[current_room].visited = true;
       }
       // ///////////////////////////////////////////////////////////////////////////
// // finish door tracking and update door crossing info
       // ///////////////////////////////////////////////////////////////////////////
if (door_crossing.valid)
{
           door_crossing.set_entering_data(current_room, nominal_rooms);
           nominal_rooms[door_crossing.leaving_room_index].doors[door_crossing.leaving_door_index].connects_to_door = door_crossing.entering_door_index;
           nominal_rooms[door_crossing.leaving_room_index].doors[door_crossing.leaving_door_index].connects_to_room = door_crossing.entering_room_index;
           nominal_rooms[current_room].doors[door_crossing.entering_door_index].visited = true;
           nominal_rooms[current_room].doors[door_crossing.entering_door_index].connects_to_door = door_crossing.leaving_door_index;
           nominal_rooms[current_room].doors[door_crossing.entering_door_index].connects_to_room = door_crossing.leaving_room_index;
           door_crossing.valid = false;
       }
       localised = true;
   		change_rect = true;
   		//current_door = choose_next_door(current_room);
       return {STATE::GOTO_DOOR, 0.0f, 0.0f};  // SUCCESS
   }
   // continue turning
   return {STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};
}


SpecificWorker::RetVal SpecificWorker::localise(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
	// initialise robot pose at origin. Necessary to reser pose accumulation
	robot_pose.setIdentity();
	robot_pose.translate(Eigen::Vector2d(0.0,0.0));
	localised = false;


	// if error high but not at room centre, go to centering step
	// compute mean of LiDAR points as room center estimate


	if(const auto center = center_estimator.estimate(points); center.has_value())
	{
		if (center.value().norm() > params.RELOCAL_CENTER_EPS )
			return{STATE::GOTO_ROOM_CENTER, 0.0f, 0.0f};


		// If close enough to center -> stop and move to TURN
		if (center.value().norm() < params.RELOCAL_CENTER_EPS )
			return {STATE::TURN, 0.0f, 0.0f};
	}
	qWarning() << __FUNCTION__ << "Not able to estimate room center from walls, continue localising.";
	return {STATE::LOCALISE, 0.0f, 0.0f};
}


SpecificWorker::RetVal SpecificWorker::process_state(
		const RoboCompLidar3D::TPoints &data,
		const Corners &corners,
		QGraphicsScene *scene,
		QGraphicsScene *scene_room)
{
	switch(state)
	{
	case STATE::GOTO_DOOR:
		{
			//qInfo() << "Estoy en el go to door" ;
			return goto_door(data,scene_room);
		}

	case STATE::ORIENT_TO_DOOR:
		{
			//qInfo() << "Estoy en el orient to door" ;
			return orient_to_door(data);
		}

	case STATE::CROSS_DOOR:
		{
			//qInfo() << "Estoy en el cross door" ;
			return cross_door(data);
		}

	case STATE::GOTO_ROOM_CENTER:
		{
			//qInfo() << "Estoy en el go to room center" ;
			return goto_room_center(data);
		}

	case STATE::TURN:
		{
			//qInfo() << "Estoy en el turn" ;
			return turn(corners);
		}
	case STATE::LOCALISE:
	default:
		{
			//qInfo() << "Estoy en el localise" ;
			return localise(data,scene);
		}


	}
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

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, std::optional<Eigen::Vector2d> center, QGraphicsScene* scene)
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
	// --- Dibujar el centro si existe ---
	if (center.has_value())
	{
		const QColor red(Qt::red);
		const QPen center_pen(red, 12);
		const QBrush center_brush(red, Qt::SolidPattern);

		// Punto pequeño (círculo) de radio 25
		auto cp = scene->addEllipse(-25, -25, 50, 50, center_pen, center_brush);
		cp->setPos(center->x(), center->y());
		draw_points.push_back(cp);
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

