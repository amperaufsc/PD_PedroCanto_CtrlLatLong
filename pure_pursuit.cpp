/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2018:
     - Sonja Brits  <britss@ethz.ch>
     - Juraj Kabzan <kabzanj@gmail.com>
     
    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "pure_pursuit.hpp"
#include <sstream>

// ROS Msgs
#include "visualization_msgs/MarkerArray.h"

namespace ns_pure_pursuit {
// Constructor
PurePursuit::PurePursuit(ros::NodeHandle &nh) : nh_(nh) {
    pub_closest_point_ = nh.advertise<visualization_msgs::MarkerArray>("/control/pure_pursuit/marker", 1);

    if (!nh.param<double>("controller/speed/p", speed_p, 0.01)) {
        ROS_WARN_STREAM("Did not load controller/speed/p. Standard value is: " << 0.01);
    }
    if (!nh.param<double>("controller/steering/p", steering_p, 0.01)) {
        ROS_WARN_STREAM("Did not load controller/steering/p. Standard value is: " << 0.01);
    }
};

// Getters
fsd_common_msgs::ControlCommand PurePursuit::getControlCommand() const { return control_command_; }

// Setters
void PurePursuit::setMaxSpeed(double &max_speed) {
    max_speed_ = max_speed;
}

void PurePursuit::setCenterLine(const geometry_msgs::Polygon &center_line) {
    center_line_ = center_line;
}

void PurePursuit::setState(const fsd_common_msgs::CarState &state) {
    state_ = state;
}

void PurePursuit::setVelocity(const fsd_common_msgs::CarStateDt &velocity) {
    velocity_ = velocity;
}

void PurePursuit::runAlgorithm() {
    createControlCommand();
}

void PurePursuit::createControlCommand() {

    if (center_line_.points.empty()) {
        control_command_.throttle.data       = static_cast<float>(-1.0);
        control_command_.steering_angle.data = 0.0;
        return;
    }

    const auto it_center_line = std::min_element(center_line_.points.begin(), center_line_.points.end(),
                                                             [&](const geometry_msgs::Point32 &a,
                                                                 const geometry_msgs::Point32 &b) {
                                                                 const double da = std::hypot(state_.car_state.x - a.x,
                                                                                              state_.car_state.y - a.y);
                                                                 const double db = std::hypot(state_.car_state.x - b.x,
                                                                                              state_.car_state.y - b.y);

                                                                 return da < db;
                                                             });
                                                             
    const auto i_center_line  = std::distance(center_line_.points.begin(), it_center_line);
    const auto size           = center_line_.points.size();
    const auto i_next         = (i_center_line + 10) % size;

    geometry_msgs::Point32 next_point = center_line_.points[i_next];  
      
    // Pedro de Bem
    {
    // algumas constantes
        int sign;
        double vref;
        double kappa = 1.0;
        double kappaP = 0.1;
        double tangent_x = center_line_.points[i_center_line+1].x - center_line_.points[i_center_line-1].x;
        double tangent_y = center_line_.points[i_center_line+1].y - center_line_.points[i_center_line-1].y;
        double velocidade = std::hypot(state_.car_state_dt.car_state_dt.x , state_.car_state_dt.car_state_dt.y);
        double velocidade_x = state_.car_state_dt.car_state_dt.x*std::cos(state_.car_state.theta); // frame global
        double velocidade_y = state_.car_state_dt.car_state_dt.x*std::sin(state_.car_state.theta); // frame global
        double distance_x = it_center_line->x - state_.car_state.x;
        double distance_y = it_center_line->y - state_.car_state.y;
        double distance_x_next = next_point.x - state_.car_state.x;
        double distance_y_next = next_point.y - state_.car_state.y;

    // controle lateral

        // erro angular:

        // produto vetorial
        double nominador_ang = (velocidade_x) * (tangent_y)
                             - (velocidade_y) * (tangent_x);

        // multiplicação da magnitude dos vetores
        double denominador_ang = std::hypot(tangent_x , tangent_y) * velocidade;

        // arcsin
        double angular_error = std::asin(nominador_ang/denominador_ang);

        // erro lateral:

        // produto vetorial
        double pvlat = (distance_x_next) * (tangent_y)
                     - (distance_y_next) * (tangent_x);

        // garantir que o erro lateral aponta pra pista
        if(std::asin(pvlat) > 0){sign = -1;}
        else{sign = 1;}

        double lateral_error = std::hypot(distance_x , distance_y);

        double ajuste_lateral = sign*std::atan(kappa*(lateral_error/velocidade)); 

        if(velocidade<1){ajuste_lateral = 0;} // evitar erros quando carro está parado, lim(v->0) 1/(velocidade) = inf
    
        // sinal de controle    [-pi/2 , pi/2]    negativo:->   positivo:<-
        control_command_.steering_angle.data = 0.7*static_cast<float>(angular_error+ajuste_lateral)
                                             + 0.3*control_command_.steering_angle.data; 
    
    // Speed Controller

        if(i_center_line > 960 | i_center_line < 160){ // simular motion planning
            vref = 6;
        }
        else{
            vref = 4;
        }

        // sinal de controle
        control_command_.throttle.data = static_cast<float>(kappaP*(vref - velocidade));
    }

    // Visualize
    publishMarkers(it_center_line->x, it_center_line->y, next_point.x, next_point.y);

}

void PurePursuit::publishMarkers(double x_pos, double y_pos, double x_next, double y_next) const{
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker marker;
    marker.color.r            = 1.0;
    marker.color.a            = 1.0;
    marker.pose.position.x    = x_pos;
    marker.pose.position.y    = y_pos;
    marker.pose.orientation.w = 1.0;
    marker.type               = visualization_msgs::Marker::SPHERE;
    marker.action             = visualization_msgs::Marker::ADD;
    marker.id                 = 0;
    marker.scale.x            = 0.5;
    marker.scale.y            = 0.5;
    marker.scale.z            = 0.5;
    marker.header.stamp       = ros::Time::now();
    marker.header.frame_id    = "map";
    markers.markers.push_back(marker);

    marker.pose.position.x = x_next;
    marker.pose.position.y = y_next;
    marker.color.b         = 1.0;
    marker.id              = 1;
    markers.markers.push_back(marker);

    pub_closest_point_.publish(markers);
}

}
