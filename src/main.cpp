#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <math.h>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0 -
  // max length of the track
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    //normal component of way point
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  /* Start in lane 1 which is the middle lane; left lane is 0 */
  int lane = 1;

  /* Set some reference velocity in MPH - taken 49.5 since we dont want to
  cross 50 MPH */
  double ref_vel = 49.5;

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data - these values come from the simulator
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // My Code starts here  

          //way points are the points on the map. we can find the closest waypoint 
          //closest waypoint could be behind the car but next waypoint could be where to go next
          /* Actual path planner points */

          /* Get the size of the previous path vector. The simulator
          actually gives the previous path */
          int prev_size = previous_path_x.size();

          ///**
          // * TODO: define a path made up of (x,y) points that the car will visit
          // *   sequentially every .02 seconds
          // */
          ///* If car has to visit a points every 0.02 seconds then in 1 second there
          //should be 50 points. And if we keep the distance between 2 points as 0.5 meter,
          //then the speed of the car becomes 0.5 * 50 = 25 m/second (50 MPH) */
          //double dist_inc = 0.4;
          //for (int i = 0; i < 50; ++i) {
          //    /* To stay in the lanes Frenet co-ordinates are very useful */
          //    /* We get the next iteration, otherwise our first point will be exactly where
          //    car is at and we would not be transitioning */
          //    double next_s = car_s + (i + 1) * dist_inc;
          //    /* We are in the middle lane, which means 4m of left lane + 2m of middle lane 
          //    since we are in the middle of the middle lane, so we are 6m from the double yellow
          //    lane from the side of left lane*/
          //    /* In other words we are 1 and a half lanes from the way points */
          //    double next_d = 6;

          //    /* Now lets transform s and d values to x and y cordinates */
          //    std::vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          //    next_x_vals.push_back(xy[0]);
          //    next_y_vals.push_back(xy[1]);
          //}
#if 1
          /* Create a widely spaced vector points spaced at 30m each, later we will
          interpolate these points with spline and fill in more points */
          std::vector<double> points_x{};
          std::vector<double> points_y{};

          /* Create reference variable for x , y and yaw. they could be either the
          starting point of the car or the end point of the previous path */
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);


          /* Check whether the previous car state is nearly empty or has some points */
          if (prev_size < 2) {
              double previous_car_x = car_x - cos(car_yaw);
              double previous_car_y = car_y - sin(car_yaw);

              /* Use two points that make path tangent to the car */
              points_x.push_back(previous_car_x);
              points_x.push_back(car_x);

              points_y.push_back(previous_car_y);
              points_y.push_back(car_y);
              /* Use the previous path's end point as the reference */
          }
          else {
              /* Redefine reference state as the end point of the previous path */
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              /* Calculate the angle based on the previous points */
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              points_x.push_back(ref_x_prev);
              points_x.push_back(ref_x);

              points_y.push_back(ref_y_prev);
              points_y.push_back(ref_y);
          }

          std::vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s,
              map_waypoints_x, map_waypoints_y);
          std::vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s,
              map_waypoints_x, map_waypoints_y);
          std::vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s,
              map_waypoints_x, map_waypoints_y);

          points_x.push_back(next_wp0[0]);
          points_x.push_back(next_wp1[0]);
          points_x.push_back(next_wp2[0]);

          points_y.push_back(next_wp0[1]);
          points_y.push_back(next_wp1[1]);
          points_y.push_back(next_wp2[1]);

          for (int i = 0; i < points_x.size(); i++) {
              /*Shift car reference angle to 0 degrees */
              double shift_x = points_x[i] - ref_x;
              double shift_y = points_y[i] - ref_y;

              /* Transform to local cordinates */
              points_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
              points_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          /* Create a spline */
          tk::spline s;

          /* Set some points in the spline */
          s.set_points(points_x, points_y);

          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          /* Start adding the previous points to the path planner */
          /* Instead of recreating points from the scratch, add points
          left from the previous path */
          for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          /* calculate how to break up spline points */
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          ///Here we are dealing with two different types of points : one is the widely spaced 
          ///spline points points_x and points_y, which are first five anchor points and not previous path points.
          ///The other one(next_x_vals) is the previous path points 

          for (int i = 0; i < (50 - previous_path_x.size()); i++) {
              /* Dividing by 2.24 since it has to m/sec */
              double N = (target_dist / (0.02 * ref_vel / 2.24));
              /* Adding on the number of hash marks on x-axis starting with 0 */
              double x_point = x_add_on + (target_x) / N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              /* Transform back to global co-ordinates */
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

              x_point += x_ref;
              y_point += y_ref;

              next_x_vals.push_back(x_point);
              next_x_vals.push_back(y_point);
          }
#endif

          json msgJson; 

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}