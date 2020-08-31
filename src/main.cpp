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

//Static constant expressions added by me 
static constexpr int LEFT_LANE{ 0 };
static constexpr int MIDDLE_LANE{ 1 };
static constexpr int RIGHT_LANE{ 2 };
static constexpr int NO_LANE{ -1 };
static constexpr double CONSTANT_VEL_VAL{ 0.224f };
static constexpr double MAX_VEL{ 49.5f };


int main() {
  uWS::Hub h;

  /* Load up map values for waypoint's x,y,s and d normalized normal vectors 
   way points are the points on the map. we can find the closest waypoint 
   closest waypoint could be behind the car but next waypoint could be where to go next */
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  /* Waypoint map to read from. Has 180 way points along the center of the track */
  string map_file_ = "../data/highway_map.csv";
  /* The max s value before wrapping around the track back to 0 -
  max length of the track */
  double max_s = 6945.554;

  /* Read the map file */
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  /* Get the x, y, s, d_x, d_y values of all the waypoints and 
  add it to the vector */
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
  int ego_lane = MIDDLE_LANE;

  /* Set some reference velocity in MPH - taken as 0, maximum being 49.5 */
  //double ref_vel = 0.0;
  double ref_vel = MAX_VEL;

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ego_lane]
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

            /* Main car's localization Data - these values come from the simulator */
            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = j[1]["yaw"];
            double car_speed = j[1]["speed"];

            /* Previous path data given to the Planner */
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];

            /* Previous path's end s and d values */
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            /* Sensor Fusion Data, a list of all other cars on the same side
            of the road */
            auto sensor_fusion = j[1]["sensor_fusion"];

            // My Code starts here  
            /* Get the size of the previous path vector. The simulator
            actually gives the previous path */
            int prev_size = previous_path_x.size();

            /* If there are previous points , then the car_s will be previous end point.
            This is beacuse the starting point will be the previous path's end-point */
            if (prev_size > 0) {
                car_s = end_path_s;
            }

            ///* Variable to define car lane */
            //int targer_car_lane;
            ///* Variable to define change in speed */
            //double ego_speed_change;
            ///* Variable to define direction of lanes */
            //int target_car_ahead = false;
            //int target_car_left = false;
            //int target_car_right = false;
            //int target_car_close = false;

            ///* Prediction - Get all the sensor fusion value of all the cars to know if there is any vehicle
            //in the ego car's lane and the velocities of the other vehicles */
            //for (int i = 0; i < sensor_fusion.size(); i++) {
            //    /* d value gives what lane other cars are in */
            //    float d_target = sensor_fusion[i][6];

            //    /* Check in which lane the cars are present */
            //    if (d_target > 0 && d_target < 4) {
            //        targer_car_lane = LEFT_LANE;
            //    }
            //    else if (d_target > 4 && d_target < 8) {
            //        targer_car_lane = MIDDLE_LANE;
            //    }
            //    else if (d_target > 8 && d_target < 12) {
            //        targer_car_lane = RIGHT_LANE;
            //    }
            //    else {
            //        targer_car_lane = NO_LANE;
            //    }

            //    /* Check if the other car is in ou\r lane(between +2 and -2 from the center point
            //    of our middle lane ) and check how close it is to us */
            //    if (d_target < (2 + 4 * ego_lane + 2) && d_target >(2 + 4 * ego_lane - 2)) {
            //        double vx_target = sensor_fusion[i][3];
            //        double vy_target = sensor_fusion[i][4];
            //        /* Speed is important to predict where the car would be
            //        in future */
            //        double check_target_speed = sqrt(vx_target * vx_target + vy_target * vy_target);
            //        /* checks the s value of the other cars to check if they
            //        are nearby */
            //        double check_target_s = sensor_fusion[i][5];

            //        /* What does the car look like in the future. For this we will use the speed of the car and the
            //        previous path size.
            //         1) previous path point projects the current path of the simulator; prev_size = number of previous waypoints.
            //         2) .02 seconds = 20 milliseconds = time taken to reach the next waypoint
            //         3) check_speed = speed of the other car distance from one waypoint to other = .02 * check_speed
            //         4) prev_size * .02 * check_speed = total distance covered by the car currently in the simulator
            //         5) therefore check_car_s += ((double)prev_size *.02 * check_speed) will be the future distance */
            //        check_target_s += ((double)prev_size * .02 * check_target_speed);

            //        if ((check_target_s > car_s) && ((check_target_s - car_s) < 30)) {
            //            ref_vel = 29.5;
            //        }

          //          /* If the car is in front of us and the the gap between the other car
          //          and our car is less than 30 meters, set the flag */
          //          if ((targer_car_lane == ego_lane) && (check_target_s > car_s) && ((check_target_s - car_s) < 30)) {
          //              target_car_ahead = true;
          //              /* If the car is in the left side and the the gap between the other car
          //              and our car is less than 30 meters, set the flag */
          //          }
          //          else if ((targer_car_lane == (ego_lane - 1)) && (car_s - 30 > check_target_s < car_s + 30)) {
          //              target_car_left = true;
          //              /* If the car is in the right side and the the gap between the other car
          //              and our car is less than 30 meters, set the flag */
          //          }
          //          else if ((targer_car_lane == (ego_lane + 1)) && (car_s - 30 > check_target_s < car_s + 30)) {
          //              target_car_right = true;
          //          }
          //      }
          //  }
          //
          //
          ///* Behavioral planning : what has to be done based on the predictions */
          ///* If a car is in front of us */
          //if (target_car_ahead) {
          //    /* And if there is no car in the left side of the lane */
          //    if (ego_lane > LEFT_LANE && !target_car_left) {
          //        ego_lane--;
          //    }
          //    /* And if there is no car in the right side of the lane */
          //    else if (ego_lane < RIGHT_LANE && !target_car_right) {
          //        ego_lane++;
          //    }
          //    else {
          //        /* To do an incremental change in the velocity, if the car is too close, subtract some
          //        constant value, 0.224(it ends up being 5 m/second2)*/
          //        ego_speed_change -= CONSTANT_VEL_VAL;
          //    }
          //} else {
          //    if (ego_lane != MIDDLE_LANE) {
          //        if ((ego_lane == LEFT_LANE && !target_car_right) || (ego_lane == RIGHT_LANE && !target_car_left)) {
          //            ego_lane = MIDDLE_LANE;
          //        }
          //    }

          //    if (ref_vel < MAX_VEL)
          //        ego_speed_change += CONSTANT_VEL_VAL;
          //}   

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

          std::vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * ego_lane), map_waypoints_s,
              map_waypoints_x, map_waypoints_y);
          std::vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * ego_lane), map_waypoints_s,
              map_waypoints_x, map_waypoints_y);
          std::vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * ego_lane), map_waypoints_s,
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

          /* calculate how to break up the 30m spaced spline points */
          /* Horizontal x axis */
          double target_x = 30;
          /* The vertical y value of corresponding x value */
          double target_y = s(target_x);
          /* Value of the hypoteneuse */
          double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
          double x_add_on = 0;

          /* Here we are dealing with two different types of points : one is the widely spaced 
          spline points points_x and points_y, which are first five anchor points and not previous path points.
          The other one(next_x_vals) is the previous path points which is filled in the spline after filling the 
          five anchor points */         
          for (int i = 1; i <= (50 - previous_path_x.size()); i++) {
              //ref_vel += ego_speed_change;

              //if (ref_vel > MAX_VEL) {
              //    ref_vel = MAX_VEL;
              //} else if (ref_vel < CONSTANT_VEL_VAL) {
              //    ref_vel = CONSTANT_VEL_VAL;
              //}

              /* Calculate distance between two points within target distance(hypotenuese) of 30m. Since it takes 20millisecond to
              move from one point to another, the distance between two points is*/
              /* Dividing by 2.24 since it has to m/sec; and 0.02 is 20milliseconds in seconds */
              double target_dist_points = (0.02 * ref_vel / 2.24);
              /* Now claculate how many points within target distance(hypotenuese) of 30m. */
              double N = (target_dist / target_dist_points);
              /* Now use these same N points to break x-axis or to calculate distance between two x points */
              double dist_target_x_points = (target_x) / N;
              /* Adding on the number of points on x-axis starting with 0 */
              double x_point = x_add_on + dist_target_x_points;
              /* The s.setpoints(ptx, pty) of five anchor points helps in calculating y value corresponding to x value */
              double y_point = s(x_point);

              /* Keep adding x points to calculate the x point in the spline */
              x_add_on = x_point;

              /* Transfer to a variable to convert it back to global co-ordinates */
              double x_temp = x_point;
              double y_temp = y_point;

              /* Transform back to global co-ordinates */
              x_point = (x_temp * cos(ref_yaw) - y_temp * sin(ref_yaw));
              y_point = (x_temp * sin(ref_yaw) + y_temp * cos(ref_yaw));

              /* Add the calculated x-points to the initial x, y points of the car which we got from localization */
              x_point += ref_x;
              y_point += ref_y;

              /* Push these final calculated x,y points to the path planner */
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }

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