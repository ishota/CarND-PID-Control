#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double goodness = 0;
double pre_goodness = 0.5;
double best_goodness = 2.0;
double good_time = 10000;
double bad_cte = 5;
double cum_cte = 0;

int sim_time = 2;
int sim_iteration = 0;
int best_sim_time = 100;
int pre_sim_time = best_sim_time;
int twigle_iteration = 0;
int twigle_count = 1;
int twigle_sub_count = 1;
int escape_sw = 0;

double kp = 0.07;
double ki = 0.000073;
double kd = 0.507;
double pre_kp = kp;
double pre_ki = ki;
double pre_kd = kd;
double best_kp = kp;
double best_ki = ki;
double best_kd = kd;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  pid.Init(kp, ki, kd); //no limit best

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          // double angle = std::stod(j[1]["steering_angle"].get<string>());
          double throttle_value = 0.2;

          // init simulation
          if (speed <= 10 && cte >= 0.75 && cte <= 0.77) {
            if ( sim_time > 1) {
              sim_iteration++;
              pid.Init(kp, ki, kd);
              printf("[%d] kp:%f(%f) ki:%f(%f) kd:%f(%f)\n", sim_iteration, kp, best_kp, ki, best_ki, kd, best_kd);
            }
            escape_sw = 0;
            sim_time = 1;
            cum_cte = 0;
          } else {
            cum_cte += fabs(cte);
            if ( escape_sw == 0 ) {
              sim_time++;
            }
          }

          // Calculate steering_value
          pid.UpdateError(cte);
          double steer_value = pid.TotalError();
          goodness = cum_cte / sim_time;

          //TWIDDLE
          if ( (fabs(cte) > bad_cte || sim_time > good_time ) && escape_sw == 0) {
            twigle_iteration++;
            escape_sw = 1;

            // sim_time is over good_time
            if ( sim_time > good_time ) {

              // best_goodness
              if ( goodness < best_goodness ){
                printf(" godness %f < best_goodness %f", goodness, best_goodness);
                best_goodness = goodness;
                pre_goodness = best_goodness;
                pre_sim_time = sim_time;
                best_kp = kp;
                best_ki = ki;
                best_kd = kd;
              } else {
              // not best
                printf(" godness %f >= best_goodness %f", goodness, best_goodness);
                pre_goodness = best_goodness;
                pre_sim_time = best_sim_time;
                kp = best_kp;
                ki = best_ki;
                kd = best_kd;
              }

            // sim_time is lower than best time
            } else if ( sim_time > best_sim_time ) {
              printf(" sim_time %d > best_sim_time %d", sim_time, best_sim_time);
              best_sim_time = sim_time;
              pre_goodness = goodness;
              best_goodness = goodness;
              pre_sim_time = sim_time;
              best_kp = kp;
              best_ki = ki;
              best_kd = kd;
            } else if ( pre_sim_time > sim_time ){
            // sim_time is lower than previous sim_time
              printf(" pre_sim_time %d > sim_time %d", pre_sim_time, sim_time);
              twigle_sub_count++;
              kp = pre_kp;
              ki = pre_ki;
              kd = pre_kd;
              if ( twigle_sub_count % 2 != 0 ) {
                twigle_count++;
              }
            } else {
              pre_goodness = goodness;
              pre_sim_time = sim_time;
            }

            pre_kp = kp;
            pre_ki = ki;
            pre_kd = kd;

            printf(" tw:%d %d %d, good:%f(%f), sim_time:%d(%d), ",twigle_iteration, twigle_count, twigle_sub_count, goodness, best_goodness, sim_time, best_sim_time);
            double x = (double)rand()/RAND_MAX;
            x /= 10;

            if ( twigle_count % 3 == 0 ) {
              if ( twigle_sub_count % 2 == 0 ) {
                std::cout << " ki up:" << ki << " -> ";
                ki *= 1 + x;
                std::cout << ki;
              } else {
                std::cout << " ki down:" << ki << " -> ";
                ki *= 1 - x;
                std::cout << ki;
              }
            } else if (twigle_count % 2 == 0 ) {
              if ( twigle_sub_count % 2 == 0 ) {
                std::cout << " kd up:" << kd << " -> ";
                kd *= 1 + x;
                std::cout << kd;
              } else {
                std::cout << " kd down:" << kd << " -> ";
                kd *= 1 - x;
                std::cout << kd;
              }
            } else {
              if ( twigle_sub_count % 2 == 0 ) {
                std::cout << " kp up:" << kp << " -> ";
                kp *= 1 + x;
                std::cout << kp;
              } else {
                std::cout << " kp down:" << kp << " -> ";
                kp *= 1 - x;
                std::cout << kp;
              }
            }

            // pid coef update
            printf("\n");
          }

          throttle_value *= (1 - fabs(steer_value));

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    // std::cout << "Connected!!!" << std::endl;
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