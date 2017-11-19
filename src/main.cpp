#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid_steer;
  // TODO: Initialize the pid variable.

  // start with P control only
  pid_steer.Init(0.01, 0.0, 0.0);  // low P -> steering not sufficient in first turn -> increase P
  pid_steer.Init(0.02, 0.0, 0.0);  // increased P -> steering still not sufficient in first turn -> increase P
  pid_steer.Init(0.03, 0.0, 0.0);  // increased P -> steering still not sufficient in first turn -> increase P
  pid_steer.Init(0.04, 0.0, 0.0);  // increased P -> car is osscillating -> use small D
  pid_steer.Init(0.04, 0.0, 0.1);  // used small D -> car is not oscillating, but steering not sufficient in first turn -> increase P
  pid_steer.Init(0.05, 0.0, 0.1);  // increased P -> car is  oscillating -> increase D
  pid_steer.Init(0.05, 0.0, 0.2);  // increased D -> car goes around turn 1, but begins osscilling on bridge -> increase D
  pid_steer.Init(0.05, 0.0, 0.3);  // increased D -> car drives over bridge but crashes at second (strong) turn -> increase P
  pid_steer.Init(0.06, 0.0, 0.3);  // increased P -> car is oscillating -> increase D
  pid_steer.Init(0.06, 0.0, 0.4);  // increased D -> car is still oscillating -> increase D  
  pid_steer.Init(0.06, 0.0, 0.5);  // increased D -> car drives around track but touches borders, still a little bit oscillating -> increase D
  pid_steer.Init(0.06, 0.0, 0.6);  // increased D -> car crashes in second turn -> increase P
  pid_steer.Init(0.07, 0.0, 0.6);  // increased P -> car is oscillating an crashing  -> increase D
  pid_steer.Init(0.07, 0.0, 0.7);  // increased D -> sum cte 1251, car drives around track, but touches some borders -> increase P
  pid_steer.Init(0.08, 0.0, 0.7);  // increased P -> sum cte 1100, little bit osscillating  -> increase D
  pid_steer.Init(0.08, 0.0, 0.8);  // increased D -> sum cte 1063, still touches some borders -> increase P
  pid_steer.Init(0.09, 0.0, 0.8);  // increased D -> sum cte 923, little bit osscillating  -> increase D
  pid_steer.Init(0.09, 0.0, 0.9);  // increased D -> sum cte 972 -> increase P
  pid_steer.Init(0.10, 0.0, 0.9);  // increased P -> sum cte 890, osscillating -> increase D
  pid_steer.Init(0.10, 0.0, 1.0);  // increased D -> sum cte 828 -> increase p
  pid_steer.Init(0.11, 0.0, 1.0);  // increased D -> sum cte 769 -> increase p
  pid_steer.Init(0.12, 0.0, 1.0);  // increased D -> sum cte 873, osscillating -> increase D
  pid_steer.Init(0.12, 0.0, 1.1);  // increased D -> sum cte 698, osscillating -> increase D
  pid_steer.Init(0.12, 0.0, 1.2);  // increased D -> sum cte 706 -> increase p
  pid_steer.Init(0.13, 0.0, 1.2);  // increased P -> sum cte 691, osscillating -> increase D
  pid_steer.Init(0.13, 0.0, 1.3);  // increased D -> sum cte 649 -> increase P
  pid_steer.Init(0.14, 0.0, 1.3);  // increased P -> sum cte 633, osscillating -> increase D
  pid_steer.Init(0.14, 0.0, 1.4);  // increased D -> sum cte 618, osscillating -> increase D
  pid_steer.Init(0.14, 0.0, 1.5);  // increased D -> sum cte 645 -> increase P
  pid_steer.Init(0.15, 0.0, 1.5);  // increased P -> sum cte 658 -> go back to best values and increase I
  pid_steer.Init(0.14, 0.001, 1.4); // increased I -> sum cte 535 ->  increase I
  pid_steer.Init(0.14, 0.002, 1.4); // increased I -> sum cte 542 -> decrease I
  pid_steer.Init(0.14, 0.0005, 1.4);// decreased I -> sum cte 542 -> go back to best I value and increase D
  pid_steer.Init(0.14, 0.001, 1.5); // increased D -> sum cte 491 -> increase D
  pid_steer.Init(0.14, 0.001, 1.6); // increased D -> sum cte 531 -> go back to best D value and increase P
  pid_steer.Init(0.15, 0.001, 1.5); // increased P -> sum cte 503, osscillating -> last try: increase D
  pid_steer.Init(0.15, 0.001, 1.6); // increased P -> sum cte 524 -> go back to best values
  pid_steer.Init(0.14, 0.001, 1.5); // best values

  double sum_cte_steer = 0.0;

  PID pid_throttle;
  pid_throttle.Init(0.9, 0.000001, 0.5);
  double sum_cte_throttle = 0.0;

  unsigned step = 0;

  h.onMessage([&pid_steer, &pid_throttle, &sum_cte_steer, &sum_cte_throttle, &step](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte_steer = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = 0.0;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid_steer.UpdateError(cte_steer);
          steer_value = pid_steer.TotalError();
          // restrict values to range [-1,1]
          if (steer_value > 1.0f) steer_value = 1.0;
          else if (steer_value < -1.0f) steer_value = -1.0f;

          // control the speed
          double speed_tgt = 50.0;
          double cte_throttle = speed - speed_tgt;
          double throttle_value;
          pid_throttle.UpdateError(cte_throttle);
          throttle_value = pid_throttle.TotalError();
          // restrict to range [0,1]
          if (throttle_value > 1.0f) throttle_value = 1.0;
          else if (throttle_value < 0.0f) throttle_value = 0.0f;

          // performance measurement
          sum_cte_steer += fabs(cte_steer);
          sum_cte_throttle += fabs(cte_throttle);
          const int steps_one_lap = 1140;
    
          step++;
          if (step > steps_one_lap) {
             std::cout << "END OF LAP " << std::endl;
             std::cout << "sum_cte_steer: " << sum_cte_steer << std::endl;
             std::cout << "sum_cte_throttle: " << sum_cte_throttle << std::endl;
             return;
          }
          // DEBUG
          std::cout << "step: " << step << std::endl;
          std::cout << "cte_steer: " << cte_steer << ", steer_value: " << steer_value << std::endl;
          std::cout << "cte_throttle: " << cte_steer << ", throttle_value: " << throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
