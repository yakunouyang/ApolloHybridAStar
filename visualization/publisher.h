//
// Created by 欧阳亚坤 on 2022/12/21.
//
#pragma once
#include <string>
#include <queue>
#include <memory>

#include "easywsclient.hpp"
#include "marker_array.h"

namespace visualization {

class Publisher {
public:
  explicit Publisher(const std::string &frame, const std::string &server = "ws://localhost:3699");

  void publish(const MarkerArray &msg);

  void spin();

private:

  std::unique_ptr<easywsclient::WebSocket> client_;
};

}