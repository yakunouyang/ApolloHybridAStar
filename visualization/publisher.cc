//
// Created by 欧阳亚坤 on 2022/12/21.
//

#include "publisher.h"


namespace visualization {

Publisher::Publisher(const std::string &frame, const std::string &server) {
  client_ = std::unique_ptr<easywsclient::WebSocket>(easywsclient::WebSocket::from_url(server));
}

void Publisher::publish(const MarkerArray &msg) {
  auto data = nlohmann::json(msg).dump();
  client_->send(data);
  client_->poll();
}

void Publisher::spin() {

}

}
