//
// Created by yakunouyang on 2022/9/19.
//
#pragma once

//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     MarkerArray data = nlohmann::json::parse(jsonString);

#include "nlohmann/json.hpp"
#include "color.h"

namespace visualization {
using nlohmann::json;

struct Stamp {
  int64_t nsec = 0;
  int64_t sec = 0;
};

struct Header {
  std::string frame_id;
  int64_t seq = 0;
  Stamp stamp;
};

struct Lifetime {
  int64_t nsec = 0;
  int64_t sec = 0;
};

struct Orientation {
  double w = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Vector3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Pose {
  Orientation orientation;
  Vector3 position;
};

namespace MarkerConst {
constexpr uint8_t ARROW=0;
constexpr uint8_t CUBE=1;
constexpr uint8_t SPHERE=2;
constexpr uint8_t CYLINDER=3;
constexpr uint8_t LINE_STRIP=4;
constexpr uint8_t LINE_LIST=5;
constexpr uint8_t CUBE_LIST=6;
constexpr uint8_t SPHERE_LIST=7;
constexpr uint8_t POINTS=8;
constexpr uint8_t TEXT_VIEW_FACING=9;
constexpr uint8_t MESH_RESOURCE=10;
constexpr uint8_t TRIANGLE_LIST=11;
constexpr uint8_t ADD=0;
constexpr uint8_t MODIFY=0;
constexpr uint8_t DELETE=2;
constexpr uint8_t DELETEALL=3;
}

struct Marker {
  int64_t action = MarkerConst::ADD;
  visualization::Color color;
  std::vector<visualization::Color> colors;
  bool frame_locked = false;
  Header header;
  int64_t id = 1;
  Lifetime lifetime;
  std::string mesh_resource;
  bool mesh_use_embedded_materials = false;
  std::string ns;
  std::vector<Vector3> points;
  Pose pose;
  Vector3 scale;
  std::string text;
  int64_t type = 0;
};

struct MarkerArray {
  std::vector<Marker> markers;
};
}


namespace nlohmann {

inline void to_json(json &j, const visualization::Color &x) {
  j["a"] = x.a();
  j["b"] = x.b();
  j["g"] = x.g();
  j["r"] = x.r();
}

inline void to_json(json &j, const visualization::Stamp &x) {
  j = json::object();
  j["nsec"] = x.nsec;
  j["sec"] = x.sec;
}

inline void to_json(json &j, const visualization::Header &x) {
  j = json::object();
  j["frame_id"] = x.frame_id;
  j["seq"] = x.seq;
  j["stamp"] = x.stamp;
}

inline void to_json(json &j, const visualization::Lifetime &x) {
  j = json::object();
  j["nsec"] = x.nsec;
  j["sec"] = x.sec;
}

inline void to_json(json &j, const visualization::Vector3 &x) {
  j = json::object();
  j["x"] = x.x;
  j["y"] = x.y;
  j["z"] = x.z;
}

inline void to_json(json &j, const visualization::Orientation &x) {
  j = json::object();
  j["w"] = x.w;
  j["x"] = x.x;
  j["y"] = x.y;
  j["z"] = x.z;
}

inline void to_json(json &j, const visualization::Pose &x) {
  j = json::object();
  j["orientation"] = x.orientation;
  j["position"] = x.position;
}

inline void to_json(json &j, const visualization::Marker &x) {
  j = json::object();
  j["action"] = x.action;
  j["color"] = x.color;
  j["colors"] = x.colors;
  j["frame_locked"] = x.frame_locked;
  j["header"] = x.header;
  j["id"] = x.id;
  j["lifetime"] = x.lifetime;
  j["mesh_resource"] = x.mesh_resource;
  j["mesh_use_embedded_materials"] = x.mesh_use_embedded_materials;
  j["ns"] = x.ns;
  j["points"] = x.points;
  j["pose"] = x.pose;
  j["scale"] = x.scale;
  j["text"] = x.text;
  j["type"] = x.type;
}

inline void to_json(json &j, const visualization::MarkerArray &x) {
  j = json::object();
  j["markers"] = x.markers;
}
}
