//
// Created by yenkn on 2021/07/28.
//
#pragma once
#include "math/aabox2d.h"
#include "math/polygon2d.h"
#include "math/line_segment2d.h"

namespace common {
namespace math {

class Circle2d {
public:
  Circle2d(double x, double y, double r): center_(x, y), r_(r) {}

  inline bool IsPointIn(const Vec2d &point) const {
    return center_.DistanceTo(point) <= r_;
  }

  inline bool HasOverlap(const Circle2d &circle) const {
    return center_.DistanceTo(circle.center_) <= r_ + circle.r_;
  }

  inline bool HasOverlap(const Polygon2d &box) const {
    if(!AABox2d(center_, r_ * 2, r_ * 2).HasOverlap(box.AABoundingBox())) {
      return false;
    }

    std::vector<Vec2d> corners;
    box.GetAllVertices(&corners);

    for(auto &vec: corners) {
      if(IsPointIn(vec)) {
        return true;
      }
    }

    for(int i = 0; i < corners.size(); i++) {
      if(LineSegment2d(corners[i], corners[(i+1) % corners.size()]).DistanceTo(center_) <= r_) {
        return true;
      }
    }

    return false;
  }

private:
  Vec2d center_;
  double r_;
};

}
}
