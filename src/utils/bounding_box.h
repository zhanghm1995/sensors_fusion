// Copyright (C) 2018  Haiming Zhang, Research Center of Intelligent Vehicle, Beijing Institute of Technology

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef DEPTH_CLUSTERING_BOUNDINGBOX_H
#define DEPTH_CLUSTERING_BOUNDINGBOX_H
#include <cstdint>

namespace sensors_fusion{

/**
 * @brief      Pixel coordinates structure
 */
struct PixelCoord {
    PixelCoord() : _x(0), _y(0) {}
    PixelCoord(int16_t x, int16_t y) : _x(x), _y(y) {}
    PixelCoord operator+(const PixelCoord& other) const {
      return PixelCoord(_x + other._x, _y + other._y);
    }

    int16_t _x;
    int16_t _y;
};

/**
 * @brief      Class for store image 2D bounding box data and offer some useful funtions.
 *
 *
 */
class BoundingBox {
public:
  //constructor
  BoundingBox(){}
  BoundingBox(PixelCoord top_left,PixelCoord bottom_right):
          _top_left(top_left),
          _bottom_right(bottom_right){}

  //get what we want
  const int& beg_x()const{
    return _top_left._x;
  }
  int& beg_x(){
    return _top_left._x;
  }
  const int& beg_y()const{
    return _top_left._y;
  }
  int& beg_y(){
    return _top_left._y;
  }
  const int& end_x()const{
    return _bottom_right._x;
  }
  int& end_x(){
    return _bottom_right._x;
  }
  const int& end_y()const{
    return _bottom_right._y;
  }
  int& end_y(){
    return _bottom_right._y;
  }

  const int& width()const{
    return (_bottom_right._x-_top_left._x);
  }
  const int& height()const{
    return (_bottom_right._y-_top_left._y);
  }

protected:
  PixelCoord _top_left;
  PixelCoord _bottom_right;


};


}



#endif //DEPTH_CLUSTERING_BOUNDINGBOX_H
