// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Copyright 2012 Andrew Tridgell, all rights reserved.
// Adapted 2014 by Diogo Almeida, diogormalmeida@gmail.com
// THIS VERSION ASSUMES THAT THE VECTORIAL PART COMES FIRST!!!!!

#ifndef QUATERNION_H_D
#define QUATERNION_H_D

#include <math.h>

class Quaternion_D
{
public:
    float        q1, q2, q3, q4;

    // constructor creates a quaternion equivalent
    // to roll=0, pitch=0, yaw=0
    Quaternion_D() {
        q1 = 0; q2 = q3 = q4 = 1;
    }

    // setting constructor
    Quaternion_D(const float _q1, const float _q2, const float _q3, const float _q4) :
        q1(_q1), q2(_q2), q3(_q3), q4(_q4) {
    }

    // function call operator
    void operator        ()(const float _q1, const float _q2, const float _q3, const float _q4)
    {
        q1 = _q1; q2 = _q2; q3 = _q3; q4 = _q4;
    }

    // check if any elements are NAN
    bool        is_nan(void)
    {
        return isnan(q1) || isnan(q2) || isnan(q3) || isnan(q4);
    }

    // implements sign_l
    void  sign_l(void)
    {
        if(q4<0)
          {q1=-q1;q2=-q2;q3=-q3;q4=-q4;} 
    }

    // return the rotation matrix equivalent for this quaternion
    void        rotation_matrix(Matrix3f &m);

    // convert a vector from earth to body frame
    void        earth_to_body(Vector3f &v);

    // create a quaternion from Euler angles
    void        from_euler(float roll, float pitch, float yaw);

    // create eulers from a quaternion
    void        to_euler(float *roll, float *pitch, float *yaw);

    // creates the conjugate
    Quaternion_D conjugate(void);

    // gets a quaternion just with the z part of this one
    Quaternion_D get_z(void);

    // Allows to obtain the quaternion expressing the rotations in x and y.
    

};


Quaternion_D mult_quat(Quaternion_D q, Quaternion_D p);
Quaternion_D mult_quat_inv(Quaternion_D q, Quaternion_D qz);

#endif // QUATERNION_H_D
