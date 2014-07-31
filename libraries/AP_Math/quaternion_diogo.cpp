/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * quaternion.cpp
 * Copyright (C) Andrew Tridgell 2012
 * Adapted 2014 by Diogo Almeida, diogormalmeida@gmail.com
 * 
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Math.h"
#include "quaternion_diogo.h"

// return the rotation matrix equivalent for this quaternion
void Quaternion_D::rotation_matrix(Matrix3f &m)
{
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q4q1 = q4 * q1;
    float q4q2 = q4 * q2;
    float q4q3 = q4 * q3;
    float q3q3 = q3 * q3;

    m.a.x = 1-2*(q2q2 + q3q3);
    m.a.y =   2*(q1q2 - q4q3);
    m.a.z =   2*(q1q3 + q4q2);
    m.b.x =   2*(q1q2 + q4q3);
    m.b.y = 1-2*(q1q1 + q3q3);
    m.b.z =   2*(q2q3 - q4q1);
    m.c.x =   2*(q1q3 - q4q2);
    m.c.y =   2*(q2q3 + q4q1);
    m.c.z = 1-2*(q1q1 + q2q2);
}

// convert a vector from earth to body frame
void Quaternion_D::earth_to_body(Vector3f &v)
{
    Matrix3f m;
    // we reverse z before and afterwards because of the differing
    // quaternion conventions from APM conventions.
    v.z = -v.z;
    rotation_matrix(m);
    v = m * v;
    v.z = -v.z;
}

// create a quaternion from Euler angles
void Quaternion_D::from_euler(float roll, float pitch, float yaw)
{
    float cr2 = cosf(roll*0.5f);
    float cp2 = cosf(pitch*0.5f);
    float cy2 = cosf(yaw*0.5f);
    float sr2 = sinf(roll*0.5f);
    float sp2 = sinf(pitch*0.5f);
    float sy2 = sinf(yaw*0.5f);

    q4 = cr2*cp2*cy2 + sr2*sp2*sy2;
    q1 = sr2*cp2*cy2 - cr2*sp2*sy2;
    q2 = cr2*sp2*cy2 + sr2*cp2*sy2;
    q3 = cr2*cp2*sy2 - sr2*sp2*cy2;
}

// create eulers from a quaternion
void Quaternion_D::to_euler(float *roll, float *pitch, float *yaw)
{
    if (roll) {
        *roll = (atan2f(2.0f*(q4*q1 + q2*q3),
                       1 - 2.0f*(q1*q1 + q2*q2)));
    }
    if (pitch) {
        // we let safe_asin() handle the singularities near 90/-90 in pitch
        *pitch = safe_asin(2.0f*(q4*q2 - q3*q1));
    }
    if (yaw) {
        *yaw = atan2f(2.0f*(q4*q3 + q1*q2),
                      1 - 2.0f*(q2*q2 + q3*q3));
    }
}

// conjugates the quaternion
Quaternion_D Quaternion_D::conjugate(void)
{

    Quaternion_D ret(-q1,-q2,-q3,q4);

    return ret;

}

Quaternion_D Quaternion_D::get_z(void)
{

    float r3=0,r4=0,norm = 0;

    norm = safe_sqrt(q3*q3+q4*q4);

    r3 = q3/norm;
    r4 = q4/norm;

    Quaternion_D ret(0,0,r3,r4);

    return ret;


}

// multiplies two quaternions
Quaternion_D mult_quat(Quaternion_D q, Quaternion_D p)
{
    float r1=0,r2=0,r3=0,r4=0,norm=0;

    r1 = q.q1*p.q4+q.q2*p.q3-q.q3*p.q2+q.q4*p.q1;
    r2 = -q.q1*p.q3+q.q2*p.q4+q.q3*p.q1+q.q4*p.q2;
    r3 = q.q1*p.q2-q.q2*p.q1+q.q3*p.q4+q.q4*p.q3;
    r4 = -q.q1*p.q1-q.q2*p.q2-q.q3*p.q3+q.q4*p.q4;

    norm = safe_sqrt(r1*r1+r2*r2+r3*r3+r4*r4);
    r1 = r1/norm;
    r2 = r2/norm;
    r3 = r3/norm;
    r4 = r4/norm;

    Quaternion_D ret(r1,r2,r3,r4);

    return ret;

}

Quaternion_D mult_quat_inv(Quaternion_D q, Quaternion_D qz)
{

    float r1=0,r2=0,r3=0,r4=0,qz1=0,qz2=0,qz3=0,qz4=0,norm=0;

    norm = safe_sqrt(qz.q1*qz.q1+qz.q2*qz.q2+qz.q3*qz.q3+qz.q4*qz.q4);
    qz1 = qz.q1/norm;
    qz2 = qz.q2/norm;
    qz3 = qz.q3/norm;
    qz4 = qz.q4/norm;


    r1 = q.q1*qz4-q.q2*qz3+q.q3*qz2-q.q4*qz1;
    r2 = q.q1*qz3+q.q2*qz4-q.q3*qz1-q.q4*qz2;
    r3 = -q.q1*qz2+q.q2*qz1+q.q3*qz4-q.q4*qz3;
    r4 = q.q1*qz1+q.q2*qz2+q.q3*qz3+q.q4*qz4;

    norm = safe_sqrt(r1*r1+r2*r2+r3*r3+r4*r4);
    r1 = r1/norm;
    r2 = r2/norm;
    r3 = r3/norm;
    r4 = r4/norm;

    Quaternion_D ret(r1,r2,r3,r4);

    return ret;


}
