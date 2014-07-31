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

// Copyright 2014 Diogo Almeida - diogormalmeida@gmail.com, all rights reserved.

// Implements functions required for the fast and saturating controller

// Required variables

#ifndef SATURATING_CONTROLLER
#define SATURATING_CONTROLLER

#include <quaternion_diogo.h>
#include <math.h>
#include <AP_Math.h>

#define phi_multiplier 1
#define torque_multiplier (1)

#define phi_low ((10*phi_multiplier*PI)/180)
#define theta_low ((15*PI)/180)
#define c_phi (0.817/(torque_multiplier*phi_multiplier))//0.817
#define c_theta 0.109
#define delta_phi ((5*PI)/180)
#define delta_theta ((5*PI)/180)
#define small_delta_phi (0.1999)
#define small_delta_z (0.0961)
#define d_ortho small_delta_phi
#define phi_up (175*PI/180)
#define theta_up (175*PI/180)
#define r_phi 0.5//0.75
#define r_theta 0.5
#define v_phi_max 1.425
#define v_theta_max 0.624
#define v_phi 0.1
#define v_theta 0.1
#define J_x 0.020232//0.020232
#define J_z 0.04//0.014

#define arm 0.3 // m
#define max_motor_thrust 0.575 //kg
#define c_T (0.0007*9.80665)//0.00067941//(max_motor_thrust/1000) // max input -> max thrust [N/s]
#define c_D (c_T/10) // ??? [N.m/s]

#define torque_xy_max (0.15/torque_multiplier)//0.15// ??? [N.m]
#define torque_z_max 0.03 // ??? [N.m]

// Trigger params

#define alpha_1 1
#define alpha_2 0.01
#define alpha_3 0.01

// RC params
#define rT_min 1030
#define rT_max 1982
#define T_min 0
#define T_max 20

#define rR_min 1030
#define rR_max 1982
#define R_min  -45
#define R_max  45

#define rP_min 1030
#define rP_max 1982
#define P_min  -45
#define P_max  45

#define rY_min 1030
#define rY_max 1982
#define Y_min  -180
#define Y_max  180

#define U_MAX 982
#define U_MIN 82

#define high_thres 1500
#define safe_lim 25


float sign(float val){
   if(val > 0)
      return 1;
   else if (val == 0)
      return 0;
   else
      return -1;

}

float lambda_f(float up,float low,float val);
float integral_lambda_f(float up, float low, float val);
float xi_f(float up, float low, float f1, float f2, float val);
inline float double_xi_f(float up1, float up2, float low1, float low2,float f1, float f2,float val);

inline float compute_phi(float qp);
inline float compute_theta(float qw);

Vector3<float> compute_art_torques(Quaternion_D * qxy, Quaternion_D * qz,float phi, float theta);

float compute_phi_dot(Quaternion_D * qxy,Vector3<float> * omega_f);
float compute_theta_dot(Quaternion_D * qxy, Quaternion_D * qz, Vector3<float> * omega_f);

inline float compute_switch_curve_phi(float phi);
inline float compute_switch_curve_theta(float theta);

float compute_torque_phi(Quaternion_D * qxy, float phi, float theta,Vector3<float> * omega_f);
float compute_torque_z(Quaternion_D * qxy, Quaternion_D * qz,float theta,Vector3<float> * omega_f);

float compute_acc_damping_phi(float phi_dot, float T_phi);
inline float compute_dec_damping_phi(float phi_dot, float T_phi);
inline float compute_star_damping_phi(float phi_dot,float dec_phi, float acc_phi, float switch_phi);
float compute_damping_phi(Quaternion_D * qxy, Vector3<float> * omega_f,float phi,float theta);

float compute_acc_damping_z(float theta_dot_hole, float T_z);
float compute_dec_damping_z(float theta_dot_hole, float T_z);
float compute_star_damping_z(float theta_dot,float theta_dot_hole,float dec_theta, float acc_theta, float switch_theta);
float compute_damping_z(Quaternion_D * qxy,Quaternion_D * qz, Vector3<float> * omega_f,float phi,float theta);

float compute_kxy(Vector3<float> * art_torques,Vector3<float> * omega_f, Quaternion_D * qxy,float d_phi);
float compute_kz(Vector3<float> * art_torques,Vector3<float> * omega_f,float d_z);

Matrix3<float> compute_D_matrix(Quaternion_D * qxy, Quaternion_D * qz, Vector3<float> * omega_f, Vector3<float> * art_torques);

Vector3<float> fast_and_saturating_controller(Quaternion_D *current_att, Quaternion_D *desired_att, Vector3<float> *omega_f, Matrix3<float> * D_trig, Vector3<float> * T_trig,Quaternion_D *ret_qxy,Quaternion_D *ret_qz);

float map(float x, float in_min, float in_max, float out_min, float out_max);

void to_motors(float Thrust, Vector3<float> torques, uint16_t * u1,uint16_t * u2,uint16_t * u3,uint16_t * u4);

void compute_lin_torques(Quaternion_D q_b, Quaternion_D q_d,Quaternion_D * q_k,Vector3<float> * T_x, Vector3<float> * T_y,Vector3<float> * T_z,Vector3<float> * T_p,Vector3<float> * T_w );

bool trig_func1( Quaternion_D q_b, Quaternion_D q_d, Vector3<float> omega, Matrix3<float> D_k, Vector3<float> omega_k, Vector3<float> T_k, float v_dot_k, float * alpha,Vector3<float> * T_star);
bool trig_func2(Vector3<float> omega, Matrix3<float> D_k, Vector3<float> omega_k,Vector3<float> * lin_T, float v_dot_k, Quaternion_D q_b, Quaternion_D q_d, Quaternion_D q_k, Vector3<float> * dT, float * debug);
bool trig_func3(Vector3<float> omega,Vector3<float> omega_k, Quaternion_D qb,Quaternion_D qd, Quaternion_D qk);
#endif
