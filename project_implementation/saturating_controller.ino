#include <quaternion_diogo.h>
#include <saturating_controller.h>


/*
* Lambda function: Saturation that vanishes close to PI
*/
float lambda_f(float up,float low,float val)
{

	if(val >= 0 && val <= low)
		return val;
	else if(val>low && val <= up)
		return low;
	else if(val>up && val <= PI)
		return low*(val-PI)/(up-PI);
	else
		return 0; // safeguard?

}

/*
* Integral lambda function: Computes the integral of the lambda function
*/
float integral_lambda_f(float up, float low, float val)
{
	if(val >= 0 && val <= low)
		return 0.5f*val*val;
	else if(val > low && val <= up)
		return 0.5f*low*low+low*(val-low);
	else if(val > up && val <= PI)
		return 0.5f*low*low+low*(up-low)+0.5f*low*(val*val-up*up)/(up-PI)-PI*low*(val-up)/(up-PI);
	else
		return 0;
}
/*
* Xi function: Linear interpolation between f1 and f2
*/
float xi_f(float up, float low, float f1, float f2, float val)
{
	if(val <= low)
		return f1;
	else if(val > low && val <= up)
		return (up-val)/(up-low)*f1 + (val-low)/(up-low)*f2;
	else
		return f2;
}

/*
* Double Xi function: Linear interpolation from f1 to f2 and back to f1
*/
inline float double_xi_f(float up1, float up2, float low1, float low2,float f1, float f2,float val)
{
	return xi_f(low2,low1,f1,xi_f(up2,up1,f2,f1,val),val);
}

/*
* Safe Arccos: Computs the arc cosine of a value, dealing with eventual rounding errors on x
*/

float safe_acos (float x)
  {
	  if (x < -1.0) 
	  	x = -1.0 ;
	  else if (x > 1.0) 
	  	x = 1.0 ;

	  return acos (x) ;
  }

/*
* Compute Phi: Returns the displacement angle of the thrust direction
*/
inline float compute_phi(float qp)
{
	return 2*safe_acos(qp);
}

/*
* Compute Theta: Returns the displacement angle of the yaw angle
*/
inline float compute_theta(float qw)
{
	return 2*safe_acos(qw);
}

/*
*	Compute artificial torques: computes the artificial torques vector for the control signal
*/
Vector3<float> compute_art_torques(Quaternion_D * qxy, Quaternion_D * qz,float phi, float theta)
{
	float qx=qxy->q1,qy=qxy->q2,qzz=qz->q3,qp=qxy->q4,qw=qz->q4;
	float A=0,B=0,T_1=0,T_2=0,T_3=0;

	if(qp!=1)
		A=c_phi*lambda_f(phi_up,phi_low,phi)/safe_sqrt(1-qp*qp)-qp*qp*qp*c_theta*integral_lambda_f(theta_up,theta_low,theta);
	else
		A=0;


	if(qw!=1)
		B=qzz*qp*qp*qp*c_theta*lambda_f(theta_up,theta_low,theta)/safe_sqrt(1-qw*qw);
	else
		B=0;
	

	T_1 = qx*A+B*qy;
	T_2 = qy*A-B*qx;
	T_3 = B*qp;

	Vector3<float> T(T_1,T_2,T_3);

	return T;
}

/*
*	Compute Phi dot: Computes the time derivative of the displacement angle of the thurst axis
*/
float compute_phi_dot(Quaternion_D * qxy,Vector3<float> * omega_f)
{
	float qx=0,qy=0,qp=0,wx=0,wy=0;

	qx = qxy->q1;
	qy = qxy->q2;
	qp = qxy->q4;
	wx = omega_f->x;
	wy = omega_f->y;

	if(qp!=1)
		return -(qx*wx+qy*wy)/safe_sqrt(1-qp*qp);
	else
		return 0;


}

/*
*	Compute Theta dot: Computes the time derivative of the yaw error angle
*/
float compute_theta_dot(Quaternion_D * qxy, Quaternion_D *qz, Vector3<float> *omega_f)
{
	float qx=0,qy=0,qzz=0,qp=0,qw=0,wx=0,wy=0,wz=0,A=0;

	qx = qxy->q1;
	qy = qxy->q2;
	qzz = qz->q3;
	qp = qxy->q4;
	qw = qz->q4;
	wx = omega_f->x;
	wy = omega_f->y;
	wz = omega_f->z;

	if(qw!=1)
		A = qzz/safe_sqrt(1-qw*qw);
	else
		A=0;

	if(qp!=1)
		return -A*qy*wx/qp + A*qx*wy/qp - A*wz;

	return -A*wz;

}

float compute_theta_dot_hole(Quaternion_D *qz, Vector3<float> *omega_f)
{
	float qzz=0,qw=0,wz=0;

	qzz = qz->q3;
	qw = qz->q4;
	wz = omega_f->z;

	if(qw!=1)
		return -qzz*wz/safe_sqrt(1-qw*qw);

	return 0;

}


/*
*	Compute Switch curve Phi: Computes the switch curve between accelerating and decelerating damping on the thrust axis movement
*/
inline float compute_switch_curve_phi(float phi)
{
	return -safe_sqrt(v_phi_max*v_phi_max-2*torque_xy_max*(phi_low-phi)/J_x);
}

/*
*	Compute Switch curve Theta: Computes the switch curve between accelerating and decelerating damping on the yaw
*/
inline float compute_switch_curve_theta(float theta)
{
	return -safe_sqrt(v_theta_max*v_theta_max-2*torque_z_max*(theta_low-theta)/J_z);
}

/*
* Computes the 'available torque' in the phi direction (discounting the torque used by theta movement)
*/
float compute_torque_phi(Quaternion_D *qxy,float phi, float theta,Vector3<float> *omega_f)
{
	float qx=0,qy=0,qp=0,A=0,B=0;

	qx = qxy->q1;
	qy = qxy->q2;
	qp = qxy->q4;

	if (qp!=1)
		A = c_phi * lambda_f(phi_up,phi_low,phi)/safe_sqrt(1-qp*qp);
	else
		A = 0;

	B = -c_theta * qp*qp*qp *integral_lambda_f(theta_up,theta_low,theta);

	return safe_sqrt(A*qx*A*qx+A*qy*A*qy)-safe_sqrt(B*qx*B*qx+B*qy*B*qy);
}

/*
* Computes the torque in the yaw direction
*/
float compute_torque_z(Quaternion_D * qxy, Quaternion_D * qz,float theta,Vector3<float> * omega_f)
{
	float qzz=0, qw=0,qp=0;

	qp = qxy->q4;
	qzz = qz->q3;
	qw = qz->q4;

	if (qw!=1)
		return safe_sqrt(sq(qzz*qp*qp*qp*qp*c_theta*lambda_f(theta_up,theta_low,theta)/safe_sqrt(1-qw*qw)));

	return 0;

}

/*
* Computes the accelerating damping on the thrust axis
*/
float compute_acc_damping_phi(float phi_dot, float T_phi)
{
	if(phi_dot > v_phi)
		return (-T_phi+torque_xy_max)/phi_dot;
	
	if(phi_dot > 0 && phi_dot <= v_phi)
		return (-T_phi+torque_xy_max)/v_phi;

	return 0;
}

/*
* Computes the decelerating damping on the thrust axis
*/
inline float compute_dec_damping_phi(float phi_dot, float T_phi)
{
	return -(T_phi+torque_xy_max)/phi_dot;
}

/*
* Computes the star damping gain on the thrust axis
*/
inline float compute_star_damping_phi(float phi_dot,float dec_phi, float acc_phi, float switch_phi)
{
	return xi_f(r_phi*switch_phi,switch_phi,dec_phi,acc_phi,phi_dot);
}

/*
* Computes the final damping gain on the thrust axis
*/
float compute_damping_phi(Quaternion_D * qxy,Quaternion_D * qz, Vector3<float> * omega_f,float phi,float theta)
{
	float phi_dot=0,switch_phi=0,T_phi=0, phi_star = 0,phi_dec = 0,phi_acc = 0;

	phi_dot = compute_phi_dot(qxy,omega_f);
	switch_phi = compute_switch_curve_phi(phi);
	T_phi = compute_torque_phi(qxy,phi,theta,omega_f);
	phi_dec = compute_dec_damping_phi(phi_dot,T_phi);
	phi_acc = compute_acc_damping_phi(phi_dot,T_phi);
	phi_star = compute_star_damping_phi(phi_dot,phi_dec,phi_acc,switch_phi);

	//hal.console->printf_P(PSTR("phi:[%.7f,%.7f,%.7f,%.7f,%.7f]\r\n"),phi_dec,phi_star,phi_acc,phi_dot,T_phi);

	return double_xi_f(phi_up-delta_phi,phi_up,phi_low,phi_low+delta_phi,small_delta_phi,phi_star,phi);
}

float compute_acc_damping_z(float theta_dot_hole, float T_z)
{
	if(theta_dot_hole > v_theta)
		return (-T_z+torque_z_max)/theta_dot_hole;
	
	if(theta_dot_hole > 0 && theta_dot_hole <= v_theta)
		return (-T_z+torque_z_max)/v_theta;

	return 0;
}

float compute_dec_damping_z(float theta_dot_hole, float T_z)
{
	if(theta_dot_hole < -v_theta)
		return (-T_z-torque_z_max)/theta_dot_hole;
	
	if(theta_dot_hole < 0 && theta_dot_hole >= -v_theta)
		return (-T_z-torque_z_max)/v_theta;

	return 0;
}
float compute_star_damping_z(float theta_dot,float theta_dot_hole,float dec_theta, float acc_theta, float switch_theta)
{
	return xi_f(r_theta*switch_theta,switch_theta,dec_theta,acc_theta,theta_dot);
}
float compute_damping_z(Quaternion_D * qxy,Quaternion_D * qz, Vector3<float> * omega_f,float phi,float theta)
{
	float theta_dot=0,theta_dot_hole=0,switch_theta=0,T_z=0, d_star_z=0 ,d_dec_z=0, d_acc_z=0, d_xi=0;

	theta_dot = compute_theta_dot(qxy,qz,omega_f);
	theta_dot_hole = compute_theta_dot_hole(qz,omega_f);
	switch_theta = compute_switch_curve_theta(theta);
	T_z = compute_torque_z(qxy, qz,theta, omega_f);

	d_dec_z = compute_dec_damping_z(theta_dot_hole, T_z);
	d_acc_z = compute_acc_damping_z(theta_dot_hole, T_z);
	d_star_z = compute_star_damping_z(theta_dot,theta_dot_hole,d_dec_z,d_acc_z,switch_theta);
	d_xi = double_xi_f(theta_up-delta_theta,theta_up,theta_low,theta_low+delta_theta,small_delta_z,d_star_z,theta);



	return xi_f(phi_up,phi_up-delta_phi,d_xi,small_delta_z,phi);
}

/*
*	Computes the gain that allows to saturate the xy torque
*/
float compute_kxy(Vector3<float> * art_torques,Vector3<float> * omega_f, Quaternion_D * qxy,float d_phi)
{
	float qx=0, qy=0, qp=0, A=0, D1=0, D2=0, D3=0, D4=0, a=0, b=0, c=0, root = 0, k = 0;

	qx = qxy->q1;
	qy = qxy->q2;
	qp = qxy->q4;

	if(qp!=1)
		A=1/(1-qp*qp);
	else
		A=0;

	D1 = A*(d_phi*qx*qx+d_ortho*qy*qy);
	D2 = A*(qx*qy*(d_phi-d_ortho));
	D3 = D2;
	D4 = A*(d_phi*qy*qy+d_ortho*qx*qx);

	a = sq(D1*omega_f->x+D2*omega_f->y)+sq(D3*omega_f->x+D4*omega_f->y);
	b = -2*(art_torques->x*(D1*omega_f->x+D2*omega_f->y)+art_torques->y*(D3*omega_f->x+D4*omega_f->y));
	c = art_torques->x*art_torques->x+art_torques->y*art_torques->y-torque_xy_max*torque_xy_max;

	root = b*b-4*a*c;

	if(root < 0 || a == 0)
		return 1.0f;

	if(root > b*b)
		k = (-b+safe_sqrt(root))/(2*a);
	else
		k = (-b-safe_sqrt(root))/(2*a);

	if (k < 0 || k > 1)
		return 1;

	return k;
}

/*
*	Computes the gain that allows to saturate the z control torque
*/
float compute_kz(Vector3<float> * art_torques,Vector3<float> * omega_f,float d_z)
{
	float T_z = 0, wz = 0, a=0, b=0, c=0, root=0, k=0;

	T_z = art_torques->z;
	wz = omega_f->z;

	a = d_z*wz*d_z*wz;
	b = -2*T_z*d_z*wz;
	c = T_z*T_z-torque_z_max*torque_z_max;

	root = b*b-4*a*c;

	if(root < 0 || a ==0)
		return 1;

	if(root > b*b)
		k = (-b+safe_sqrt(root))/(2*a);
	else
		k = (-b-safe_sqrt(root))/(2*a);

	if (k < 0 || k > 1)
		return 1;

	//hal.console->printf_P(PSTR("k_z: [%.7f]\r\n"),k);
	return k;
}	

/*
*	Computes the damping matrix
*/
Matrix3<float> compute_D_matrix(Quaternion_D *qxy, Quaternion_D *qz, Vector3<float> *omega_f, Vector3<float> *art_torques)
{
	float qx=0, qy=0, qzz=0, qp=0, qw=0, wx=0, wy=0, wz=0, d_phi=0, d_z=0, k_xy=0, k_z=0,d1=0,d2=0,d3=0,d4=0,d5=0,d6=0,d7=0,d8=0,d9=0;
	float A=0, B=0;

	qx = qxy->q1;
	qy = qxy->q2;
	qzz = qz->q3;
	qp = qxy->q4;
	qw = qz->q4;

	wx = omega_f->x;
	wy = omega_f->y;
	wz = omega_f->z;

	d_phi = compute_damping_phi(qxy,qz,omega_f,compute_phi(qp),compute_theta(qw));
	d_z = compute_damping_z(qxy,qz,omega_f,compute_phi(qp),compute_theta(qw));

	k_xy = compute_kxy(art_torques,omega_f,qxy,d_phi);
	k_z = compute_kz(art_torques,omega_f,d_z);


	if(qp!=1){
		A = k_xy*d_phi/(1-qp*qp);
		B = k_xy*d_ortho/(1-qp*qp);
	}else{
		A = 0;
		B = 0;
	}

	d1 = A*qx*qx+B*qy*qy;
	d2 = qx*qy*(A-B);
	d4 = d2;
	d5 = A*qy*qy+B*qx*qx;
	d9 = k_z*d_z;

	

	Matrix3<float> ret(d1,d2,d3,d4,d5,d6,d7,d8,d9);

	return ret;

}

/*
*	Implements the fast and saturating attitude controller
*/
Vector3<float> fast_and_saturating_controller( Quaternion_D *current_att,Quaternion_D *desired_att, Vector3<float> *omega_f,Matrix3<float> * D_trig, Vector3<float> * T_trig,Quaternion_D *ret_qxy,Quaternion_D *ret_qz)
{
	Quaternion_D qe,qz,qxy;

	qe = mult_quat(current_att->conjugate(),*desired_att); // attitude error
	qe.sign_l();
	qz = qe.get_z();
	qxy = mult_quat_inv(qe,qz);

	*ret_qxy = qxy;
	*ret_qz = qz;
	//hal.console->printf_P(PSTR("qe:[%.7f,%.7f,%.7f,%.7f]\r\n"),qe.q1,qe.q2,qe.q3,qe.q4);
	//hal.console->printf_P(PSTR("qz:[%.2f,%.2f,%.7f,%.7f]\r\n"),qz.q1,qz.q2,qz.q3,qz.q4);
	//hal.console->printf_P(PSTR("qxy:[%.7f,%.7f,%.2f,%.7f]\r\n"),qxy.q1,qxy.q2,qxy.q3,qxy.q4);
  	*T_trig = compute_art_torques(&qxy,&qz,compute_phi(qxy.q4),compute_theta(qz.q4));
  	*D_trig = compute_D_matrix(&qxy,&qz,omega_f,T_trig);

  	 

  	return (*T_trig)-(*D_trig)*(*omega_f);
}

/*
*
*/
float map_f(float x, float in_min, float in_max, float out_min, float out_max)
{
	if(x > in_max)
		x = in_max;
	if(x < in_min)
		x = in_min;

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	float ret = 0;

	if(x > in_max)
		x = in_max;
	if(x < in_min)
		x = in_min;

  	ret = round((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);

  	if (ret > out_max)
  		ret = out_max;
  	if (ret < out_min)
  		ret = out_min;

  	return ret;
}

/*
*	Converts from Thrust + Torques to motor inputs
*/
void to_motors(float Thrust, Vector3<float> torques, uint16_t * u1,uint16_t * u2,uint16_t * u3,uint16_t * u4)
{
	float u1f = 0, u2f = 0, u3f = 0, u4f = 0;
	float offx = 0.25, offy = -0.1, offz = 0, offset = 0;

	u1f = 0.5/(arm*c_T)*(torques.y) - 0.25/(c_D)*(torques.z) + 0.25/(c_T)*(Thrust) - (2*c_D*offy - c_T*arm*offz)/(4*c_D*c_T*arm);
	u2f = -0.5/(arm*c_T)*(torques.y) - 0.25/(c_D)*(torques.z) + 0.25/(c_T)*(Thrust) + (2*c_D*offy + c_T*arm*offz)/(4*c_D*c_T*arm);
	u3f = 0.5/(arm*c_T)*(torques.x) + 0.25/(c_D)*(torques.z+offset) + 0.25/(c_T)*(Thrust) - (2*c_D*offx + c_T*arm*offz)/(4*c_D*c_T*arm);
	u4f = -0.5/(arm*c_T)*(torques.x) + 0.25/(c_D)*(torques.z+offset) + 0.25/(c_T)*(Thrust) + (2*c_D*offx - c_T*arm*offz)/(4*c_D*c_T*arm);

	if(u1f > U_MAX)
		u1f = 0;
	else if (u1f < U_MIN)
		u1f = 0;

	if(u2f > U_MAX)
		u2f = 0;
	else if (u2f < U_MIN)
		u2f = 0;


	if(u3f > U_MAX)
		u3f = 0;
	else if (u3f < U_MIN)
		u3f = 0;

	if(u4f > U_MAX)
		u4f = 0;
	else if (u4f < U_MIN)
		u4f = 0;


	*u1 = (uint16_t) u1f+1000; // to radio values
	*u2 = (uint16_t) u2f+1000; // to radio values
	*u3 = (uint16_t) u3f+1000; // to radio values
	*u4 = (uint16_t) u4f+1000; // to radio values
}

/*
* Triggering Function 1 : Evaluates the state of the system and states if a triggering event should occur
*
*	Function 1 computes w^T T(q_k) + safe_sqrt(sq(w)*)abs(T^\star(q)) - w_hat'*D_k*w_k <= w_k'*D_k*w_k
*/
bool trig_func1( Quaternion_D q_b, Quaternion_D q_d, Vector3<float> omega, Matrix3<float> D_k, Vector3<float> omega_k, Vector3<float> T_k, float v_dot_k, float * alpha,Vector3<float> * T_star)
{
	float v_dot_e = 0;
	float qx=0, qy=0, qzz=0, qp=0, qw=0,qp2=0, A=0, B=0, C=0, phi = 0, theta =0, T1=0, T2=0, T3=0 ;
	Vector3<float> omega_e;
	Quaternion_D qe, qxy, qz;

	omega_e = omega - omega_k;
	
	qe = mult_quat(q_b.conjugate(),q_d);
	qe.sign_l();
	qz = qe.get_z();
	qxy = mult_quat_inv(qe,qz);

	qx = qxy.q1;
	qy = qxy.q2;
	qzz = qz.q3;
	qp = qxy.q4;
	qw = qz.q4;
	qp2 = qp*qp;

	phi = 2*safe_acos(qp);
	theta = 2*safe_acos(qw);

	if(qp!=1)
		A = c_phi*lambda_f(phi_up,phi_low,phi)/safe_sqrt(1-qp2);

	if(qw!=1)
		B = safe_sqrt(sq(qzz*qp2*qp))*c_theta*lambda_f(theta_up,theta_low,theta)/safe_sqrt(1-qw*qw);

	C = safe_sqrt(sq(qp2*qp))*c_theta*theta*theta_low;

	
	T1 = safe_sqrt(sq(qx))*A + safe_sqrt(sq(qx))*C + safe_sqrt(sq(qy))*B;

	T2 = safe_sqrt(sq(qy))*A + safe_sqrt(sq(qy))*C + safe_sqrt(sq(qx))*B;

	T3 = safe_sqrt(sq(qp))*B;

	//hal.console->printf_P(PSTR("[%.7f]\r\n"),T_1);

	T_star->x = T1;
	T_star->y = T2;
	T_star->z = T3;

	v_dot_e = (omega_e * D_k) * omega_k;

	(*alpha) = (omega) * (T_k) + safe_sqrt(omega*omega)*safe_sqrt((*T_star)*(*T_star)) - v_dot_e;

	if (*alpha > -alpha_1*v_dot_k){
		//hal.console->printf_P(PSTR("[%.7f,%.7f]\r\n"),alpha,v_dot_k);
		return true;
	}
	
	return false;

}

/*
* Compute linearized torques: computes the linearization of T(q) around the current attitude 
*/
void compute_lin_torques(Quaternion_D q_b, Quaternion_D q_d,Quaternion_D * q_k,Vector3<float> * T_x, Vector3<float> * T_y,Vector3<float> * T_z,Vector3<float> * T_p,Vector3<float> * T_w)
{
	
	float A=0, B=0, C=0, D=0, E=0, F=0, G=0;
	float qx=0, qy=0, qzz=0, qp=0, qw=0;
	float phi = 0, theta = 0;
	Quaternion_D qxy, qz, qe;
	float lambda_phi = 0, lambda_theta =0, qp3 = 0, qp2 = 0;


	qe = mult_quat(q_b.conjugate(),q_d);
	qe.sign_l();
	qz = qe.get_z();
	qxy = mult_quat_inv(qe,qz);

	qx = qxy.q1;
	qy = qxy.q2;
	qzz = qz.q3;
	qp = qxy.q4;
	qp2 = qp*qp;
	qp3 = qp2*qp;
	qw = qz.q4;


	*q_k = qe;

	phi = 2*safe_acos(qp);
	theta = 2*safe_acos(qw);

	lambda_phi = lambda_f(phi_up,phi_low,phi);
	lambda_theta = lambda_f(theta_up,theta_low,theta);

	

	A = safe_sqrt(1-qp2);
	B = safe_sqrt(1-qw*qw);
	C = c_theta * integral_lambda_f(theta_up,theta_low,theta);
	if(A!=0)
		D = c_phi / A;
	if(B!=0)
		E = c_theta/B;

	F = safe_acos(qw);
	G = safe_acos(qp);

	T_x->x = D*lambda_phi - qp3*C;
	T_x->y = -qzz*qp3*E*lambda_theta;
	T_x->z = 0;

	T_y->x = -T_x->y;
	T_y->y = T_x->x;
	T_y->z = 0;

	T_z->x = qy*qp3*E*lambda_theta;
	T_z->y = -qx*qp3*E*lambda_theta;
	T_z->z = qp*qp3*E*lambda_theta;

	if(phi >= 0 && phi <= phi_low){
		float parc2 = qzz*qp2*E*lambda_theta;
		if(A != 0){
			float parc1 = -2*D/A + 2*qp*D*G/(A*A) - 3*qp2*C;

			T_p->x = parc1*qx + 3*qy*parc2;
			T_p->y = parc1*qy - 3*qx*parc2;
			T_p->z = 4*qp*parc2;
		}else{
			float parc1 = - 3*qp2*C;

			T_p->x = parc1*qx + 3*qy*parc2;
			T_p->y = parc1*qy - 3*qx*parc2;
			T_p->z = 4*qp*parc2;
		}

	}else if(phi> phi_low && phi <= phi_up){
		float parc2 = qzz*qp2*E*lambda_theta;
		if(A != 0){
			float parc1 = phi_low*D*qp/(A*A) - 3*qp2*C;
			T_p->x = parc1*qx + 3*qy*parc2;
			T_p->y = parc1*qy - 3*qx*parc2;
			T_p->z = 4*qp*parc2;
		}else{
			float parc1 = - 3*qp2*C;
			T_p->x = parc1*qx + 3*qy*parc2;
			T_p->y = parc1*qy - 3*qx*parc2;
			T_p->z = 4*qp*parc2;
		}
	}else{
		float parc2 = qzz*qp2*E*lambda_theta;
		if(A != 0){
			float parc1 = -2*D*phi_low/(A*(phi_up-PI)) + phi_low*D*qp*(2*G-PI)/(A*A*(phi_up-PI)) - 3*qp2*C;
			T_p->x = parc1*qx + 3*qy*parc2;
			T_p->y = parc1*qy - 3*qx*parc2;
			T_p->z = 4*qp*parc2;
		}else{
			float parc1 = - 3*qp2*C;
			T_p->x = parc1*qx + 3*qy*parc2;
			T_p->y = parc1*qy - 3*qx*parc2;
			T_p->z = 4*qp*parc2;
		}
	}

	if(theta >= 0 && theta <= theta_low){

		if(B!=0){
			T_w->x = 4*qp3*qx*E*F + 2*qw*qzz*qp3*qy*E*F/(B*B) - 2*qzz*qp3*qy*E/B;
			T_w->y = 4*qp3*qy*E*F - 2*qw*qzz*qp3*qx*E*F/(B*B) + 2*qzz*qp3*qx*E/B;
			T_w->z = 2*qzz*qp3*qp*qw*E*F/(B*B) - 2*qzz*qp3*qp*E/B;
		}else{
			T_w->x = 4*qp3*qx*E*F;
			T_w->y = 4*qp3*qy*E*F;
			T_w->z = 0;
		}
	}else if ( theta > theta_low && theta <= theta_up){

		if(B!=0){
			T_w->x = 2*theta_low*qp3*qx*E + theta_low*qw*qzz*qp3*qy*E/(B*B);
			T_w->y = 2*theta_low*qp3*qy*E - theta_low*qw*qzz*qp3*qx*E/(B*B);
			T_w->z = theta_low*qzz*qp3*qp*qw*E/(B*B);
		}else{
			T_w->x = 2*theta_low*qp3*qx*E;
			T_w->y = 2*theta_low*qp3*qy*E;
			T_w->z = 0;
		}

	}else{

		if(B!=0){
			T_w->x = -2*qp3*qx*theta_low*E*(PI-2*F)/(theta_up-PI) - 2*qzz*qp3*qy*theta_low*E/(B*(theta_up-PI)) + qzz*qp3*qw*qy*theta_low*(2*F-PI)*E/(B*B*(theta_up-PI));
			T_w->y = -2*qp3*qy*theta_low*E*(PI-2*F)/(theta_up-PI) + 2*qzz*qp3*qx*theta_low*E/(B*(theta_up-PI)) - qzz*qp3*qw*qx*theta_low*(2*F-PI)*E/(B*B*(theta_up-PI));
			T_w->z = -2*qzz*qp3*qp*E*theta_low/(B*(theta_up-PI)) - qzz*qp*qp3*qw*E*theta_low*(PI-2*F)/(B*B*(theta_up-PI));
		}else{
			T_w->x = -2*qp3*qx*theta_low*E*(PI-2*F)/(theta_up-PI);
			T_w->y = -2*qp3*qy*theta_low*E*(PI-2*F)/(theta_up-PI);
			T_w->z = 0;
		}

	}

	return;
}

/*
* Triggering Function 2 : Evaluates the state of the system and states if a triggering event should occur
*
*	Function 2 computes a linearization of T(q)
*/
bool trig_func2(Vector3<float> omega, Matrix3<float> D_k, Vector3<float> omega_k,Vector3<float> lin_T[5], float v_dot_k, Quaternion_D q_b, Quaternion_D q_d, Quaternion_D q_k, Vector3<float> * dT, float * debug)
{

	float alpha = 0;
	float v_dot_e = 0;
	float dx=0, dy=0, dz=0, dp=0, dw=0;
	Vector3<float> omega_e;
	Quaternion_D qe, qz, qxy, qzk, qxyk;


	qe = mult_quat(q_b.conjugate(),q_d);
	qe.sign_l();
	qz = qe.get_z();
	qxy = mult_quat_inv(qe,qz);

	qzk = q_k.get_z();
	qxyk = mult_quat_inv(q_k,qzk);

	dx = qxy.q1 - qxyk.q1;
	dy = qxy.q2 - qxyk.q2;
	dz = qz.q3 - qzk.q3;
	dp = qxy.q4 - qxyk.q4;
	dw = qz.q4 - qzk.q4; 

	omega_e = omega - omega_k;

	v_dot_e = (omega_e * D_k) * omega_k;

	*dT = lin_T[0] * dx + lin_T[1] * dy + lin_T[2] * dz + lin_T[3] * dp + lin_T[4] * dw;

	alpha = -omega * (*dT) - v_dot_e;
	*debug = lin_T[0].x;



	if(alpha > -alpha_2*v_dot_k)
		return true;
	else
		return false;
}

/*
* Triggering Function 3 : Evaluates the state of the system and states if a triggering event should occur
*
*	Function 3 implements the heuristic strategy: if the state shifts more than a certain percentage, a triggering event occurs
*/
bool trig_func3(Vector3<float> omega,Vector3<float> omega_k, Quaternion_D qb,Quaternion_D qd, Quaternion_D qk)
{
	
	float norm = 0;
	Vector3<float> omega_e;
	Quaternion_D qe;

	qe = mult_quat(qb.conjugate(),qd);
	qe.sign_l();
	omega_e = omega - omega_k;

	norm = safe_sqrt(sq(qe.q1-qk.q1)+sq(qe.q2-qk.q2)+sq(qe.q3-qk.q3)+sq(qe.q4-qk.q4)+sq(omega_e.x)+sq(omega_e.y)+sq(omega_e.z));

	

	if(norm > alpha_3)
		return true;
	else
		return false;
}