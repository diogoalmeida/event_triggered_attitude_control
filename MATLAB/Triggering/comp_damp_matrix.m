function [ D phi_dot theta_dot switch_phi switch_theta] = comp_damp_matrix(qx,qy,qz,qp,qw,w, T_z, T_phi, art_torques, var)
%COMP_DAMP_MATRIX Given the state [qx qy qz qp qw w], computes the damping
%matrix D to be used in the saturating controller.
    
    
    phi = 2 * acos(qp);
    theta = 2 * acos(qw);
    
    if qp ~= 1
        
        phi_dot = -(qx*w(1) + qy*w(2)) / sqrt(1-qp^2);
        
    else
        
        phi_dot = 0;
        
    end
    
    if qw ~= 1 && qp ~= 0
        
        theta_dot = -(qz*qy*w(1) - qz*qx*w(2)) / (qp * sqrt(1-qw^2)) - qz*w(3) / sqrt(1-qw^2);
    
    else
        
        theta_dot = 0;
        
    end
    
    if qw ~=1
        
        theta_dot_hole = -qz*w(3) / sqrt(1-qw^2);
        
    else
        
        theta_dot_hole = 0;
        
    end
    
    
    % compute damping constants for the yaw angle
    
    if theta_dot_hole < - var.v_theta
        
        d_z_dec = (-T_z - var.tau_z) / theta_dot_hole;
        
    else if theta_dot_hole < 0 && theta_dot_hole >= -var.v_theta
            
            d_z_dec = (-T_z - var.tau_z) / var.v_theta;
            
        else
            
            d_z_dec = 0;
            
        end
        
    end
    
    
    if theta_dot_hole > var.v_theta
        
        d_z_acc = (-T_z + var.tau_z) / theta_dot_hole;
        
    else if theta_dot_hole > 0 && theta_dot_hole <= var.v_theta
            
            d_z_acc = (-T_z + var.tau_z) / var.v_theta;
            
        else
            
            d_z_acc = 0;
            
        end
        
    end
    
    switch_theta = - sqrt (var.v_theta_max^2 - 2*(var.tau_z*(var.theta_low-theta))/var.J_z);
    
    d_star_z = xi_function(var.r_theta*switch_theta,switch_theta,theta_dot,d_z_dec,d_z_acc);
    
    d_z = xi_function(var.phi_up,var.phi_up-var.delta_phi,phi,double_xi_function(var.theta_up-var.delta_z,var.theta_up,var.theta_low,var.theta_low+var.delta_z,theta,var.small_delta_z,d_star_z),var.small_delta_z);
    
    
    % compute damping constants for the thrust vector displacement angle
    
    d_phi_dec = -(T_phi + var.tau_xy) / phi_dot;
    
    if phi_dot > var.v_phi
        
        d_phi_acc = (-T_phi + var.tau_xy) / phi_dot;
        
    else if phi_dot > 0 && phi_dot <= var.v_phi
            
            d_phi_acc = (-T_phi + var.tau_xy) / var.v_phi;
            
        else
            
            d_phi_acc = 0;
            
        end
        
    end
    
    switch_phi = -sqrt(var.v_phi_max^2-2*(var.tau_xy*(var.phi_low-phi))/var.J_x);
    
    d_phi_star = xi_function(var.r_phi*switch_phi,switch_phi,phi_dot,d_phi_dec,d_phi_acc);
    
    d_phi = double_xi_function(var.phi_up-var.delta_phi,var.phi_up,var.phi_low,var.phi_low+var.delta_phi,phi,var.small_delta_phi,d_phi_star);
    
    
    % Compute saturating gains
    
    T_xy = art_torques(1:2);
    T_z = art_torques(3);
    
    if qp ~= 1
        
        D_xy = (d_phi*[qx^2 qx*qy; qx*qy qy^2] + var.small_delta_phi * [qy^2 -qx*qy; -qx*qy qx^2]) ./ (1-qp^2);
        
    else
        
        D_xy = zeros(2,2);
        
    end
    
    
    vec = D_xy*w(1:2);
    
    a = vec(1)^2+vec(2)^2;
    b = -2*T_xy(1)*vec(1) - 2*T_xy(2)*vec(2);
    c = T_xy(1)^2+T_xy(2)^2-var.tau_xy^2;
    
    sq = b^2-4*a*c;

    if  sq < 0 || a == 0
        
        k_xy = 1;

    else if sq > b^2

            k_xy = (-b+sqrt(sq))/(2*a);

        else

            k_xy = (-b-sqrt(sq))/(2*a);

        end

    end

    if k_xy > 1

        k_xy = 1;

    else if k_xy < 0

            k_xy = 1;

        end

    end
    
    
    a = (d_z*w(3))^2;
    b = (-2*T_z*d_z*w(3));
    c = T_z^2 - var.tau_z^2;
    sq = b^2-4*a*c;
 
    
   if sq < 0 || a == 0
           
           k_z = 1;
           
       else if sq > b^2
               
               k_z = (-b + sqrt(sq))/(2*a);
               
           else if sq <= b^2
                   
                   k_z = (-b - sqrt(sq))/(2*a);
                   
               end
               
           end
           
       end
       
       if k_z > 1
           
           k_z = 1;
           
       else if k_z < 0
               
               k_z = 0;
               
           end
           
       end
    
    D = [k_xy*D_xy zeros(2,1); zeros(1,2) k_z*d_z];
    
    
    
    
end

