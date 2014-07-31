function [ art_torques, T_z, T_phi ] = comp_art_torques( qx,qy,qz,qp,qw, var)
%COMP_ART_TORQUES Computes the artificial torques used by the saturating
%controller. Given the error elements qx,qy,qz,qp and qw, computes the artificial torques along the three frame axis.
    
    
    phi = 2*acos(qp);
    theta = 2*acos(qw);
    
    
    if qp~=1
        
        A = var.c_phi * delta_function(var.phi_up,var.phi_low,phi) / sqrt(1-qp^2);
        
    else
        
        A = 0;
        
    end
    
    B = qp^3 * var.c_theta * delta_integral(var.theta_up,var.theta_low,theta);
    
    if qw ~=1
        
        C = qz * qp^3 * var.c_theta * delta_function(var.theta_up,var.theta_low,theta) / sqrt(1-qw^2);
        
    else
        
        C = 0;
        
    end
    
    
    art_torques = [(A-B)*qx + C*qy; (A-B)*qy - C*qx; C*qp];
    T_z = abs(C*qp);
    
    if qp~=1
        
        T_phi_phi = var.c_phi * delta_function(var.phi_up,var.phi_low,phi) / sqrt(1-qp^2) * [qx;qy;0];
        
    else
        
        T_phi_phi = 0;
        
    end
    
    T_theta_phi = - var.c_theta * qp^3 * delta_integral(var.theta_up,var.theta_low,theta) * [qx; qy; 0];
    
    
    T_phi = norm(T_phi_phi)-norm(T_theta_phi);
    
        

end

