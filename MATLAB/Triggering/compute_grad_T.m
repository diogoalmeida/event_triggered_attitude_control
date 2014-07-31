function [ grad_T ] = compute_grad_T( qxy,qzz,var )
%COMPUTE_GRAD_T Computes the constant elements for the torque gradient
    
    qx = qxy(1);
    qy = qxy(2);
    qz = qzz(3);
    qp = qxy(4);
    qw = qzz(4);
    
    F = acos(qw);
    G = acos(qp);
    
    phi = 2*G;
    theta = 2*F;
    
    A = sqrt(1-qp^2);
    B = sqrt(1-qw^2);
    C = var.c_theta * delta_integral(var.theta_up,var.theta_low,theta);
    
    if A ~= 0
        D = var.c_phi / A;
    else
        D = 0;
    end
    
    if B ~= 0
        E = var.c_theta / B;
    else
        E = 0;
    end
    
    
    
    T_x = [D * delta_function(var.phi_up,var.phi_low,phi) - qp^3 * C;
           -qz * qp^3 * E * delta_function(var.theta_up,var.theta_low,theta);
           0];
    T_y = [qz * qp^3 * E * delta_function(var.theta_up,var.theta_low,theta);
           D * delta_function(var.phi_up,var.phi_low,phi) - qp^3 * C;
           0];
    T_z = [qy * qp^3 * E * delta_function(var.theta_up,var.theta_low,theta);
           -qx * qp^3 * E * delta_function(var.theta_up,var.theta_low,theta);
           qp^4 * E * delta_function(var.theta_up,var.theta_low,theta)];
       
    if phi >= 0 && phi <= var.phi_low
        
        if A ~= 0
            T_p = [ (-2*D/A + 2*qp*D*G / A^2 - 3*qp^2 *C)*qx + 3*qz*qy*qp^2*E*delta_function(var.theta_up,var.theta_low,theta);
                    (-2*D/A + 2*qp*D*G / A^2 - 3*qp^2 *C)*qy - 3*qz*qx*qp^2*E*delta_function(var.theta_up,var.theta_low,theta);
                    4*qz*qp^3*E*delta_function(var.theta_up,var.theta_low,theta)];
        else
            T_p = [ ( - 3*qp^2 *C)*qx + 3*qz*qy*qp^2*E*delta_function(var.theta_up,var.theta_low,theta);
                    ( - 3*qp^2 *C)*qy - 3*qz*qx*qp^2*E*delta_function(var.theta_up,var.theta_low,theta);
                    4*qz*qp^3*E*delta_function(var.theta_up,var.theta_low,theta)];
        end
        
    else if phi > var.phi_low && phi <= var.phi_up
            
            if A ~= 0
                T_p = [(var.phi_low*D*qp / A^2 - 3*qp^2*C)*qx + 3*qz*qy*qp^2*E*delta_function(var.theta_up,var.theta_low,theta);
                       (var.phi_low*D*qp / A^2 - 3*qp^2*C)*qy - 3*qz*qx*qp^2*E*delta_function(var.theta_up,var.theta_low,theta);
                       4*qz*qp^3*E*delta_function(var.theta_up,var.theta_low,theta)];
            else
                T_p = [(- 3*qp^2*C)*qx + 3*qz*qy*qp^2*E*delta_function(var.theta_up,var.theta_low,theta);
                       (- 3*qp^2*C)*qy - 3*qz*qx*qp^2*E*delta_function(var.theta_up,var.theta_low,theta);
                       4*qz*qp^3*E*delta_function(var.theta_up,var.theta_low,theta)];
                   
            end
            
        else if phi > var.phi_up && phi <= pi
                
                if A ~= 0
                    
                    T_p = [(-2*D*var.phi_low / (A*(var.phi_up-pi)) + D*var.phi_low*qp*(2*G-pi) / (A^2*(var.phi_up-pi)) - 3*qp^2*C)*qx + 3*qz*qy*qp^2*E*delta_function(var.theta_up,var.theta_low,theta);
                           (-2*D*var.phi_low / (A*(var.phi_up-pi)) + D*var.phi_low*qp*(2*G-pi) / (A^2*(var.phi_up-pi)) - 3*qp^2*C)*qy - 3*qz*qx*qp^2*E*delta_function(var.theta_up,var.theta_low,theta);
                           4*qz*qp^3*E*delta_function(var.theta_up,var.theta_low,theta)];
                else
                    
                    T_p = [(- 3*qp^2*C)*qx + 3*qz*qy*qp^2*E*delta_function(var.theta_up,var.theta_low,theta);
                           (- 3*qp^2*C)*qy - 3*qz*qx*qp^2*E*delta_function(var.theta_up,var.theta_low,theta);
                           4*qz*qp^3*E*delta_function(var.theta_up,var.theta_low,theta)];
                end
                
            end
            
        end
        
    end
    
    
    if theta >= 0 && theta <= var.theta_low
        
        if B ~= 0
            
            T_w = [4*qp^3*qx*E*F + 2*qw*qz*qp^3*qy*E*F / B^2 - 2*qz*qp^3*qy*E / B;
                   4*qp^3*qy*E*F - 2*qw*qz*qp^3*qx*E*F / B^2 + 2*qz*qp^3*qx*E / B;
                   2*qz*qp^4*qw*E*F / B^2 - 2*qz*qp^4*E / B];
               
        else
            
            T_w = [4*qp^3*qx*E*F;
                   4*qp^3*qy*E*F;
                   0];
               
        end
        
    else if theta > var.theta_low && theta <= var.theta_up
            
            if B ~= 0
                
                T_w = [2*var.theta_low*qp^3*qx*E + var.theta_low*qw*qz*qp^3*qy*E / B^2;
                       2*var.theta_low*qp^3*qy*E - var.theta_low*qw*qz*qp^3*qx*E / B^2;
                       var.theta_low*qz*qp^4*qw*E / B^2];
                   
            else
                
                T_w = [2*var.theta_low*qp^3*qx*E;
                       2*var.theta_low*qp^3*qy*E;
                       0];
                   
            end
            
        else if theta > var.theta_up && theta <= pi
                
                if B ~= 0
                    
                    T_w = [-2*qp^3*qx*var.theta_low*(pi-2*F)/(var.theta_up-pi) - 2*qz*qp^3*qy*var.theta_low*E/(B*(var.theta_up-pi)) + qz*qp^3*qw*qy*var.theta_low*(2*F-pi)*E/(B^2*(var.theta_up-pi));
                           -2*qp^3*qy*var.theta_low*(pi-2*F)/(var.theta_up-pi) + 2*qz*qp^3*qx*var.theta_low*E/(B*(var.theta_up-pi)) - qz*qp^3*qw*qx*var.theta_low*(2*F-pi)*E/(B^2*(var.theta_up-pi));
                           -2*qz*qp^4*E*var.theta_low/(B*(var.theta_up-pi)) - qz*qp^4*qw*E*var.theta_low*(pi-2*F)/(B^2*(var.theta_up-pi))];
                       
                else
                    
                     T_w = [-2*qp^3*qx*var.theta_low*(pi-2*F)/(var.theta_up-pi);
                           -2*qp^3*qy*var.theta_low*(pi-2*F)/(var.theta_up-pi) ;
                           0];
                       
                end
                
            end
            
        end
        
    end
    
    grad_T = [T_x T_y T_z T_p T_w];
    
end

