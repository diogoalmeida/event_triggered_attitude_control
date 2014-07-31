function [ q w ] = quadcopter( torques,w, var, q, t_s )
%QUADCOPTER Simulates the quadcopter behavior for inputs u=[u1; u2; u3; u4]
    
    
        % from quaternion to rotation matrix that maps the inertial frame
        % into the body frame
        R = [q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(4)*q(3)), 2*(q(1)*q(3)-q(4)*q(2));
             2*(q(2)*q(1)-q(4)*q(3)), -q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(4)*q(1));
             2*(q(3)*q(1)+q(4)*q(2)), 2*(q(3)*q(2)-q(4)*q(1)), -q(1)^2-q(2)^2+q(3)^2+q(4)^2];
         
         
        
                
        
        J=[var.J_x 0 0;
           0 var.J_x 0;
           0  0 var.J_z];
      
      
        
        % Cross product tensor
        
        w_cross = [0, -w(3) w(2);
                   w(3), 0, -w(1);
                   -w(2), w(1), 0];
        
        
        % The angular speed is given by Euler's equation
        

        w = w + (J\(J*w_cross*w) + J\torques)*t_s;
 
        
        
        % the evolution of the attitude is given by the attitude kinematics
        % [Survey of Attitude Representations]
        
        E = [q(4), -q(3), q(2); 
            q(3), q(4), -q(1);
            -q(2), q(1), q(4);
            -q(1), -q(2), -q(3)];
        

        q = (q + (0.5*E*w)*t_s);

%         % From the paper
%         
%          W_r=[q_loc(4) q_loc(3) -q_loc(2);
%                -q_loc(3) q_loc(4) q_loc(1);
%                q_loc(2) -q_loc(1) q_loc(4);
%                -q_loc(1) -q_loc(2) -q_loc(3)];
%         
%         q(i,:) = (q_loc - (0.5*W_r*w(i,:)')*t_s)';
       
        q = q./(norm(q));
        
        q = q';
        w = w';

        
        

end

