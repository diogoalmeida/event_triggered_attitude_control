function [ torques phi theta ] = controller(q_d,q_b,w,var)
%CONTROLLER Implements the fast and saturating controller. Receives the
%desired pose q_d, and the current attitude [q_b w]. Assumes a global
%structure var with all the variables required for the proper function of
%the controller.
    
    % 1) Given q_b and q_d, compute the error term q = q_b^hat * q_d.
    
    q = quat_mult(quat_conjugate(q_b),q_d);
    
    q = q.*sign_l(q(4)); % ensure sign regularity (q and -q represent the same rotation, so no harm done)
    
    % Decompose the error quaternion q into an xy error and a z error, q_xy
    % and q_z
    
    q_z = get_z_from_quat(q);
    q_xy = quat_mult_inv(q_z,q);
    
    % get the individual members of each quaternion for easier readability
    % later on
    qx = q_xy(1);
    qy = q_xy(2);
    qp = q_xy(4);
    qz = q_z(3);
    qw = q_z(4);
    
    
    % 2) Compute artificial torques
    
    [art_torques T_z T_phi] = comp_art_torques(qx,qy,qz,qp,qw,var);
    
    % 3) Compute damping
    
    D = comp_damp_matrix(qx,qy,qz,qp,qw,w,T_z,T_phi,art_torques,var);
    
    % 4) Return control signal
    
    torques = (art_torques - D*w)';
    
    % Debug variables
    
    phi = 2*acos(qp);
    theta = 2*acos(qw);


end

