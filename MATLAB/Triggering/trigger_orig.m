function [ trig v_dot alpha T_star] = trigger_analit( old_x, q_b, q_d, old_T, T, w, D_k,tau, var )
%TRIGGER_ANALIT Implements the analitical triggering function
    
    w_error = w - old_x(6:8);
    
    
    v_dot = w*tau' - T'*w';
    
    q_error = quat_mult(quat_conjugate(q_b),q_d);
        
    q_hat = q_error;
    q_hat = q_hat.*sign_l(q_hat(4));

    q_zv = get_z_from_quat(q_hat);
    q_xy = quat_mult_inv(q_zv,q_hat);


    q_x = q_xy(1);
    q_y = q_xy(2);
    q_p = q_xy(4);

    q_z = q_zv(3);
    q_w = q_zv(4);
    
    
    phi = 2*acos(q_p);
    theta = 2*acos(q_w);
    
    if q_p ~= 1
        A = 1/sqrt(1-q_p^2);
    else
        A = 0;
    end
    
    if q_w ~=1
        B = 1/sqrt(1-q_w^2);
    else
        B= 0;
    end
    
    T_star = comp_art_torques(q_x,q_y,q_z,q_p,q_w,var);
    
    
    alpha = w*old_T - w*T_star - w_error*D_k*old_x(6:8)' ;
    
    if alpha > 0.0*old_x(6:8)*D_k*old_x(6:8)'
        
        trig = 1;
        
    else
        
        trig = 0;
        
    end


end

