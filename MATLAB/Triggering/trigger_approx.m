function [ trig v_dot dT] = trigger_approx( old_x, q_b, q_d, old_T, T, w, D_k,tau,grad_T, var )
%TRIGGER_ANALIT Implements the approximated triggering function. Grad_T is
%a 5 element vector with T_x,T_y,T_z,T_p and T_w.
    
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
    
    
    qx = q_x - old_x(1);
    qy = q_y - old_x(2);
    qz = q_z - old_x(3);
    qp = q_p - old_x(4);
    qw = q_w - old_x(5);
    
    dT = grad_T(:,1).*qx + grad_T(:,2).*qy + grad_T(:,3).*qz + grad_T(:,4).*qp + grad_T(:,5).*qw;
    
    alpha = -w*dT - w_error*D_k*old_x(6:8)';
    
    dT = grad_T(:,1).*qx;
    
    if alpha > 0.1*old_x(6:8)*D_k*old_x(6:8)'
        
        trig = 1;
        
    else
        
        trig = 0;
        
    end


end

