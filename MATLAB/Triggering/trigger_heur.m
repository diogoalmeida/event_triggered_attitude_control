function [ trig v_dot ] = trigger_heur( old_x , q_b, q_d, w, T, qk, tau )
%TRIGGER_HEUR Implements the heuristic triggering function
    
    
    v_dot = w*tau' - T'*w';
    w_error = w - old_x(6:8);
    
    q_error = quat_mult(quat_conjugate(q_b),q_d);
        
    q_hat = q_error;
    q_hat = q_hat.*sign_l(q_hat(4));



    e = [q_hat - qk, w_error];
    
    alpha = norm(e)
    
    if alpha > 0.1
        
        trig = 1;
        
    else
        
        trig = 0;
        
    end

end

