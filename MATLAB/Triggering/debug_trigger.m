
% Creates the simulation environment for the saturating controller to run

phi_multiplier = 1;
torque_multiplier = 1;

var = struct('phi_low',10*phi_multiplier*pi/180,'theta_low', ...
            15*pi/180,'phi_up',175*pi/180,'theta_up',175*pi/180, ...
            'delta_phi',5*pi/180,'delta_z',5*pi/180, ...
            'small_delta_phi',0.1999,'small_delta_z',0.0961, ...
            'c_phi',0.817/(torque_multiplier*phi_multiplier), ...
            'c_theta',0.109*1,'v_phi',0.1,'v_phi_max',1.425, ...
            'v_theta',0.1,'v_theta_max',0.624,'r_phi',0.5, ...
            'r_theta',0.75,'J_x',0.020232,'J_z',0.1068,'tau_xy', ...
            0.15/torque_multiplier,'tau_z',0.03*0);

load('data.mat');

F = 50;

T = length(w)/F;

q_d = angle_to_quat([deg2rad(45) 0 0]);

torques = zeros(T*F,3);

old_x = [0 0 0 1 1 w(1,:)];
D_k = zeros(3,3);
D_log = zeros(T*F,3,3);
old_T = zeros(3,T*F);
%old_T = zeros(3,1);
curr_T = zeros(3,1);
v_dot = zeros(T*F,1);
v_dot_k = zeros(T*F,1);
alpha = zeros(T*F,1);
trig = ones(T*F,1);
T_star = zeros(3,T*F);
grad_T = zeros(3,5);

i = 0;
for t = 1/F:1/F:T
    
    i = i+1;
    
    q_b = angle_to_quat([deg2rad(phi(round(i))) 0 0]);
    
    w_c = [w(round(i),1) 0 0]';
    
    
    if round(i) == 1
        %[ trig(round(i)) v_dot(round(i)) alpha(round(i)) T_star(:,i) ]  = trigger_analit( old_x, q_b, q_d, old_T(:,round(i)), curr_T, w_c', D_k, zeros(1,3), var );
        [ trig(round(i)) v_dot(round(i)) T_star(:,i) ]  = trigger_approx( old_x, q_b, q_d, old_T, curr_T, w_c', D_k, zeros(1,3), grad_T, var );
        trig(round(i)) =1;
    else
        %[ trig(round(i)) v_dot(round(i)) alpha(round(i)) T_star(:,i) ]  = trigger_analit( old_x, q_b, q_d, old_T(:,round(i)-1), curr_T, w_c', D_k, torques(round(i-1),:), var );
        [ trig(round(i)) v_dot(round(i)) T_star(:,i) ]  = trigger_approx( old_x, q_b, q_d, old_T, curr_T, w_c', D_k, torques(round(i),:), grad_T, var );
    end
    
    if trig(round(i)) == 1
        [torques(round(i),:) trash trash D_k qxyk qzk art_torques] = controller(q_d,q_b,w_c, var);
        old_x =[qxyk(1) qxyk(2) qzk(3) qxyk(4) qzk(4) w(round(i),:)];
        old_T(:,round(i)) = art_torques;
        
        grad_T = compute_grad_T(qxyk,qzk,var);
        %old_T = art_torques;
        curr_T = art_torques;
        %v_dot_k(round(i)) = v_dot(round(i));
        v_dot_k(round(i)) = -old_x(6:8) * D_k * old_x(6:8)';
        D_log(round(i),:,:) = D_k;
    else
        [trash trash trash trash trash trash art_torques] = controller(q_d,q_b,w_c, var);
        curr_T = art_torques;
        torques(round(i),:) = torques(round(i)-1,:);
        v_dot_k(round(i)) = v_dot_k(round(i)-1);
        old_T(:,round(i)) = old_T(:,round(i)-1);
    end
        
        
    
    
end
    

%%

%%

fontsize = 15;
line = 1;



figure(1);
subplot(4,1,1);
hold on
title('Control torques');
plot(1/F:1/F:T,torques(:,1),'r','Linewidth',line);
%plot(t_s:t_s:T,torques(:,2),'g','Linewidth',line);
%plot(t_s:t_s:T,torques(:,3),'k','Linewidth',line);
%plot(t_s:t_s:T,sqrt(torques(:,1).^2+torques(:,2).^2),'c','Linewidth',line);
%legend({'$\tau_x$','$\tau_y$','$\tau_z$','$||\tau_{xy}||$'},'interpreter', 'latex','fontsize',fontsize);

% figure(6)
% hold on
% title('Switch curve and angle velocity');
% plot(t_s:t_s:T,phi_dotv(:));
% plot(t_s:t_s:T,s_surf_phi(:),'k.');
% plot(t_s:t_s:T,theta_dotv(:),'g');
% plot(t_s:t_s:T,s_surf_theta(:),'r.');
% legend({'$\dot \varphi$','$s(\varphi)$'},'interpreter', 'latex','fontsize',fontsize);

subplot(4,1,2);
hold on
title('Angular velocities');
plot(1/F:1/F:T,w(:,1),'k','Linewidth',line);
%figure(70)
subplot(4,1,3);
hold on
title('$\varphi$ and $\vartheta$','interpreter','latex');
plot(1/F:1/F:T,phi(1:end),'Linewidth',line);
%plot(t_s:t_s:T,theta,'r','Linewidth',line);
subplot(4,1,4);
hold on
title('Sampling instants');
stem(1/F:1/F:T,trig,'Marker','none');

figure(2)
subplot(4,1,1)
title('v_dot_k');
plot(1/F:1/F:T,v_dot_k,'k','Linewidth',line);
%figure(70)
subplot(4,1,2);
hold on
title('$\alpha$','interpreter','latex');
plot(1/F:1/F:T,alpha,'Linewidth',line);
%plot(t_s:t_s:T,theta,'r','Linewidth',line);
subplot(4,1,3);
hold on
title('T','interpreter','latex');
plot(1/F:1/F:T,old_T(1,:),'Linewidth',line);
subplot(4,1,4);
hold on
title('T star','interpreter','latex');
plot(1/F:1/F:T,T_star(1,:),'Linewidth',line);

