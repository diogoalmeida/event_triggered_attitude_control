%% Runs quad model with real torques

var = struct('phi_low',10*phi_multiplier*pi/180,'theta_low',15*pi/180,'phi_up',175*pi/180,'theta_up',175*pi/180,'delta_phi',5*pi/180,'delta_z',5*pi/180,'small_delta_phi',0.1999,'small_delta_z',0.0961,'c_phi',0.817/(torque_multiplier*phi_multiplier),'c_theta',0.109,'v_phi',0.1,'v_phi_max',1.425,'v_theta',0.1,'v_theta_max',0.624,'r_phi',0.75,'r_theta',0.75,'J_x',0.020232,'J_z',0.1077,'tau_xy',0.15/torque_multiplier,'tau_z',0.03*0);

load('data.mat');

F = 115;

T = length(w)/F;

w = zeros(T*F,3);
q_b = zeros(T*F,4);

for t = 1/F:1/F:T-1/F
    
    i = t*F;
    
    [q_b(round(i)+1,:) w(round(i)+1,:)] = quadcopter(torques(round(i),:)'.*3.45,w(round(i),:)',var,q_b(round(i),:)',1/F);
end

%%


fontsize = 15;
line = 1;



figure(2);
subplot(3,1,1);
hold on
title('Control torques');
plot(1/F:1/F:T,torques(:,1),'r','Linewidth',line);
%plot(t_s:t_s:T,torques(:,2),'g','Linewidth',line);
%plot(t_s:t_s:T,torques(:,3),'k','Linewidth',line);
%plot(t_s:t_s:T,sqrt(torques(:,1).^2+torques(:,2).^2),'c','Linewidth',line);
%legend({'$\tau_x$','$\tau_y$','$\tau_z$','$||\tau_{xy}||$'},'interpreter', 'latex','fontsize',fontsize);


subplot(3,1,2);
hold on
title('Angular velocities');
plot(1/F:1/F:T,w(:,1),'k','Linewidth',line);
%figure(70)
subplot(3,1,3);
hold on
title('$\varphi$ and $\vartheta$','interpreter','latex');
plot(1/F:1/F:T,phi(1:end-1),'Linewidth',line);
%plot(t_s:t_s:T,theta,'r','Linewidth',line);