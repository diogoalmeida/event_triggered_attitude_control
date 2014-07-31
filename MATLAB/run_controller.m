
% Creates the simulation environment for the saturating controller to run

phi_multiplier = 1;
torque_multiplier = 1;

var = struct('phi_low',10*phi_multiplier*pi/180,'theta_low',15*pi/180,'phi_up',175*pi/180,'theta_up',175*pi/180,'delta_phi',5*pi/180,'delta_z',5*pi/180,'small_delta_phi',0.1999,'small_delta_z',0.0961,'c_phi',0.817/(torque_multiplier*phi_multiplier),'c_theta',0.109,'v_phi',0.1,'v_phi_max',1.425,'v_theta',0.1,'v_theta_max',0.624,'r_phi',0.75,'r_theta',0.75,'J_x',0.0085,'J_z',0.014,'tau_xy',0.15/torque_multiplier,'tau_z',0.03);


F = 1000;

T = 2.5; %s

w = zeros(T*F,3);
q_b = zeros(T*F,4);

q_d = angle_to_quat([deg2rad(0) 0 0]);

torques = zeros(T*F,3);

q_b(1,:) = [-0.992 -0.087 0.008 0.087];%angle_to_quat([deg2rad(0) 0 0]);
w(1,:) = [0 0 1.7];

phi = zeros(T*F,1);
theta = zeros(T*F,1);

for t = 1/F:1/F:T-1/F
    
    i = t*F;
    
    [torques(round(i),:) phi(round(i)) theta(round(i))] = controller(q_d',q_b(round(i),:)',w(round(i),:)', var);
    
    [q_b(round(i)+1,:) w(round(i)+1,:)] = quadcopter(torques(round(i),:)',w(round(i),:)',var,q_b(round(i),:)',1/F);
    
end
    

%%

%%

fontsize = 15;
line = 1;



figure(1);
subplot(2,1,1);
hold on
title('Control torques');
plot(1/F:1/F:T,torques(:,1),'r','Linewidth',line);
plot(1/F:1/F:T,torques(:,2),'g','Linewidth',line);
plot(1/F:1/F:T,torques(:,3),'k','Linewidth',line);
plot(1/F:1/F:T,sqrt(torques(:,1).^2+torques(:,2).^2),'c','Linewidth',line);
legend({'$\tau_x$','$\tau_y$','$\tau_z$','$||\tau_{xy}||$'},'interpreter', 'latex','fontsize',fontsize);

% figure(6)
% hold on
% title('Switch curve and angle velocity');
% plot(t_s:t_s:T,phi_dotv(:));
% plot(t_s:t_s:T,s_surf_phi(:),'k.');
% plot(t_s:t_s:T,theta_dotv(:),'g');
% plot(t_s:t_s:T,s_surf_theta(:),'r.');
% legend({'$\dot \varphi$','$s(\varphi)$'},'interpreter', 'latex','fontsize',fontsize);

% subplot(3,1,2);
% hold on
% title('Angular velocities');
% plot(1/F:1/F:T,w(:,1),'r','Linewidth',line);
% plot(1/F:1/F:T,w(:,2),'g','Linewidth',line);
% plot(1/F:1/F:T,w(:,3),'b','Linewidth',line);
% %figure(70)
subplot(2,1,2);
hold on
title('$\varphi$ and $\vartheta$','interpreter','latex');
plot(1/F:1/F:T,phi,'Linewidth',line);
plot(1/F:1/F:T,theta,'r','Linewidth',line);

