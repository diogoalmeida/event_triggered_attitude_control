
% Creates the simulation environment for the saturating controller to run

phi_multiplier = 1;
torque_multiplier = 1;

% var = struct('phi_low',10*phi_multiplier*pi/180,'theta_low',...
%              15*pi/180,'phi_up',175*pi/180,'theta_up',175*pi/180,...
%              'delta_phi',5*pi/180,'delta_z',5*pi/180,...
%              'small_delta_phi',0.1999,'small_delta_z',0.0961,...
%              'c_phi',0.817/(torque_multiplier*phi_multiplier),...
%              'c_theta',0.109,'v_phi',0.1,'v_phi_max',1.425,'v_theta',0.1,...
%              'v_theta_max',0.624,'r_phi',0.5,'r_theta',0.75,'J_x',0.02,...
%              'J_z',0.032,'tau_xy',0.15/torque_multiplier,'tau_z',0.03);
         
var = struct('phi_low',10*phi_multiplier*pi/180,'theta_low',...
             15*pi/180,'phi_up',175*pi/180,'theta_up',175*pi/180,...
             'delta_phi',5*pi/180,'delta_z',5*pi/180,...
             'small_delta_phi',0.1999,'small_delta_z',0.0961,...
             'c_phi',0.817/(torque_multiplier*phi_multiplier),...
             'c_theta',0.109,'v_phi',0.1,'v_phi_max',1.425,'v_theta',0.1,...
             'v_theta_max',0.624,'r_phi',0.75,'r_theta',0.75,'J_x',0.0085,...
             'J_z',0.014,'tau_xy',0.15/torque_multiplier,'tau_z',0.03);



F = 50;

T = 2.5; %s

w = zeros(T*F,3);
q_b = [zeros(T*F,3) ones(T*F,1)];

q_d = angle_to_quat([deg2rad(0) 0 0]);

torques = zeros(T*F,3);

q_b(1,:) = [-0.992 -0.087 0.008 0.087];%angle_to_quat([deg2rad(-45) 0 0]);
w(1,:) = [0 0 1.7];

phi = zeros(T*F,1);
theta = zeros(T*F,1);
trig = ones(T*F,1);
D_k = zeros(3,3);
old_x = [0 0 0 1 1 w(1,:)];
old_T = zeros(3,1);
curr_T = zeros(3,1);
v_dot = zeros(T*F,1);
grad_T = zeros(3,5);
qk = zeros(1,4);
phi_dot = zeros(T*F,1);
theta_dot = zeros(T*F,1);
switch_phi = zeros(T*F,1);
switch_theta = zeros(T*F,1);
v_dot = zeros(T*F,1);

for t = 1/F:1/F:T-1/F
    
    i = t*F;
    
    %[ trig(round(i)) v_dot(round(i)) ] = trigger_orig( old_x, q_b(round(i),:), q_d, old_T, curr_T, w(round(i),:), D_k, torques(round(i),:), var );
    %[ trig(round(i)) v_dot(round(i)) ]  = trigger_analit( old_x, q_b(round(i),:), q_d, old_T, curr_T, w(round(i),:), D_k, torques(round(i),:), var );
    %[ trig(round(i)) v_dot(round(i)) ]  = trigger_approx( old_x, q_b(round(i),:), q_d, old_T, curr_T, w(round(i),:), D_k, torques(round(i),:), grad_T, var );
    [ trig(round(i)) v_dot(round(i)) ] = trigger_heur( old_x , q_b(round(i),:), q_d, w(round(i),:), curr_T, qk, torques(round(i),:) );
    
    if round(i) == 1
        trig(round(i)) = 1;
    end
    
    if trig(round(i)) == 1
        [torques(round(i),:) trash trash D_k qxyk qzk art_torques qk phi_dot theta_dot switch_phi switch_theta v_dot(round(i))] = controller(q_d,q_b(round(i),:),w(round(i),:)', var);
        
        grad_T = compute_grad_T(qxyk,qzk,var);
        
        old_x =[qxyk(1) qxyk(2) qzk(3) qxyk(4) qzk(4) w(round(i),:)];
        
        old_T = art_torques;
        curr_T = art_torques;
        
        phi(round(i)) = 2*acos(qxyk(4));
        theta(round(i)) = 2*acos(qzk(4));
    else
        [trash trash trash D qxyk qzk art_torques li li li li li trash] = controller(q_d,q_b(round(i),:),w(round(i),:)', var);
        
        if round(i) > 1
            torques(round(i),:) = torques(round(i)-1,:);
        end
        
        curr_T = art_torques;
        
        phi(round(i)) = 2*acos(qxyk(4));
        theta(round(i)) = 2*acos(qzk(4));
        v_dot(round(i)) = w(round(i),:)*torques(round(i),:)' - art_torques'*w(round(i),:)';
    end
    
    [q_b(round(i)+1,:) w(round(i)+1,:)] = quadcopter(torques(round(i),:)',w(round(i),:)',var,q_b(round(i),:)',1/F);
    
    
    
    
    
    
    
end
    

%%

%%

fontsize = 10;
line = 1;
marker = 5;
height = 0.368;
tick_size = 9;

figure(1);
subplot('Position',[0.1 0.6 0.8 height]);
hold on
%title('Control torques');
plot(1/F:1/F:T,torques(:,1),'r','Linewidth',line);
plot(1/F:1/F:T,torques(:,2),'g','Linewidth',line);
plot(1/F:1/F:T,torques(:,3),'k','Linewidth',line);
plot(1/F:1/F:T,sqrt(torques(:,1).^2+torques(:,2).^2),'b--','Linewidth',line);
%h_leg=legend({'$\tau_x$'},'interpreter', 'latex','fontsize',fontsize);%,'$\tau_y$','$\tau_z$','$\|\tau_{xy}\|$'%legend({'$\tau_x$','$\tau_y$','$\tau_z$','$\|\tau_{xy}\|$'},'interpreter', 'latex','fontsize',fontsize);
%legend({'$\tau_x$','$\tau_y$','$\tau_z$','$\|\tau_{xy}\|$'},'interpreter', 'latex','fontsize',fontsize);
set(gca,'FontSize',tick_size)
set(gca,'XTickLabel',[]);
ylabel('[N.m]');

subplot('Position',[0.1 0.15 0.8 height]);
hold on
%title('$\varphi$ and $\vartheta$','interpreter','latex');
plot(1/F:1/F:T,rad2deg(phi),'Linewidth',line);
plot(1/F:1/F:T,rad2deg(theta),'r','Linewidth',line);
%legend({'$\varphi$'},'interpreter', 'latex','fontsize',fontsize);
%legend({'$\varphi$','$\vartheta$'},'interpreter','latex','fontsize',fontsize);
%set(gca,'XTickLabel',0:1:T);
set(gca,'XTickLabel',[]);
%x=xlabel('Time [s]');
%set(x, 'Units', 'Normalized', 'Position', [0.5, -0.05, 0]);
ylabel('Degrees');
set(gca,'FontSize',tick_size)

subplot('Position',[0.1 0.07 0.8 height/9]);
hold on
stem(1/F:1/F:T,trig,'Marker','none');
axis([0 T 0 1]);
set(gca,'YTickLabel',[]);
set(gca,'FontSize',tick_size)
%set(gca,'XTick', 0:1:T);
set(gca,'XTick', 0:0.5:T);
x=xlabel('Time [s]','fontsize',10);
set(x, 'Units', 'Normalized', 'Position', [0.5, -0.5, 0]);

% figure(1);
% subplot('Position',[0.05 0.6 0.43 height]);
% hold on
% %title('Control torques');
% plot(1/F:1/F:T,torques(:,1),'r','Linewidth',line);
% %plot(1/F:1/F:T,torques(:,2),'g','Linewidth',line);
% %plot(1/F:1/F:T,torques(:,3),'k','Linewidth',line);
% %plot(1/F:1/F:T,sqrt(torques(:,1).^2+torques(:,2).^2),'b--','Linewidth',line);
% %h_leg=legend({'$\tau_x$'},'interpreter', 'latex','fontsize',fontsize);%,'$\tau_y$','$\tau_z$','$\|\tau_{xy}\|$'
% %legend({'$\tau_x$','$\tau_y$','$\tau_z$','$\|\tau_{xy}\|$'},'interpreter', 'latex','fontsize',fontsize);
% set(gca,'FontSize',tick_size)
% set(gca,'XTickLabel',[]);
% ylabel('[N.m]');
% 
% subplot('Position',[0.56 0.6 0.43 height]);
% hold on
% %title('Angular velocities');
% plot(1/F:1/F:T,w(:,1),'r','Linewidth',line);
% %plot(1/F:1/F:T,w(:,2),'g','Linewidth',line);
% %plot(1/F:1/F:T,w(:,3),'b','Linewidth',line);
% %legend({'$\omega_x$'},'interpreter', 'latex','fontsize',fontsize);
% %legend({'$\omega_x$','$\omega_y$','$\omega_z$'},'interpreter','latex','fontsize',fontsize);
% set(gca,'XTickLabel',[]);
% set(gca,'FontSize',tick_size)
% ylabel('[rad/s]');
% 
% subplot('Position',[0.05 0.15 0.43 height]);
% hold on
% plot(1/F:1/F:T,v_dot,'k','Linewidth',line);
% %legend({'$\dot V(\mathbf{x})$'},'interpreter','latex','fontsize',fontsize);
% %set(gca,'XTickLabel',0:1:T);
% set(gca,'XTickLabel',[]);
% ylabel('[N.m.rad/s]');
% set(gca,'FontSize',tick_size)
% %title('Switch curve and angle velocity');
% % plot(1/F:1/F:T,phi_dot(:),'Linewidth',line);
% % plot(1/F:1/F:T,switch_phi(:),'k--','linewidth',line);
% % plot(1/F:1/F:T,theta_dot(:),'g','Linewidth',line);
% % plot(1/F:1/F:T,switch_theta(:),'r-.','linewidth',line);
% % legend({'$\dot \varphi$','$s(\varphi)$','$\dot \vartheta$','$s(\vartheta)$'},'interpreter', 'latex','fontsize',fontsize);
% % ylabel('[rad/s]');
% %xlabel('Time [s]');
% 
% subplot('Position',[0.56 0.15 0.43 height]);
% hold on
% %title('$\varphi$ and $\vartheta$','interpreter','latex');
% plot(1/F:1/F:T-1/F,rad2deg(phi(1:end-1)),'Linewidth',line);
% %plot(1/F:1/F:T-1/F,rad2deg(theta(1:end-1)),'r','Linewidth',line);
% %legend({'$\varphi$'},'interpreter', 'latex','fontsize',fontsize);
% %legend({'$\varphi$','$\vartheta$'},'interpreter','latex','fontsize',fontsize);
% %set(gca,'XTickLabel',0:1:T);
% set(gca,'XTickLabel',[]);
% ylabel('Degrees');
% set(gca,'FontSize',tick_size)
% 
% 
% subplot('Position',[0.05 0.1 0.43 height/9]);
% hold on
% stem(1/F:1/F:T,trig,'Marker','none');
% axis([0 T 0 1]);
% set(gca,'YTickLabel',[]);
% set(gca,'FontSize',tick_size)
% %set(gca,'XTick', 0:1:T);
% set(gca,'XTick', 0:0.5:T);
% x=xlabel('Time [s]','fontsize',10);
% set(x, 'Units', 'Normalized', 'Position', [0.5, -0.5, 0]);
% 
% 
% subplot('Position',[0.56 0.1 0.43 height/9]);
% hold on
% %title('Sampling instants');
% stem(1/F:1/F:T,trig,'Marker','none');
% axis([0 T 0 1]);
% set(gca,'YTickLabel',[]);
% set(gca,'FontSize',tick_size)
% %set(gca,'XTick', 0:1:T);
% set(gca,'XTick', 0:0.5:T);
% x=xlabel('Time [s]','fontsize',10);
% set(x, 'Units', 'Normalized', 'Position', [0.5, -0.5, 0]);
% 
% %set(tick_h,'XLim',x_size,'YLim',y_size);
% 
% % figure(6)
% % hold on
% % title('Switch curve and angle velocity');
% % plot(t_s:t_s:T,phi_dotv(:));
% % plot(t_s:t_s:T,s_surf_phi(:),'k.');
% % plot(t_s:t_s:T,theta_dotv(:),'g');
% % plot(t_s:t_s:T,s_surf_theta(:),'r.');
% % legend({'$\dot \varphi$','$s(\varphi)$'},'interpreter', 'latex','fontsize',fontsize);
% 
% % subplot(4,1,2);
% % hold on
% % title('Angular velocities');
% % plot(1/F:1/F:T,w(:,1),'r','Linewidth',line);
% % plot(1/F:1/F:T,w(:,2),'g','Linewidth',line);
% % plot(1/F:1/F:T,w(:,3),'b','Linewidth',line);
% % %figure(70)

