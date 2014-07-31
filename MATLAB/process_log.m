%% Reads a log from the APM board and creates a matrix with motor values

fid = fopen('motor_test.txt');

tline = fgetl(fid);

M = [];

while ischar(tline)
    if tline(1:3) == 'MOT'
        temp = textscan(tline,'%s%n%n%n%n%s','delimiter',',');
        M(end+1,:) = [temp{2:5}];
    end
    tline = fgetl(fid);
end
fclose(fid);

fid = fopen('motor_test.txt');

tline = fgetl(fid);

A = [];

while ischar(tline)
    if tline(1:3) == 'ATT'
        temp = textscan(tline,'%s%n%n%n%n%n%n%n%s','delimiter',',');
        A(end+1,:) = [temp{2:8}];
    end
    tline = fgetl(fid);
end
fclose(fid);

fid = fopen('motor_test.txt');

tline = fgetl(fid);

B = [];

while ischar(tline)
    if tline(1:4) == 'DFLT'
        temp = textscan(tline,'%s%n%n%s','delimiter',',');
        if temp{2} == 1
            B(end+1,:) = [temp{2:3}];
        end
    end
    tline = fgetl(fid);
end
fclose(fid);

fid = fopen('motor_test.txt');

tline = fgetl(fid);

C = [];

while ischar(tline)
    if tline(1:3) == 'D16'
        temp = textscan(tline,'%s%n%n%s','delimiter',',');
        C(end+1,:) = [temp{2:3}];
    end
    tline = fgetl(fid);
end
fclose(fid);


fid = fopen('motor_test.txt');

tline = fgetl(fid);

D = [];

while ischar(tline)
    if tline(1:4) == 'DFLT'
        temp = textscan(tline,'%s%n%n%s','delimiter',',');
        if temp{2} == 3
            D(end+1,:) = [temp{2:3}];
        end
    end
    tline = fgetl(fid);
end
fclose(fid);

fid = fopen('motor_test.txt');

tline = fgetl(fid);

E = [];

while ischar(tline)
    if tline(1:4) == 'DFLT'
        temp = textscan(tline,'%s%n%n%s','delimiter',',');
        if temp{2} == 4
            E(end+1,:) = [temp{2:3}];
        end
    end
    tline = fgetl(fid);
end
fclose(fid);

fid = fopen('motor_test.txt');

tline = fgetl(fid);

G = [];

while ischar(tline)
    if tline(1:4) == 'DFLT'
        temp = textscan(tline,'%s%n%n%s','delimiter',',');
        if temp{2} == 5
            G(end+1,:) = [temp{2:3}];
        end
    end
    tline = fgetl(fid);
end
fclose(fid);

fid = fopen('motor_test.txt');

tline = fgetl(fid);

H = [];

while ischar(tline)
    if tline(1:4) == 'DFLT'
        temp = textscan(tline,'%s%n%n%s','delimiter',',');
        if temp{2} == 6
            H(end+1,:) = [temp{2:3}];
        end
    end
    tline = fgetl(fid);
end
fclose(fid);


F = 50;

if length(M(:,1))/F > 2.5
    T = 2.5;
else
    T = (length(M(:,1))-1)/F;
end

fontsize = 15;
line = 1;
height = 0.368;%0.16;

%% Plot stuff (trig)
figure(100);
%subplot('Position',[0.12 0.8 0.8 height]);
subplot('Position',[0.1 0.6 0.8 height]);
hold on
%title('Control torques');
plot(1/F:1/F:T,M(1:T*F,1)/10000,'r','Linewidth',line);
%plot(0:T,zeros(T+1,1),'k','Linewidth',line);
set(gca,'XTickLabel',[]);
ylabel('[N.m]');


% subplot('Position',[0.12 0.56 0.8 height]);
% hold on
% %title('Ang vel');
% plot(1/F:1/F:T,B(1:T*F,2),'r','Linewidth',line);
% plot(0:T,zeros(T+1,1),'b','Linewidth',line);
% set(gca,'XTickLabel',[]);
% ylabel('[rad/s]');

% subplot('Position',[0.12 0.35 0.8 height]);
% hold on
% %v_dot_k
% plot(1/F:1/F:T,D(1:T*F,2),'k','Linewidth',line);
% set(gca,'XTickLabel',[]);
% ylabel('[N.m.rad/s]');

%subplot('Position',[0.12 0.15 0.8 height]);
subplot('Position',[0.1 0.15 0.8 height]);

hold on
%title('$\varphi$','interpreter','latex');
plot(1/F:1/F:T,abs(48-A(1:T*F,2)),'Linewidth',line)
set(gca,'XTickLabel',[]);
%set(gca,'XTickLabel',[0:0.5:T]);
ylabel('Degrees');
%xlabel('Time [s]');

subplot('Position',[0.1 0.08 0.8 height/8]);
hold on
%title('Sampling instants');
stem(1/F:1/F:T,C(1:T*F,2),'Marker','none')
axis([0 T 0 1]);
set(gca,'YTickLabel',[]);
x=xlabel('Time [s]','fontsize',10);
set(x, 'Units', 'Normalized', 'Position', [0.5, -0.5, 0]);


w = [B(1:T*F,2) zeros(length(B(1:T*F,2)),2)];
phi = A(1:T*F,2);
torques = [M(1:T*F,1)/10000 zeros(length(M(1:T*F,1)),2)];
trig = C(1:T*F,2);
T_k = G(1:T*F,2);

% figure(200);
% subplot(4,1,1);
% hold on
% title('v_dot_k');
% plot(1/F:1/F:T,D(1:T*F,2),'Linewidth',line);
% subplot(4,1,2);
% hold on
% title('alpha');
% plot(1/F:1/F:T,E(1:T*F,2),'Linewidth',line);
% subplot(4,1,3);
% hold on
% title('T_k');
% plot(1/F:1/F:T,G(1:T*F,2),'Linewidth',line);
% subplot(4,1,4);
% hold on
% title('T star');
% plot(1/F:1/F:T,H(1:T*F,2),'Linewidth',line);
% save('data.mat','w','phi','torques','T_k');

%% 2x2 version
% 
% height = 2.3*height;
% tick_size = 9;
% figure(100);
% subplot('Position',[0.05 0.6 0.43 height]);
% hold on
% %title('Control torques');
% plot(1/F:1/F:T,M(1:T*F,1)/10000,'r','Linewidth',line);
% %plot(0:T,zeros(T+1,1),'k','Linewidth',line);
% set(gca,'XTickLabel',[]);
% set(gca,'FontSize',tick_size)
% ylabel('[N.m]');
% 
% subplot('Position',[0.56 0.6 0.43 height]);
% hold on
% %title('Ang vel');
% plot(1/F:1/F:T,B(1:T*F,2),'r','Linewidth',line);
% %plot(0:T,zeros(T+1,1),'b','Linewidth',line);
% set(gca,'XTickLabel',[]);
% set(gca,'FontSize',tick_size)
% ylabel('[rad/s]');
% 
% subplot('Position',[0.05 0.15 0.43 height]);
% hold on
% %v_dot_k
% plot(1/F:1/F:T,D(1:T*F,2),'k','Linewidth',line);
% set(gca,'XTickLabel',[]);
% set(gca,'FontSize',tick_size)
% ylabel('[N.m.rad/s]');
% %  x=xlabel('Time [s]','fontsize',10);
% %  set(x, 'Units', 'Normalized', 'Position', [0.5, -0.5, 0]);
% 
% subplot('Position',[0.56 0.15 0.43 height]);
% hold on
% %title('$\varphi$','interpreter','latex');
% plot(1/F:1/F:T,abs(45-A(1:T*F,2)),'Linewidth',line)
% axis([0 T 0 105]);
% set(gca,'XTickLabel',[]);
% set(gca,'FontSize',tick_size)
% ylabel('Degrees');
% %  x=xlabel('Time [s]','fontsize',10);
% %  set(x, 'Units', 'Normalized', 'Position', [0.5, -0.5, 0]);
% 
% subplot('Position',[0.05 0.1 0.43 height/9]);
% hold on
% %title('Sampling instants');
% stem(1/F:1/F:T,C(1:T*F,2),'Marker','none')
% axis([0 T 0 1]);
% set(gca,'YTickLabel',[]);
% set(gca,'FontSize',tick_size)
% set(gca,'XTick', 0:0.5:T);
% x=xlabel('Time [s]','fontsize',10);
% set(x, 'Units', 'Normalized', 'Position', [0.5, -0.5, 0]);
% 
% subplot('Position',[0.56 0.1 0.43 height/9]);
% hold on
% %title('Sampling instants');
% stem(1/F:1/F:T,C(1:T*F,2),'Marker','none')
% axis([0 T 0 1]);
% set(gca,'YTickLabel',[]);
% set(gca,'FontSize',tick_size)
% set(gca,'XTick', 0:0.5:T);
% x=xlabel('Time [s]','fontsize',10);
% set(x, 'Units', 'Normalized', 'Position', [0.5, -0.5, 0]);
% 
% %% Plot stuff (cont)
% 
% % figure(100);
% % subplot('Position',[0.12 0.8 0.8 height]);
% % hold on
% % %title('Control torques');
% % plot(1/F:1/F:T,M(1:T*F,1)/10000,'r','Linewidth',line);
% % plot(0:T,zeros(T+1,1),'k','Linewidth',line);
% % set(gca,'XTickLabel',[]);
% % ylabel('[N.m]');
% % 
% % 
% % subplot('Position',[0.12 0.55 0.8 height]);
% % hold on
% % %title('Ang vel');
% % plot(1/F:1/F:T,B(1:T*F,2),'r','Linewidth',line);
% % plot(0:T,zeros(T+1,1),'b','Linewidth',line);
% % set(gca,'XTickLabel',[]);
% % ylabel('[rad/s]');
% % 
% % subplot('Position',[0.12 0.3 0.8 height]);
% % hold on
% % %v_dot_k
% % plot(1/F:1/F:T,D(1:T*F,2),'k','Linewidth',line);
% % set(gca,'XTickLabel',[]);
% % ylabel('[N.m.rad/s]');
% % 
% % subplot('Position',[0.12 0.07 0.8 height]);
% % hold on
% % %title('$\varphi$','interpreter','latex');
% % plot(1/F:1/F:T,abs(45-A(1:T*F,2)),'Linewidth',line)
% % ylabel('Degrees');
% % xlabel('Time [s]');
% 
% % w = [B(1:T*F,2) zeros(length(B(1:T*F,2)),2)];
% % phi = A(1:T*F,2);
% % torques = [M(1:T*F,1)/10000 zeros(length(M(1:T*F,1)),2)];
% % trig = C(1:T*F,2);
% % T_k = G(1:T*F,2);
% % 
% % figure(200);
% % subplot(4,1,1);
% % hold on
% % title('v_dot_k');
% % plot(1/F:1/F:T,D(1:T*F,2),'Linewidth',line);
% % subplot(4,1,2);
% % hold on
% % title('alpha');
% % plot(1/F:1/F:T,E(1:T*F,2),'Linewidth',line);
% % subplot(4,1,3);
% % hold on
% % title('T_k');
% % plot(1/F:1/F:T,G(1:T*F,2),'Linewidth',line);
% % subplot(4,1,4);
% % hold on
% % title('T star');
% % plot(1/F:1/F:T,H(1:T*F,2),'Linewidth',line);
% % 
% % save('data.mat','w','phi','torques','T_k');
% % 
% %  