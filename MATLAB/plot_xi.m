%% Plots a sample delta function for report purposes

x_low = 0.6;
x_up = 3.0;
x = 0:0.001:pi;


for i=1:length(x)
    y(i) = delta_function(x_up,x_low,x(i));
end


line = 1;
font = 12;
figure(1)
hold on
plot(x,y,'k','linewidth',line);
axis([0 pi 0 x_low + 0.1])
title('$\Lambda_{\xi_l}^{\xi_u}(\xi)$','interpreter','Latex','Fontsize',font);
xlabel('$\xi$','interpreter','Latex','Fontsize',font);
ylabel('$\Lambda$','interpreter','Latex','Fontsize',font);
plot(x_low, 0:0.01:x_low+0.1,'k.','markersize',2);
plot(x_up, 0:0.01:x_low+0.1,'k.','markersize',2);
format_ticks(gca,{'\xi_l','\xi_u'},{'0','\xi_l'},[x_low,x_up],[0,x_low],[],[],[],'interpreter','Latex');