x_1 = 2;
x_2 = 3.0;
x = 0:0.0001:5;
y = zeros(1,length(x));

for i=1:length(x)
    y(i) = xi_function(x_2,x_1,x(i),x(i)^2,x(i));
end

line = 1;
font = 12;
figure(1)
hold on
plot(x,y,'k','linewidth',line);
axis([0 5 0 5+0.1])
title('$\chi_{\xi_1}^{\xi_2}(\xi,\psi_1(\mathbf{x}),\psi_2(\mathbf{x}))$','interpreter','Latex','Fontsize',font);
xlabel('$\xi$','interpreter','Latex','Fontsize',font);
ylabel('$\chi$','interpreter','Latex','Fontsize',font);
plot(x_1, 0:0.05:4,'k.','markersize',2);
plot(x_2, 0:0.05:3,'k.','markersize',2);
format_ticks(gca,{'\xi_1','\xi_2'},{'0'},[x_1,x_2],[0],[],[],[],'interpreter','Latex');