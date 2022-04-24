function MPC_plot(x, u)

name = {"pos", "velocity", "theta", "theta dot", "V servo"}; 

figure
subplot(2,3,1)
plot(x(1,:));
title(name{1});

subplot(2,3,2)
plot(x(2,:));
title(name{2});

subplot(2,3,4)
plot(x(3,:));
title(name{3});

subplot(2,3,5)
plot(x(4,:));
title(name{4});

subplot(2,3,6)
plot(u(1,:));
title(name{5});

end