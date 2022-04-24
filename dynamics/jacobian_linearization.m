function [A, B] = jacobian_linearization(x)

x1 = x(1);
x2 = x(2);
x3 = x(3);
x4 = x(4);

g = 9.81;
r_g = 0.0254;
L = 0.4255;
K = 10;
tau = 0.1;

a = 5 * g * r_g / (7 * L);
b = (10 * L / 7) * (r_g / L)^2;

A = [0, 1, 0 ,0 ;
     0, 0, a*cos(x3) + b*(L/2-x1)* x4^2 *cos(x3) * sin(x3), -b*(L/2-x1) * cos(x3)^2 * x4;
     0,0,0,1;
     0,0,0, -1/tau];

B = [0;0;0;K/tau];
end