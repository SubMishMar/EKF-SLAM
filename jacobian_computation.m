clear all
clc;
syms xr yr xl yl d b
d = sqrt((xl-xr)^2+(yl-yr)^2);
b = atan((yl-yr)/(xl-xr));

H_r = simplify(jacobian([d, b], [xr, yr]))
H_l = simplify(jacobian([d, b], [xl, yl]))


% x_k = xk_1 + vk*dt*cos(thetak_1);
% y_k = yk_1 + vk*dt*sin(thetak_1);
% thetak = thetak_1 + omegak*dt;

syms xk_1 yk_1 thetak_1 vk omegak dt

X = [xk_1 + vk*dt*cos(thetak_1);
     yk_1 + vk*dt*sin(thetak_1);
     thetak_1 + omegak*dt];
     
F_x = simplify(jacobian(X, [xk_1, yk_1, thetak_1]))
F_u = simplify(jacobian(X, [vk, omegak]))