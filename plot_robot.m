function plot_robot(rk, rgb)
x_k = rk(1);
y_k = rk(2);
theta_k = rk(3);

l = 3;
w = 1;

R = [cos(theta_k), -sin(theta_k);
     sin(theta_k), cos(theta_k)];
low_left_corner = [x_k; y_k] + R*[-l/2; -w/2];
up_left_corner = [x_k; y_k] + R*[-l/2; w/2];
right_corner = [x_k; y_k] + R*[l/2; 0];
x_coors = [low_left_corner(1); up_left_corner(1); right_corner(1)];
y_coors = [low_left_corner(2); up_left_corner(2); right_corner(2)];
plot(x_k, y_k, '+');
hold on;
fill(x_coors, y_coors, rgb);          
end