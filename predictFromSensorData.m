function [x, P] = predictFromSensorData(x, P, sensor_reading, index, R)
  assert(size(sensor_reading,1) == 1);
  assert(size(sensor_reading,2) == 2);
  robot_pose = x(1:3);
  x_r = robot_pose(1);
  y_r = robot_pose(2);
  theta_r = robot_pose(3);
  
  rho = sensor_reading(1);
  bearing = sensor_reading(2);
  
  x_lmk = x_r + rho*cos(theta_r+bearing);
  y_lmk = y_r + rho*sin(theta_r+bearing);
  
  landmark_location = [x_lmk, y_lmk];
  
  Gr = [1 0  -rho*sin(theta_r+bearing);
        0 1   rho*cos(theta_r+bearing)];
  Gobs = [cos(theta_r+bearing) -rho*sin(theta_r+bearing);
          sin(theta_r+bearing)  rho*cos(theta_r+bearing)];
          
  x_idx = 3+2*index-1;
  y_idx = 3+2*index;
  
  x(x_idx) = x_lmk;
  x(y_idx) = y_lmk;
  P(x_idx:y_idx, x_idx:y_idx) = Gr*P(1:3, 1:3)*Gr' + Gobs*R*Gobs';
  P(x_idx:y_idx, [1:x_idx-1, y_idx+1:end]) = Gr * P(1:3,[1:x_idx-1, y_idx+1:end]);
  P([1:x_idx-1, y_idx+1:end], x_idx:y_idx) = P(x_idx:y_idx, [1:x_idx-1, y_idx+1:end])'; 
end