function [sensor_output, H_r, H_l] = sensor_model(robot_pose, lmk_position, R)
  assert(length(robot_pose)==3);
  assert(length(lmk_position)==2);
  x_robot = robot_pose(1);
  y_robot = robot_pose(2);
  theta_robot = robot_pose(3);
  
  x_lmk = lmk_position(1);
  y_lmk = lmk_position(2);

  d2 = (x_lmk - x_robot)^2 + (y_lmk - y_robot)^2;  
  
  d = sqrt(d2);
  bearing = atan2(y_lmk - y_robot, x_lmk - x_robot)-theta_robot;
  
  sensor_output = [d; bearing] + [sqrt(R(1))*randn(1,1); sqrt(R(4))*randn(1,1)];
  
  H_r = [(x_robot - x_lmk)/d, (y_robot - y_lmk)/d, 0;
         (y_lmk-y_robot)/d2, (x_robot - x_lmk)/d2, -1];
  H_l = [(x_lmk - x_robot)/d, (y_lmk - y_robot)/d;
         (y_robot - y_lmk)/d2, (x_lmk - x_robot)/d2];       
end
