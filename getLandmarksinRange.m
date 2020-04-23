function lmks_idx = getLandmarksinRange(rk_true, sensor_range, landmark_XY);
  assert(length(rk_true)==3);
  assert(size(landmark_XY, 2) == 2);  
  
  x_robot = rk_true(1);
  y_robot = rk_true(2);
  lmks_idx = [];
  for i=1:size(landmark_XY, 1)
    distance = sqrt((x_robot-landmark_XY(i, 1))^2 + (y_robot-landmark_XY(i, 2))^2);
    if sensor_range > distance
      lmks_idx = [lmks_idx; i];
    end
  end  
  
end
