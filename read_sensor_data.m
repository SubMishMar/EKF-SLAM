function sensor_data = read_sensor_data (robot_pose, landmark_XY, R)
  assert(size(landmark_XY, 2) == 2);
  sensor_data = [];
  for i = 1:size(landmark_XY, 1)
    current_lmk_pos = landmark_XY(i,:);
    [sensor_output,~,~] = sensor_model(robot_pose, current_lmk_pos, R);
    sensor_data = [sensor_data; sensor_output'];
  end
end
