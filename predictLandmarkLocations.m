function [x, P] = predictLandmarkLocations(x, P, sensor_readings, landmark_indices, R)
  assert(size(sensor_readings, 2) == 2);
  
  for i = 1:size(sensor_readings, 1)
    sensor_reading = sensor_readings(i, :);
    landmark_index = landmark_indices(i);
    
    [x, P] = predictFromSensorData(x, P, sensor_reading, landmark_index, R);
  end
  
end