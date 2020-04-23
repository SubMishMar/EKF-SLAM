close all;

generate_landmarks

rk_true = zeros(3, 1);
rk = rk_true;
rk_motion_model = rk_true;

%% Tuning Params
q_tuning = [0.7;0.7];
Q_tuning = diag(q_tuning.^2);
r_tuning = [0.20; 3*pi/180];
R_tuning = diag(r_tuning.^2);

% System noise
q = [0.5;0.5];
Qk = diag(q.^2);
% Measurement noise
m = [0.15; 1*pi/180];
Rk = diag(m.^2);

dt = 0.01;

sensor_range = 40;

mapped_lmk_idx = [];
no_ts = 20000;

figure(1)
plot(landmark_XY(:, 1), landmark_XY(:, 2), 'r+','MarkerSize',20,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
hold on;
%plot_robot(rk_true, [1, 0, 0]);
%hold on;
%plot_robot(rk, [1, 0, 0]);
%hold on;

uk = [2; 0.1];
P = zeros(3+2*no_of_lmks, 3+2*no_of_lmks);
P(1:3, 1:3) = diag([0, 0, 0*180/pi]);
x = zeros(3+2*no_of_lmks, 1);
x(1:3) = [0; 0; 0];
rk = x(1:3);
rk_true = rk;
rk_motion_model = rk_true;
tracePrxy = [];
estimated_robot_pose = [];
motion_model_pose = [];
true_robot_pose = [];
for i=1:no_ts     
  %% Truth
  uk_true = uk + [sqrt(Qk(1))*randn(1,1); sqrt(Qk(4))*randn(1,1)];
  [rk_true, ~, ~, ~] = motion_model(rk_true, uk_true, dt, ...
                            zeros(3+2*no_of_lmks, 3+2*no_of_lmks), zeros(2,2));
  
  %% Prediction
  rk = x(1:3);
  [rk, P, Ak, Bk] = motion_model(rk, uk, dt, P, Q_tuning);
  [rk_motion_model, ~, ~, ~] = motion_model(rk_motion_model, uk, dt, zeros(3+2*no_of_lmks, 3+2*no_of_lmks), zeros(2,2));
  motion_model_pose = [motion_model_pose; rk_motion_model'];
  x(1:3) = rk;
  
  nearest_lmks_idx = [];
  if no_of_lmks > 0
    nearest_lmks_idx = getLandmarksinRange(rk_true, sensor_range, landmark_XY);
  end
  if isempty(mapped_lmk_idx) & ~isempty(nearest_lmks_idx)
    mapped_lmk_idx = [mapped_lmk_idx; nearest_lmks_idx];
    lmks_XY = landmark_XY(mapped_lmk_idx,:);
    sensor_readings= read_sensor_data (rk_true, lmks_XY, Rk);
    [x, P] = predictLandmarkLocations(x, P, sensor_readings, nearest_lmks_idx, R_tuning);
  else
    if ~isempty(nearest_lmks_idx)
      new_indices = [];
      old_indices = [];
      for j=1:length(nearest_lmks_idx)
        if isempty(find(nearest_lmks_idx(j) == mapped_lmk_idx))
          new_indices = [new_indices; nearest_lmks_idx(j)];
        else
          old_indices = [old_indices; nearest_lmks_idx(j)];
        end
      end
      if ~isempty(new_indices)
        new_lmk_idx = new_indices;
        mapped_lmk_idx = [mapped_lmk_idx; new_lmk_idx];
        new_lmks_XY = landmark_XY(new_lmk_idx,:);
        sensor_readings = read_sensor_data (rk_true, new_lmks_XY, Rk);
        [x, P] = predictLandmarkLocations(x, P, sensor_readings, new_lmk_idx, R_tuning);
        
      end
      
      if ~isempty(old_indices)
        %% This is the correction process
        old_lmk_idx = old_indices;
        old_lmks_XY = landmark_XY(old_lmk_idx,:);
        sensor_readings = read_sensor_data (rk_true, old_lmks_XY, Rk);
        [x, P] = update(x, P, sensor_readings, old_lmk_idx, R_tuning); 
      end
    end
  end
  
  
  %% Plotting stuff
  if mod(i, 100) == 0
    plot(rk_true(1), rk_true(2), 'r.', 'LineWidth', 5, 'MarkerSize',15,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
    hold on;
    plot(rk(1), rk(2), 'g.','MarkerSize',10,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
    hold on;
    %plot_robot(rk, [0, 1, 0]);
    %hold on;
    %plot_robot(rk_true, [1, 0, 0]);
    %hold on;
    plot_covEllipse(rk, P(1:2, 1:2), 3, 20);
    hold on;
    mapped_lmk_XY = landmark_XY(mapped_lmk_idx,:);
    x_indices = 3+2*mapped_lmk_idx-1;
    y_indices = x_indices+1;
    for count = 1:length(mapped_lmk_idx)
      current_lmk_xy = [x(x_indices(count)), x(y_indices(count))];
      plot_covEllipse(current_lmk_xy, P(x_indices(count):y_indices(count), x_indices(count):y_indices(count)), 3, 20);
      hold on;
      plot(current_lmk_xy(1), current_lmk_xy(2), 'gd');
      hold on;
    end
   end
   drawnow
   tracePrxy = [tracePrxy; trace(P(1:2, 1:2))];
   estimated_robot_pose = [estimated_robot_pose;x(1:3)'];
   true_robot_pose = [true_robot_pose;rk_true'];
end

figure(1)
grid;
axis square;
axis equal;
xlabel('X[m]', 'FontSize',12,'FontWeight','bold','Color','k');
ylabel('Y[m]', 'FontSize',12,'FontWeight','bold','Color','k');

estimation_err_x = true_robot_pose(:,1) - estimated_robot_pose(:,1);
estimation_err_y = true_robot_pose(:,2) - estimated_robot_pose(:,2);
estimation_err_theta = true_robot_pose(:,3) - estimated_robot_pose(:,3);

%%
figure(2)
subplot(311)
p = plot(true_robot_pose(:,1), '-r', 'LineWidth', 5);
hold on;
q = plot(estimated_robot_pose(:,1), '-g', 'LineWidth', 3);
hold on;
r = plot(estimation_err_x, '-b', 'LineWidth', 1);
hold off;
grid;
legend([p,q,r], {'true_{x}[m]', 'est_{x}[m]', 'error_{x}[m]'});

subplot(312)
p = plot(true_robot_pose(:,2), '-r', 'LineWidth', 5);
hold on;
q = plot(estimated_robot_pose(:,2), '-g', 'LineWidth', 3);
hold on;
r = plot(estimation_err_y, '-b', 'LineWidth', 1);
hold off;
grid;
legend([p,q,r], {'true_{y}[m]', 'est_{y}[m]', 'error_{y}[m]'});

subplot(313)
p = plot(true_robot_pose(:,3), '-r', 'LineWidth', 5);
hold on;
q = plot(estimated_robot_pose(:,3), '-g', 'LineWidth', 3);
hold on;
r = plot(estimation_err_theta, '-b', 'LineWidth', 1);
hold off;
grid;
legend([p,q,r], {'true_{yaw}', 'est_{yaw}', 'error_{yaw}'});

%%
figure(3)
p = plot(abs(estimation_err_x), '-r', 'LineWidth', 2);
hold on;
q = plot(abs(estimation_err_y), '-g', 'LineWidth', 2);
hold on;
r = plot(abs(estimation_err_theta), '-b', 'LineWidth', 2);
hold on;
s = plot(sqrt(tracePrxy), '-k', 'LineWidth', 2);
hold off;
grid;
legend([p,q,r,s], {'error_x[m]', 'error_y[m]', 'error_{yaw}[m]', 'trace(P_{xy})'});