function plot_stuff (x, P, lmk_indices, landmark_XY, r_true)
  x_indices = 3+2*lmk_indices-1;
  y_indices = x_indices+1;
  
  robot_pose = x(1:3);
  plot(r_true(1), r_true(2), 'ro', 'MarkerSize',15,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[1,0,0]);
  hold on;
  plot(robot_pose(1), robot_pose(2), 'go','MarkerSize',10,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0,1,0]);
  hold on;
  for i=1:length(lmk_indices)
    landmark_x = x(x_indices(i));
    landmark_y = x(y_indices(i));
    landmark_x_true = landmark_XY(lmk_indices(i), 1);
    landmark_y_true = landmark_XY(lmk_indices(i), 2);
    P_llxy = P(x_indices(i):x_indices(i)+1, x_indices(i):x_indices(i)+1);
    plot(landmark_x, landmark_y, 'gd');
    hold on;
    plot(landmark_x_true, landmark_y_true, 'b+','MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0,1,0]);
    hold on;
    %plot_covEllipse(robot_pose, P(1:2, 1:2), 3, 20)
    %hold on;
    plot_covEllipse([landmark_x, landmark_y], P_llxy, 3, 20)
    hold on;
  end
end
