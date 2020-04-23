function [x, P] = update(x, P, sensor_readings, old_lmk_idx, R)
  x_indices = 3+2*old_lmk_idx-1;
  y_indices = x_indices+1;
  x_lmk = [x(x_indices), x(y_indices)];
  assert(size(sensor_readings,1) == size(x_lmk,1));
  assert(size(sensor_readings,2) == size(x_lmk,2));
  x_r = x(1:3);
  for i = 1:size(sensor_readings, 1)
    yi = sensor_readings(i,:)';
    li = x_lmk(i,:)';
    [yi_predicted, H_r, H_l] = sensor_model(x_r, li, zeros(2,2));
    z = yi - yi_predicted;
    z(2) = atan2(sin(z(2)), cos(z(2)));
    Prr = P(1:3, 1:3);
    Prl = P(1:3, x_indices(i):y_indices(i));
    Plr = Prl';
    Pll = P(x_indices(i):y_indices(i), x_indices(i):y_indices(i));
    Pmr = P(4:end, 1:3);
    Pml = P(4:end, x_indices(i):y_indices(i));
    P_RL = [Prr, Prl;
            Plr, Pll];
    Z = [H_r, H_l] * P_RL * [H_r';H_l']+R;
    K = [Prr Prl; Pmr Pml]*[H_r';H_l']*inv(Z);
    x = x + K*z;
    P = P - K*Z*K';
  end
end  