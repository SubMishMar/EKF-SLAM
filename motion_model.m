function [rk, P, Ak, Bk] = motion_model(rk_1, uk, dt, P, Qk)
  assert(length(rk_1) == 3);
  assert(length(uk) == 2);
  assert(size(Qk, 1) == 2);
  assert(size(Qk, 2) == 2);
  rk_1 = rk_1(:); % forcing it be column vector
  uk = uk(:); % forcing it to be column vector
  
  xk_1 = rk_1(1);
  yk_1 = rk_1(2);
  thetak_1 = rk_1(3);
  
  vk = uk(1);
  omegak = uk(2);
  
  x_k = xk_1 + vk*dt*cos(thetak_1);
  y_k = yk_1 + vk*dt*sin(thetak_1);
  thetak = thetak_1 + omegak*dt;
  rk = [x_k; y_k; thetak];
 
  Ak = [1 0 -vk*sin(thetak)*dt;
        0 1 +vk*cos(thetak)*dt;
        0 0  1];
  
  Bk = [cos(thetak)*dt 0;
        sin(thetak)*dt 0;
        0                dt;];
  
  Prr = P(1:3, 1:3);  
  Prm = P(1:3, 4:end);
  Prr = Ak*Prr*Ak' + Bk*Qk*Bk'; 
  Prm = Ak*Prm;
  Pmr = Prm';
  
  P(1:3, 1:3) = Prr;
  P(1:3, 4:end) = Prm;
  P(4:end, 1:3) = Pmr;
end