function [p_d,dot_p_d,ddot_p_d]=circular_path_temp(s_d,dot_s_d,ddot_s_d,p0,radius,alfa)
  p0x = p0(1);
  p0y = p0(2)+radius;
  p0z = p0(3);

  %POSIZIONE
  p_dx = ones(1,length(s_d))*p0x;
  p_dy= p0y - radius*cos(alfa*s_d);
  p_dz= p0z - radius*sin(alfa*s_d);
  p_d=[p_dx;p_dy;p_dz];

  %VELOCITA
  dot_p_dx = zeros(1,length(dot_s_d));
  dot_p_dy = radius.*(alfa).*dot_s_d.*sin(alfa.*s_d);
  dot_p_dz = -radius.*(alfa).*dot_s_d.*cos(alfa.*s_d);
  dot_p_d=[dot_p_dx;dot_p_dy;dot_p_dz];
 
  %ACCELERAZIONE
  ddot_p_dx = zeros(1,length(ddot_s_d));
  ddot_p_dy = radius.*(alfa).*(dot_s_d.*dot_s_d.*alfa.*cos(alfa.*s_d)+ddot_s_d.*sin(2*alfa*s_d));
  ddot_p_dz = -radius.*(alfa).*(-dot_s_d.*dot_s_d.*alfa.*sin(alfa.*s_d)+ddot_s_d.*cos(2*alfa*s_d));
  ddot_p_d=[ddot_p_dx;ddot_p_dy;ddot_p_dz];
end