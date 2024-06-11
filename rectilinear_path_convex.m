function [p_d,dot_p_d,ddot_p_d,dddot_p_d]=rectilinear_path_convex(s_d,dot_s_d,ddot_s_d,dddot_s_d,pi,pf)
p_d=(1-s_d)*pi+s_d*pf;
dot_p_d=(pf-pi)*dot_s_d;
ddot_p_d=(pf-pi)*ddot_s_d;
dddot_p_d=(pf-pi)*dddot_s_d;
end