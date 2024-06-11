
clear all
close all

%% SAMPLING TIME
Ts=0.001;

%% PLANNER
t_iniz = 0;
% ttot=20; %duration
tdead=10; %dead time to evaluate the steady-state

%
x0=0; y0=0; z0=-1;
xf=1; yf=1; zf=-1;
dot_x0=0; dot_y0=0; dot_z0=0;
alfa=pi;
vel = 0.3;
ttot = norm([xf,yf]-[x0,y0])/vel %duration = norm / velocity

[s_d,dot_s_d,ddot_s_d,dddot_s_d,tot_time,t] = planner2(Ts,t_iniz,ttot,tdead);

% figure
% plot(t,s_d)
% 
% figure
% plot(t,dot_s_d)
% 
% figure
% plot(t,ddot_s_d)
% 
% figure
% plot(t,dddot_s_d)



[psi_d,dot_psi_d,ddot_psi_d,dddot_psi_d] = rectilinear_path_convex(s_d,dot_s_d,ddot_s_d,dddot_s_d,0,deg2rad(0));
[x_d,dot_x_d,ddot_x_d,dddot_x_d]=rectilinear_path_convex(s_d,dot_s_d,ddot_s_d,dddot_s_d,x0,xf);
[y_d,dot_y_d,ddot_y_d,dddot_y_d]=rectilinear_path_convex(s_d,dot_s_d,ddot_s_d,dddot_s_d,y0,yf);
[z_d,dot_z_d,ddot_z_d,dddot_z_d]=rectilinear_path_convex(s_d,dot_s_d,ddot_s_d,dddot_s_d,z0,zf);

p_d=[x_d;y_d;z_d];
dot_p_d=[dot_x_d;dot_y_d;dot_z_d];
ddot_p_d=[ddot_x_d;ddot_y_d;ddot_z_d];

% [p_d,dot_p_d,ddot_p_d] = circular_path_temp(s_d,dot_s_d,ddot_s_d,[x0,y0,z0],1,alfa);

%data for Simulink
pos_0 = [x0 y0 z0];
lin_vel_0 = [dot_x0 dot_y0 dot_z0];
w_bb_0 = [0 0 0];
csi_d=p_d(1:3,:); dot_csi_d=dot_p_d(1:3,:); ddot_csi_d=ddot_p_d(1:3,:);

figure
plot(t,p_d);
legend

figure
plot(t,dot_p_d)

figure
plot(t,ddot_p_d)

% figure
% plot(t,dddot_p_d)




