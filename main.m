
clear all
close all

%% PASSIVITY BASED CONTROL
Ts = 1e-3;

m = 1;
g = 9.81;
r = 5;

intv=[];
for i=1:r
    v(i).vec = zeros(6,1);
    intv = [intv,v(i).vec];
end

% BUTTERWORTH FILTER
[b,a] = butter(r,1,"low","s");
k = gainvalue(r,a(2:end));

%% SAMPLING TIME
Ts=0.001;
ttot = 0;
xf = 0; yf = 0;
totalTime = 0;
time_vec = [];

x_d = [];
dot_x_d = [];
ddot_x_d = [];
dddot_x_d = [];

y_d = [];
dot_y_d = [];
ddot_y_d = [];
dddot_y_d = [];

z_d = [];
dot_z_d = [];
ddot_z_d = [];
dddot_z_d = [];

psi_d = [];
dot_psi_d = [];
ddot_psi_d = [];
dddot_psi_d = [];

% path = [0,0;
%                     1,1;
%                     2,2;
%                     3,3;
%                     4,4;
%                     5,5;
%                     15,15];

reversePath = load("reversePath.mat").reversePath;
path = flip(reversePath);
% path = path(1:8,:);

for i=1:size(path,1)-1
    %% PLANNER

    % Note that t_iniz=0 because every trajectory is computed
    % as if it started from t=0.
    % But later, they will be set to start at t=totalTime
    t_iniz = 0;
    x0=path(i,1); y0=path(i,2); z0=-1;
    xf=path(i+1,1); yf=path(i+1,2); zf=-1;
    dot_x0=0; dot_y0=0; dot_z0=0;
    alfa=pi;

    % The velocity with which the drone executes the trajectory
    vel = 0.3;

    % The duration of the trajectory depends on the distance and the
    % velocity
    trajectory_duration = norm([xf,yf]-[x0,y0])/vel;
    ttot = trajectory_duration; % duration = norm / velocity

    % The duration of the steady state
    % After the trajectory has been executed
    tdead = trajectory_duration/3; 
    
    % Compute the s(t)
    [s_d,dot_s_d,ddot_s_d,dddot_s_d,tot_time,t] = planner2(Ts,t_iniz,ttot,tdead);
    
    % Build the time vector to plot the whole trajectories since t=0
    t = t+totalTime;
    time_vec = [time_vec, t];

    % Update the total time since t=0 to the last trajectory
    totalTime = totalTime + ttot + tdead;

    % Compute the i-th primitive
    [x_d_new,dot_x_d_new,ddot_x_d_new,dddot_x_d_new] = rectilinear_path_convex(s_d,dot_s_d,ddot_s_d,dddot_s_d,x0,xf);
    [y_d_new,dot_y_d_new,ddot_y_d_new,dddot_y_d_new] = rectilinear_path_convex(s_d,dot_s_d,ddot_s_d,dddot_s_d,y0,yf);
    [z_d_new,dot_z_d_new,ddot_z_d_new,dddot_z_d_new] = rectilinear_path_convex(s_d,dot_s_d,ddot_s_d,dddot_s_d,z0,zf);
    [psi_d_new,dot_psi_d_new,ddot_psi_d_new,dddot_psi_d_new] = rectilinear_path_convex(s_d,dot_s_d,ddot_s_d,dddot_s_d,0,deg2rad(0));
    
    % Put all the primitives since t=0 in one vector
    x_d = [x_d, x_d_new];
    dot_x_d = [dot_x_d, dot_x_d_new];
    ddot_x_d = [ddot_x_d, ddot_x_d_new];
    dddot_x_d = [dddot_x_d, dddot_x_d_new];

    y_d = [y_d, y_d_new];
    dot_y_d = [dot_y_d, dot_y_d_new];
    ddot_y_d = [ddot_y_d, ddot_y_d_new];
    dddot_y_d = [dddot_y_d, dddot_y_d_new];

    z_d = [z_d, z_d_new];
    dot_z_d = [dot_z_d, dot_z_d_new];
    ddot_z_d = [ddot_z_d, ddot_z_d_new];
    dddot_z_d = [dddot_z_d, dddot_z_d_new];

    psi_d = [psi_d, psi_d_new];
    dot_psi_d = [dot_psi_d, dot_psi_d_new];
    ddot_psi_d = [ddot_psi_d, ddot_psi_d_new];
    dddot_psi_d = [dddot_psi_d, dddot_psi_d_new];

end

p_d=[x_d;y_d;z_d];
dot_p_d=[dot_x_d;dot_y_d;dot_z_d];
ddot_p_d=[ddot_x_d;ddot_y_d;ddot_z_d];

%data for Simulink
pos_0 = [x0 y0 z0];
lin_vel_0 = [dot_x0 dot_y0 dot_z0];
w_bb_0 = [0 0 0];
csi_d=p_d(1:3,:); dot_csi_d=dot_p_d(1:3,:); ddot_csi_d=ddot_p_d(1:3,:);

simDuration = totalTime + 5;

figure
plot(time_vec,p_d);
legend('xd','yd','zd')

figure
plot(time_vec,dot_p_d)

figure
plot(time_vec,ddot_p_d)

% figure
% plot(t,dddot_p_d)


time_vec = sort(time_vec);

