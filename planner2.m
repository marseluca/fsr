function [s_d,dot_s_d,ddot_s_d,dddot_s_d,tot_time,t]=planner2(Ts,Ti,Tf,Tdead)
%% PLANNER
t_iniz = Ti;

ttot = Tf;
tdead=Tdead; %dead time to evaluate the steady-state
tot_time = ttot + tdead;
t1=linspace(0,ttot,round(ttot/Ts));
t=linspace(0,ttot+tdead,round(ttot/Ts)+round(tdead/Ts));

%% ARCLENGTH
s= zeros(1,length(t1)); dot_s = zeros(1,length(t1)); ddot_s = zeros(1,length(t1));
%Initial and final conditions
s0=0; sf=1; dot_s0= 0; dot_sf=0; ddot_s0=0; ddot_sf=0; dddot_s0 = 0; dddot_sf = 0;

%% 7-th order polynomial
a0=0; a1=0; a2=0; a3=0; a4=0; a5=0; a6=0; a7=0;

%% Computing coefficients
    A = [t_iniz^7, t_iniz^6, t_iniz^5, t_iniz^4, t_iniz^3, t_iniz^2, t_iniz, 1;
        ttot^7, ttot^6, ttot^5, ttot^4, ttot^3, ttot^2, ttot, 1;
        7*t_iniz^6, 6*t_iniz^5, 5*t_iniz^4, 4*t_iniz^3, 3*t_iniz^2, 2*t_iniz, 1, 0;
        7*ttot^6, 6*ttot^5, 5*ttot^4, 4*ttot^3, 3*ttot^2, 2*ttot, 1, 0;
        42*t_iniz^5, 30*t_iniz^4, 20*t_iniz^3, 12*t_iniz^2, 6*t_iniz, 2, 0, 0;
        42*ttot^5, 30*ttot^4, 20*ttot^3, 12*ttot^2, 6*ttot, 2, 0, 0;
        210*t_iniz^4, 120*t_iniz^3, 60*t_iniz^2, 24*t_iniz, 6, 0, 0, 0;
        210*ttot^4, 120*ttot^3, 60*ttot^2, 24*ttot, 6, 0, 0, 0];
    b = [s0 sf dot_s0 dot_sf ddot_s0 ddot_sf dddot_s0 dddot_sf]';
    a_temp = A\b;
    a7 = a_temp(1);
    a6 = a_temp(2);
    a5 = a_temp(3);
    a4 = a_temp(4);
    a3 = a_temp(5);
    a2 = a_temp(6);
    a1 = a_temp(7);
    a0 = a_temp(8);
    
    %% trajectories
    s(:)=a7*t1.^7 + a6*t1.^6 + a5*t1.^5 +a4*t1.^4 +a3*t1.^3 +a2*t1.^2 +a1*t1 +a0;
    dot_s(:) = 7*a7*t1.^6 + 6*a6*t1.^5 + 5*a5*t1.^4 +4*a4*t1.^3 +3*a3*t1.^2 +2*a2*t1 +a1;
    ddot_s(:) = 42*a7*t1.^5 + 30*a6*t1.^4 + 5*4*a5*t1.^3 +4*3*a4*t1.^2 +3*2*a3*t1 +2*a2;
    dddot_s(:) = 210*a7*t1.^4 + 120*a6*t1.^3 + 3*5*4*a5*t1.^2 + 2*4*3*a4*t1 +3*2*a3;

    %% addition of the stead-state terms
    s_d(:)=[s zeros(1,round(tdead/Ts))+sf]; 
    dot_s_d(:)=[dot_s zeros(1,round(tdead/Ts))+dot_sf];
    ddot_s_d(:)=[ddot_s zeros(1,round(tdead/Ts))+ddot_sf];
    dddot_s_d(:)=[dddot_s zeros(1,round(tdead/Ts))+dddot_sf];
end