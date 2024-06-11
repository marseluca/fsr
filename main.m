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