%% Mass Spring Damper
 
%% INTRO
clear all
close all
clc

%% FOLDERS
addpath([cd,'/Models'])

%% Model Name and simulink parameters

par.sym.TStart = 0;
par.sym.Outputdt = 0.01;
par.sym.Ttot = 300;      % Time simulation (s)
par.sym.TStop = par.sym.Ttot;
par.sym.max_dt = 0.01;
par.sym.solver = 'ode45';  % Numerical method that we will use in order to solve the differential equations
par.sym.cut = ( 0.1 * par.sym.Ttot/par.sym.Outputdt);   %Cut for time histories

model_name = 'massa_molla_smorzatore';

%% Analysis

% Force
par.T = .5; %Period of the wave (s)
par.H = 20; %Height of the wave (m)
w_wav = 1/par.T*2*pi;


%% System Parameters
m = 18.55;
b = 15;
k = 200;
K_opt = w_wav^2 * m - k;
B_opt = b;


%% Simulation
k_vec = [1000:50:5000];
c_vec = [0:3:100];


numel = length(k_vec)*length(c_vec);
count =0;

f = waitbar(count/numel,sprintf('%d of %d simulations completed',count,numel));

mean_power = zeros(length(k_vec),length(c_vec));
min_apparent = zeros(length(k_vec),length(c_vec));
mean_apparent = zeros(length(k_vec),length(c_vec));

for ii=1:length(k_vec)
    for jj=1:length(c_vec)
        K = k_vec(ii);
        B = c_vec(jj);
        in = Simulink.SimulationInput(model_name);
        in = in.setVariable('K',K,'Workspace','global');
        in = in.setVariable('B',B,'Workspace','global');
        simOut = sim(in);
        LL = floor(length(simOut.TIME)/2);
        mean_power(ii,jj) = mean(simOut.POWER(LL:end));
        min_apparent(ii,jj) = mean(mink(simOut.Power_apparent(LL:end),10));
        mean_apparent(ii,jj) = mean(simOut.Power_apparent(LL:end));
        count=count+1;
        sprintf('%d of %d simulations completed',count,numel)
        waitbar(count/numel,f,sprintf('%d of %d simulations completed',count,numel))
    end
end
delete(f)

%% Results
zz = [-500:1:500];

figure()
surf(k_vec',c_vec',mean_power')
hold on
plot3(K_opt*ones(length(zz),1),B_opt*ones(length(zz),1),zz','LineWidth',2)
title('Active Power')
% plot(c_vec,mean_power)

figure()
surf(k_vec',c_vec',min_apparent')
hold on
plot3(K_opt*ones(length(zz),1),B_opt*ones(length(zz),1),zz','LineWidth',2)
title('Min Apparent Power')

figure()
surf(k_vec',c_vec',mean_apparent')
hold on
plot3(K_opt*ones(length(zz),1),B_opt*ones(length(zz),1),zz','LineWidth',2)
title('Mean Apparent Power')

error('fine')

