%% Point Absorber Simulink Model


%% INTRO
clear all
close all
clc

%% FUNCTIONS FOLDER

addpath([cd,'/Functions_model/']) %It contains functions like the Regular wave Analysis
addpath(genpath([cd,'/MODELSandPARAMETERS/']))% It contains the simulink models 
addpath([cd,'/Hulls/'])% It contains all the hulls
addpath([cd,'/Wave Forces/'])
% Parameters
load('cylinder128.mat')% It contains hydrodynamic data for the body coming from the LIS. File
load('FORCE_T0.625_H0.01_D8000_CYL.mat') % Load the matlab file with all the forces of the irregular wave
par.hull = hull;

%% Model Name and simulink parameters
L=1.28;
par.sym.TStart = 0;
par.sym.Outputdt = 0.01;
par.sym.Ttot = 200;      % Time simulation (s)
par.sym.TStop = par.sym.Ttot;
par.sym.max_dt = 0.01;
par.sym.solver = 'ode45'; %Numerical method that we will use in order to solve the differential equations
par.sym.cut = ( 0.1 * par.sym.Ttot/par.sym.Outputdt);  % Cut for time histories
model_name = 'Point_Absorber_1D';


par.hull.BetaPitch = 0;% Non linear damping coefficient


%% Analysis

%% Wave forces Characteristics


dir = 1; % direction
par.ramp_cycle = 5; % Transient period of the dynamics in the beginning of the simulation
par.WAVE.TIME = wave.TIME';
par.WAVE.forces{1,1}.TAU(:,3) = wave.TAU';
par.WAVE.ETA = wave.ETA';

%Hydrostatic Forces
Diam = 0.16; %Diameter of the sphere
R = Diam/2; %Radius of the sphere
g = 9.81;  % m/s^2 accelaration of gravity
volume = pi*R^2; %volume of the sphere in m^3

%Drag Forces
density_p = 1025;
Cdx = 1.16;
Cdz = 1.16;
Ax =Diam;
Az =Diam;
density_hull = 922.5;

m =volume*density_hull;
inertia = 1/2*m*R^2; % Inertia 

% Moorings of the point Absorber

weight = 0;% The total weight of the mooring line kg


Smax = 0.67*0.08;

Initial_L = 0.32 ;% Initial length of the mooring line
Anchor = [0 ;0 ;-30]; %Coordinates of the anchor
C = [0 ;0 ; 30-Initial_L]; % Coordinates of the mooring point


%% PTO Damping



%% Flags
Pto_flag = 1;%Put 1 if you want the force of PTO
drag_flag = 1;%Put 1 if you want the Drag forces
viscous_flag = 0;%Put 1 if you want the viscous flag
Drift_flag = 1;%Put 1 if you want the Drift flag
Hydrostatic_flag = 0; % If the body is submerged put 0(Always with moorings)
mooring_flag = 1 ;% Without moorings put 0/With moorings put 1
wave_flag = 1 ;% If you dont want waves put 0
pitch_flag = 1;


temp_mass = zeros(6); % Create the mass matrix
temp_mass(1,1) = m;
temp_mass(2,2) = m;
temp_mass(3,3) = m;
temp_mass(4,4) = 1/4*m*R^2;
temp_mass(5,5) = inertia;
temp_mass(6,6) = 1/4*m*R^2;


%% Map

k_vec = [1000:10:2500];
c_vec = [0:2:100];
numel = length(k_vec)*length(c_vec);
count =0;

f = waitbar(count/numel,sprintf('%d of %d simulations completed',count,numel));

for ii=1:length(k_vec)
    for jj=1:length(c_vec)
        K_stiffness = k_vec(ii);
        Bpto = c_vec(jj);
        Sinit = (density_p*g*volume - m*g)/K_stiffness;
        in = Simulink.SimulationInput(model_name);
        in = in.setVariable('K_stiffness',K_stiffness,'Workspace','global');
        in = in.setVariable('Bpto',Bpto,'Workspace','global');
        in = in.setVariable('Sinit',Sinit,'Workspace','global');
        simOut = sim(in);
        mean_power(ii,jj) = mean(simOut.POWER);
        mean_tot_pow(ii,jj) = mean(simOut.TOT_POW);
        count=count+1;
        sprintf('%d of %d simulations completed',count,numel)
        waitbar(count/numel,f,sprintf('%d of %d simulations completed',count,numel))
    end
end

delete(f)





%% PLOTS
figure()
surf(k_vec,c_vec,mean_power')
title('POW')

figure()
surf(k_vec,c_vec,mean_tot_pow')
title('TOT_POW')

save map_H0.0075_T1.mat

error('fine')




