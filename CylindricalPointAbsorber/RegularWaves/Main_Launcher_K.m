
%% Point Absorber Simulink Model 


%% INTRO
clear all
close all
clc

%% FUNCTIONS FOLDER
addpath([cd,'/Functions']) %It contains functions like the Regular wave Analysis
addpath([cd,'/Hulls/IBAMR Hulls'])
addpath([cd,'/MODELSandPARAMETERS'])% It contains the simulink models 
addpath(genpath([cd,'/MODELSandPARAMETERS_MA']))
addpath([cd,'/Hulls'])% It contains all the hulls

% Parameters
load('cylinder8_04.mat')% It contains hydrodynamic data for the body coming from the LIS. File


par.hull = hull;

%% Model Name and simulink parameters

par.sym.TStart = 0;
par.sym.Outputdt = 0.01;
par.sym.Ttot = 5000;      % Time simulation (s)
par.sym.TStop = par.sym.Ttot;
par.sym.max_dt = 0.01;
par.sym.cut = ( 0.1 * par.sym.Ttot/par.sym.Outputdt);   %Cut for time histories

par.hull.BetaPitch = 0;% Non linear damping coefficient


%% Analysis

% Wave

L = 1.28;
par.T = 0.625; %Period of the wave (s)
par.H = 0.01; %Height of the wave (m)
dir = 1; % direction
par.ramp_cycle = 5; % Transient period of the dynamics in the beginning of the simulation
[WAVE] = FUNC_RegularWaveAnalysis_3DOF(par,dir);% It produces data about the wave(excitation forces)
par.WAVE = WAVE;

%Hydrostatic Forces
Diam = 0.16; %Diameter of the sphere
L = 8*Diam;
R = Diam/2; %Radius of the sphere
g = 9.81;  % m/s^2 accelaration of gravity
volume = pi*R^2; %volume of the sphere in m^3
density_hull = 922.5;

%Drag Forces
density_p = 1025;
Cdx = 1.16;
Cdz = 1.16;
Ax =Diam;
Az =Diam;


m =volume*density_hull;
inertia = 1/2*m*R^2; 

% Moorings of the point Absorber

weight = 0;% The total weight of the mooring line kg


Smax = 0.67*0.08;
Initial_L = 0.32 ;% Initial length of the mooring line
Anchor = [0 ;0 ;-30]; %Coordinates of the anchor
C = [0 ;0 ; 30-Initial_L]; % Coordinates of the mooring point




%% PTO Damping

Bpto = 18;
K_opt = 3720;

%% Flags
Pto_flag = 1;%Put 1 if you want the force of PTO
drag_flag = 1;%Put 1 if you want the Drag forces
viscous_flag = 0;%Put 1 if you want the viscous flag
Drift_flag = 1;%Put 1 if you want the Drift flag
Hydrostatic_flag = 0; % If the body is submerged put 0(Always with moorings)
mooring_flag = 1 ;% Without moorings put 0/With moorings put 1
wave_flag = 1 ;% If you dont want waves put 0
pitch_flag = 1;
couple_flag = 1;


temp_mass = zeros(6); % Create the mass matrix
temp_mass(1,1) = m;
temp_mass(2,2) = m;
temp_mass(3,3) = m;
temp_mass(4,4) = 1/4*m*R^2;
temp_mass(5,5) = inertia;
temp_mass(6,6) = 1/4*m*R^2;




T_tran = 50;        %transient length

str = strcat('Choose the ESC algorithm (insert number)\n',...
    '- 1 : Sliding Mode ESC\n',...
    '- 2 : Self Driving ESC\n',...
    '- 3 : Relay Mode ESC\n',...
    '- 4 : Least-Sq. ESC\n',...
    '- 5 : Perturbation Based ESC\n');
str=(sprintf(str));

IN = input(str);

switch IN
    case 1
        disp('Sliding Mode ESC selected')
        model_name = 'Point_Absorber_1D_ESC_SM_MM';
        T_avg = 2*par.T;
        w_wav = 1/par.T*2*pi;
        N_delay = T_avg/par.sym.max_dt-1;

        W_K = w_wav*0.1;

        rho_K = 0.001;   
        beta_K = 0.06;
        k_K = 11;          
        
    case 2
        disp('Self Driving ESC selected')
        model_name = 'Point_Absorber_1D_ESC_Self_driving_MA';
        w_wav=1/par.T*2*pi;
        w_L = w_wav*0.01;

        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;

        sigma = 1.e-11;

        eta_K = 0.08;                       
        lambda_K = 100000;           
        sprintf('eta*lambda K = %f',eta_K* lambda_K)

        m1_K0 = 1;
        m2_K0 = 1;
        Q1_K0 = 10;         
        Q2_K0 = 1;

        k_K=1;
        
    case 3
        disp('Relay Mode ESC selected')
        model_name = 'Point_Absorber_1D_ESC_Dither_Free_MA';
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        w = w_wav;
        a1 = 50;                
        w_L = w_wav*1;

        N_buff = 40;            
        R = 1;                  
        amp = 35;                     

        omega = w_wav/100;
        Sat = inf;

        k_K=1;               
        
    case 4
        disp('Least-Sq. ESC selected')
        model_name = 'Point_Absorber_1D_ESC_LSQ_MM';
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        w_K = w_wav/40;            

        W_K = w_wav*0.5;
        N_buff = 40;

        a_K = 40;

        k_K = 3000;     
        Sat = 3;

    case 5 
        disp('Perturbation Based ESC selected')
        model_name = 'Point_Absorber_1D_ESC_classic_LOG_MM';
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        w = w_wav/50;
        a1 = 40;                 
        phase = 0   *pi/180;
        W_K = w_wav*0.5;
        w_1 = w*0.7;
        w_L = w*0.1;          
        k_K = 400;                   
        Sat = 0.03;    
        
    otherwise
        error('Incorrect Input')
end




%%
open_system(model_name,'loadonly'); 

str = strcat('Choose your preference (insert number)\n',...
    '- 1 : Single Initial Condition\n',...
    '- 2 : Multiple Initial Conditions (one for each simulation)\n');
str=(sprintf(str));

IN = input(str);

switch IN
    case 1
        disp('Single Initial Condition selected')
        K_0_vec = [6000];
    case 2
        disp('Multiple Initial Conditions selected')
        K_0_vec = [1000 2000 3000 4000 5000 6000];
        error('Incorrect input')
end


K_init_vec = K_0_vec/k_K;

for ii=1:length(K_0_vec)
     K_init = K_init_vec(ii);
     K_0 = K_0_vec(ii);
     out=sim(model_name);
     K_hat_vec(:,ii) = K_hat;
     Pot_filtrata_vec(:,ii) = Pot_filtrata;
     sprintf('%d of %d completed', ii,length(K_0_vec))
end



f = figure();
subplot(2,1,1)
plot(out,K_opt*ones(length(out),1),'r--','LineWidth', 2)
hold on
plot(out,K_hat_vec)
legendCell = {'K_{opt}'};
legend(legendCell)
title('Stiffness (N/m)')
xlabel('time (s)')


subplot(2,1,2)
plot(out,Pot_filtrata_vec)
title('Power (W)')
xlabel('time (s)')

bFig(1:2) = axes(f);
for k = 1:numel(bFig)
    bFig(k) = subplot(2,1,k);
end
for i=1:2
    linkaxes([bFig(1), bFig(2)], 'x');
end


error('fine')

