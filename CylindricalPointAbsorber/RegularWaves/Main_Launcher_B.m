
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
par.T = 1; %Period of the wave (s)
par.H = .0075; %Height of the wave (m)
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



weight = 0;% The total weight of the mooring line kg


%%

K_stiffness = 1530;
B_opt = 30;

Smax = 0.67*0.08;
Sinit = (density_p*g*volume - m*g)/K_stiffness;
Initial_L = 0.32 ;% Initial length of the mooring line
Anchor = [0 ;0 ;-30]; %Coordinates of the anchor
C = [0 ;0 ; 30-Initial_L]; % Coordinates of the mooring point




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
        model_name = 'Point_Absorber_1D_ESC_B_SM_MM';
        T_avg = 2*par.T;
        w_wav = 1/par.T*2*pi;
        N_delay = T_avg/par.sym.max_dt-1;

        W_K = w_wav*0.1;

        rho_B = 0.0002;   
        beta_B = 0.02;     
        k_B = 0.5;            
        
    case 2
        disp('Self Driving ESC selected')
        model_name = 'Point_Absorber_1D_ESC_B_Self_driving_MA';
        w_wav=1/par.T*2*pi;
        w_1 = w_wav*0.7;
        w_2 = w_wav*0.005;
        w_L = w_wav*0.01;

        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;

        sigma = 1.e-11;

        eta_B = 0.05;           
        lambda_B = 800;          

        sprintf('eta*lambda B = %f',eta_B* lambda_B)

        m1_B0 = 0;
        m2_B0 = .1;
        Q1_B0 = 10;
        Q2_B0 = 1;

        k_B=1;
        
    case 3
        disp('Relay Mode ESC selected')
        model_name = 'Point_Absorber_1D_ESC_B_Dither_Free_MA';
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        w = w_wav;
        a1_B = 5;                
        w_L = w_wav*1;        

        N_buff = 40;            
        R_B = 0.1;     
        amp_B = 5;
        
        omega_B = w_wav/100;     
        Sat = inf;
        k_B = 1;                       
        
    case 4
        disp('Least-Sq. ESC selected')
        model_name = 'Point_Absorber_1D_ESC_B_LSQ_MM';
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        w_B = w_wav/100;     

        W_K = w_wav*1;
        N_buff = 30;    

        a_B = 4;

        k_B = 40;
        Sat = 3;   

    case 5 
        disp('Perturbation Based ESC selected')
        model_name = 'Point_Absorber_1D_ESC_B_classic_LOG_MM';
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        w_B = w_wav/23;
        a1_B = 3;                 
        phase = 0   *pi/180;
        W_K = w_wav*1;
        w_1 = w_B*1;    
        w_L = w_B*0.1;          
        k_B = 130;                   
        Sat = 0.003;        
        
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
        B_0_vec =20;
    case 2
        disp('Multiple Initial Conditions selected')
        B_0_vec = [20 40 120 240 300];
    otherwise
        error('Incorrect input')
end

B_init_vec = B_0_vec/k_B;

for ii=1:length(B_0_vec)
    B_0 = B_0_vec(ii);
    B_init = B_init_vec(ii);
    out=sim(model_name);
    B_hat_vec(:,ii) = B_hat;
    Pot_filtrata_vec(:,ii) = Pot_filtrata;
    sprintf('%d of %d completed',ii,length(B_0_vec))
end

figure()
subplot(2,1,1)
plot(out,B_opt*ones(length(out),1),'b--','LineWidth', 2)

hold on
plot(out,B_hat_vec)
title('Damping coeff.')
xlabel('t(s)')
legend('C_{opt}')
ylabel('Ns/m')

subplot(2,1,2)
plot(out,Pot_filtrata_vec)
title('Power')
xlabel('t(s)')
ylabel('W')

error('fine')



