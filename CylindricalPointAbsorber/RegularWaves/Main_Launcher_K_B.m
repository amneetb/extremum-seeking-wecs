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
par.sym.solver = 'ode45';  % Numerical method that we will use in order to solve the differential equations
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

K_opt = 3720;
B_opt = 18;


T_tran = 50;        %transient length

str = strcat('Choose the ESC algorithm (insert number)\n',...
    '- 1 : Sliding Mode ESC\n',...
    '- 2 : Self Driving\n',...
    '- 3 : Relay Mode ESC\n',...
    '- 4 : Least-Sq. ESC\n',...
    '- 5 : Perturbation Based ESC\n', ...
    '- 6 : Perturbation Based ESC tot\n',...
    '- 7 : Haring ES\n');
str=(sprintf(str));

IN = input(str);

switch IN
    case 1
        disp('Sliding Mode ESC selected')
        model_name = 'Point_Absorber_1D_ESC_K_B_SM_MM';
        T_avg = 2*par.T;
        w_wav = 1/par.T*2*pi;
        N_delay = T_avg/par.sym.max_dt-1;

        W_K = w_wav*0.1;

        rho_K = 0.005;   
        beta_K = 0.06;   
        k_K = 16;       

        rho_B = 0.003;   
        beta_B = 0.07;  
        k_B = 1.2;             
        
    case 2
        disp('Self Driving ESC selected')
        model_name = 'Point_Absorber_1D_ESC_K_B_Self_driving_MM';
        w_wav=1/par.T*2*pi;

        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;

        sigma = 1.e-11;

        eta_K = 0.08;           
        eta_B = 0.05;           
        lambda_K = 500000;           
        lambda_B = 1000;          

        m1_K0 = 1;          
        m1_B0 = 0;
        m2_K0 = 1;
        m2_B0 = .1;
        Q1_K0 = 10;         
        Q1_B0 = 10;
        Q2_K0 = 1;         
        Q2_B0 = 1;          

        k_K = 1;
        k_B = 1;

        w_L = w_wav*0.5;    
        
    case 3
        disp('Relay Mode ESC selected')
        model_name = 'MMS_ESC_K_B_dither_free';
        
        model_name = 'Point_Absorber_1D_ESC_K_B_Dither_Free_MM';
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        w = w_wav;

        a1 = 50;                
        a1_B = 5;                
        w_L_B = w_wav*0.5;
        w_L_K = w_wav*0.5;

        N_buff_K = 40;            
        N_buff_B = 40;

        k_K = 1;                 
        k_B = 1;                
        R = 1;                
        R_B = 0.2;              
        amp = 35;                 
        amp_B = 0.5;              

        omega = w_wav/40;
        omega_B = w_wav/50;
        
    case 4
        disp('Least-Sq. ESC selected')
        model_name = 'Point_Absorber_1D_ESC_K_B_LSQ_MM';
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        N_buff = 20;
        W_K = w_wav*1;       

        w_K = w_wav/30;
        w_B = w_wav/45;

        a_K = 40;
        a_B = 5;

        k_K = 13000;       
        k_B = 130;          

        Sat = 3;  
    case 5 
        disp('Perturbation Based ESC selected')
        model_name = 'Point_Absorber_1D_ESC_K_B_classic_LOG_MM';
        w_wav = 1/par.T * 2 * pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;

        W_K = w_wav*1;        

        %  K
        w_K = w_wav/20;
        a_K = 40;                 
        w_1_K = w_K*0.5;        
        w_L_K = w_K*1;        
        k_K = 800;                  

        % B
        w_B = w_wav/23;
        a_B = 3;                    
        w_1_B = w_B*1;        
        w_L_B = w_B*0.1;            
        k_B = 80;                

        phase = 0   *pi/180;
        Sat = .007;     
        
    case 6 
        disp('Perturbation Based ESC total selected')
        model_name = 'Point_Absorber_1D_ESC_K_B_classic_LOG_MM_tot';
        w_wav = 1/par.T * 2 * pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;

        W_K = w_wav*0.5;

        % K
        w_K = w_wav/20;
        a_K = 40;                 
        w_1_K = w_K*0.7;
        w_L_K = w_K*0.5;
        k_K = 800;                   

        % B
        w_B = w_wav/23;
        a_B = 3;                    
        w_1_B = w_B*0.7;
        w_L_B = w_B*0.5;            
        k_B = 100;                

        phase = 0   *pi/180;
        Sat = .007; 
        
    case 7 
        disp('Haring method selected')
        model_name = 'Point_Absorber_1D_ESC_K_B_Haring_MM';
        w_wav = 1/par.T * 2 * pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        N_MOPP = par.T*2/par.sym.max_dt-1;

        W_K = w_wav*0.1;        

        % K
        w_K = w_wav/50;
        a_K = 40;                 
        w_1_K = w_K*0.5;        
        w_L_K = w_K*.1;        
        k_K = 40000;                   

        % B
        w_B = w_wav/60;
        a_B = 3;                    
        w_1_B = w_B*1;       
        w_L_B = w_B*0.1;            
        k_B = 400;                

        phase = 0   *pi/180;
        Sat = .007;    
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
        B_0_vec =350;
        K_0_vec = 6000;
    case 2
        disp('Multiple Initial Conditions selected')
        B_0_vec = [30 120  200 350];
        K_0_vec = [1000 2500  4500 6000];
    otherwise
        error('Incorrect input')
end


B_init_vec = B_0_vec/k_B;
K_init_vec = K_0_vec/k_K;

for ii=1:length(K_0_vec)
    B_0 = B_0_vec(ii);
    B_init = B_init_vec(ii);
    K_0 = K_0_vec(ii);
    K_init = K_init_vec(ii);
    out=sim(model_name);
    B_hat_vec(:,ii) = B_hat;
    K_hat_vec(:,ii) = K_hat;
    Pot_filtrata_vec(:,ii) = Pot_filtrata;
    time_vec(:,ii) = out;
    sprintf('%d of %d completed', ii,length(K_0_vec))
end



figure()
subplot(3,1,1)
plot(out,B_opt*ones(length(out),1),'b--','LineWidth', 2)
hold on
plot(time_vec,B_hat_vec)
legend('C_{opt}')
title('Damping (Ns/m)')
xlabel('time(s)')

subplot(3,1,2)
plot(out,K_opt*ones(length(out),1),'r--','LineWidth', 2)
hold on
plot(time_vec,K_hat_vec)
legend('K_{opt}')
title('Stiffness (N/m)')
xlabel('time(s)')

subplot(3,1,3)
plot(time_vec,Pot_filtrata_vec)
title('Power (W)')
xlabel('time(s)')

error('fine')


