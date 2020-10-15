%% Mass Spring Damper

%% INTRO
clear all
close all
clc

%% FOLDERS
addpath(genpath([cd,'/Models']))

%% Model Name and simulink parameters

par.sym.TStart = 0;
par.sym.Outputdt = 0.01;
par.sym.Ttot = 5000;      % Time simulation (s)
par.sym.TStop = par.sym.Ttot;
par.sym.max_dt = 0.01;
par.sym.cut = ( 0.1 * par.sym.Ttot/par.sym.Outputdt);   %Cut for time histories


%% Analysis

% Force
par.T = 0.5; %Period of the wave (s)
par.H = 20; %Height of the wave (m)
w_wav = 1/par.T*2*pi;

%% System Parameters
m = 18.55;
k = 200;
b = 15;
K_opt = w_wav^2*m-k;
B = b;


%% Control Parameters
T_tran = 50;


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
        model_name = 'MMS_ESC_sliding_mode';
        T_avg = 2*par.T;
        w_wav = 1/par.T*2*pi;
        N_delay = T_avg/par.sym.max_dt-1;

        W_K = w_wav*0.1;

        rho_K = 0.001;   
        beta_K = 0.1;
        k_K = 10;        
        
    case 2
        disp('Self Driving ESC selected')
        model_name = 'MMS_ESC_self_driving';
        
        w_wav=1/par.T*2*pi;
        w_L = w_wav*0.1;

            T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;

        sigma = 1.e-11;

        eta_K = 0.03;                
        lambda_K = 120000;          
        sprintf('eta*lambda K = %f\n',eta_K* lambda_K)

        m1_K0 = 1;
        m2_K0 = 1;
        Q1_K0 = 10;         
        Q2_K0 = 1;

        k_K=1;
        
    case 3
        disp('Relay Mode ESC selected')
        model_name = 'MMS_ESC_dither_free';
        
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;        
        N_delay = T_avg/par.sym.max_dt-1;
        w = w_wav;
        a1 = 50;                
        w_L = w_wav*0.1;

        N_buff = 40;            
        R = 5.e-4;                  
        amp = 15;                    

        omega = w_wav/200;          
        Sat = inf;

        k_K=3500;                  
        
    case 4
        disp('Least-Sq. ESC selected')
        model_name = 'MMS_ESC_LSQ';
        
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        w_K = w_wav/200;

        W_K = w_wav*0.2;
        N_buff = 40;

        a_K = 10;

        k_K = 6000;
        Sat = 1;

    case 5 
        disp('Perturbation Based ESC selected')
        model_name = 'MMS_ESC_classic_LOG';
        
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        w = w_wav/20;
        a1 = 20;                 
        phase = 0   *pi/180;
        W_K = w_wav*1;
        w_1 = w*0.7;

        w_L = w*0.1;         
        k_K = 2200;               
        Sat = .003;  
        
    otherwise
        error('Incorrect Input')
end

str = strcat('Choose your preference (insert number)\n',...
    '- 1 : Single Initial Condition\n',...
    '- 2 : Multiple Initial Conditions (one for each simulation)\n');
str=(sprintf(str));

IN = input(str);

switch IN
    case 1
        disp('Single Initial Condition selected')
        K_0_vec = [7000];
    case 2
        disp('Multiple Initial Conditions selected')
        K_0_vec = [1000 2000 3000 4000 5000 6000 7000];
    otherwise
        error('Incorrect input')
end



%% Simulation


K_init_vec = K_0_vec/k_K;

for ii=1:length(K_0_vec)
     K_init = K_init_vec(ii);
     K_0 = K_0_vec(ii);
     out=sim(model_name);
     K_hat_vec(:,ii) = K_hat;
     Pot_filtrata_vec(:,ii) = Pot_filtrata;
end

%% Results
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










