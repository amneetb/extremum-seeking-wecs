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
K = w_wav^2*m-k; 
B_opt = b;


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
        model_name = 'MMS_ESC_B_SM';
        T_avg = 2*par.T;
        w_wav = 1/par.T*2*pi;
        N_delay = T_avg/par.sym.max_dt-1;


        W_K = w_wav*0.1;

        rho_B = 0.0002;   
        beta_B = 0.02;     
        k_B = .2;        
        
    case 2
        disp('Self Driving ESC selected')
        model_name = 'MMS_ESC_B_self_driving';
        
        w_wav=1/par.T*2*pi;
        w_L = w_wav*0.1;

        T_avg = 2*par.T;                
        N_delay = T_avg/par.sym.max_dt-1;

        sigma = 1.e-11;

        eta_B = 0.035;           
        lambda_B = 500;          

        sprintf('eta*lambda B = %f',eta_B* lambda_B)

        m1_B0 = 0;
        m2_B0 = .1;
        Q1_B0 = 10;
        Q2_B0 = 1;

        k_B=1;
        
    case 3
        disp('Relay Mode ESC selected')
        model_name = 'MMS_ESC_B_dither_free';
        
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;                
        N_delay = T_avg/par.sym.max_dt-1;
        w = w_wav;
        a1_B = 3;                
        w_L = w_wav*0.3;

        N_buff = 40;           
        R_B = 0.06;     
        amp_B = 2;     

        omega_B = w_wav/200;
        Sat = inf;
        k_B = 1;              
        
    case 4
        disp('Least-Sq. ESC selected')
        model_name = 'MMS_ESC_B_LSQ';
        
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        w_B = w_wav/200;

        W_K = w_wav*0.2;
        N_buff = 20;       

        a_B = 1.5;  

        k_B = 15;        
        Sat = 1.2;      

    case 5 
        disp('Perturbation Based ESC selected')
        model_name = 'MMS_ESC_B_classic_LOG';
        
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        w_B = w_wav/200;
        a1_B = 2;                 
        phase = 0   *pi/180;
        W_K = w_wav *0.3;
        w_1 = w_B   *0.1;
        w_L = w_B*0.1;          
        k_B = 25;                     
        Sat = 0.003;        
        
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
        B_0_vec =5;
    case 2
        disp('Multiple Initial Conditions selected')
        B_0_vec = [5 20 40 120];
    otherwise
        error('Incorrect input')
end



B_init_vec = B_0_vec/k_B;

for ii=1:length(B_0_vec)
     B_init = B_init_vec(ii);
     B_0 = B_0_vec(ii);
     out=sim(model_name);
     B_hat_vec(:,ii) = B_hat;
     Pot_filtrata_vec(:,ii) = Pot_filtrata;
end

%% Results
f = figure();
subplot(2,1,1)
plot(out,B_opt*ones(length(out),1),'r--','LineWidth', 2)
hold on
plot(out,B_hat_vec)
legendCell = {'C_{opt}'};
legend(legendCell)
title('Damping (Ns/m)')
xlabel('time (s)')

% legend('ESC','Opt')

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









