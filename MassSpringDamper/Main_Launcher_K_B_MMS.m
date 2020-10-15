%% Mass Spring Damper
 
%% INTRO
clear all
close all
clc

%% FOLDERS
addpath(genpath([cd,'/Models']))

%% Simulation Parameters
%% Model Name and simulink parameters

par.sym.TStart = 0;
par.sym.Outputdt = 0.01;
par.sym.Ttot = 10000;      % Time simulation (s)
par.sym.TStop = par.sym.Ttot;
par.sym.max_dt = 0.01;
par.sym.cut = ( 0.1 * par.sym.Ttot/par.sym.Outputdt);   


%% Analysis

% Force
par.T = .5; %Period of the wave (s)
par.H = 20; %Height of the wave (m)
w_wav = 1/par.T*2*pi;


%% System Parameters
m = 18.55;
k = 200;
b = 15;             
K_opt = w_wav^2*m-k; 
B_opt = b;

T_tran=50;

str = strcat('Choose the ESC algorithm (insert number)\n',...
    '- 1 : Sliding Mode ESC\n',...
    '- 2 : Self Driving + SM ESC\n',...
    '- 3 : Relay Mode ESC\n',...
    '- 4 : Least-Sq. ESC\n',...
    '- 5 : Perturbation Based ESC\n', ...
    '- 6 : Perturbation Based ESC tot\n');
str=(sprintf(str));

IN = input(str);

switch IN
    case 1
        disp('Sliding Mode ESC selected')
        model_name = 'MMS_ESC_K_B_SM';
        
        T_avg = 2*par.T;
        w_wav = 1/par.T*2*pi;
        N_delay = T_avg/par.sym.max_dt-1;


        W_K = w_wav*0.1;

        rho_K = 0.005;   
        beta_K = 0.06;      
        k_K = 8;        

        rho_B = 0.002;    
        beta_B = 0.07;     	
        k_B = 0.8;        
        
    case 2
        disp('Self Driving ESC selected')
        model_name = 'MMS_ESC_K_B_mixed_3';
        
        w_wav=1/par.T*2*pi;


        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;

        sigma = 1.e-11;

        eta_K = 0.05;           
        lambda_K = 90000;           
        eta_B = 0.035;           
        lambda_B = 500;          

        m1_K0 = 1;          
        m1_B0 = 1;
        m2_K0 = 1;
        m2_B0 = .1;
        Q1_K0 = 10;         
        Q1_B0 = 1;
        Q2_K0 = 1;         
        Q2_B0 = 1;         

        k_K = 1;
        k_B = 1;
        
        %SM K + SD B        
%         W_K = w_wav*0.1;
%         W_B = w_wav*0.01;
        
        %SD K + SM B
        W_K = w_wav*0.01;
        W_B = w_wav*0.1;
        
        
        %SM
        rho_K = 0.0006;   
        beta_K = 0.06;
        k_K = 10;        

        rho_B = 0.0006;   
        beta_B = 0.06;      
        k_B = .55;        
        
    case 3
        disp('Relay Mode ESC selected')
        model_name = 'MMS_ESC_K_B_dither_free';
        
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        w = w_wav;

        a1 = 50;          
        a1_B = 5;        

        w_L_B = w_wav*0.2;
        w_L_K = w_wav*0.01;

        N_buff_K = 40;             
        N_buff_B = 40;

        k_K = 1;                  % 1 molto più influente di N_buff
        k_B = 1;                % 1 molto più influente di N_buff

        R = 2.5;                % 2.5 se grande ho instabilità
        R_B = 0.08;              % 0.4
        amp = 20;               % 60 se piccolo converge dopo, se grande ho rumore   
        amp_B = 1;              % 5 se piccolo converge dopo, se grande ho rumore ma converge un po prima

        omega = w_wav/180;
        omega_B = w_wav/220;
        
    case 4
        disp('Least-Sq. ESC selected')
        model_name = 'MMS_ESC_K_B_LSQ';
        
        w_wav=1/par.T*2*pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;
        N_buff = 40;
        W_K = w_wav*0.2;       %*0.1

        w_K = w_wav/140;        %140
        w_B = w_wav/130;        %30

        a_K = 40;
        a_B = 4;

        k_K = 2500;       %300000
        k_B = 20;          %20

        w_L_K = w_K*0.2;
        w_L_B = w_B*0.2;
        Sat = .05;    
    case 5 
        disp('Perturbation Based ESC selected')
        model_name = 'MMS_ESC_K_B_classic_LOG';
        
        w_wav = 1/par.T * 2 * pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;

        W_K = w_wav*1;

        % Parametri K
        w_K = w_wav/20;    
        a_K = 25;                 
        w_1_K = w_K*0.5;
        w_L_K = w_K*0.1;
        k_K = 800;                  

        % Parametri B
        w_B = w_wav/23;     
        a_B = 3;                    
        w_1_B = w_B*1;
        w_L_B = w_B*0.1;            
        k_B = 300;                

        phase = 0   *pi/180;
        Sat = .03; 
        Sat_B = 0.003;
        
    case 6 
        disp('Perturbation Based ESC total selected')
        model_name = 'MMS_ESC_K_B_classic_LOG_tot';
        
        w_wav = 1/par.T * 2 * pi;
        T_avg = 2*par.T;
        N_delay = T_avg/par.sym.max_dt-1;

        W_K = w_wav*0.3;

        % Parametri K
        w_K = w_wav/150;
        a_K = 25;                 
        w_1_K = w_K*0.5;
        w_L_K = w_K*0.5;
        k_K = 1200;                  

        % Parametri B
        w_B = w_wav/30;
        a_B = 0.3;                   
        w_1_B = w_B*0.5;
        w_L_B = w_B*0.1;            
        k_B = 400;                

        phase = 0   *pi/180;
        Sat = .03; 
        Sat_B = 0.003;
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
        B_0_vec =120;
        K_0_vec = 6500;
    case 2
        disp('Multiple Initial Conditions selected')
        B_0_vec = [5 20 40 120];
        K_0_vec = [1000 2000 4000 6000 7000];
    otherwise
        error('Incorrect input')
end



B_init_vec = B_0_vec/k_B;
K_init_vec = K_0_vec/k_K;

for ii=1:length(B_0_vec)
    B_0 = B_0_vec(ii);
    B_init = B_init_vec(ii);
    K_0 = K_0_vec(ii);
    K_init = K_init_vec(ii);
    out=sim(model_name);
    B_hat_vec(:,ii) = B_hat;
    K_hat_vec(:,ii) = K_hat;
    Pot_filtrata_vec(:,ii) = Pot_filtrata;
end

f = figure();
subplot(3,1,1)
plot(out,B_opt*ones(length(out),1),'b--','LineWidth', 2)
hold on
plot(out,B_hat_vec)
legendCell = {'C_{opt}'};
legend(legendCell)
title('Damping (Ns/m)')
xlabel('time (s)')

subplot(3,1,2)
plot(out,K_opt*ones(length(out),1),'r--','LineWidth', 2)
hold on
plot(out,K_hat_vec)
legendCell = {'K_{opt}'};
legend(legendCell)
title('Stiffness (N/m)')
xlabel('time (s)')

subplot(3,1,3)
plot(out,Pot_filtrata_vec)
title('Power(W)')
xlabel('time (s)')

bFig(1:3) = axes(f);
for k = 1:numel(bFig)
    bFig(k) = subplot(3,1,k);
end
for i=1:numel(bFig)-1
    linkaxes([bFig(i), bFig(i+1)], 'x');
end

figure()
plot(out,F_ext./(max(F_ext)),'r')
hold on
plot(out,x_dot./(max(x_dot)),'g')
legend('Ext. Force','velocity')






