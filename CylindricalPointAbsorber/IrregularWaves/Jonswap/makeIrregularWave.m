clear all;
close all
clc;

T_max = 2.49;            
T_min = 0.21;           

omega_begin = 2*pi/T_max;
omega_end = 2*pi/T_min;

N = 100;
Ohm = linspace(omega_begin, omega_end, N);

Hs = 0.0075;
Tp = 1;
depth = 0.65;            
gravity = 9.81;

seed = 1234;
rng(seed);
[ S, Amp, Phase ] = JONSWAP(Ohm, Hs, Tp);


alpha =  (depth/gravity)*Ohm.^2;
beta =  alpha.*tanh(alpha).^(-0.5);
kd = (alpha + beta.^2.*cosh(beta).^(-2))./(tanh(beta) + beta.*cosh(beta).^(-2));
k = kd/depth;

k_newton = zeros(1,N);
for i = 1:N
    dispersion2 = @(kguess,omega) omega^2 - gravity * kguess * tanh(kguess*depth);
    omega = Ohm(i);
    dispersion1 =  @(kguess) dispersion2(kguess,omega);
    k_newton(i) = fzero(dispersion1,omega^2/gravity);
end




%%
Ls = 2*pi/k(25);
x = 10.6;

t = 0:0.01:20000;
eta = zeros(1,length(t));

for i = 1:length(t)
    for j = 1: N      
       eta(i) = eta(i) + Amp(j)*cos(k(j)*x - Ohm(j)*t(i) + Phase(j));
    end
end

figure(1);
plot(t,eta)
title('time domain evolution of the wave')
xlabel('time (s)')
ylabel('eta (m)')

figure(2);
plot(Ohm,S)
title('wave spectrum')
xlabel('omega (rad*s)')


%% Calculate T_e


m__1 = trapz(Ohm,S.*Ohm.^-1);
m_2 = trapz(Ohm,S.*Ohm.^2);
m_0 = trapz(Ohm,S);
T_e = m__1/m_0*2*pi;
T_z = sqrt(m_0/m_2)*2*pi;
T_p_formula = (trapz(Ohm,S.^5))/(trapz(Ohm,S.^5.*Ohm))/0.95*2*pi;
m__2 = trapz(Ohm,S.*Ohm.^-2);
m_1 = trapz(Ohm,S.*Ohm.^1);
qty = 2*pi*(m__2*m_1)/(m_0^2*1.025);
if qty <= 1.25*T_e
    T_p_paper = qty;
else
    T_p_paper = 1.25*T_e;
end

Hm0 = 4*sqrt(trapz(Ohm,S));

NFFT = 2^(nextpow2(length(t)));

wave.TIME = t;
wave.Ohm = Ohm;
wave.ETA = eta;
wave.S = S;
wave.Hm0 = Hm0;
wave.Te = T_e;
wave.Tp = Tp;
wave.Pw0 = 0.49*wave.Hm0^2*wave.Te; 
wave.PwS = 0.49*Hs^2*wave.Tp; 
wave.NFFT = NFFT;
wave.Phase = Phase;
wave.Amp = Amp;

difference_in_power = (wave.Pw0-wave.PwS)/wave.PwS;

sprintf('error in power estimation: %f %%',difference_in_power)

filename = 'T1_H0.0075_D20000_CYL.mat';

save_dir = fullfile(cd,'\Waves');
save([save_dir,'/',filename],'wave')

%% Force Generation


addpath([cd,'/Hulls'])% It contains all the hulls

% Load Hull Parameters
load('cylinder128.mat')% It contains hydrodynamic data for the body coming from the LIS. File
par.hull = hull;


[wave] = Irr_Force(wave,par);

figure
plot(wave.TIME,wave.TAU)
title('Wave Heaving Force')

filename = 'FORCE_T1_H0.0075_D20000_CYL.mat';

save_dir = fullfile(cd,'\Wave Forces');
save([save_dir,'/',filename],'wave')
