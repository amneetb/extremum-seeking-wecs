function [wave] = Irr_Force(wave,par)

t = wave.TIME;
Ohm = wave.Ohm;
Amp = wave.Amp;
Phase = wave.Phase;

TAU = zeros(length(t),1);

%% RAMP DEFINITION
ramp_cycle = 5;      % Number of cycle for wave ramp
t_ramp = 0:0.01:ramp_cycle*wave.Tp;
ramp_1 = t_ramp/(ramp_cycle*wave.Tp);                 % Slope
ramp_2 = ones(length(t)-length(t_ramp),1);      % Constant profile
ramp = [ramp_1  ramp_2'];

%% WAVE PROFILE
wave.ETA = wave.ETA.*ramp;



%% FROUDE-KRYLOV COEFFICIENTS
% Heave axis

tot_cont = 0;

for ii=1:length(Ohm)
    fz = interp1(par.hull.WAVE.prop.w_vec,par.hull.WAVE.forces{1,1}.f(:,3),Ohm(ii));     % Froude-krylov coefficient interpolation (N/m)
    contribution = fz*exp(1i*(Ohm(ii)*t+Phase(ii)))*Amp(ii);                                                         % Wave force acting on the par.hull (N)
    tot_cont = tot_cont + contribution;
end

tot_cont = real(tot_cont);

TAU = tot_cont.*ramp;

wave.TAU = TAU;

end

