function [WAVE] = REG_WAVE_FORCES( par,T,H ,dir)

% Questa function riceve in ingresso i parametri dell'onda regolare T e H e dello scafo, e
% restituisce in output la forzante dovuta all'onda(pitch), comprendente le
% forze di Froude-Krilov e di diffrazione

%% Tauw time function
t = (0:par.sym.max_dt:2*par.sym.Ttot);                                          %(s) Vettore tempi 

TAU = zeros(length(t),6);

% surge Force
f1=interp1(par.hull.WAVE.prop.w_vec,par.hull.WAVE.forces{dir,1}.f(:,1),(2*pi/T)); %calcolo il coeff di Froude Krylov + diffrazione a quella frequenza
max_tau1 = H/2*abs(f1);                                   %ampiezza forzante beccheggio
Fx = max_tau1 * sin(2*pi/T*t);                           %andamento temporale forzante beccheggio(SENZA SFASAMENTO CON ETA)
TAU(:,1) = Fx;

% heave Force
f3=interp1(par.hull.WAVE.prop.w_vec,par.hull.WAVE.forces{dir,1}.f(:,3),(2*pi/T)); %calcolo il coeff di Froude Krylov + diffrazione a quella frequenza
max_tau3 = H/2*abs(f3);                                   %ampiezza forzante beccheggio
Fz = max_tau3 * sin(2*pi/T*t);                           %andamento temporale forzante beccheggio(SENZA SFASAMENTO CON ETA)
TAU(:,3) = Fx;

% Pitch
f5=interp1(par.hull.WAVE.prop.w_vec,par.hull.WAVE.forces{dir,1}.f(:,5),(2*pi/T)); %calcolo il coeff di Froude Krylov + diffrazione a quella frequenza
max_tau5 = H/2*abs(f5);                                   %ampiezza forzante beccheggio
Fry = max_tau5 * sin(2*pi/T*t);                           %andamento temporale forzante beccheggio(SENZA SFASAMENTO CON ETA)
TAU(:,5) = Fry;


WAVE.TIME = t;
end