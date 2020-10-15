function [WAVE] = FUNC_RegularWaveAnalysis_3DOF(par,dir)

% This function builds a structure that contains the regular wave profile
% and the regular wave forces along surge, heave and pitch directions. The
% regular wave properties are given in "Launcher_SM_PEWEC_3DOF_RegWaves.m" 
% file.
% 
%
% Author: Nicola Pozzi
%
%
% ##### Syntax: 
% [wave] = FUNC_RegularWaveAnalysis_3DOF(par,dir)
%
%
% ##### Input description:
% "InputPar" is a structure built in the "Launcher_SM_PEWEC_3DOF_RegWaves.m"
% and it contains the following fields:
%
%   --> sym:      simulation parameters
%       - model:  Simulink model name
%       - dt:     step time for data saving (s)
%       - Ttot:   stop time of the numerical simulation (s)
%       - dt_max: maximum step size for solver (s)
%       - solver: numerical solver type
%
%   --> wave:         wave parameters
%       - T:          regular wave period (s)
%       - H:          regular wave heigth (m)
%       - ramp_cycle: integer number of cycles for initial wave ramp
%       - dir_index:  index that identify the wave forces related to a
%                     specific wave direction. This index is an integer 
%                     between 1 and the length of the cell array of the 
%                     wave forces 
%
%   --> res: results option
%       - Ncycles: integer number of cycles with respect to performances are avaluated
%
%   --> gyro:         gyroscope parameters
%       - ms:         gyroscope frame mass (kg)
%       - Is:         gyroscope frame inertia (kgm^2)
%       - mg:         gyroscope mass (kg)
%       - Ig:         gyroscope inertia (kgm^2)
%       - phid:       gyroscope angular velocity (rad/s)
%       - nGyros:     number of gyroscopes (-)
%
%   --> ctrl: control parameters 
%       - c:  linear damping coefficient (Nms/rad)
%       - k:  linear stiffness coefficient (Nm/rad)
%
%   --> PTO:    Power Take Off parameters
%       - tau:  gear ratio (-)
%       - Tnom: PTO nominal torque (Nm)
%       - Tmax: PTO maximum torque (Nm)
%       - wnom: PTO nominal angular velocity (rad/s)
%       - wmax: PTO maximum angular velocity (rad/s)
%       - Pnom: PTO nominal power (W)
%       - Pmax: PTO maximum power (W)
%
% "hull" is a structure that contains hull parameters, which are organised 
% in the following fields:
%   --> ENV:            structure containing the ENVIRONMENT data:
%       - WD:           water depth (m)
%       - rho_w:        water density (kg/m^3)
%
%   --> FDA:            structure containing the frequency domain analysis
%                       (FDA)
%       - hydrostatic:  structure that contains hydrostatic anaysis 
%                       results
%         + M:          mass matrix
%         + COG:        center of gravity vector (m), measured with respect  
%                       to the water plane
%         + LPV:        lowest point of the vessel (m), measured with
%                       respect to the water plane
%         + COB:        center of buoyancy (m)
%         + WP_area:    water plane area (m^2)
%         + BG:         distance between COG and COB (m)
%         + GMX:        metacentric height relative to the rotation
%                       around x axis (roll motion) (m)
%         + GMY:        metacentric height relative to the rotation
%                       around y axis (pitch motion) (m)
%         + BMX:        distance between COB and GMX (m) (BMX = GMX+BG)
%         + BMY:        distance between COB and GMY (m) (BMY = GMY+BG)
%         + MX:         restoring moment with respect x axis, per 
%                       degree of rotarion (Nm/deg)
%         + MY:         restoring moment with respect y axis, per 
%                       degree of rotarion (Nm/deg)
%         + K:          stiffness matrix
%       - hydrodynamic: structure that contains the dynamic parameters
%                       of the floating structure
%         + A:          added mass matrix, computed for the input
%                       frequency vector
%         + B:          damping matrix, computed for the input
%                       frequency vector
%         + Response:   structure that contains the hull's frequency 
%                       response, computed for the assigned wave 
%                       directions
%         + dir:        wave direction (deg)
%         + RAO:        Response Amplitude Operator computed along the
%                       six DOFs (col) and for all the assigned
%                       frequencies (row)
%              
%   --> WAVE:           structure containing the wave properties and forces
%       - prop:         structure containing the wave properties
%         + w_vec:      wave radian frequency vector (rad/s)
%         + T_vec:      wave period vector (s)
%         + k_vec:      wave number vector
%         + lambda_vec: wave length vector (m)
%         + D_l_vec:    depth - wave length ratio vector 
%         + kD_vec:     wave number - depth product vector
%         + DIR_vec:    wave direction vector
%       - forces:       structure that contains Froude-Krylov coefficients
%       - dir:          wave direction (deg)
%       - f:            Froude-Krylov coefficients computed along the six
%                       DOFs (col) and for all the assigned frequencies
%                       (row)
%       - Drift_forces: structure that contains the mean wave drift force
%                       coefficients
%         + dir:        wave direction (deg)
%         + Drift:      mean wave drift force coefficients computed along
%                       the six DOFs (col) and for all the 
%                       assigned frequencies (row) 
%
%   --> FDA:            Frequency Domain Analysis
%       - A:            added mass matrix
%       - B:            radiation matrix
%       - K:            hydrostatic stiffness
%       - f:            Froude-Krylov coefficients
%       - RAO:          Response Amplitude Operator
%       - w:            radian frequency vector (rad)
%       - k_vec:        wave number vector
%       - M:            mass matrix
%       - COG:          hull centre of gravity measured with respect to the 
%                       water level
%       - WD:           water depth (m)
%       - rho_w:        water density (kg/m^3)
%       - Drift:        mean drift force coefficients
%
%   --> TDM:            time domain motions
%       - Ainf:         asimptotic value of added mass matrix for 
%                       w-->infinite
%       - x:            convolution integral SS approximation, surge
%       - x_ry:         convolution integral SS represetation, coupling 
%                       surge-pitch
%       - z:            convolution integral SS approximation, heave
%       - ry_x:         convolution integral SS represetation, coupling 
%                       pitch-surge
%       - ry:           convolution integral SS represetation, pitch
%
%   --> GEOM:           hull geometry and mooring attachment point position
%       - W:            hull width (m)
%       - H:            hull heigth (m)
%       - L:            hull length (m)
%       - xC:           horizontal mooring linking point coordinate in hull 
%                       reference frame
%       - zC:           vertical mooring linking point coordinate in hull
%                       reference frame
%       - H0G:          vertical hull COG coordinate, measured from the 
%                       seabed in static conditions

%%
% ##### Output description:
% "wave" is a structure that contains regular wave parameters, which are 
% organised in the following fields:
%   --> Fd_x:       mean drift force value along surge axis (N)
%   --> Fx:         wave force vector along surge axis (N)
%   --> Fz:         wave force vector along heave axis (N)
%   --> Fry:        wave force vector along pitch axis (Nm)
%   --> t:          time vector (s)
%   --> eta:        wave profile (m)
%   --> T:          regular wave period (s)
%   --> H:          regulat wave height (m)
%   --> ramp_cycle: integer number of cycles for initial wave ramp
%
%
% ##### Changelog:
% 25/05/2016: creation


%% TIME BASE DEFINITION
t = (0:par.sym.max_dt:par.sym.Ttot);      % Time vector (s)
TAU = zeros(length(t),6);

%% REGULAR WAVE PARAMETERS
T = par.T;                        % Wave period (s)
H = par.H;                        % Wave heigth (m)
ramp_cycle = par.ramp_cycle;      % Number of cycle for wave ramp  


%% RAMP DEFINITION
t_ramp = 0:par.sym.max_dt:ramp_cycle*T;        % Time vector (s)       
ramp_1 = t_ramp/(ramp_cycle*T);                 % Slope
ramp_2 = ones(length(t)-length(t_ramp),1);      % Constant profile
ramp = [ramp_1  ramp_2'];                       % Ramp vector


%% WAVE PROFILE
eta = H/2*sin(2*pi/T*t);        % Sinusoidal wave profile (m)
eta = eta.*ramp;                % Ramped wave profile (m)


%% DRIFT FORCE SURGE MOTION
fd_x = interp1(par.hull.WAVE.prop.w_vec, par.hull.WAVE.Drift_forces{dir,1}.Drift(:,1), (2*pi/T));        % Normalized surge wave drift force interpolation (N/m^2)
WAVE.Drift_forces{1,1}.MEAN_DRIFT(:,1) = (H/2)^2 * abs(fd_x);                                                                            % Surge wave drift force (N)                                     


%% FROUDE-KRYLOV COEFFICIENTS

% Surge axis
fx = interp1(par.hull.WAVE.prop.w_vec,par.hull.WAVE.forces{dir,1}.f(:,1),(2*pi/T));     % Froude-krylov coefficient interpolation (N/m)
%Fx = H/2*abs(fx)*sin((2*pi/T)*t+phase(fx));         % Wave force acting on the par.hull (N)
Fx = H/2*abs(fx)*sin((2*pi/T)*t+angle(fx));    
TAU(:,1) = Fx.*ramp;                                                                                       % Ramped wave force acting on the par.hull (N)

% Sway axis
fy = interp1(par.hull.WAVE.prop.w_vec,par.hull.WAVE.forces{dir,1}.f(:,2),(2*pi/T));     % Froude-krylov coefficient interpolation (N/m)
%Fy = H/2*abs(fy)*sin((2*pi/T)*t+phase(fy));                                                         % Wave force acting on the par.hull (N)
Fy = H/2*abs(fy)*sin((2*pi/T)*t+angle(fy));   
TAU(:,2) = Fy.*ramp;                                                                                      % Ramped wave force acting on the par.hull (N)

% Heave axis
fz = interp1(par.hull.WAVE.prop.w_vec,par.hull.WAVE.forces{dir,1}.f(:,3),(2*pi/T));     % Froude-krylov coefficient interpolation (N/m)
%Fz = H/2*abs(fz)*sin((2*pi/T)*t+phase(fz));                                                         % Wave force acting on the par.hull (N)
Fz = H/2*abs(fz)*sin((2*pi/T)*t+angle(fz));    
TAU(:,3) = Fz.*ramp;                                                                                      % Ramped wave force acting on the par.hull (N)

% Roll sxis
frx = interp1(par.hull.WAVE.prop.w_vec,par.hull.WAVE.forces{dir,1}.f(:,4),(2*pi/T));    % Froude-krylov coefficient interpolation (Nm/m)
%Frx = H/2*abs(frx)*sin((2*pi/T)*t+phase(frx));                                                      % Wave force acting on the par.hull (Nm)
Frx = H/2*abs(frx)*sin((2*pi/T)*t+angle(frx));  
TAU(:,4) = Frx.*ramp;                                                                                    % Ramped wave force acting on the par.hull (Nm)

% Pitch sxis
fry = interp1(par.hull.WAVE.prop.w_vec,par.hull.WAVE.forces{dir,1}.f(:,5),(2*pi/T));    % Froude-krylov coefficient interpolation (Nm/m)
%Fry = H/2*abs(fry)*sin((2*pi/T)*t+phase(fry));                                                      % Wave force acting on the par.hull (Nm)
Fry = H/2*abs(fry)*sin((2*pi/T)*t+angle(fry));  
TAU(:,5) = Fry.*ramp;                                                                                    % Ramped wave force acting on the par.hull (Nm)

% Yaw sxis
frz = interp1(par.hull.WAVE.prop.w_vec,par.hull.WAVE.forces{dir,1}.f(:,5),(2*pi/T));    % Froude-krylov coefficient interpolation (Nm/m)
%Frz = H/2*abs(frz)*sin((2*pi/T)*t+phase(frz));                                                      % Wave force acting on the par.hull (Nm)
Frz = H/2*abs(frz)*sin((2*pi/T)*t+angle(frz)); 
TAU(:,6) = Frz.*ramp;                                                                                    % Ramped wave force acting on the par.hull (Nm)

%% WAVE STRUCTURE DEFINITION
% wave.Fd_x = Fd_x; 
% wave.Fx = Fx';
% wave.Fy = Fy';
% wave.Fz = Fz';
% wave.Frx = Frx'; 
% wave.Fry = Fry'; 
% wave.Frz = Frz';                                       
% wave.t = t';
% wave.eta = eta'; 
% wave.T = T;
% wave.H = H;
% wave.ramp_cycle = ramp_cycle;
WAVE.TIME = t';
WAVE.forces{1,1}.TAU = TAU;
WAVE.ETA = eta';
WAVE.T = T;
WAVE.H = H;

end