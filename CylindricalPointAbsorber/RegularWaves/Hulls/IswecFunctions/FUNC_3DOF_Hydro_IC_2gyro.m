function par = FUNC_3DOF_Hydro_IC_2gyro(par)

%% INITIAL CONDITIONS

par.gyro.ms = 0;
par.gyro.nGyros = 0;
par.gyro.mg = 0;

% Surge motion
par.IC.hull.x0 = 0;

% Heave motion
par.IC.hull.z0 = -((par.gyro.ms+par.gyro.mg)*par.gyro.nGyros*9.81/par.hull.FDA.hydrostatic.K(3,3));

% Exctracting mooring forces in the initial condition
Fmx_0 = interp2(par.moor.x_vec,par.moor.h_vec,par.moor.Fmx_mat',par.hull.GEOM.xC,par.moor.geom.h_nom);
Fmz_0 = interp2(par.moor.x_vec,par.moor.h_vec,par.moor.Fmz_mat',par.hull.GEOM.zC,par.moor.geom.h_nom);

% Pitch motion
par.IC.hull.ry0 = (Fmx_0*sqrt(par.hull.GEOM.xC^2+par.hull.GEOM.zC^2)*sin(atan(par.hull.GEOM.zC/par.hull.GEOM.xC))-...
    Fmz_0*sqrt(par.hull.GEOM.xC^2+par.hull.GEOM.zC^2)*cos(atan(par.hull.GEOM.zC/par.hull.GEOM.xC)))/par.hull.FDA.hydrostatic.K(5,5);

end