function par = FUNC_UD_PendulumInertia(par)
% Torino, 05/07/2016
% Autori: Giovanni Bracco, Mauro Bonfanti.
 
% Lato del cubo equivalente
par.pendulum.l = (par.pendulum.m / 7700)^(1/3);
 
% Matrice d'inerzia
Ip = 1/12 * par.pendulum.m * 2 * par.pendulum.l^2;
par.pendulum.Ip = diag([1 1 1] * Ip);

end

