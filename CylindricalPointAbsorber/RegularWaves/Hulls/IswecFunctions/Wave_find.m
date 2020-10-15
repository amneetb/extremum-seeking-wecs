function [WaveForce,T,H,filename] = Wave_find(Te,Hs)
% Author: Mauro Bonfanti

Waves=dir('Wave_forces\*.mat');
addpath([cd,'\Wave_forces'])

for kk=1:length(Waves)
    
    % CRio
    filename = Waves(kk).name;
    load(filename)
    disp(filename)
    T = WaveForce.Te;
    H = WaveForce.Hm0;
    err_Te = abs(Te-T);
    disp(err_Te)
    err_Hs = abs(Hs-H);
    if err_Te < 0.2 && err_Hs < 0.2
        break
    end

end
end

