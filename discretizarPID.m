function ftd = discretizarPID(controlador)
%DISCRETIZARPID Summary of this function goes here
%   Detailed explanation goes here

Kp = controlador.Kp;
Kd = controlador.Kd;
Ki = controlador.Ki;
T = controlador.T;
Tf = 1;

ft = pid(Kp,Ki,Kd,Tf);
ftd = c2d(ft, T, 'Tustin');
end

