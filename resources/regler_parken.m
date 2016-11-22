clear;
clc;

%%
% y_p = v0*sin(phi+delta_h) 
% phi_p = v0/l * sin(delta_v - delta_h)/cos(delta_v)

v = 1;
l = 0.21;
A = [0, v; 0, 0];
B = [0, v; v/l, -v/l]; 
C = eye(2);

R = place(A,B,[-4 -4]) % Eigenwerte vorgeben
F = inv(C/(B*R-A)*B) % Vorfilter berechnen

%% Implementierung

y_soll = 0;
phi_soll = 0.1;

y_ist = 0.0;
phi_ist = 0;

% Berechnung der Stellgrößen
delta_v = F(1,1)*y_soll + F(1,2)*phi_soll - (R(1,1)*y_ist + R(1,2)*phi_ist);
delta_h = F(2,1)*y_soll + F(2,2)*phi_soll - (R(2,1)*y_ist + R(2,2)*phi_ist);






