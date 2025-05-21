%{
Thomas Abmayr // Hochschule München
Modul: Vorlesung Geosensor Fusion // Vertiefung Navigation
Thema: Cartesian Motion und Trajektorienberechnung
Aufgaben: 7.3.1 & 7.3.2
%}

clear all;
close all;
fig = figure;

% Laden der realen Pose und Odometriedaten
load('+init_/pose_real.mat');   % Enthält pose_real [x, y, theta]
load('+init_/delta_real.mat');  % Enthält delta_real [Strecke, Winkeländerung]

% Start- und Endpunkt der Trajektorie definieren
pos_a = [pose_real(1,1), pose_real(1,2)];          % Startposition
pos_b = [pose_real(end,1), pose_real(end,2)];      % Endposition
v_a = [0,900];   % Startgeschwindigkeit
v_b = [900, 0];   % Endgeschwindigkeit
a_a = [0, 0];     % Startbeschleunigung
a_b = [0, 0];     % Endbeschleunigung
mid_pos = [400, 350]; 

% Berechnen der Trajektorie mittels quintischem Polynom
[ax, ay] = calcTrajWithMidpoint(pos_a,mid_pos, pos_b, v_a, v_b, a_a, a_b);

% Initialisierung der kartesischen Bewegung (7.3.1)
numSteps = length(delta_real);
traj_1 = zeros(numSteps+1, 3);
traj_1(1,:) = [pos_a, 0];  % Startpose mit Winkel

% Figur mit Hintergrundbild vorbereiten
clf(fig);
img = imread('+init_/myImg3.jpg');
imshow(img);
hold on;
[h, w, ~] = size(img);
text(10, h-20,'(c)  Taha Haouas, HM, Muc.DAI (2025)', 'Color','white','FontSize', 10)

axis([0 w 0 h]);
grid on;
title('Cartesian Motion & Trajektorie');

% Schleife zur Trajektorienberechnung und Darstellung
for k = 1:numSteps
    t = (k-1) / numSteps; % Zeit normalisiert auf [0,1]
    
    % Berechnung der Trajektorienpunkte aus quintischem Polynom
    traj_1(k+1,1) = ax(1) + ax(2)*t + ax(3)*t^2 + ax(4)*t^3 + ax(5)*t^4 + ax(6)*t^5;
    traj_1(k+1,2) = ay(1) + ay(2)*t + ay(3)*t^2 + ay(4)*t^3 + ay(5)*t^4 + ay(6)*t^5;
    
    % Winkel anhand der Tangentenrichtung berechnen
    traj_1(k+1,3) = atan2d(traj_1(k+1,2) - traj_1(k,2), traj_1(k+1,1) - traj_1(k,1));
    
    % Trajektorie zeichnen
    plot(traj_1(1:k+1,1), traj_1(1:k+1,2), 'b-', 'LineWidth', 2);
    
    % Real gefahrene Pose zeichnen (Vergleich mit idealer Trajektorie)
    plot(pose_real(1:k+1,1), pose_real(1:k+1,2), 'w-', 'LineWidth', 2);
    
    % Punkte für real und berechnete Trajektorie markieren
    plot(traj_1(k+1,1), traj_1(k+1,2), 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
    plot(pose_real(k+1,1), pose_real(k+1,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
    
    drawnow;
    pause(0.05);
end

% Speichern der berechneten Trajektorie
save('+out_/traj_1.mat', 'traj_1');

%% Lokale Funktion zur Berechnung der quintischen Trajektorie
function [ax, ay] = calcTraj(pos_a, pos_b, v_a, v_b, a_a, a_b)
    % Quintisches Polynom zur Berechnung der Trajektorie zwischen zwei Punkten
    
    M = [1 0 0 0 0 0;
         0 1 0 0 0 0;
         0 0 2 0 0 0;
         1 1 1 1 1 1;
         0 1 2 3 4 5;
         0 0 2 6 12 20];
    
    bx = [pos_a(1); v_a(1); a_a(1); pos_b(1); v_b(1); a_b(1)];
    by = [pos_a(2); v_a(2); a_a(2); pos_b(2); v_b(2); a_b(2)];
    
    ax = M\bx;  % Lösung des Gleichungssystems für x-Koordinaten
    ay = M\by;  % Lösung des Gleichungssystems für y-Koordinaten
end

function [ax, ay] = calcTrajWithMidpoint(pos_a, mid_pos, pos_b, v_a, v_b, a_a, a_b)
    % Quintisches Polynom mit zusätzlichem Punkt bei t=0.5
    M = [1 0 0 0 0 0;
         0 1 0 0 0 0;
         0 0 2 0 0 0;
         1 0.5^1 0.5^2 0.5^3 0.5^4 0.5^5;
         1 1 1 1 1 1;
         0 1 2 3 4 5];
    
    bx = [pos_a(1); v_a(1); a_a(1); mid_pos(1); pos_b(1); v_b(1)];
    by = [pos_a(2); v_a(2); a_a(2); mid_pos(2); pos_b(2); v_b(2)];
    
    ax = M\bx;
    ay = M\by;
end