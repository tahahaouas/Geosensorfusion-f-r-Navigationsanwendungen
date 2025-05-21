%{
11.03.2024
Thomas Abmayr // Hochschule München
Modul: Vorlesung Geosensor Fusion // Vertiefung Navigation
Thema: Koppelnavigation mit homogenen Matrizen
Aufgaben 7.2.1.1 bis 7.2.1.4
%}

%clear all;
close all;
fig = figure;

% Odometriedaten laden (delta_real.mat enthält Zeilen: [Strecke, Winkeländerung] in Grad)
load('+init_/delta_real.mat');   

% Initiale homogene Transformationsmatrix für den Startzustand:
% Startpose: [100, 100, 90] (x, y, theta in Grad)
theta0 = 90;
M = [cosd(theta0) -sind(theta0) 100;
     sind(theta0)  cosd(theta0) 100;
     0             0           1];

% Alle Transformationsmatrizen speichern (für spätere Analyse)
M_all = zeros(3,3, length(delta_real));
M_all(:,:,1) = M;

% Tabelle zur Speicherung der Koordinaten erstellen (Spalten: x, y, Theta)
Koordinaten = zeros(length(delta_real), 3);
Koordinaten(1,:) = [100, 100, 90];

% Figur vorbereiten mit Hintergrundbild
clf(fig);
img = imread('+init_/myImg3.jpg');
imshow(img);
hold on;
[h, w, ~] = size(img);
text(10, h-20,'(c)  Taha Haouas, HM, Muc.DAI (2025)', 'Color','white','FontSize', 10)

axis([0 w 0 h]);
grid on;
title('Koppelnavigation mit homogenen Matrizen');

% Erstellen eines UI-Controls zur Live-Anzeige der Koordinaten
hLiveBox = uicontrol('Style', 'text', 'Units', 'normalized', ...
    'Position', [0.05 0.85 0.2 0.1], 'String', '', 'FontSize', 10, ...
    'BackgroundColor', 'white');

% Schleife durch alle Odometriedaten
for k = 1 : length(delta_real)-1
    % --- Aufgabe 7.2.1.1: Homogene Aktualisierung ---
    % Odometriedaten: d = gefahrene Strecke, dtheta = Winkeländerung (in Grad)
    d = delta_real(k+1, 1);
    dtheta = delta_real(k+1, 2);
    
    % Erstellen der Inkrement-Transformationsmatrix 
    T = [cosd(dtheta) -sind(dtheta) d;
         sind(dtheta)  cosd(dtheta)  0;
         0             0            1];
    
    % Aktualisieren der Gesamttransformation im Weltkoordinatensystem
    M = M * T;
    M_all(:,:,k+1) = M;
    
    % Aktuelle Position und Orientierung extrahieren
    pos = [M(1,3), M(2,3)];
    theta = atan2d(M(2,1), M(1,1));  % Orientierung (Yaw) in Grad
    
    % Speichern der aktuellen Pose in der Tabelle
    Koordinaten(k+1,:) = [pos, theta];
    
    % Trajektorie plotten (Linie zwischen vorheriger und aktueller Position)
    if k > 1
        prev_pos = [M_all(1,3,k), M_all(2,3,k)];
        plot([prev_pos(1) pos(1)], [prev_pos(2) pos(2)], 'm-', 'LineWidth', 3);
    else
        plot(pos(1), pos(2), 'm.', 'MarkerSize', 20);
    end
    
    % --- Aufgabe 7.2.1.2: Darstellung des lokalen Koordinatensystems ---
    % Festlegen der Achsenlänge (hier 0.2 Einheiten)
    origin = pos;
    x_axis_end = origin + 0.2 * [M(1,1), M(2,1)];
    y_axis_end = origin + 0.2 * [M(1,2), M(2,2)];
    
    % Lokale Achsen plotten: x-Achse in Rot, y-Achse in Grün
    plot([origin(1) x_axis_end(1)], [origin(2) x_axis_end(2)], 'r-', 'LineWidth', 2);
    plot([origin(1) y_axis_end(1)], [origin(2) y_axis_end(2)], 'g-', 'LineWidth', 2);
    
    % --- Aufgabe 7.2.1.3: Rotation überprüfen ---
    % Extrahieren der 2x2 Rotationsmatrix aus M
    R_current = M(1:2, 1:2);
    if isRot(R_current)
        rotText = 'Rotation OK';
    else
        rotText = 'Rotation NICHT OK';
    end
    % Optional: Anzeige der Rotationsüberprüfung an einer festen Position
    text(10, 20, rotText, 'Color', 'yellow', 'FontSize', 10);
    
    % --- Aufgabe 7.2.1.4: RPY-Darstellung (hier vorwiegend Yaw) ---
    % Einbetten der 2x2-Rotation in eine 3x3-Matrix
    R_3d = [R_current, [0;0]; 0 0 1];
    [roll, pitch, yaw] = getRPY(R_3d);
    txt = sprintf('Yaw: %.2f°', yaw);
    text(10, 40, txt, 'Color', 'cyan', 'FontSize', 10);
    
    % Live-Anzeige der aktuellen Koordinaten in der uicontrol-Box
    liveText = sprintf('x: %.2f\ny: %.2f\nTheta: %.2f°\nLiveKoordinaten', pos(1), pos(2), theta);
    set(hLiveBox, 'String', liveText);
    
    drawnow;
    pause(0.05);
end

%% Anzeige der gesammelten Koordinaten in einer separaten Tabelle (uitable)
fTable = figure('Name','Koordinaten Tabelle');
uitable('Data', Koordinaten, 'ColumnName', {'x','y','Theta (°)'}, ...
    'Units','Normalized', 'Position',[0 0 1 1]);

%% --- Lokale Methoden ---

function res = isRot(R)
    % isRot überprüft, ob eine gegebene R eine gültige Rotationsmatrix ist.
    tol = 1e-3;
    res = max(max(abs(R' - inv(R)))) < tol && abs(det(R) - 1) < tol;
end

function [roll, pitch, yaw] = getRPY(R)
    % getRPY extrahiert die Roll-, Pitch- und Yaw-Winkel aus einer 3x3 Rotationsmatrix R.
    yaw = atan2d(R(2,1), R(1,1));
    cg = cosd(yaw);
    sg = sind(yaw);
    pitch = atan2d(-R(3,1), R(1,1)*cg + R(2,1)*sg);
    roll  = atan2d(R(1,3)*sg - R(2,3)*cg, R(2,2)*cg - R(1,2)*sg);
end
