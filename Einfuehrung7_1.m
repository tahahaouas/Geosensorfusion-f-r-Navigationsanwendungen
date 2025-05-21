%{
23.02.2017
11.03.2024
Vorname:Taha
Nachname:Haouas

Thomas Abmayr // Hochschule München
Modul:  Vorlesung Geosensor Fusion // Vertiefung Navigation
Thema:  Koppelnav.für einen 2D Roboter mit einfacher Odom
%}

%clear all;
close all;
fig = figure();

% load init params from mat files
load ('+init_/pose_real.mat');   % Contains variable "pose_real"
load ('+init_/delta_real.mat');  % Contains variable "delta_real"


% temp Variable wegen Template
load('+out_/pose_koppel.mat');   % Load a template for the dead reckoning trajectory
pose_tmp = pose_koppel;
pose_koppel = [];

% set init pose for dead reckoning trajectory
pose_koppel(1, 1:3) = [100, 100, 90]; 

% Loop through the odometry data to update and plot the trajectories
for k = 1 : length(delta_real)-1 
           
    %% Plot background image
    clf(fig);
    img = imread('+init_/myImg3.jpg');
    imshow(img);
    [h, w, ~] = size(img);
    text(10, h-20, '(c) Taha Haouas, HM, Muc.Dai (2025)', 'Color', 'white', 'FontSize', 10);
    axis([0 w 0 h]);
    hold on;
    grid on;
        
    %% Plot the real trajectory (Aufgabe 7.1.1)
    % Plot the actual trajectory from pose_real.mat up to current index
    plot(pose_real(1:k+1,1), pose_real(1:k+1,2), 'w-', 'LineWidth', 8);
    text(pose_real(k+1,1), pose_real(k+1,2)+20, 'Real Robot Loc', 'Color', 'blue', 'FontSize', 10);
    
    %% Calculate and plot dead reckoning (Aufgabe 7.1.2)
    % Compute the new pose based on current dead reckoning pose and odometry
    pose_koppel(k+1, 1:3) = motionModel(pose_koppel(k, 1:3), delta_real(k+1, 1:2));  
    % Plot the dead reckoning trajectory
    plot(pose_koppel(1:k+1,1), pose_koppel(1:k+1,2), 'm-', 'LineWidth', 3);
    plot(pose_koppel(end,1), pose_koppel(end,2), 'c+', 'LineWidth', 2);
    text(pose_koppel(end,1), pose_koppel(end,2)+20, 'Dead Reckoning', 'Color', 'red', 'FontSize', 10);
        
    title('Dead Reckoning vs Real Trajectory');
    drawnow;
    
    pause(0.05);
    
end

%% Local Methoden: 

% motionModel
function new_pose = motionModel(current_pose, delta)
    % motionModel computes the new pose based on the current pose and odometry.
    % current_pose: [x, y, theta]
    % delta: [distance, angle_change]
    d = delta(1);
    dtheta = delta(2);
    
    new_x = current_pose(1) + d * cosd(current_pose(3) + dtheta);
    new_y = current_pose(2) + d * sind(current_pose(3) + dtheta);
    new_theta = current_pose(3) + dtheta;
    
    new_pose = [new_x, new_y, new_theta];
end


