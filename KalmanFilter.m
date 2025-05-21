%{
23.02.2017
02.06.2024
Thomas Abmayr // Hochschule München
Modul:  Vorlesung Geosensor Fusion // Vertiefung Navigation
Thema:  Koppelnav.für einen 2D Roboter mit einfacher Odom
%}

clear all;
close all;
fig = figure();

% init video
%vw = VideoWriter("./+out_/P10_VN_EKF_Template.mp4", 'MPEG-4');
%vw.FrameRate = 8;
%open(vw);

%% load init params from mat files
load ('+init_/pose_real.mat');
load ('+init_/delta_real.mat');
load ('+init_/X.mat');
load ('+init_/Y.mat');
load ('+init_/Z_kp1.mat', 'Z_kp1');

%% init params
pose(1, 1:3) = [100, 100, 90]; 
x(1, 1:3) = [100, 100, 90]; 
V = [7, 0; 0, 0.3]; % motion
W = [7, 0; 0, 1.5]; % measurement
P = [7 0 0; 0, 0, 7; 0, 0, 0.7]; %P_dach_k;

%% Laufe die Trajektorie in diskreten Schritten ab
for k = 1 : length(delta_real)-1 
           
    %% plot
    clf (fig);
    fig = figure(1);
    img = imread('myImg3.jpg');
    imshow(img);
    [h,w,c] = size(img);
    text(10, h-20,'(c) Taha Haouas, HM, Muc.Dai (2025)', 'Color','white','FontSize', 10)
    axis([-0 w -0 h]);
    hold on;
    grid on;
        
    plot(pose_real(1:k+1,1), pose_real(1:k+1,2), 'w-', 'LineWidth',8); % plot ideal trajectory
    text(pose_real(k+1,1), pose_real(k+1,2)+20,'Real Robot Loc', 'Color','blue','FontSize', 10)
   
    % landmarks
    d = sqrt( (X(2)-pose_real(k,1))^2 + (Y(2)-pose_real(k,2))^2 );
    plot([pose_real(k,1),X(2)], [pose_real(k,2),Y(2)],'w-', 'LineWidth',2);
    d = sqrt( (X(1)-pose_real(k,1))^2 + (Y(1)-pose_real(k,2))^2 );
    plot([pose_real(k,1),X(1)], [pose_real(k,2),Y(1)],'w-', 'LineWidth',2);

    %% calc and plot EKF 

    % TODO: Überschreibe pose_tmp mit Ergebnis von Aufgabe 7.10.2
    x = pose_real(k+1, 1:3); % [x, P] = navlib.calc_ekf( Z_kp1(k+1, :), x, P, delta_real(k+1, 1:2), V, W, X, Y );
    pose(k+1, 1:3) = x;
    
    plot(pose(1:k+1,1), pose(1:k+1,2), 'm-', 'LineWidth',3);
    plot( pose(end,1), pose(end,2), 'c+', 'LineWidth',2);
    text(pose(end,1), pose(end,2)+20,'Kalman Filter', 'Color','red','FontSize', 10)
        
    str = 'Extended Kalman Filter';
    title(str);
    text(10, 20, str,'Color', 'b','FontSize', 12, 'fontweight','bold' );

    %% video
    %tmpImg = frame2im(getframe);
    %writeVideo(vw, imresize(tmpImg, [549*2, 693*2]));
    drawnow;
    pause(0.05);
    
end

%% save
%save ('+out_/pose_EKF.mat', 'pose');
%close(vw);

