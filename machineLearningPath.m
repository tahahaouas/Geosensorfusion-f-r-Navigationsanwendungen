%{
26.04.2024
Thomas Abmayr // Hochschule München
Modul:  Vorlesung Geosensor Fusion // Vertiefung Navigation
Thema:  Koppelnav. für einen 2D Roboter mit einfacher Odom
%}

%clear all;
close all;
fig = figure();

% init video
%vw = VideoWriter("./+out_/P5_VN_FdFwdNetz.mp4", 'MPEG-4');
%vw.FrameRate = 8;
%open(vw);

%% Lade initiale Parameter
load ('+init_/pose_real.mat'); % Echte Position des Roboters
load ('+init_/GPS.mat');       % GPS-Messwerte

%% Setze Initialwerte für die geschätzte Pose mit DL
pose_DL = zeros(size(pose_real));  
pose_DL(1, :) = [100, 100, 90];

%% Initialisiere das MLP-Netzwerk
net = feedforwardnet(1, 'trainlm'); % 10 Neuronen, Levenberg-Marquardt-Training
net.trainParam.showWindow = false;   % Deaktiviert MATLAB-GUI beim Training

%% Trajektorie durchlaufen
for k = 1 : length(GPS)-1 

    % --- Zeichnen der Umgebung ---
    clf(fig);
    fig = figure(1);
    img = imread('myImg3.jpg');
    imshow(img);
    [h, w, ~] = size(img);
    text(10, h-20, '(c) Taha Haouas, HM, Muc.DAI (2025)', 'Color','white','FontSize', 10);
    axis([-0 w -0 h]);
    hold on;
    grid on;

    % Zeichne echte (weiße) Trajektorie
    plot(pose_real(1:k+1,1), pose_real(1:k+1,2), 'w-', 'LineWidth',8);
    plot(pose_real(k+1,1), pose_real(k+1,2), 'b+', 'LineWidth',4);
    text(pose_real(k+1,1), pose_real(k+1,2)+20, 'Real Robot Loc', 'Color','blue','FontSize', 10);

    % Zeichne GPS-Messwerte (grün)
    plot(GPS(1:k+1,1), GPS(1:k+1,2), 'b*');

    % --- Feedforward-Netzwerk verwenden für Trajektorienschätzung ---
    sw = 15;
    delay = 3;
    if k > sw  % Sliding Window von 15 Werten
        gps_window = GPS(k-14:k, :);
        pose_DL(k+1, 1:2) = calc_feedFwd(gps_window, net, delay); % Schätzung mit MLP
        pose_DL(k+1, 3) = 0; % Orientierung bleibt 0, da MLP nur x,y schätzt
    else
        pose_DL(k+1, :) = pose_DL(1, :); % Anfangswerte übernehmen
    end

    % Zeichne geschätzte Trajektorie (magenta)
    plot(pose_DL(1:k+1, 1), pose_DL(1:k+1,2), 'm-', 'LineWidth',3);
    plot(pose_DL(k+1,1), pose_DL(k+1,2), 'm+', 'LineWidth',4);
    text(pose_DL(k+1,1)+10, pose_DL(k+1,2)+40, 'Berechnung mit Feedforward Netz', 'Color','g','FontSize', 10);

    title('Trajektorenschätzung mit Feedforward Netz');

    % Videoaufnahme
    %tmpImg = frame2im(getframe);
    %writeVideo(vw, imresize(tmpImg, [549*2, 693*2]));
    drawnow;
    pause(0.05);

end

%% Speichern der geschätzten Trajektorie
%save ('+out_/pose_NeurNet.mat', 'pose_DL');
%close(vw);


function x = calc_feedFwd(myGPS, net, delay)
    N = size(myGPS, 1) - 1; % Get the number of elements - 1
    X = (0:N) / N; % Normalize indices from 0 to 1
    
    T = myGPS'; % Transpose myGPS for training
    
    net = train(net, X, T); % Train the neural network
    
    x = net(0.8); % Evaluate network at 0.8
end
