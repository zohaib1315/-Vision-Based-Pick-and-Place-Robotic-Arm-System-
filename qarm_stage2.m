%% =========================================================
%  Q-Arm Stage 2: Vision-Based Object Detection
%  ---------------------------------------------------------
%  Simulates a top-down camera detecting colored objects
%  on a table, converts pixel coordinates to robot XYZ,
%  then feeds the detected position into the IK solver.
%
%  Requirements: Image Processing Toolbox
%                Computer Vision Toolbox
%% =========================================================

clc; clear; close all;

%% ---- 1. LOAD ROBOT (carry over from Stage 1) ---------------
fprintf('Loading Q-Arm...\n');
robot = importrobot('QARM.urdf');
robot.DataFormat = 'row';
robot.Gravity     = [0, 0, -9.81];

jointLimits = [-2.967,  2.967;
               -1.483,  1.483;
               -1.658,  1.309;
               -3.010,  3.010];

homeConfig = robot.homeConfiguration;

% IK setup
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations     = 3000;
ik.SolverParameters.MaxTime           = 15;
ik.SolverParameters.GradientTolerance = 1e-12;
ik.SolverParameters.SolutionTolerance = 1e-10;

%% ---- 2. CAMERA PARAMETERS ----------------------------------
% Simulated top-down camera looking straight down at the table.
% The camera is mounted above the robot base.
%
%  Camera frame:
%    - Image center = robot base origin (0, 0)
%    - Camera height above table = 0.75 m
%    - Field of view covers a 0.8m x 0.8m area of the table
%    - Table surface Z = 0.02 m (slightly above base)

camHeight    = 0.75;       % metres above table
tableZ       = 0.02;       % table surface height in robot frame
imgWidth     = 640;        % pixels
imgHeight    = 480;        % pixels
fovX         = 0.8;        % metres — physical width camera can see
fovY         = 0.6;        % metres — physical height camera can see

% Pixels per metre
ppm_x = imgWidth  / fovX;   % 800 px/m
ppm_y = imgHeight / fovY;   % 800 px/m

% Image center corresponds to robot base (0,0)
cx = imgWidth  / 2;   % 320
cy = imgHeight / 2;   % 240

fprintf('Camera FOV   : %.2f m x %.2f m\n', fovX, fovY);
fprintf('Resolution   : %d x %d px\n', imgWidth, imgHeight);
fprintf('Scale        : %.1f px/m\n', ppm_x);

%% ---- 3. GENERATE SYNTHETIC SCENE --------------------------
% Simulate what the camera sees: colored objects on a table.
% In real hardware this would be a live camera frame.

fprintf('\nGenerating synthetic camera scene...\n');

% Create blank table image (light beige)
scene = uint8(ones(imgHeight, imgWidth, 3) * 220);
scene(:,:,1) = 210; scene(:,:,2) = 200; scene(:,:,3) = 180;

% Add grid lines to simulate table surface
for x = 1:80:imgWidth
    scene(:, x, :) = 180;
end
for y = 1:60:imgHeight
    scene(y, :, :) = 180;
end

% ---- Define objects on the table ----------------------------
% Each object: [world_X, world_Y, color_name, RGB]
objects = {
    [0.22,  0.05],  'red',    [220,  40,  40];
    [0.10,  0.18],  'green',  [ 40, 180,  40];
    [0.18, -0.10],  'blue',   [ 40,  80, 220];
    [0.28,  0.12],  'yellow', [230, 210,  30];
};

nObjects   = size(objects, 1);
objRadius  = 18;   % pixels — represents ~2.2cm object

% Draw objects onto scene
for i = 1:nObjects
    worldPos = objects{i,1};
    objColor = objects{i,3};

    % Convert world XY → pixel UV
    u = round(cx + worldPos(1) * ppm_x);
    v = round(cy - worldPos(2) * ppm_y);   % Y flipped (image Y goes down)

    % Draw filled circle
    [uu, vv] = meshgrid(1:imgWidth, 1:imgHeight);
    mask = (uu - u).^2 + (vv - v).^2 <= objRadius^2;
    for c = 1:3
        channel = scene(:,:,c);
        channel(mask) = objColor(c);
        scene(:,:,c) = channel;
    end

    % Draw black border
    borderMask = (uu-u).^2 + (vv-v).^2 <= objRadius^2 & ...
                 (uu-u).^2 + (vv-v).^2 >= (objRadius-2)^2;
    for c = 1:3
        channel = scene(:,:,c);
        channel(borderMask) = 20;
        scene(:,:,c) = channel;
    end

    % Label
    text_x = max(1, u - 15);
    text_y = max(1, v - objRadius - 5);
    fprintf('Object %d (%s): world=[%.2f, %.2f]  pixel=[%d, %d]\n', ...
        i, objects{i,2}, worldPos, u, v);
end

% Show the synthetic camera image
figure('Name','Camera View — Synthetic Scene','NumberTitle','off','Color','w');
imshow(scene);
title('Simulated Top-Down Camera View', 'FontSize', 12);
hold on;

% Draw crosshair at image center (robot base projection)
plot(cx, cy, 'w+', 'MarkerSize', 20, 'LineWidth', 2);
text(cx+5, cy-10, 'Base', 'Color','white','FontSize',9,'FontWeight','bold');

%% ---- 4. COLOR-BASED OBJECT DETECTION -----------------------
fprintf('\n--- Vision Detection ---\n');

% Target: detect the RED object (pick target)
% In a real system this threshold would be tuned to lighting conditions
targetColor = 'red';

% Convert to HSV for more robust color detection
sceneHSV = rgb2hsv(scene);
H = sceneHSV(:,:,1);
S = sceneHSV(:,:,2);
V = sceneHSV(:,:,3);

% Red hue range in HSV (red wraps around 0/1)
redMask = ((H < 0.05 | H > 0.95) & S > 0.5 & V > 0.3);

% Clean up noise with morphological operations
redMask = imopen(redMask,  strel('disk', 3));
redMask = imclose(redMask, strel('disk', 5));
redMask = imfill(redMask, 'holes');

% Find connected components
cc     = bwconncomp(redMask);
stats  = regionprops(cc, 'Centroid', 'Area', 'BoundingBox');

fprintf('Detected %d red region(s)\n', cc.NumObjects);

% Pick the largest region (most likely the object, not noise)
if cc.NumObjects == 0
    error('No red object detected in scene.');
end
[~, idx] = max([stats.Area]);
detected  = stats(idx);
centroid  = detected.Centroid;   % [u, v] in pixels

fprintf('Detected centroid (pixels): u=%.1f  v=%.1f\n', centroid);

% Draw detection overlay on camera view
rectangle('Position', detected.BoundingBox, ...
    'EdgeColor','yellow','LineWidth',2);
plot(centroid(1), centroid(2), 'y+', 'MarkerSize',20, 'LineWidth',3);
text(centroid(1)+10, centroid(2), 'DETECTED', ...
    'Color','yellow','FontSize',10,'FontWeight','bold');

%% ---- 5. PIXEL → ROBOT WORLD COORDINATES -------------------
fprintf('\n--- Coordinate Transform ---\n');

% Convert pixel centroid back to world XY
detectedX = (centroid(1) - cx) / ppm_x;
detectedY = (cy - centroid(2)) / ppm_y;   % flip Y back
detectedZ = tableZ + 0.03;                % object sits ~3cm above table

detectedWorldPos = [detectedX, detectedY, detectedZ];

fprintf('Pixel centroid       : [%.1f, %.1f] px\n', centroid);
fprintf('Detected world pos   : X=%.4f  Y=%.4f  Z=%.4f  m\n', detectedWorldPos);
fprintf('Ground truth pos     : X=%.4f  Y=%.4f  m\n', objects{1,1});
fprintf('Detection error      : %.4f m\n', ...
    norm(detectedWorldPos(1:2) - objects{1,1}));

%% ---- 6. IK: MOVE TO DETECTED OBJECT -----------------------
fprintf('\n--- IK: Moving to detected object ---\n');

[detectedConfig, detectedErr] = solveIK(ik, detectedWorldPos, jointLimits, 20);

fprintf('Target               : [%.4f, %.4f, %.4f] m\n', detectedWorldPos);
fprintf('IK solution          : YAW=%.1f°  SHOULDER=%.1f°  ELBOW=%.1f°  WRIST=%.1f°\n', ...
    rad2deg(detectedConfig));
fprintf('IK position error    : %.5f m\n', detectedErr);

%% ---- 7. FULL PIPELINE VISUALIZATION -----------------------
% Show the complete pipeline: camera → detection → robot motion

placeXYZ  = [-0.15, 0.15, 0.05];
liftXYZ   = [detectedWorldPos(1), detectedWorldPos(2), detectedWorldPos(3)+0.18];

[liftConfig,  ~]       = solveIK(ik, liftXYZ,  jointLimits, 15);
[placeConfig, placeErr] = solveIK(ik, placeXYZ, jointLimits, 20);

fprintf('\nPlace IK error       : %.5f m\n', placeErr);

waypoints = {homeConfig; detectedConfig; liftConfig; placeConfig; homeConfig};
segLabels = {'Vision-guided approach','Picking object', ...
             'Transporting','Returning home'};
nSteps    = 50;

figure('Name','Q-Arm — Vision-Guided Pick & Place','NumberTitle','off','Color','w');
show(robot, homeConfig, 'Visuals','off','Frames','on','PreservePlot',false);
hold on;

% Draw pick and place markers
plot3(detectedWorldPos(1), detectedWorldPos(2), detectedWorldPos(3), 's', ...
    'MarkerSize',16,'MarkerFaceColor',[0.9 0.1 0.1], ...
    'MarkerEdgeColor','k','DisplayName','Detected Object (Red)');
plot3(placeXYZ(1), placeXYZ(2), placeXYZ(3), 's', ...
    'MarkerSize',16,'MarkerFaceColor',[0.1 0.7 0.1], ...
    'MarkerEdgeColor','k','DisplayName','Place Zone');
objHandle = plot3(detectedWorldPos(1), detectedWorldPos(2), detectedWorldPos(3), ...
    'o','MarkerSize',12,'MarkerFaceColor','red','MarkerEdgeColor','k');

% Draw camera frustum (top-down view hint)
patch([-fovX/2 fovX/2 fovX/2 -fovX/2], ...
      [-fovY/2 -fovY/2 fovY/2 fovY/2], ...
      [camHeight camHeight camHeight camHeight], ...
      [0.6 0.8 1.0], 'FaceAlpha',0.15,'EdgeColor','cyan', ...
      'DisplayName','Camera FOV');

legend('Location','northeast');
xlim([-0.6, 0.6]); ylim([-0.6, 0.6]); zlim([-0.05, 0.8]);
axis manual;
view(45, 25);
ax = gca;
ax.CameraViewAngleMode = 'manual';
ax.DataAspectRatioMode = 'manual';
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

isAttached = false;
for seg = 1:numel(waypoints)-1
    q1 = waypoints{seg};
    q2 = waypoints{seg+1};
    title(ax, ['Vision Pipeline — ' segLabels{seg}], 'FontSize',12);
    for t = linspace(0,1,nSteps)
        q_interp = (1-t)*q1 + t*q2;
        % Attach at pick
        if seg == 2 && t > 0.8
            isAttached = true;
        end
        % Detach at place
        if seg == 4 && t > 0.5
            isAttached = false;
        end
        % Move object WITH robot
        if isAttached
            T = getTransform(robot, q_interp, 'END-EFFECTOR');
            pos = T(1:3,4);

            set(objHandle, 'XData', pos(1), ...
                       'YData', pos(2), ...
                       'ZData', pos(3));
        end
        % After release → snap to place
        if ~isAttached && seg >= 4
            set(objHandle, 'XData', placeXYZ(1), ...
                       'YData', placeXYZ(2), ...
                       'ZData', placeXYZ(3));
        end
        show(robot, q_interp,'Visuals','off','Frames','on', ...
            'Parent',ax,'PreservePlot',false);
        view(ax,45,25);
        xlim(ax,[-0.6,0.6]); ylim(ax,[-0.6,0.6]); zlim(ax,[-0.05,0.8]);
        drawnow limitrate;
        pause(0.02);
    end
end
title(ax,'Vision-Guided Pick & Place Complete ✓','FontSize',12);

%% ---- 8. DETECTION SUMMARY FIGURE --------------------------
figure('Name','Stage 2 Summary','NumberTitle','off','Color','w');

% Left: camera view with detection
subplot(1,2,1);
imshow(scene); hold on;
rectangle('Position',detected.BoundingBox,'EdgeColor','y','LineWidth',2);
plot(centroid(1),centroid(2),'y+','MarkerSize',20,'LineWidth',3);
plot(cx, cy, 'w+','MarkerSize',15,'LineWidth',2);
title('Camera View + Detection','FontSize',11);

% Right: top-down robot map
subplot(1,2,2); hold on;
% Draw workspace boundary circle
theta = linspace(0, 2*pi, 100);
plot(0.55*cos(theta), 0.55*sin(theta), '--', 'Color',[0.7 0.7 0.7], ...
    'LineWidth',1,'DisplayName','Workspace boundary');
% Draw all objects
colors_rgb = {[0.9 0.1 0.1],[0.1 0.7 0.1],[0.1 0.3 0.9],[0.9 0.8 0.1]};
for i = 1:nObjects
    pos_i = objects{i,1};
    plot(pos_i(1), pos_i(2), 'o', 'MarkerSize',18, ...
        'MarkerFaceColor', colors_rgb{i}, 'MarkerEdgeColor','k', ...
        'DisplayName', objects{i,2});
end
% Highlight detected object
plot(detectedWorldPos(1), detectedWorldPos(2), 'y*', ...
    'MarkerSize',20,'LineWidth',2,'DisplayName','Detected');
% Place zone
plot(placeXYZ(1), placeXYZ(2), 'k^', 'MarkerSize',14, ...
    'MarkerFaceColor','green','DisplayName','Place zone');
% Robot base
plot(0, 0, 'k+','MarkerSize',15,'LineWidth',3,'DisplayName','Robot base');

axis equal; grid on;
xlim([-0.6 0.6]); ylim([-0.6 0.6]);
xlabel('X (m)'); ylabel('Y (m)');
title('Top-Down Robot Map','FontSize',11);
legend('Location','southoutside','NumColumns',3,'FontSize',8);

sgtitle('Stage 2: Vision-Based Object Detection Pipeline','FontSize',13,'FontWeight','bold');

fprintf('\n✓ Stage 2 complete!\n');
fprintf('Next: Stage 3 — Simulink model with Simscape Multibody.\n');

%% =========================================================
%  LOCAL FUNCTIONS
%% =========================================================
function [bestConfig, bestErr] = solveIK(ik, targetXYZ, jointLimits, nRestarts)
    targetTform = trvec2tform(targetXYZ) * eul2tform([0 pi 0]);
    weights     = [1, 1, 1, 0.1, 0.1, 0.1];
    bestErr     = inf;
    bestConfig  = zeros(1,4);
    for k = 1:nRestarts
        if k == 1
            initGuess = deg2rad([0, 45, -30, 0]);
        else
            initGuess = jointLimits(:,1)' + ...
                rand(1,4).*(jointLimits(:,2)-jointLimits(:,1))';
        end
        [cfg, info] = ik('END-EFFECTOR', targetTform, weights, initGuess);
        if info.PoseErrorNorm < bestErr
            bestErr    = info.PoseErrorNorm;
            bestConfig = cfg;
        end
        if bestErr < 0.005; break; end
    end
end
