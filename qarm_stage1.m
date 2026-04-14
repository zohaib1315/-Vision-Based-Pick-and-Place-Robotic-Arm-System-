%% =========================================================
%  Q-Arm Stage 1 (Fixed): Load Robot, Visualize, FK & IK
%  ---------------------------------------------------------
%  Fixes:
%    - Correct base frame handling (world vs base_link)
%    - Better IK initial guess to avoid singularities
%    - Multiple IK restarts for reliability
%    - Fixed workspace scatter plot
%% =========================================================

clc; clear; close all;

%% ---- 1. LOAD THE ROBOT -------------------------------------
fprintf('Loading Q-Arm URDF...\n');

robot = importrobot('QARM.urdf');
robot.DataFormat = 'row';
robot.Gravity     = [0, 0, -9.81];

fprintf('Robot base name : %s\n', robot.BaseName);
fprintf('Bodies          : %d\n', robot.NumBodies);
fprintf('Joints          : %d\n', numel(robot.homeConfiguration));

% ---- Joint limits (hardcoded from URDF for easy reference) ----
jointNames  = {'YAW','SHOULDER','ELBOW','WRIST'};
jointLimits = [-2.967,  2.967;   % YAW
               -1.483,  1.483;   % SHOULDER
               -1.658,  1.309;   % ELBOW
               -3.010,  3.010];  % WRIST

fprintf('\n%-10s  %-8s  %-8s\n','Joint','Lower°','Upper°');
for i = 1:4
    fprintf('%-10s  %+6.1f    %+6.1f\n', jointNames{i}, ...
        rad2deg(jointLimits(i,1)), rad2deg(jointLimits(i,2)));
end

%% ---- 2. HOME POSE ------------------------------------------
homeConfig = robot.homeConfiguration;  % [0 0 0 0]

figure('Name','Q-Arm — Home Pose','NumberTitle','off');
show(robot, homeConfig, 'Visuals','off','Frames','on');
title('Q-Arm — Home Pose (all joints = 0°)');
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45,25);

%% ---- 3. FORWARD KINEMATICS ---------------------------------
fprintf('\n--- Forward Kinematics ---\n');

testConfig = deg2rad([30, 20, -15, 10]);
T_ee = getTransform(robot, testConfig, 'END-EFFECTOR');
pos  = T_ee(1:3,4);

fprintf('Input  (deg): YAW=30  SHOULDER=20  ELBOW=-15  WRIST=10\n');
fprintf('EE position : X=%.4f  Y=%.4f  Z=%.4f  m\n', pos);

figure('Name','Q-Arm — Test Config','NumberTitle','off');
show(robot, testConfig, 'Visuals','off','Frames','on');
hold on;
plot3(pos(1),pos(2),pos(3),'r*','MarkerSize',14,'LineWidth',2);
title('Q-Arm — Test Config (FK result = red star)');
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45,25);

%% ---- 4. INVERSE KINEMATICS (fixed) -------------------------
fprintf('\n--- Inverse Kinematics (fixed) ---\n');

% Set up IK solver
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations     = 3000;
ik.SolverParameters.MaxTime           = 15;
ik.SolverParameters.GradientTolerance = 1e-12;
ik.SolverParameters.SolutionTolerance = 1e-10;


% Position matters more than orientation for pick-and-place
weights = [1, 1, 1, 0.1, 0.1, 0.1];

% ---- Helper: multi-restart IK for reliability ---------------
function [bestConfig, bestErr] = solveIK(ik, targetXYZ, jointLimits, nRestarts)
    targetTform = trvec2tform(targetXYZ) * eul2tform([0 pi 0]);
    weights = [1, 1, 1, 0.1, 0.1, 0.1];
    bestErr     = inf;
    bestConfig  = zeros(1,4);

    for k = 1:nRestarts
        if k == 1
            % First try: elbow-up guess (good starting pose)
            initGuess = deg2rad([0, 45, -30, 0]);
        else
            % Remaining tries: random within limits
            initGuess = jointLimits(:,1)' + ...
                rand(1,4) .* (jointLimits(:,2) - jointLimits(:,1))';
        end

        [cfg, info] = ik('END-EFFECTOR', targetTform, weights, initGuess);

        if info.PoseErrorNorm < bestErr
            bestErr    = info.PoseErrorNorm;
            bestConfig = cfg;
        end

        if bestErr < 0.005  % good enough — stop early
            break;
        end
    end
end

% ---- Pick target --------------------------------------------
pickXYZ = [0.22, 0.05, 0.10];
liftXYZ = [pickXYZ(1), pickXYZ(2), pickXYZ(3) + 0.15];
fprintf('\nPick target  : [%.2f, %.2f, %.2f] m\n', pickXYZ);

[pickConfig, pickErr] = solveIK(ik, pickXYZ, jointLimits, 20);

fprintf('IK solution  : YAW=%.1f°  SHOULDER=%.1f°  ELBOW=%.1f°  WRIST=%.1f°\n', ...
    rad2deg(pickConfig));
fprintf('Position error: %.5f m  ', pickErr);
if pickErr < 0.01
    fprintf('✓ Good solution\n');
elseif pickErr < 0.05
    fprintf('~ Acceptable\n');
else
    fprintf('✗ Poor — target may be near workspace boundary\n');
end
% Verify with FK
T_pick = getTransform(robot, pickConfig, 'END-EFFECTOR');
fprintf('FK verify    : [%.4f, %.4f, %.4f] m  (should match target)\n', ...
    T_pick(1:3,4)');

% ---- Place target -------------------------------------------
placeXYZ = [-0.15, 0.15, 0.12];
placeLiftXYZ = [placeXYZ(1), placeXYZ(2), placeXYZ(3) + 0.18];
fprintf('\nPlace target : [%.2f, %.2f, %.2f] m\n', placeXYZ);

[placeConfig, placeErr] = solveIK(ik, placeXYZ, jointLimits, 20);
[liftConfig, liftErr] = solveIK(ik, liftXYZ, jointLimits, 15);
[placeLiftConfig, ~] = solveIK(ik, placeLiftXYZ, jointLimits, 15);

fprintf('IK solution  : YAW=%.1f°  SHOULDER=%.1f°  ELBOW=%.1f°  WRIST=%.1f°\n', ...
    rad2deg(placeConfig));
fprintf('Position error: %.5f m  ', placeErr);
if placeErr < 0.01
    fprintf('✓ Good solution\n');
elseif placeErr < 0.05
    fprintf('~ Acceptable\n');
else
    fprintf('✗ Poor — target may be near workspace boundary\n');
end
fprintf('Lift error: %.5f\n', liftErr);
%% ---- 4. ANIMATION (3D view locked) -------------------------
waypoints = {
    homeConfig;
    pickConfig;     % go to object
    liftConfig;         % lift object
    placeLiftConfig;    % move above place
    placeConfig;        % lower to place
    homeConfig
};
segLabels = {
    'Move to pick';
    'Lift object';
    'Move above place';
    'Lower to place';
    'Return home'
};
nSteps    = 50;
 
hFig = figure('Name','Q-Arm — Pick & Place Animation', ...
              'NumberTitle','off','Color','w');
 
% Draw scene once — before the loop
show(robot, homeConfig, 'Visuals','off','Frames','on', ...
    'PreservePlot', false);
hold on;
 
% Place target markers
plot3(pickXYZ(1),  pickXYZ(2),  pickXYZ(3),  's', ...
    'MarkerSize',14,'MarkerFaceColor',[0 0.8 0], ...
    'MarkerEdgeColor','k','DisplayName','Pick target');
plot3(placeXYZ(1), placeXYZ(2), placeXYZ(3), 's', ...
    'MarkerSize',14,'MarkerFaceColor',[0.9 0.1 0.1], ...
    'MarkerEdgeColor','k','DisplayName','Place target');
legend('Location','northeast');
 
% Fix axis limits based on robot reach (~0.6m)
xlim([-0.6, 0.6]);
ylim([-0.6, 0.6]);
zlim([-0.05, 0.65]);
axis manual;          % <-- LOCK axes so show() can't rescale
 
% Set and LOCK the 3D view
view(45, 25);
ax = gca;
ax.CameraViewAngleMode = 'manual';   % prevents zoom reset
ax.DataAspectRatioMode = 'manual';   % prevents aspect reset
 
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

objHandle = plot3(pickXYZ(1), pickXYZ(2), pickXYZ(3), ...
    'o','MarkerSize',12,'MarkerFaceColor','red','MarkerEdgeColor','k');
% Animate
isAttached = false;   % object initially NOT attached
for seg = 1:numel(waypoints)-1
    q1 = waypoints{seg};
    q2 = waypoints{seg+1};
    title(ax, ['Q-Arm — ' segLabels{seg}], 'FontSize', 12);
    for t = linspace(0, 1, nSteps)
        q_interp = (1-t)*q1 + t*q2;
        % Attach at pick
        if seg == 2 && t > 0.8
            isAttached = true;
        end
        % Detach at place
        if seg == 5 && t > 0.2
            isAttached = false;
        end
        % Update object position ONLY when attached
        if isAttached
            T = getTransform(robot, q_interp, 'END-EFFECTOR');
            pos = T(1:3,4);
            set(objHandle, 'XData', pos(1), ...
                   'YData', pos(2), ...
                   'ZData', pos(3));
        end
        % When released → snap to place location
        if ~isAttached && seg >= 5
            set(objHandle, 'XData', placeXYZ(1), ...
                   'YData', placeXYZ(2), ...
                   'ZData', placeXYZ(3));
        end
 
        show(robot, q_interp, ...
            'Visuals','off', ...
            'Frames','on', ...
            'Parent', ax, ...
            'PreservePlot', false);   % redraws robot only, keeps markers
 
        % Re-lock view every frame (show() tries to reset it)
        view(ax, 45, 25);
        xlim(ax, [-0.6, 0.6]);
        ylim(ax, [-0.6, 0.6]);
        zlim(ax, [-0.05, 0.65]);
 
        drawnow limitrate;
        pause(0.02);
    end
end
title(ax, 'Q-Arm — Pick & Place Complete ✓', 'FontSize', 12);
 
%% ---- 5. WORKSPACE CLOUD (fixed) ----------------------------
fprintf('Generating workspace cloud...\n');
 
nSamples = 6000;
reachPts = zeros(nSamples, 3);
 
for k = 1:nSamples
    q = jointLimits(:,1)' + rand(1,4).*(jointLimits(:,2)-jointLimits(:,1))';
    T = getTransform(robot, q, 'END-EFFECTOR');
    reachPts(k,:) = T(1:3,4)';
end
 
% Remove outliers (points beyond 0.8m from origin — bad IK artifacts)
dist = vecnorm(reachPts, 2, 2);
reachPts = reachPts(dist < 0.8, :);
 
figure('Name','Q-Arm — Reachable Workspace','NumberTitle','off','Color','w');
scatter3(reachPts(:,1), reachPts(:,2), reachPts(:,3), ...
    8, reachPts(:,3), 'filled', 'MarkerFaceAlpha', 0.35);
colormap('jet');
cb = colorbar; cb.Label.String = 'Z height (m)';
clim([min(reachPts(:,3)), max(reachPts(:,3))]);  % fix color scaling
 
hold on;
plot3(pickXYZ(1),  pickXYZ(2),  pickXYZ(3), '^', ...
    'MarkerSize',12,'MarkerFaceColor','g','MarkerEdgeColor','k','DisplayName','Pick');
plot3(placeXYZ(1), placeXYZ(2), placeXYZ(3),'^', ...
    'MarkerSize',12,'MarkerFaceColor','r','MarkerEdgeColor','k','DisplayName','Place');
legend('Location','best');
 
title('Q-Arm Reachable Workspace');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis equal; grid on; view(45, 25);
 
fprintf('✓ Done! Ready for Stage 2.\n');