classdef Context
    methods (Static)
        function ctx = initContext()
            data = load(fullfile(pwd, "modelData", "arm_data.mat"), ...
                "q_marker_int1", "q_retBlue1", "q_grip1", "robot");

            ctx.P = competitionController.Context.params();
            ctx.robot = data.robot;
            ctx.ik = inverseKinematics("RigidBodyTree", ctx.robot);
            ctx.scanQ = data.q_marker_int1(:, 1);
            ctx.scanSweepQs = competitionController.Context.buildScanSweep(ctx.scanQ);
            ctx.blueDropQ = data.q_retBlue1(:, end);
            ctx.greenDropQ = competitionController.Planning.solveMirroredDrop(ctx.ik, ctx.robot, ctx.blueDropQ);
            ctx.homeQ = data.q_retBlue1(:, 1);
            ctx.closeSequenceLength = size(data.q_grip1, 2);
            ctx.lastArmQ = ctx.homeQ;
            ctx.state = "MOVE_SCAN";
            ctx.scanTargetIndex = 1;
            ctx.scanDirection = 1;
            ctx.settleCount = 0;
            ctx.holdCount = 0;
            ctx.emptyScanCount = 0;
            ctx.retryCount = 0;
            ctx.completedCount = 0;
            ctx.stepCount = 0;
            ctx.verifyCount = 0;
            ctx.refineCount = 0;
            ctx.phaseOrder = ["PHASE1_FIXED", "PHASE2_SHAPE", "PHASE3_ORIENTATION", "PHASE4_RANDOM"];
            ctx.phaseIndex = 1;
            ctx.phase = ctx.phaseOrder(ctx.phaseIndex);
            ctx.target = competitionController.Context.emptyTarget();
            ctx.phase1Targets = competitionController.Phase1Handler.buildTargets(ctx);
            ctx.phase1TargetIndex = 1;
            ctx.phase2Targets = repmat(struct("enabled", false, "target", competitionController.Context.emptyTarget()), 0, 1);
            ctx.phase2TargetIndex = 1;
            ctx.completedTargets = zeros(0, 3);
            ctx.failedTargets = zeros(0, 3);
            ctx.lastCameraTform = eye(4);

            fprintf('[PHASE] starting -> %s\n', ctx.phase);
        end

        function P = params()
            P.cam.fx = 525.0;
            P.cam.fy = 525.0;
            P.cam.cx = 320.0;
            P.cam.cy = 240.0;

            P.det.minBlobArea = 120;
            P.det.maxBlobArea = 70000;
            P.det.minDepth = 0.05;
            P.det.maxDepth = 2.50;
            P.det.ringPad = 12;
            P.det.minHeight = 0.005;
            P.det.depthBand = 0.08;
            P.det.rgbForegroundThreshold = 252;
            P.det.rgbDarkThreshold = 245;
            P.det.minSat = 0.20;
            P.det.minDepthBlobArea = 60;
            P.det.maxDepthBlobArea = 120000;
            P.det.maxForegroundCoverage = 0.55;
            P.det.whiteMinValue = 0.72;
            P.det.whiteMaxSat = 0.18;
            P.det.blackMaxValue = 0.22;
            P.det.blackMaxSat = 0.35;
            P.det.baseMaskTopFrac = 0.72;
            P.det.baseMaskLeftFrac = 0.22;
            P.det.baseMaskRightFrac = 0.78;

            P.workspace.x = [-0.10 1.10];
            P.workspace.y = [-1.00 1.00];
            P.workspace.z = [-0.03 0.55];
            P.workspace.baseRejectX = [-0.20 0.20];
            P.workspace.baseRejectY = [-0.24 0.24];
            P.workspace.baseRejectZMax = 0.30;

            P.class.uprightBottleHeight = 0.16;
            P.class.uprightCanHeight = 0.09;
            P.class.cubeHeight = 0.045;
            P.class.markerLength = 0.12;
            P.class.longObjectLength = 0.12;
            P.class.maxReasonableHeight = 0.60;
            P.class.maxBottleHeight = 0.32;
            P.class.maxCanHeight = 0.20;
            P.class.maxMarkerHeight = 0.08;
            P.class.maxSpamHeight = 0.10;
            P.class.maxCubeHeight = 0.08;
            P.class.nominalBottleHeight = 0.18;
            P.class.nominalCanHeight = 0.11;
            P.class.nominalMarkerHeight = 0.03;
            P.class.nominalSpamHeight = 0.06;
            P.class.nominalCubeHeight = 0.045;

            P.motion.scanStep = 0.08;
            P.motion.travelStep = 0.09;
            P.motion.graspStep = 0.04;
            P.motion.arriveTol = 0.025;
            P.motion.graspTol = 0.015;
            P.motion.pregraspLift = 0.11;
            P.motion.graspClearance = 0.006;
            P.motion.postLift = 0.16;
            P.motion.minGraspZ = 0.05;
            P.motion.maxGraspZ = 0.075;
            P.motion.xyNudgeGain = 0.0;
            P.motion.maxPregraspZ = 0.22;
            P.motion.maxLiftZ = 0.28;
            P.motion.graspBiasXY = [0.000; 0.000];
            P.motion.graspBiasZ = -0.025;

            P.sim.baseFixedStep = 0.01;
            P.sim.fixedStep = 0.02;
            speedScale = P.sim.fixedStep / P.sim.baseFixedStep;
            P.motion.scanStep = P.motion.scanStep * speedScale;
            P.motion.travelStep = P.motion.travelStep * speedScale;
            P.motion.graspStep = P.motion.graspStep * speedScale;

            P.closeHoldSteps = max(8, round(36 / speedScale));
            P.openHoldSteps = max(4, round(10 / speedScale));
            P.verifyHoldSteps = max(4, round(8 / speedScale));
            P.scanSettlingSteps = max(2, round(3 / speedScale));
            P.refineSettlingSteps = max(2, round(2 / speedScale));
            P.maxEmptyScans = 40;
            P.recoveryEmptyScans = 6;
            P.maxTargetRetries = 5;
            P.memoryRadius = 0.08;
            P.refineRadius = 0.12;
            P.grasp.minClearance = 0.015;
            P.retry.yawOffsets = [0, pi / 2, -pi / 2, pi, pi / 4];
            P.retry.zOffsets = [0, 0.010, 0.010, 0.015, -0.005];
            P.phaseAdvanceEmptyScans = 8;

            P.drop.blue = [-0.50 0.35 0.10];
            P.drop.green = [-0.50 -0.35 0.10];
        end

        function target = emptyTarget()
            target = struct( ...
                "position", zeros(3, 1), ...
                "qPre", zeros(6, 1), ...
                "qGrasp", zeros(6, 1), ...
                "qLift", zeros(6, 1), ...
                "qDrop", zeros(6, 1), ...
                "graspPosition", zeros(3, 1), ...
                "graspYaw", 0, ...
                "score", -inf, ...
                "label", "unknown", ...
                "color", "unknown", ...
                "height", 0, ...
                "source", "vision", ...
                "dropBin", "green", ...
                "basePosition", zeros(3, 1), ...
                "baseYaw", 0, ...
                "variantIndex", 1);
        end

        function scanGoal = currentScanGoal(ctx)
            scanGoal = ctx.scanSweepQs(:, ctx.scanTargetIndex);
        end

        function sweepQs = buildScanSweep(scanQ)
            yawOffsets = linspace(-pi / 2, pi / 2, 5);
            sweepQs = repmat(scanQ(:), 1, numel(yawOffsets));
            for k = 1:numel(yawOffsets)
                q = scanQ(:);
                q(1) = wrapToPi(scanQ(1) + yawOffsets(k));
                q(6) = wrapToPi(scanQ(6) - 0.35 * sign(yawOffsets(k)));
                sweepQs(:, k) = q;
            end
        end

        function camTF = parseCameraTform(cameraTform, fallback)
            camTF = fallback;
            if isempty(cameraTform)
                return;
            end

            vals = double(cameraTform);
            if isequal(size(vals), [4 4])
                camTF = vals;
            elseif isequal(size(vals), [3 4])
                camTF = [vals; 0 0 0 1];
            elseif isvector(vals) && numel(vals) == 16
                camTF = reshape(vals, 4, 4);
            elseif isvector(vals) && numel(vals) == 12
                camTF = [reshape(vals, 3, 4); 0 0 0 1];
            end
        end

        function [nextQ, arrived] = stepArm(currentQ, targetQ, maxStep, tol)
            delta = targetQ(:) - currentQ(:);
            dist = norm(delta);
            if dist <= tol
                nextQ = targetQ(:);
                arrived = true;
                return;
            end

            scale = min(maxStep / max(dist, 1e-9), 1.0);
            nextQ = currentQ(:) + scale * delta;
            arrived = false;
        end

        function memory = rememberPosition(memory, position)
            memory(end + 1, :) = position(:)';
        end

        function tf = nearMemory(position, memory, radius)
            tf = false;
            if isempty(memory)
                return;
            end

            dist = vecnorm(memory - position(:)', 2, 2);
            tf = any(dist < radius);
        end
    end
end
