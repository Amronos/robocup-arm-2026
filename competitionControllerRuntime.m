function [gripperStatus, currentConfig, stop] = competitionControllerRuntime(rgbFrame, depthFrame, cameraTform, targetGrasped)
persistent ctx

if isempty(ctx)
    ctx = localInitContext();
end

gripperStatus = 0;
currentConfig = ctx.lastArmQ;
stop = false;

camTF = localParseCameraTform(cameraTform, ctx.lastCameraTform);
ctx.lastCameraTform = camTF;
ctx.stepCount = ctx.stepCount + 1;

if isempty(rgbFrame) || isempty(depthFrame)
    [currentConfig, arrived] = localStepArm(ctx.lastArmQ, localCurrentScanGoal(ctx), ctx.P.motion.scanStep, ctx.P.motion.arriveTol);
    ctx.lastArmQ = currentConfig;
    if arrived
        ctx.state = "SCAN";
    end
    return;
end

switch ctx.state
    case "MOVE_SCAN"
        [ctx.lastArmQ, arrived] = localStepArm(ctx.lastArmQ, localCurrentScanGoal(ctx), ctx.P.motion.scanStep, ctx.P.motion.arriveTol);
        currentConfig = ctx.lastArmQ;
        gripperStatus = 0;
        if arrived
            ctx.state = "SCAN";
            ctx.settleCount = 0;
        end

    case "SCAN"
        currentConfig = localCurrentScanGoal(ctx);
        gripperStatus = 0;
        ctx.lastArmQ = currentConfig;
        ctx.settleCount = ctx.settleCount + 1;
        if ctx.settleCount < ctx.P.scanSettlingSteps
            return;
        end

        switch ctx.phase
            case "PHASE1_FIXED"
                [ctx, hasTarget] = localDispatchQueuedTarget(ctx.phase1Targets, ctx.phase1TargetIndex, ctx, "phase1TargetIndex");
                if hasTarget
                    localLogPhaseTarget(ctx);
                    ctx.state = "MOVE_PREGRASP";
                    return;
                end
                ctx = localAdvancePhase(ctx, "phase1 queue drained");
                return;

            case "PHASE2_SHAPE"
                queue = localPerceiveScene(rgbFrame, depthFrame, camTF, ctx);
                if isempty(ctx.phase2Targets)
                    ctx.phase2Targets = localBuildPhase2Targets(queue, ctx);
                    ctx.phase2TargetIndex = 1;
                end
                [ctx, hasTarget] = localDispatchQueuedTarget(ctx.phase2Targets, ctx.phase2TargetIndex, ctx, "phase2TargetIndex");
                if hasTarget
                    localLogPhaseTarget(ctx);
                    ctx.state = "MOVE_PREGRASP";
                    return;
                end
                if ~isempty(ctx.phase2Targets)
                    ctx = localAdvancePhase(ctx, "phase2 fixed-slot queue drained");
                    return;
                end
                ctx = localHandleEmptyPhaseScan(ctx, "no usable phase2 slots");
                return;

            otherwise
                queue = localPerceiveScene(rgbFrame, depthFrame, camTF, ctx);
                queue = localFilterQueueForPhase(queue, ctx);
                if isempty(queue)
                    ctx = localHandleEmptyPhaseScan(ctx, "no valid targets");
                    return;
                end

                ctx.emptyScanCount = 0;
                ctx.retryCount = 0;
                ctx.target = queue(1);
                localLogPhaseTarget(ctx);
                ctx.state = "MOVE_PREGRASP";
        end

    case "MOVE_PREGRASP"
        [ctx.lastArmQ, arrived] = localStepArm(ctx.lastArmQ, ctx.target.qPre, ctx.P.motion.travelStep, ctx.P.motion.arriveTol);
        currentConfig = ctx.lastArmQ;
        gripperStatus = 0;
        if arrived
            if ctx.target.source == "fixed"
                ctx.state = "MOVE_GRASP";
            else
                ctx.state = "REFINE_GRASP";
                ctx.refineCount = 0;
            end
        end

    case "REFINE_GRASP"
        currentConfig = ctx.target.qPre;
        ctx.lastArmQ = currentConfig;
        gripperStatus = 0;
        ctx.refineCount = ctx.refineCount + 1;
        if ctx.refineCount < ctx.P.refineSettlingSteps
            return;
        end

        queue = localPerceiveScene(rgbFrame, depthFrame, camTF, ctx);
        refinedTarget = localSelectRefinedTarget(queue, ctx.target, ctx);
        if ~isempty(refinedTarget)
            ctx.target = refinedTarget;
            fprintf('[REFINE] %s %s pos=[%.3f %.3f %.3f] h=%.3f grasp=[%.3f %.3f %.3f] yaw=%.2f\n', ...
                ctx.target.color, ctx.target.label, ...
                ctx.target.position(1), ctx.target.position(2), ctx.target.position(3), ctx.target.height, ...
                ctx.target.graspPosition(1), ctx.target.graspPosition(2), ctx.target.graspPosition(3), ctx.target.graspYaw);
        end
        ctx.state = "MOVE_GRASP";

    case "MOVE_GRASP"
        [ctx.lastArmQ, arrived] = localStepArm(ctx.lastArmQ, ctx.target.qGrasp, ctx.P.motion.graspStep, ctx.P.motion.graspTol);
        currentConfig = ctx.lastArmQ;
        gripperStatus = 0;
        if arrived
            ctx.state = "CLOSE";
            ctx.holdCount = 0;
        end

    case "CLOSE"
        currentConfig = ctx.target.qGrasp;
        ctx.lastArmQ = currentConfig;
        gripperStatus = 1;
        ctx.holdCount = ctx.holdCount + 1;
        if ctx.holdCount >= max(ctx.P.closeHoldSteps, ctx.closeSequenceLength + 4)
            ctx.state = "VERIFY";
            ctx.verifyCount = 0;
        end

    case "VERIFY"
        currentConfig = ctx.target.qGrasp;
        ctx.lastArmQ = currentConfig;
        gripperStatus = 1;
        ctx.verifyCount = ctx.verifyCount + 1;
        if logical(targetGrasped)
            ctx.state = "LIFT";
            ctx.holdCount = 0;
            ctx.verifyCount = 0;
        elseif ctx.verifyCount < ctx.P.verifyHoldSteps
            return;
        else
            ctx.retryCount = ctx.retryCount + 1;
            ctx.failedTargets = localRememberPosition(ctx.failedTargets, ctx.target.position);
            if ctx.retryCount >= ctx.P.maxTargetRetries
                ctx.state = "MOVE_SCAN";
                ctx.target = localEmptyTarget();
            else
                ctx.state = "MOVE_PREGRASP";
            end
            ctx.verifyCount = 0;
        end

    case "LIFT"
        [ctx.lastArmQ, arrived] = localStepArm(ctx.lastArmQ, ctx.target.qLift, ctx.P.motion.travelStep, ctx.P.motion.arriveTol);
        currentConfig = ctx.lastArmQ;
        gripperStatus = 1;
        if arrived
            ctx.state = "MOVE_BIN";
        end

    case "MOVE_BIN"
        [ctx.lastArmQ, arrived] = localStepArm(ctx.lastArmQ, ctx.target.qDrop, ctx.P.motion.travelStep, ctx.P.motion.arriveTol);
        currentConfig = ctx.lastArmQ;
        gripperStatus = 1;
        if arrived
            ctx.state = "OPEN";
            ctx.holdCount = 0;
        end

    case "OPEN"
        currentConfig = ctx.target.qDrop;
        ctx.lastArmQ = currentConfig;
        gripperStatus = 0;
        ctx.holdCount = ctx.holdCount + 1;
        if ctx.holdCount >= ctx.P.openHoldSteps
            ctx.completedCount = ctx.completedCount + 1;
            ctx.completedTargets = localRememberPosition(ctx.completedTargets, ctx.target.position);
            ctx.target = localEmptyTarget();
            ctx.state = "MOVE_SCAN";
        end

    case "STOP"
        currentConfig = ctx.lastArmQ;
        gripperStatus = 0;
        stop = true;

    otherwise
        ctx.state = "MOVE_SCAN";
end
end

function ctx = localInitContext()
data = load(fullfile(pwd, "modelData", "arm_data.mat"), "q_marker_int1", "q_retBlue1", "q_grip1", "robot");
ctx.P = localParams();
ctx.robot = data.robot;
ctx.ik = inverseKinematics("RigidBodyTree", ctx.robot);
ctx.scanQ = data.q_marker_int1(:, 1);
ctx.scanSweepQs = localBuildScanSweep(ctx.scanQ);
ctx.blueDropQ = data.q_retBlue1(:, end);
ctx.greenDropQ = localSolveMirroredDrop(ctx.ik, ctx.robot, ctx.blueDropQ);
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
ctx.phaseOrder = ["PHASE1_FIXED", "PHASE2_SHAPE", "PHASE3_ORIENTATION", "PHASE4_RANDOM_BIN"];
ctx.phaseIndex = 1;
ctx.phase = ctx.phaseOrder(ctx.phaseIndex);
ctx.target = localEmptyTarget();
ctx.phase1Targets = localBuildPhase1Targets(ctx);
ctx.phase1TargetIndex = 1;
ctx.phase2Targets = repmat(struct("enabled", false, "target", localEmptyTarget()), 0, 1);
ctx.phase2TargetIndex = 1;
ctx.completedTargets = zeros(0, 3);
ctx.failedTargets = zeros(0, 3);
ctx.lastCameraTform = eye(4);
end

function P = localParams()
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
P.motion.scanStep = 0.05;
P.motion.travelStep = 0.06;
P.motion.graspStep = 0.025;
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
P.closeHoldSteps = 60;
P.openHoldSteps = 18;
P.verifyHoldSteps = 12;
P.scanSettlingSteps = 5;
P.refineSettlingSteps = 4;
P.maxEmptyScans = 40;
P.recoveryEmptyScans = 6;
P.maxTargetRetries = 2;
P.memoryRadius = 0.08;
P.refineRadius = 0.12;
P.grasp.minClearance = 0.015;
P.phaseAdvanceEmptyScans = 8;
P.phase2.slotMatchRadius = 0.16;
P.phase2.x = [0.74 0.96];
P.phase2.y = [-0.22 0.12];
P.phase3.x = [0.32 0.72];
P.phase3.y = [-0.24 0.18];
P.phase4.x = [-0.02 0.56];
P.phase4.y = [-0.76 -0.38];
P.drop.blue = [-0.50 0.35 0.10];
P.drop.green = [-0.50 -0.35 0.10];
end

function target = localEmptyTarget()
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
    "dropBin", "green");
end

function [ctx, hasTarget] = localDispatchQueuedTarget(targets, targetIndex, ctx, indexField)
hasTarget = false;

while targetIndex <= numel(targets)
    candidate = targets(targetIndex);
    targetIndex = targetIndex + 1;
    if ~candidate.enabled
        continue;
    end
    ctx.target = candidate.target;
    ctx.(indexField) = targetIndex;
    ctx.retryCount = 0;
    hasTarget = true;
    return;
end
ctx.(indexField) = targetIndex;
end

function scanGoal = localCurrentScanGoal(ctx)
scanGoal = ctx.scanSweepQs(:, ctx.scanTargetIndex);
end

function sweepQs = localBuildScanSweep(scanQ)
yawOffsets = linspace(-pi/2, pi/2, 5);
sweepQs = repmat(scanQ(:), 1, numel(yawOffsets));
for k = 1:numel(yawOffsets)
    q = scanQ(:);
    q(1) = wrapToPi(scanQ(1) + yawOffsets(k));
    q(6) = wrapToPi(scanQ(6) - 0.35 * sign(yawOffsets(k)));
    sweepQs(:, k) = q;
end
end

function camTF = localParseCameraTform(cameraTform, fallback)
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

function queue = localPerceiveScene(rgbFrame, depthFrame, camTF, ctx)
P = ctx.P;
rgbFrame = uint8(rgbFrame);
depthFrame = double(depthFrame);

maskDepth = localDepthForegroundMask(depthFrame, P);
maskColor = localObjectColorMask(rgbFrame, P);
maskRgb = imdilate(maskColor, strel("disk", 2)) & maskDepth;
maskRgb = imopen(maskRgb, strel("disk", 2));
maskRgb = imclose(maskRgb, strel("disk", 4));
maskRgb = imfill(maskRgb, "holes");
maskRgb = bwareaopen(maskRgb, P.det.minDepthBlobArea);

mask = maskDepth & imdilate(maskColor, strel("disk", 3));
mask = imopen(mask, strel("disk", 2));
mask = imclose(mask, strel("disk", 4));
mask = imfill(mask, "holes");
mask = bwareaopen(mask, P.det.minDepthBlobArea);
mask = localSuppressRobotBaseMask(mask, P);
maskRgb = localSuppressRobotBaseMask(maskRgb, P);

cc = bwconncomp(mask, 8);
props = regionprops(cc, "BoundingBox", "Centroid", "Area", "PixelIdxList", "Orientation", "MajorAxisLength", "MinorAxisLength");

queue = repmat(localEmptyTarget(), 0, 1);
rawCandidates = 0;
frameArea = numel(mask);
rejectPose = 0;
rejectWorkspace = 0;
rejectMemory = 0;
rejectIK = 0;
rejectSelf = 0;
softUnsafe = 0;
for i = 1:numel(props)
    if props(i).Area < P.det.minDepthBlobArea || props(i).Area > P.det.maxDepthBlobArea
        continue;
    end
    if props(i).Area > 0.85 * frameArea
        continue;
    end
    rawCandidates = rawCandidates + 1;

    bbox = props(i).BoundingBox;
    [rgbPatch, depthPatch, validDepth] = localCropPatches(rgbFrame, depthFrame, bbox);
    if nnz(validDepth) < 20
        continue;
    end

    [position, height, poseOK] = localEstimatePose(props(i), depthFrame, bbox, validDepth, camTF, P);
    if ~poseOK
        rejectPose = rejectPose + 1;
        continue;
    end

    if localNearMemory(position, ctx.completedTargets, P.memoryRadius) || ...
            localNearMemory(position, ctx.failedTargets, 0.04)
        rejectMemory = rejectMemory + 1;
        continue;
    end

    [label, color] = localClassifyObject(rgbPatch, depthPatch, validDepth, bbox, height, props(i), P);
    [position, height, hardReject, qualityPenalty] = localSanitizeTargetEstimate(position, height, label, P);
    if hardReject
        rejectWorkspace = rejectWorkspace + 1;
        continue;
    end
    if localIsRobotSelfCandidate(position, bbox, size(mask), P)
        rejectSelf = rejectSelf + 1;
        continue;
    end
    [graspScore, isSafe] = localGraspScore(props(i), props, position, bbox, height, label, P);
    graspScore = graspScore * qualityPenalty;
    if ~isSafe
        graspScore = 0.6 * graspScore;
        softUnsafe = softUnsafe + 1;
    end

    [qPre, qGrasp, qLift, graspPosition, graspYaw] = localPlanTarget(position, height, props(i).Orientation, label, ctx);
    if ~all(isfinite([qPre; qGrasp; qLift])) || all(qGrasp == 0)
        rejectIK = rejectIK + 1;
        continue;
    end

    entry = localEmptyTarget();
    entry.position = position;
    entry.qPre = qPre;
    entry.qGrasp = qGrasp;
    entry.qLift = qLift;
    entry.qDrop = localDropTarget(label, color, ctx);
    entry.graspPosition = graspPosition;
    entry.graspYaw = graspYaw;
    entry.score = localPointValue(label, color) * graspScore;
    entry.label = label;
    entry.color = color;
    entry.height = height;
    entry.source = "vision";
    entry.dropBin = localBinForLabelColor(label, color);
    queue(end+1, 1) = entry; %#ok<AGROW>
end

if isempty(queue)
    fprintf('[PERCEPTION] rgbPx=%d depthPx=%d blobs=%d graspable=0 pose=%d ws=%d self=%d mem=%d unsafe=%d ik=%d\n', ...
        nnz(maskRgb), nnz(maskDepth), rawCandidates, rejectPose, rejectWorkspace, rejectSelf, rejectMemory, softUnsafe, rejectIK);
    return;
end

[~, order] = sort([queue.score], "descend");
queue = queue(order);
fprintf('[PERCEPTION] rgbPx=%d depthPx=%d blobs=%d graspable=%d pose=%d ws=%d self=%d mem=%d unsafe=%d ik=%d\n', ...
    nnz(maskRgb), nnz(maskDepth), rawCandidates, numel(queue), rejectPose, rejectWorkspace, rejectSelf, rejectMemory, softUnsafe, rejectIK);
end

function queue = localFilterQueueForPhase(queue, ctx)
if isempty(queue)
    return;
end

switch ctx.phase
    case "PHASE2_SHAPE"
        keep = false(numel(queue), 1);
        for idx = 1:numel(queue)
            pos = queue(idx).position;
            keep(idx) = any(queue(idx).label == ["can", "bottle"]) && ...
                pos(1) >= ctx.P.phase2.x(1) && pos(1) <= ctx.P.phase2.x(2) && ...
                pos(2) >= ctx.P.phase2.y(1) && pos(2) <= ctx.P.phase2.y(2);
        end
        queue = queue(keep);

    case "PHASE3_ORIENTATION"
        keep = false(numel(queue), 1);
        for idx = 1:numel(queue)
            pos = queue(idx).position;
            keep(idx) = pos(1) >= ctx.P.phase3.x(1) && pos(1) <= ctx.P.phase3.x(2) && ...
                pos(2) >= ctx.P.phase3.y(1) && pos(2) <= ctx.P.phase3.y(2);
        end
        queue = queue(keep);

    case "PHASE4_RANDOM_BIN"
        keep = false(numel(queue), 1);
        for idx = 1:numel(queue)
            pos = queue(idx).position;
            keep(idx) = pos(1) >= ctx.P.phase4.x(1) && pos(1) <= ctx.P.phase4.x(2) && ...
                pos(2) >= ctx.P.phase4.y(1) && pos(2) <= ctx.P.phase4.y(2);
        end
        queue = queue(keep);
end
end

function mask = localObjectColorMask(rgbFrame, P)
hsvPatch = rgb2hsv(rgbFrame);
h = hsvPatch(:, :, 1);
s = hsvPatch(:, :, 2);
v = hsvPatch(:, :, 3);
redMask = ((h <= 0.04) | (h >= 0.96)) & s > 0.30 & v > 0.18;
yellowMask = (h >= 0.10 & h <= 0.18) & s > 0.28 & v > 0.22;
greenMask = (h >= 0.22 & h <= 0.42) & s > 0.22 & v > 0.15;
blueMask = (h >= 0.52 & h <= 0.72) & s > 0.20 & v > 0.15;
purpleMask = (h >= 0.72 & h <= 0.88) & s > 0.20 & v > 0.15;
whiteMask = v >= P.det.whiteMinValue & s <= P.det.whiteMaxSat;
blackMask = v <= P.det.blackMaxValue & s <= P.det.blackMaxSat;
mask = redMask | yellowMask | greenMask | blueMask | purpleMask | whiteMask | blackMask;
end

function mask = localSuppressRobotBaseMask(mask, P)
[h, w] = size(mask);
top = max(1, floor(P.det.baseMaskTopFrac * h));
left = max(1, floor(P.det.baseMaskLeftFrac * w));
right = min(w, ceil(P.det.baseMaskRightFrac * w));
mask(top:h, left:right) = false;
end

function [rgbPatch, depthPatch, validDepth] = localCropPatches(rgbFrame, depthFrame, bbox)
[h, w, ~] = size(rgbFrame);
x1 = max(1, floor(bbox(1)));
y1 = max(1, floor(bbox(2)));
x2 = min(w, ceil(bbox(1) + bbox(3)));
y2 = min(h, ceil(bbox(2) + bbox(4)));
rgbPatch = rgbFrame(y1:y2, x1:x2, :);
depthPatch = depthFrame(y1:y2, x1:x2);
validDepth = isfinite(depthPatch) & depthPatch > 0.05 & depthPatch < 2.50;
end

function [position, height, hardReject, qualityPenalty] = localSanitizeTargetEstimate(position, height, label, P)
position = position(:);
height = max(height, P.det.minHeight);
hardReject = false;
qualityPenalty = 1.0;

if position(1) < P.workspace.x(1) || position(1) > P.workspace.x(2)
    hardReject = true;
    return;
end
if position(2) < P.workspace.y(1) || position(2) > P.workspace.y(2)
    hardReject = true;
    return;
end
if position(3) < -0.12 || position(3) > P.workspace.z(2)
    hardReject = true;
    return;
end
if height > P.class.maxReasonableHeight
    hardReject = true;
    return;
end
position(3) = min(max(position(3), P.workspace.z(1)), P.workspace.z(2));

switch label
    case "bottle"
        if height > P.class.maxBottleHeight
            qualityPenalty = 0.70;
            height = min(height, P.class.maxBottleHeight);
        end
    case "can"
        if height > P.class.maxCanHeight
            qualityPenalty = 0.72;
            height = min(height, P.class.maxCanHeight);
        end
    case "marker"
        if height > P.class.maxMarkerHeight
            qualityPenalty = 0.65;
            height = min(height, P.class.maxMarkerHeight);
        end
    case "spam"
        if height > P.class.maxSpamHeight
            qualityPenalty = 0.68;
            height = min(height, P.class.maxSpamHeight);
        end
    case "cube"
        if height > P.class.maxCubeHeight
            qualityPenalty = 0.65;
            height = min(height, P.class.maxCubeHeight);
        end
end
end

function tf = localIsRobotSelfCandidate(position, bbox, imageSize, P)
tf = false;
if position(1) >= P.workspace.baseRejectX(1) && position(1) <= P.workspace.baseRejectX(2) && ...
        position(2) >= P.workspace.baseRejectY(1) && position(2) <= P.workspace.baseRejectY(2) && ...
        position(3) <= P.workspace.baseRejectZMax
    tf = true;
    return;
end

h = imageSize(1);
w = imageSize(2);
bboxCx = bbox(1) + 0.5 * bbox(3);
bboxBottom = bbox(2) + bbox(4);
if bboxBottom >= P.det.baseMaskTopFrac * h && ...
        bboxCx >= P.det.baseMaskLeftFrac * w && ...
        bboxCx <= P.det.baseMaskRightFrac * w
    tf = true;
end
end

function mask = localDepthForegroundMask(depthFrame, P)
valid = isfinite(depthFrame) & depthFrame > P.det.minDepth & depthFrame < P.det.maxDepth;
if nnz(valid) < 50
    mask = false(size(depthFrame));
    return;
end

tableDepth = prctile(depthFrame(valid), 92);
mask = valid & depthFrame < (tableDepth - P.det.depthBand);
if nnz(mask) < P.det.minDepthBlobArea
    tableDepth = prctile(depthFrame(valid), 97);
    mask = valid & depthFrame < (tableDepth - 0.03);
end
if nnz(mask) > P.det.maxForegroundCoverage * numel(mask)
    depthNorm = mat2gray(depthFrame, [P.det.minDepth P.det.maxDepth]);
    edgeMask = edge(depthNorm, 'Canny');
    edgeMask = imdilate(edgeMask, strel('disk', 2));
    mask = mask & ~edgeMask;
end
mask = imopen(mask, strel("disk", 2));
mask = imclose(mask, strel("disk", 5));
mask = imfill(mask, "holes");
mask = bwareafilt(mask, [P.det.minDepthBlobArea max(P.det.maxDepthBlobArea, P.det.minDepthBlobArea)]);
end

function [position, objHeight, poseOK] = localEstimatePose(prop, depthFrame, bbox, ~, camTF, P)
position = zeros(3, 1);
objHeight = 0;
poseOK = false;

allDepthVals = depthFrame(prop.PixelIdxList);
allDepthVals = allDepthVals(isfinite(allDepthVals) & allDepthVals > P.det.minDepth & allDepthVals < P.det.maxDepth);
depthVals = allDepthVals;
if numel(depthVals) < 20
    return;
end

objDepth = median(depthVals);
[h, w] = size(depthFrame);
x1 = max(1, floor(bbox(1)));
y1 = max(1, floor(bbox(2)));
x2 = min(w, ceil(bbox(1) + bbox(3)));
y2 = min(h, ceil(bbox(2) + bbox(4)));

pad = P.det.ringPad;
rx1 = max(1, x1 - pad);
ry1 = max(1, y1 - pad);
rx2 = min(w, x2 + pad);
ry2 = min(h, y2 + pad);
ring = depthFrame(ry1:ry2, rx1:rx2);
ringMask = true(size(ring));
ringMask((y1-ry1+1):(y2-ry1+1), (x1-rx1+1):(x2-rx1+1)) = false;
ringVals = ring(ringMask);
ringVals = ringVals(isfinite(ringVals) & ringVals > P.det.minDepth & ringVals < P.det.maxDepth);
if isempty(ringVals)
    tableDepth = prctile(depthVals, 95);
else
    tableDepth = median(ringVals);
end
objHeight = max(tableDepth - objDepth, 0);
if objHeight < P.det.minHeight
    objHeight = max(prctile(depthVals, 90) - prctile(depthVals, 10), 0.02);
end

[rowsObj, colsObj] = ind2sub(size(depthFrame), prop.PixelIdxList);
objMask = isfinite(depthFrame(prop.PixelIdxList)) & ...
          depthFrame(prop.PixelIdxList) > P.det.minDepth & ...
          depthFrame(prop.PixelIdxList) < P.det.maxDepth;
rowsObj = rowsObj(objMask);
colsObj = colsObj(objMask);
objPixDepth = depthFrame(prop.PixelIdxList);
objPixDepth = objPixDepth(objMask);

if isempty(rowsObj)
    u = prop.Centroid(1);
    v = prop.Centroid(2);
else
    localMask = false(y2 - y1 + 1, x2 - x1 + 1);
    localRows = rowsObj - y1 + 1;
    localCols = colsObj - x1 + 1;
    localIdx = sub2ind(size(localMask), localRows, localCols);
    localMask(localIdx) = true;
    distMap = bwdist(~localMask);

    coreMask = objPixDepth <= prctile(objPixDepth, 45);
    interiorThresh = max(1, 0.55 * max(distMap(:)));
    interiorMask = false(size(rowsObj));
    interiorMask(distMap(localIdx) >= interiorThresh) = true;
    centerMask = coreMask & interiorMask;
    if nnz(centerMask) < 8
        centerMask = interiorMask;
    end
    if nnz(centerMask) < 8
        centerMask = coreMask;
    end
    if nnz(centerMask) < 8
        centerMask = true(size(rowsObj));
    end

    colsCore = double(colsObj(centerMask));
    rowsCore = double(rowsObj(centerMask));
    weightVals = double(distMap(localIdx(centerMask)));
    if isempty(weightVals) || all(weightVals <= 0)
        weightVals = ones(size(colsCore));
    else
        weightVals = weightVals .^ 2;
    end
    u = sum(colsCore .* weightVals) / sum(weightVals);
    v = sum(rowsCore .* weightVals) / sum(weightVals);

    % Build a robust 3-D centroid from the object core so grasping is
    % centered on the body rather than a noisy single pixel estimate.
    zCore = double(objPixDepth(centerMask));
    if isempty(zCore)
        zCore = double(objPixDepth);
        cols3d = double(colsObj);
        rows3d = double(rowsObj);
        weight3d = ones(size(zCore));
    else
        cols3d = colsCore;
        rows3d = rowsCore;
        weight3d = weightVals;
    end
    xCore = (cols3d - P.cam.cx) .* zCore / P.cam.fx;
    yCore = (rows3d - P.cam.cy) .* zCore / P.cam.fy;
    ptsCam = [xCore(:), yCore(:), zCore(:), ones(numel(zCore), 1)]';
    ptsWorld = camTF * ptsCam;
    if ~isempty(ptsWorld)
        weight3d = weight3d(:)' / sum(weight3d(:));
        position = ptsWorld(1:3, :) * weight3d';
    end
end

u = u + P.motion.xyNudgeGain * (P.cam.cx - u);
v = v + 0.5 * P.motion.xyNudgeGain * (P.cam.cy - v);

ptCam = [(u - P.cam.cx) * objDepth / P.cam.fx; ...
         (v - P.cam.cy) * objDepth / P.cam.fy; ...
         objDepth; ...
         1];
ptWorld = camTF * ptCam;
if ~all(isfinite(position))
    position = ptWorld(1:3);
else
    % Favor the interior-point 3-D centroid and only lightly mix in the
    % pixel reprojection so perspective bias does not pull the target
    % toward object edges.
    position = 0.90 * position + 0.10 * ptWorld(1:3);
end
poseOK = all(isfinite(position)) && all(abs(position) < 5);
end

function [label, color] = localClassifyObject(rgbPatch, depthPatch, validDepth, bbox, objHeight, prop, P)
color = localDominantColor(rgbPatch);
metricWidth = bbox(3) * median(depthPatch(validDepth), "omitnan") / P.cam.fx;
metricHeight = bbox(4) * median(depthPatch(validDepth), "omitnan") / P.cam.fy;
majorExtent = max(metricWidth, metricHeight);
minorExtent = min(metricWidth, metricHeight);
aspect = max(prop.MajorAxisLength / max(prop.MinorAxisLength, 1), bbox(4) / max(bbox(3), 1));

if color == "white"
    label = "spam";
    return;
end
if color == "black"
    if majorExtent > P.class.markerLength || aspect > 2.2
        label = "marker";
    else
        label = "spam";
    end
    return;
end
if color == "purple" || color == "red_cube" || color == "blue_cube" || color == "green_cube"
    label = "cube";
    return;
end
if objHeight < P.class.cubeHeight && abs(metricWidth - metricHeight) < 0.03 && majorExtent < 0.11
    label = "cube";
    return;
end

upright = objHeight >= P.class.uprightCanHeight;
if upright
    if color == "blue"
        label = "bottle";
    elseif color == "green"
        label = "can";
    elseif color == "yellow" || color == "red"
        if objHeight >= P.class.uprightBottleHeight
            label = "bottle";
        else
            label = "can";
        end
    else
        label = "can";
    end
    return;
end

if majorExtent >= P.class.longObjectLength
    if color == "blue"
        label = "bottle";
    elseif color == "green"
        label = "can";
    elseif color == "yellow" || color == "red"
        if majorExtent > 0.17
            label = "bottle";
        else
            label = "can";
        end
    else
        label = "marker";
    end
else
    if color == "blue"
        label = "bottle";
    else
        label = "can";
    end
end

if minorExtent < 0.05 && majorExtent < 0.09 && color ~= "blue" && objHeight < 0.08
    label = "cube";
end
end

function color = localDominantColor(rgbPatch)
hsvPatch = rgb2hsv(rgbPatch);
h = hsvPatch(:, :, 1);
s = hsvPatch(:, :, 2);
v = hsvPatch(:, :, 3);
vivid = s > 0.22 & v > 0.15;

if nnz(vivid) < 20
    if mean(v(:)) > 0.55
        color = "white";
    else
        color = "black";
    end
    return;
end

votes = [
    nnz(((h < 0.05) | (h > 0.95)) & vivid), ...
    nnz((h >= 0.10 & h <= 0.18) & vivid), ...
    nnz((h >= 0.24 & h <= 0.43) & vivid), ...
    nnz((h >= 0.53 & h <= 0.72) & vivid), ...
    nnz((h >= 0.72 & h <= 0.86) & vivid)];
[~, idx] = max(votes);
names = ["red", "yellow", "green", "blue", "purple"];
color = names(idx);
end

function [score, isSafe] = localGraspScore(prop, props, position, bbox, objHeight, label, P)
fillRatio = prop.Area / max(bbox(3) * bbox(4), 1);
typeBonus = localPointValue(label, "unknown") / 30;
clearance = inf;
for i = 1:numel(props)
    other = props(i);
    if abs(other.Centroid(1) - prop.Centroid(1)) < 1e-6 && abs(other.Centroid(2) - prop.Centroid(2)) < 1e-6
        continue;
    end
    pxDist = norm(prop.Centroid - other.Centroid);
    clearance = min(clearance, pxDist * max(position(1), 0.2) / P.cam.fx);
end

if isinf(clearance)
    clearance = 0.10;
end

score = 0.40 * min(fillRatio, 1.0) + ...
        0.25 * min(objHeight / 0.18, 1.0) + ...
        0.20 * min(clearance / 0.08, 1.0) + ...
        0.15 * typeBonus;
isSafe = clearance >= P.grasp.minClearance;
end

function [qPre, qGrasp, qLift, graspPosition, yaw] = localPlanTarget(position, objHeight, orientationDeg, label, ctx)
if label == "marker"
    yaw = deg2rad(-orientationDeg);
    yaw = (pi / 2) * round(yaw / (pi / 2));
elseif any(label == ["bottle", "can", "spam"]) && objHeight < ctx.P.class.uprightCanHeight
    yaw = deg2rad(90 - orientationDeg);
else
    yaw = 0;
end

graspPosition = position(:);
effectiveHeight = localEffectiveHeightForGrasp(objHeight, label, ctx.P);
switch label
    case {"bottle", "can"}
        graspPosition(3) = graspPosition(3) - 0.55 * effectiveHeight;
    case "marker"
        graspPosition(3) = graspPosition(3) - 0.50 * effectiveHeight;
    case "spam"
        graspPosition(3) = graspPosition(3) - 0.35 * effectiveHeight;
    case "cube"
        graspPosition(3) = graspPosition(3) - 0.25 * effectiveHeight;
end
graspPosition(1:2) = graspPosition(1:2) + ctx.P.motion.graspBiasXY;
graspPosition(3) = graspPosition(3) + ctx.P.motion.graspBiasZ;
graspPosition(3) = max(graspPosition(3), ctx.P.motion.minGraspZ);
graspPosition(3) = min(graspPosition(3), ctx.P.motion.maxGraspZ);

prePosition = graspPosition + [0; 0; ctx.P.motion.pregraspLift];
liftPosition = graspPosition + [0; 0; ctx.P.motion.postLift];
prePosition(3) = min(prePosition(3), ctx.P.motion.maxPregraspZ);
liftPosition(3) = min(liftPosition(3), ctx.P.motion.maxLiftZ);

Tgrasp = localMakeT(graspPosition + [0; 0; ctx.P.motion.graspClearance], yaw);
Tpre = localMakeT(prePosition, yaw);
Tlift = localMakeT(liftPosition, yaw);

[qPre, okPre] = localSolveIk(ctx, Tpre, ctx.lastArmQ);
[qGrasp, okGrasp] = localSolveIk(ctx, Tgrasp, qPre);
[qLift, okLift] = localSolveIk(ctx, Tlift, qGrasp);

if ~(okPre && okGrasp && okLift)
    [qPre, qGrasp, qLift] = deal(zeros(6, 1));
end
end

function qDrop = localDropTarget(label, color, ctx)
binName = localBinForLabelColor(label, color);
if binName == "blue"
    qDrop = ctx.blueDropQ;
else
    qDrop = ctx.greenDropQ;
end
end

function binName = localBinForLabelColor(label, color)
if label == "bottle" || label == "marker"
    binName = "blue";
elseif label == "cube"
    if color == "blue" || color == "red"
        binName = "blue";
    else
        binName = "green";
    end
elseif label == "spam" || label == "can"
    binName = "green";
else
    binName = "green";
end
end

function effectiveHeight = localEffectiveHeightForGrasp(objHeight, label, P)
switch label
    case "bottle"
        nominalHeight = P.class.nominalBottleHeight;
    case "can"
        nominalHeight = P.class.nominalCanHeight;
    case "marker"
        nominalHeight = P.class.nominalMarkerHeight;
    case "spam"
        nominalHeight = P.class.nominalSpamHeight;
    case "cube"
        nominalHeight = P.class.nominalCubeHeight;
    otherwise
        nominalHeight = 0.08;
end

effectiveHeight = min(max(objHeight, 0.5 * nominalHeight), 1.15 * nominalHeight);
end

function value = localPointValue(label, color)
switch label
    case "can"
        switch color
            case "green"
                value = 10;
            case "yellow"
                value = 20;
            case "red"
                value = 30;
            otherwise
                value = 15;
        end
    case "bottle"
        switch color
            case "blue"
                value = 10;
            case "yellow"
                value = 20;
            case "red"
                value = 30;
            otherwise
                value = 15;
        end
    case "spam"
        value = 20;
    case "marker"
        value = 20;
    case "cube"
        value = 10;
    otherwise
        value = 5;
end
end

function [nextQ, arrived] = localStepArm(currentQ, targetQ, maxStep, tol)
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

function fixedTargets = localBuildPhase1Targets(ctx)
fixedPoseTable = [ ...
    0.20 0.40 0.15  0.0; ...
    0.20 0.60 0.08  0.0; ...
    0.20 0.70 0.07  pi/2; ...
    0.30 0.70 0.08  pi/2; ...
    0.40 0.60 0.15  0.0; ...
    0.60 0.60 0.07  pi/4];
fixedLabels = ["can", "bottle", "marker", "bottle", "can", "spam"];
fixedColors = ["green", "red", "black", "yellow", "green", "white"];
fixedBins = ["green", "blue", "blue", "blue", "green", "green"];

fixedTargets = repmat(struct("enabled", false, "target", localEmptyTarget()), numel(fixedLabels), 1);
seedQ = ctx.homeQ;
for k = 1:numel(fixedLabels)
    target = localBuildFixedTargetFromPose(fixedPoseTable(k, :), fixedLabels(k), fixedColors(k), fixedBins(k), seedQ, ctx);
    fixedTargets(k).enabled = any(target.qGrasp);
    fixedTargets(k).target = target;
    if fixedTargets(k).enabled
        seedQ = target.qDrop;
    else
        fprintf('[FIXED] skipped target %d because IK could not be planned\n', k);
    end
end
end

function targets = localBuildPhase2Targets(queue, ctx)
slotTable = [ ...
    0.80 -0.40 0.15 0.0; ...
    0.85  0.00 0.24 0.0; ...
    0.90 -0.15 0.08 0.0; ...
    0.93  0.05 0.15 0.0];
slotMode = ["vertical", "vertical", "horizontal", "vertical"];

targets = repmat(struct("enabled", false, "target", localEmptyTarget()), 0, 1);
if isempty(queue)
    return;
end

used = false(numel(queue), 1);
seedQ = ctx.homeQ;
for k = 1:size(slotTable, 1)
    slotXY = slotTable(k, 1:2)';
    bestIdx = 0;
    bestDist = inf;
    for i = 1:numel(queue)
        if used(i)
            continue;
        end
        distXY = norm(queue(i).position(1:2) - slotXY);
        if distXY < bestDist
            bestDist = distXY;
            bestIdx = i;
        end
    end
    if bestIdx == 0 || bestDist > ctx.P.phase2.slotMatchRadius
        continue;
    end

    used(bestIdx) = true;
    obs = queue(bestIdx);
    zPick = slotTable(k, 3);
    if slotMode(k) == "vertical"
        if obs.label == "bottle"
            zPick = 0.24;
        elseif obs.label == "can"
            zPick = 0.15;
        end
    end

    poseRow = [slotTable(k, 1), slotTable(k, 2), zPick, slotTable(k, 4)];
    binName = localBinForLabelColor(obs.label, obs.color);
    target = localBuildFixedTargetFromPose(poseRow, obs.label, obs.color, binName, seedQ, ctx);
    if any(target.qGrasp)
        entry.enabled = true;
        entry.target = target;
        targets(end+1, 1) = entry; %#ok<AGROW>
        seedQ = target.qDrop;
    end
end
end

function target = localBuildFixedTargetFromPose(poseRow, label, color, binName, qSeed, ctx)
target = localEmptyTarget();
graspPosition = poseRow(1:3)';
yaw = poseRow(4);

prePosition = graspPosition + [0; 0; ctx.P.motion.pregraspLift];
liftPosition = graspPosition + [0; 0; ctx.P.motion.postLift];
prePosition(3) = min(prePosition(3), ctx.P.motion.maxPregraspZ);
liftPosition(3) = min(liftPosition(3), ctx.P.motion.maxLiftZ);

Tgrasp = localMakeT(graspPosition + [0; 0; ctx.P.motion.graspClearance], yaw);
Tpre = localMakeT(prePosition, yaw);
Tlift = localMakeT(liftPosition, yaw);

[qPre, okPre] = localSolveIk(ctx, Tpre, qSeed);
[qGrasp, okGrasp] = localSolveIk(ctx, Tgrasp, qPre);
[qLift, okLift] = localSolveIk(ctx, Tlift, qGrasp);

if ~(okPre && okGrasp && okLift)
    return;
end

target.position = graspPosition;
target.qPre = qPre;
target.qGrasp = qGrasp;
target.qLift = qLift;
target.qDrop = localDropQForBin(binName, ctx);
target.graspPosition = graspPosition;
target.graspYaw = yaw;
target.score = localPointValue(label, color) + 100;
target.label = label;
target.color = color;
target.height = localEffectiveHeightForGrasp(ctx.P.class.nominalCanHeight, label, ctx.P);
target.source = "fixed";
target.dropBin = binName;
end

function qDrop = localDropQForBin(binName, ctx)
if binName == "blue"
    qDrop = ctx.blueDropQ;
else
    qDrop = ctx.greenDropQ;
end
end

function ctx = localAdvancePhase(ctx, reason)
oldPhase = ctx.phase;
ctx.phaseIndex = ctx.phaseIndex + 1;
ctx.emptyScanCount = 0;
ctx.retryCount = 0;
ctx.failedTargets = zeros(0, 3);
ctx.target = localEmptyTarget();
ctx.scanTargetIndex = 1;
ctx.scanDirection = 1;
ctx.settleCount = 0;

if ctx.phaseIndex > numel(ctx.phaseOrder)
    fprintf('[PHASE] %s complete -> stopping (%s)\n', oldPhase, reason);
    ctx.state = "STOP";
    return;
end

ctx.phase = ctx.phaseOrder(ctx.phaseIndex);
if ctx.phase == "PHASE2_SHAPE"
    ctx.phase2Targets = repmat(struct("enabled", false, "target", localEmptyTarget()), 0, 1);
    ctx.phase2TargetIndex = 1;
end
fprintf('[PHASE] %s complete -> %s (%s)\n', oldPhase, ctx.phase, reason);
ctx.state = "MOVE_SCAN";
end

function ctx = localHandleEmptyPhaseScan(ctx, reason)
ctx.emptyScanCount = ctx.emptyScanCount + 1;
fprintf('[SCAN] %s %s at step %d (empty=%d)\n', lower(char(ctx.phase)), reason, ctx.stepCount, ctx.emptyScanCount);

if ctx.phase ~= "PHASE4_RANDOM_BIN" && ctx.emptyScanCount >= ctx.P.phaseAdvanceEmptyScans
    ctx = localAdvancePhase(ctx, "phase exhausted");
    return;
end
if ctx.phase == "PHASE4_RANDOM_BIN" && ctx.emptyScanCount >= ctx.P.maxEmptyScans
    ctx.state = "STOP";
    return;
end

if ctx.emptyScanCount >= ctx.P.recoveryEmptyScans
    ctx.failedTargets = zeros(0, 3);
    ctx.retryCount = 0;
    ctx.emptyScanCount = 0;
    ctx.scanTargetIndex = 1;
    ctx.scanDirection = 1;
    ctx.settleCount = 0;
    ctx.state = "MOVE_SCAN";
else
    nextIdx = ctx.scanTargetIndex + ctx.scanDirection;
    if nextIdx > size(ctx.scanSweepQs, 2) || nextIdx < 1
        ctx.scanDirection = -ctx.scanDirection;
        nextIdx = ctx.scanTargetIndex + ctx.scanDirection;
    end
    ctx.scanTargetIndex = nextIdx;
    ctx.state = "MOVE_SCAN";
end
end

function localLogPhaseTarget(ctx)
fprintf('[TARGET][%s] %s %s score=%.2f pos=[%.3f %.3f %.3f] h=%.3f grasp=[%.3f %.3f %.3f] yaw=%.2f\n', ...
    ctx.phase, ctx.target.color, ctx.target.label, ctx.target.score, ...
    ctx.target.position(1), ctx.target.position(2), ctx.target.position(3), ctx.target.height, ...
    ctx.target.graspPosition(1), ctx.target.graspPosition(2), ctx.target.graspPosition(3), ctx.target.graspYaw);
end

function tf = localMakeT(position, yaw)
rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
rx = [1 0 0; 0 -1 0; 0 0 -1];
tf = [rz * rx, position(:); 0 0 0 1];
end

function [qArm, ok] = localSolveIk(ctx, targetTform, qSeedArm)
yawSeeds = [0, pi / 2, -pi / 2];
xyOffsets = [0 0; 0.015 0; -0.015 0; 0 0.015; 0 -0.015];
ok = false;
qArm = zeros(6, 1);
baseYaw = atan2(targetTform(2, 1), targetTform(1, 1));

for s = 1:numel(yawSeeds)
    for k = 1:size(xyOffsets, 1)
        trialPos = targetTform(1:3, 4) + [xyOffsets(k, :) 0]';
        trial = localMakeT(trialPos, baseYaw + yawSeeds(s));
        qSeedFull = [qSeedArm(:)' zeros(1, 6)];
        [qSol, info] = ctx.ik("tool0", trial, [0.25 0.25 0.25 1 1 1], qSeedFull);
        if strcmpi(info.Status, "success") || info.PoseErrorNorm < 5e-3
            qArm = qSol(1:6)';
            qArm = qArm(:);
            ok = true;
            return;
        end
    end
end
end

function qGreen = localSolveMirroredDrop(ikSolver, robot, qBlue)
qBlueFull = [qBlue(:)' zeros(1, 6)];
blueTform = getTransform(robot, qBlueFull, "tool0");
greenTform = blueTform;
greenTform(2, 4) = -0.35;
[qSol, info] = ikSolver("tool0", greenTform, [0.25 0.25 0.25 1 1 1], qBlueFull);
if strcmpi(info.Status, "success") || info.PoseErrorNorm < 5e-3
    qGreen = qSol(1:6)';
    qGreen = qGreen(:);
else
    qGreen = qBlue(:);
    qGreen(1) = wrapToPi(qGreen(1) + pi / 2);
end
end

function refinedTarget = localSelectRefinedTarget(queue, currentTarget, ctx)
refinedTarget = [];
if isempty(queue)
    return;
end

bestIdx = 0;
bestScore = -inf;
for i = 1:numel(queue)
    entry = queue(i);
    posDist = norm(entry.position(:) - currentTarget.position(:));
    if posDist > ctx.P.refineRadius
        continue;
    end

    if entry.label ~= currentTarget.label
        continue;
    end
    if currentTarget.color ~= "unknown" && entry.color ~= currentTarget.color
        continue;
    end

    matchScore = -2.5 * posDist;
    zDist = abs(entry.position(3) - currentTarget.position(3));
    matchScore = matchScore - 3.0 * zDist;
    matchScore = matchScore + 1.5;
    matchScore = matchScore + 1.0;
    matchScore = matchScore + 0.05 * entry.score;

    if matchScore > bestScore
        bestScore = matchScore;
        bestIdx = i;
    end
end

if bestIdx > 0
    refinedTarget = queue(bestIdx);
end
end

function tf = localRememberPosition(tf, position)
tf(end+1, :) = position(:)';
end

function tf = localNearMemory(position, memory, radius)
tf = false;
if isempty(memory)
    return;
end
dist = vecnorm(memory - position(:)', 2, 2);
tf = any(dist < radius);
end
