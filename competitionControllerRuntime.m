function [gripperStatus, currentConfig, stop] = competitionControllerRuntime(rgbFrame, depthFrame, cameraTform, targetGrasped)
persistent ctx

if isempty(ctx)
    ctx = competitionController.Context.initContext();
elseif isfield(ctx, "phaseIndex") && ctx.phaseIndex > numel(ctx.phaseOrder)
    fprintf('[RESET] stale controller context detected -> reinitializing controller\n');
    ctx = competitionController.Context.initContext();
elseif ~competitionController.Phase.isPhaseEnabled(ctx.phase)
    ctx = competitionController.Phase.skipDisabledPhase(ctx, "phase disabled");
end

entryState = ctx.state;
entryPhase = ctx.phase;

gripperStatus = 0;
currentConfig = ctx.lastArmQ;
stop = false;

camTF = competitionController.Context.parseCameraTform(cameraTform, ctx.lastCameraTform);
ctx.lastCameraTform = camTF;
ctx.stepCount = ctx.stepCount + 1;

if isempty(rgbFrame) || isempty(depthFrame)
    [currentConfig, arrived] = competitionController.Context.stepArm( ...
        ctx.lastArmQ, ...
        competitionController.Context.currentScanGoal(ctx), ...
        ctx.P.motion.scanStep, ...
        ctx.P.motion.arriveTol);
    ctx.lastArmQ = currentConfig;
    if arrived
        ctx.state = "SCAN";
    end
    ctx = competitionController.RuntimeDebug.emitTransitionDebug( ...
        ctx, entryPhase, entryState, currentConfig, gripperStatus, stop);
    return;
end

switch ctx.state
    case "MOVE_SCAN"
        if ctx.phase == "PHASE3_ORIENTATION" && ~isempty(ctx.phase3Targets)
            currentConfig = ctx.lastArmQ;
            ctx.state = "SCAN";
            ctx.settleCount = 0;
        else
            [ctx.lastArmQ, arrived] = competitionController.Context.stepArm( ...
                ctx.lastArmQ, ...
                competitionController.Context.currentScanGoal(ctx), ...
                ctx.P.motion.scanStep, ...
                ctx.P.motion.arriveTol);
            currentConfig = ctx.lastArmQ;
            if arrived
                ctx.state = "SCAN";
                ctx.settleCount = 0;
            end
        end

     case "SCAN"
         currentConfig = competitionController.Context.currentScanGoal(ctx);
         ctx.lastArmQ = currentConfig;
         ctx.settleCount = ctx.settleCount + 1;
         if ctx.settleCount < ctx.P.scanSettlingSteps
             return;
         end
        [ctx, handled] = competitionController.PhaseRouter.runScanPhase( ...
            ctx, rgbFrame, depthFrame, camTF);
        if handled
            return;
        end

    case "MOVE_PREGRASP"
        travelStep = ctx.P.motion.travelStep;
        if ctx.phase == "PHASE1_FIXED" && ctx.target.source == "phase1-fixed"
            travelStep = min(travelStep, ctx.P.phase1.travelStep);
        end
        if ctx.phase == "PHASE3_ORIENTATION" && ctx.target.source == "phase3-fixed"
            travelStep = min(travelStep, ctx.P.phase3.travelStep);
        end
        [ctx.lastArmQ, arrived] = competitionController.Context.stepArm( ...
            ctx.lastArmQ, ctx.target.qPre, travelStep, ctx.P.motion.arriveTol);
        currentConfig = ctx.lastArmQ;
        if arrived
            if (ctx.phase == "PHASE1_FIXED" && ctx.target.source == "phase1-fixed") || ...
                    (ctx.phase == "PHASE3_ORIENTATION" && ctx.target.source == "phase3-fixed")
                ctx.state = "MOVE_GRASP";
            else
                ctx.state = "REFINE_GRASP";
                ctx.refineCount = 0;
            end
        end

    case "MOVE_RETRY_ROTATE"
        travelStep = ctx.P.motion.travelStep;
        if ctx.phase == "PHASE1_FIXED" && ctx.target.source == "phase1-fixed"
            travelStep = min(travelStep, ctx.P.phase1.travelStep);
        end
        if ctx.phase == "PHASE3_ORIENTATION" && ctx.target.source == "phase3-fixed"
            travelStep = min(travelStep, ctx.P.phase3.travelStep);
        end
        [ctx.lastArmQ, arrived] = competitionController.Context.stepArm( ...
            ctx.lastArmQ, ctx.target.qRotate, travelStep, ctx.P.motion.arriveTol);
        currentConfig = ctx.lastArmQ;
        if arrived
            ctx.state = "MOVE_PREGRASP";
        end

    case "REFINE_GRASP"
        currentConfig = ctx.target.qPre;
        ctx.lastArmQ = currentConfig;
        ctx.refineCount = ctx.refineCount + 1;
        if ctx.refineCount < ctx.P.refineSettlingSteps
            return;
        end

        queue = competitionController.Perception.perceiveScene(rgbFrame, depthFrame, camTF, ctx);
        queue = competitionController.PhaseRouter.filterQueue(queue, ctx);
        refinedTarget = competitionController.Perception.selectRefinedTarget(queue, ctx.target, ctx);
        if ~isempty(refinedTarget)
            ctx.target = refinedTarget;
            if competitionController.RuntimeDebug.isVerbose(ctx.P)
                fprintf('[REFINE] %s %s pos=[%.3f %.3f %.3f] h=%.3f grasp=[%.3f %.3f %.3f] yaw=%.2f\n', ...
                    ctx.target.color, ctx.target.label, ...
                    ctx.target.position(1), ctx.target.position(2), ctx.target.position(3), ctx.target.height, ...
                    ctx.target.graspPosition(1), ctx.target.graspPosition(2), ctx.target.graspPosition(3), ctx.target.graspYaw);
            end
        end
        ctx.state = "MOVE_GRASP";

    case "MOVE_GRASP"
        graspStep = ctx.P.motion.graspStep;
        if ctx.phase == "PHASE1_FIXED" && ctx.target.source == "phase1-fixed"
            graspStep = min(graspStep, ctx.P.phase1.graspStep);
        end
        if ctx.phase == "PHASE3_ORIENTATION" && ctx.target.source == "phase3-fixed"
            graspStep = min(graspStep, ctx.P.phase3.graspStep);
        end
        [ctx.lastArmQ, arrived] = competitionController.Context.stepArm( ...
            ctx.lastArmQ, ctx.target.qGrasp, graspStep, ctx.P.motion.graspTol);
        currentConfig = ctx.lastArmQ;
        if arrived
            ctx.state = "CLOSE";
            ctx.holdCount = 0;
        end

    case "CLOSE"
        currentConfig = ctx.target.qGrasp;
        ctx.lastArmQ = currentConfig;
        gripperStatus = 1;
        ctx.holdCount = ctx.holdCount + 1;
        if ctx.holdCount >= max(ctx.P.closeHoldSteps, ctx.closeSequenceLength + 2)
            ctx.state = "VERIFY";
            ctx.verifyCount = 0;
        end

    case "VERIFY"
        currentConfig = ctx.target.qGrasp;
        ctx.lastArmQ = currentConfig;
        gripperStatus = 1;
        ctx.verifyCount = ctx.verifyCount + 1;
        graspConfirmed = ~isempty(targetGrasped) && logical(targetGrasped(1));
        if graspConfirmed
            ctx.state = "LIFT";
            ctx.holdCount = 0;
            ctx.verifyCount = 0;
        elseif ctx.verifyCount < ctx.P.verifyHoldSteps
            return;
        else
            ctx.retryCount = ctx.retryCount + 1;
            maxRetries = ctx.P.maxTargetRetries;
            if ctx.phase == "PHASE1_FIXED" && ctx.target.source == "phase1-fixed"
                maxRetries = ctx.P.phase1.maxTargetRetries;
            end
            if ctx.phase == "PHASE3_ORIENTATION" && ctx.target.source == "phase3-fixed"
                maxRetries = ctx.P.phase3.maxTargetRetries;
            end
            if ctx.retryCount < maxRetries
                [ctx, recovered] = competitionController.Planning.tryAlternateTargetPlan(ctx);
                if recovered
                    ctx.verifyCount = 0;
                    ctx.refineCount = 0;
                    return;
                end
            end
            fprintf('[GIVEUP] %s after %d attempts in %s\n', ...
                competitionController.RuntimeDebug.targetSummary(ctx.target), ...
                ctx.retryCount, ctx.phase);
            ctx.failedTargets = competitionController.Context.rememberPosition(ctx.failedTargets, ctx.target.position);
            if (ctx.phase == "PHASE1_FIXED" && ctx.target.source == "phase1-fixed") || ...
                    (ctx.phase == "PHASE3_ORIENTATION" && ctx.target.source == "phase3-fixed")
                ctx.state = "SCAN";
            else
                ctx.state = "MOVE_SCAN";
            end
            ctx.target = competitionController.Context.emptyTarget();
            ctx.verifyCount = 0;
        end

    case "LIFT"
        [ctx.lastArmQ, arrived] = competitionController.Context.stepArm( ...
            ctx.lastArmQ, ctx.target.qLift, ctx.P.motion.travelStep, ctx.P.motion.arriveTol);
        currentConfig = ctx.lastArmQ;
        gripperStatus = 1;
        if arrived
            ctx.state = "MOVE_BIN";
        end

    case "MOVE_BIN"
        [ctx.lastArmQ, arrived] = competitionController.Context.stepArm( ...
            ctx.lastArmQ, ctx.target.qDrop, ctx.P.motion.travelStep, ctx.P.motion.arriveTol);
        currentConfig = ctx.lastArmQ;
        gripperStatus = 1;
        if arrived
            ctx.state = "OPEN";
            ctx.holdCount = 0;
        end

    case "OPEN"
        currentConfig = ctx.target.qDrop;
        ctx.lastArmQ = currentConfig;
        ctx.holdCount = ctx.holdCount + 1;
        if ctx.holdCount >= ctx.P.openHoldSteps
            ctx.completedCount = ctx.completedCount + 1;
            ctx.completedTargets = competitionController.Context.rememberPosition(ctx.completedTargets, ctx.target.position);
            ctx.target = competitionController.Context.emptyTarget();
            if ctx.phase == "PHASE1_FIXED" || ctx.phase == "PHASE3_ORIENTATION"
                ctx.state = "SCAN";
            else
                ctx.state = "MOVE_SCAN";
            end
        end

    case "STOP"
        currentConfig = ctx.lastArmQ;
        stop = true;

    otherwise
        ctx.state = "MOVE_SCAN";
end

ctx = competitionController.RuntimeDebug.emitTransitionDebug( ...
    ctx, entryPhase, entryState, currentConfig, gripperStatus, stop);
end
