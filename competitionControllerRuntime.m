function [gripperStatus, currentConfig, stop] = competitionControllerRuntime(rgbFrame, depthFrame, cameraTform, targetGrasped)
persistent ctx

if isempty(ctx)
    ctx = competitionController.Context.initContext();
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
        [ctx.lastArmQ, arrived] = competitionController.Context.stepArm( ...
            ctx.lastArmQ, ctx.target.qPre, ctx.P.motion.travelStep, ctx.P.motion.arriveTol);
        currentConfig = ctx.lastArmQ;
        if arrived
            ctx.state = "REFINE_GRASP";
            ctx.refineCount = 0;
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
            fprintf('[REFINE] %s %s pos=[%.3f %.3f %.3f] h=%.3f grasp=[%.3f %.3f %.3f] yaw=%.2f\n', ...
                ctx.target.color, ctx.target.label, ...
                ctx.target.position(1), ctx.target.position(2), ctx.target.position(3), ctx.target.height, ...
                ctx.target.graspPosition(1), ctx.target.graspPosition(2), ctx.target.graspPosition(3), ctx.target.graspYaw);
        end
        ctx.state = "MOVE_GRASP";

    case "MOVE_GRASP"
        [ctx.lastArmQ, arrived] = competitionController.Context.stepArm( ...
            ctx.lastArmQ, ctx.target.qGrasp, ctx.P.motion.graspStep, ctx.P.motion.graspTol);
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
            if ctx.retryCount < ctx.P.maxTargetRetries
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
            ctx.state = "MOVE_SCAN";
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
            ctx.state = "MOVE_SCAN";
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
