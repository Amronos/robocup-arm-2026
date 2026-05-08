classdef Phase
    methods (Static)
        function phaseOrder = enabledPhaseOrder()
            phaseOrder = ["PHASE1_FIXED", "PHASE3_ORIENTATION"];
        end

        function tf = isPhaseEnabled(phase)
            tf = any(strcmp(competitionController.Phase.enabledPhaseOrder(), phase));
        end

        function [ctx, hasTarget] = dispatchQueuedTarget(targets, targetIndex, ctx, indexField)
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

        function ctx = advancePhase(ctx, reason)
            oldPhase = ctx.phase;
            ctx.phaseIndex = ctx.phaseIndex + 1;
            ctx.emptyScanCount = 0;
            ctx.retryCount = 0;
            ctx.failedTargets = zeros(0, 3);
            ctx.target = competitionController.Context.emptyTarget();
            ctx.scanTargetIndex = 1;
            ctx.scanDirection = 1;
            ctx.settleCount = 0;

            [ctx, foundPhase] = competitionController.Phase.selectActivePhase(ctx);
            if ~foundPhase
                fprintf('[PHASE] %s complete -> stopping (%s)\n', oldPhase, reason);
                ctx.state = "STOP";
                return;
            end

            fprintf('[PHASE] %s complete -> %s (%s)\n', oldPhase, ctx.phase, reason);
            ctx.state = "MOVE_SCAN";
        end

        function ctx = skipDisabledPhase(ctx, reason)
            if competitionController.Phase.isPhaseEnabled(ctx.phase)
                return;
            end

            oldPhase = ctx.phase;
            ctx.phaseIndex = ctx.phaseIndex + 1;
            [ctx, foundPhase] = competitionController.Phase.selectActivePhase(ctx);
            if ~foundPhase
                fprintf('[PHASE] skipping disabled %s -> stopping (%s)\n', oldPhase, reason);
                ctx.state = "STOP";
                return;
            end

            fprintf('[PHASE] skipping disabled %s -> %s (%s)\n', oldPhase, ctx.phase, reason);
            ctx.state = "MOVE_SCAN";
        end

        function ctx = handleEmptyPhaseScan(ctx, reason)
            ctx.emptyScanCount = ctx.emptyScanCount + 1;
            if competitionController.RuntimeDebug.isVerbose(ctx.P)
                fprintf('[SCAN] %s %s at step %d (empty=%d)\n', ...
                    lower(char(ctx.phase)), reason, ctx.stepCount, ctx.emptyScanCount);
            end

            if ctx.phase ~= "PHASE4_RANDOM" && ctx.emptyScanCount >= ctx.P.phaseAdvanceEmptyScans
                ctx = competitionController.Phase.advancePhase(ctx, "phase exhausted");
                return;
            end
            if ctx.phase == "PHASE4_RANDOM" && ctx.emptyScanCount >= ctx.P.maxEmptyScans
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

        function [ctx, foundPhase] = selectActivePhase(ctx)
            foundPhase = false;
            while ctx.phaseIndex <= numel(ctx.phaseOrder)
                candidate = ctx.phaseOrder(ctx.phaseIndex);
                if competitionController.Phase.isPhaseEnabled(candidate)
                    ctx.phase = candidate;
                    if ctx.phase == "PHASE3_ORIENTATION"
                        ctx.phase3Targets = competitionController.Phase3Handler.buildTargets(ctx);
                        ctx.phase3TargetIndex = 1;
                    end
                    foundPhase = true;
                    return;
                end

                ctx.phaseIndex = ctx.phaseIndex + 1;
            end
        end
    end
end
