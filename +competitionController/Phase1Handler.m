classdef Phase1Handler
    methods (Static)
        function fixedTargets = buildTargets(ctx)
            fixedPoseTable = [ ...
                0.20 0.39 0.140  pi / 2; ...
                0.20 0.60 0.08  pi / 2; ...
                0.15 0.5 0.01 0.0; ...
                0.3 0.4 0.01  pi / 2; ...
                0.402 0.68 0.03  pi / 2; ...
                0.62 0.62 0.04  pi / 5];
            fixedLabels = ["can", "bottle", "marker", "bottle", "can", "spam"];
            fixedColors = ["green", "red", "black", "yellow", "green", "white"];
            fixedBins = ["green", "blue", "blue", "blue", "green", "green"];

            fixedTargets = repmat(struct("enabled", false, "target", competitionController.Context.emptyTarget()), numel(fixedLabels), 1);
            for k = 1:numel(fixedLabels)
                target = competitionController.Planning.buildPhase1FixedTargetFromPose( ...
                    fixedPoseTable(k, :), fixedLabels(k), fixedColors(k), fixedBins(k), ctx);
                fixedTargets(k).enabled = any(target.qGrasp);
                fixedTargets(k).target = target;
                if fixedTargets(k).enabled
                    fixedTargets(k).target.source = "phase1-fixed";
                    fixedTargets(k).target.score = fixedTargets(k).target.score + 60 - k;
                else
                    fprintf('[FIXED] skipped target %d because IK could not be planned\n', k);
                end
            end
        end

        function ctx = runScanPhase(ctx)
            [ctx, hasTarget] = competitionController.Phase.dispatchQueuedTarget( ...
                ctx.phase1Targets, ctx.phase1TargetIndex, ctx, "phase1TargetIndex");
            if hasTarget
                competitionController.RuntimeDebug.logPhaseTarget(ctx);
                ctx.state = "MOVE_PREGRASP";
                return;
            end

            ctx = competitionController.Phase.advancePhase(ctx, "phase1 queue drained");
        end

        function zone = targetZone(position)
            pos = position(:);
            if numel(pos) ~= 3
                zone = "unknown";
            elseif pos(1) > 0 && pos(2) > 0
                zone = "fixed";
            else
                zone = "other";
            end
        end

        function queue = filterQueue(queue)
        end
    end
end
