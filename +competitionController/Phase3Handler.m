classdef Phase3Handler
    methods (Static)
        function ctx = runScanPhase(ctx, rgbFrame, depthFrame, camTF) %#ok<INUSD>
            if isempty(ctx.phase3Targets)
                ctx.phase3Targets = competitionController.Phase3Handler.buildTargets(ctx);
                ctx.phase3TargetIndex = 1;
            end

            [ctx, hasTarget] = competitionController.Phase.dispatchQueuedTarget( ...
                ctx.phase3Targets, ctx.phase3TargetIndex, ctx, "phase3TargetIndex");
            if hasTarget
                ctx.emptyScanCount = 0;
                ctx.retryCount = 0;
                competitionController.RuntimeDebug.logPhaseTarget(ctx);
                ctx.state = "MOVE_PREGRASP";
                return;
            end

            ctx = competitionController.Phase.advancePhase(ctx, "phase3 queue drained");
        end

        function fixedTargets = buildTargets(ctx)
            fixedPoseTable = [ ...
                0.77  0.12 0.070 pi; ...
                0.87  0.18 0.032 pi; ...
                0.87  0.00 0.055 pi; ...
                0.89 -0.13 0.130 pi];
            fixedLabels = ["spam", "cube", "bottle", "bottle"];
            fixedColors = ["white", "red", "yellow", "blue"];

            fixedTargets = repmat(struct("enabled", false, "target", competitionController.Context.emptyTarget()), numel(fixedLabels), 1);
            for k = 1:numel(fixedLabels)
                binName = competitionController.Planning.binForLabelColor(fixedLabels(k), fixedColors(k));
                target = competitionController.Planning.buildPhase3FixedTargetFromPose( ...
                    fixedPoseTable(k, :), fixedLabels(k), fixedColors(k), binName, ctx);
                fixedTargets(k).enabled = any(target.qGrasp);
                fixedTargets(k).target = target;
                if fixedTargets(k).enabled
                    fixedTargets(k).target.source = "phase3-fixed";
                    fixedTargets(k).target.score = fixedTargets(k).target.score + 75 - k;
                else
                    fprintf('[PHASE3] skipped fixed slot %d because IK could not be planned\n', k);
                end
            end
        end

        function zone = targetZone(position)
            pos = position(:);
            bounds = competitionController.Phase3Handler.regionBounds();
            if numel(pos) ~= 3
                zone = "unknown";
            elseif pos(1) >= bounds.orientationX(1) && pos(1) <= bounds.orientationX(2) && ...
                    pos(2) >= bounds.orientationY(1) && pos(2) <= bounds.orientationY(2)
                zone = "orientation";
            else
                zone = "other";
            end
        end

        function queue = filterQueue(queue)
            if isempty(queue)
                return;
            end

            bounds = competitionController.Phase3Handler.regionBounds();
            keep = false(numel(queue), 1);
            for idx = 1:numel(queue)
                pos = queue(idx).position;
                inOrientation = pos(1) >= bounds.orientationX(1) && ...
                    pos(1) <= bounds.orientationX(2) && ...
                    pos(2) >= bounds.orientationY(1) && ...
                    pos(2) <= bounds.orientationY(2);
                keep(idx) = inOrientation;
            end
            queue = queue(keep);
        end
    end

    methods (Static, Access = private)
        function bounds = regionBounds()
            bounds.orientationX = [0.74 0.98];
            bounds.orientationY = [-0.20 0.22];
        end
    end
end
