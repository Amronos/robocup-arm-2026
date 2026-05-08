classdef Phase2Handler
    methods (Static)
        function ctx = runScanPhase(ctx, rgbFrame, depthFrame, camTF)
            queue = competitionController.Perception.perceiveScene(rgbFrame, depthFrame, camTF, ctx);
            queue = competitionController.Phase2Handler.filterQueue(queue);
            competitionController.RuntimeDebug.logQueueSummary(queue, ctx, "shape-raw");

            if isempty(ctx.phase2Targets)
                ctx.phase2Targets = competitionController.Phase2Handler.buildTargets(queue, ctx);
                ctx.phase2TargetIndex = 1;
            end

            [ctx, hasTarget] = competitionController.Phase.dispatchQueuedTarget( ...
                ctx.phase2Targets, ctx.phase2TargetIndex, ctx, "phase2TargetIndex");
            if hasTarget
                ctx.emptyScanCount = 0;
                ctx.retryCount = 0;
                competitionController.RuntimeDebug.logPhaseTarget(ctx);
                ctx.state = "MOVE_PREGRASP";
                return;
            end

            ctx = competitionController.Phase.handleEmptyPhaseScan(ctx, "no usable phase2 targets");
        end

        function zone = targetZone(position)
            pos = position(:);
            bounds = competitionController.Phase2Handler.regionBounds();
            if numel(pos) ~= 3
                zone = "unknown";
            elseif pos(1) >= bounds.shapeX(1) && pos(1) <= bounds.shapeX(2) && ...
                    pos(2) >= bounds.shapeY(1) && pos(2) <= bounds.shapeY(2)
                zone = "shape";
            elseif pos(1) > 0 && pos(2) > 0
                zone = "fixed";
            else
                zone = "other";
            end
        end

        function queue = filterQueue(queue)
            if isempty(queue)
                return;
            end

            keep = false(numel(queue), 1);
            for idx = 1:numel(queue)
                zone = competitionController.Phase2Handler.targetZone(queue(idx).position);
                keep(idx) = any(queue(idx).label == ["can", "bottle"]) && ...
                    any(zone == ["fixed", "shape"]);
            end
            queue = queue(keep);
        end
    end

    methods (Static, Access = private)
        function targets = buildTargets(queue, ctx)
            slotTable = [ ...
                0.22  0.36 0.15  pi / 2; ...
                0.02  0.32 0.08  pi / 2; ...
               -0.15  0.41 0.08  pi; ...
               -0.23  0.17 0.08 -pi / 2];
            slotMode = ["vertical", "horizontal", "vertical", "vertical"];

            targets = repmat(struct("enabled", false, "target", competitionController.Context.emptyTarget()), 0, 1);
            if isempty(queue)
                return;
            end

            [~, order] = sort(arrayfun(@(q) q.position(1), queue), "descend");
            shapeQueue = queue(order);
            seedQ = ctx.homeQ;
            numSlots = min(size(slotTable, 1), numel(shapeQueue));
            for k = 1:numSlots
                obs = shapeQueue(k);
                zPick = slotTable(k, 3);
                if slotMode(k) == "vertical"
                    if obs.label == "bottle"
                        zPick = 0.24;
                    elseif obs.label == "can"
                        zPick = 0.15;
                    end
                end

                poseRow = [slotTable(k, 1), slotTable(k, 2), zPick, slotTable(k, 4)];
                binName = competitionController.Planning.binForLabelColor(obs.label, obs.color);
                target = competitionController.Planning.buildFixedTargetFromPose( ...
                    poseRow, obs.label, obs.color, binName, seedQ, ctx);
                if any(target.qGrasp)
                    target.source = "phase2";
                    target.score = target.score + 50 - k;
                    entry = struct("enabled", true, "target", target);
                    targets(end + 1, 1) = entry; %#ok<AGROW>
                    seedQ = target.qDrop;
                end
            end
        end

        function bounds = regionBounds()
            bounds.shapeX = [0.74 0.96];
            bounds.shapeY = [-0.45 0.12];
        end
    end
end
