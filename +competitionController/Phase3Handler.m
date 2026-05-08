classdef Phase3Handler
    methods (Static)
        function ctx = runScanPhase(ctx, rgbFrame, depthFrame, camTF)
            queue = competitionController.Perception.perceiveScene(rgbFrame, depthFrame, camTF, ctx);
            queue = competitionController.Phase3Handler.filterQueue(queue);
            competitionController.RuntimeDebug.logQueueSummary(queue, ctx, "dynamic");
            if isempty(queue)
                ctx = competitionController.Phase.handleEmptyPhaseScan(ctx, "no valid targets");
                return;
            end

            ctx.emptyScanCount = 0;
            ctx.retryCount = 0;
            ctx.target = queue(1);
            competitionController.RuntimeDebug.logPhaseTarget(ctx);
            ctx.state = "MOVE_PREGRASP";
        end

        function zone = targetZone(position)
            pos = position(:);
            bounds = competitionController.Phase3Handler.regionBounds();
            if numel(pos) ~= 3
                zone = "unknown";
            elseif pos(1) >= bounds.orientationX(1) && pos(1) <= bounds.orientationX(2) && ...
                    pos(2) >= bounds.orientationY(1) && pos(2) <= bounds.orientationY(2)
                zone = "orientation";
            elseif pos(1) >= bounds.randomX(1) && pos(1) <= bounds.randomX(2) && ...
                    pos(2) >= bounds.randomY(1) && pos(2) <= bounds.randomY(2)
                zone = "random";
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
                inRandom = pos(1) >= bounds.randomX(1) && ...
                    pos(1) <= bounds.randomX(2) && ...
                    pos(2) >= bounds.randomY(1) && ...
                    pos(2) <= bounds.randomY(2);
                keep(idx) = inOrientation || inRandom;
            end
            queue = queue(keep);
        end
    end

    methods (Static, Access = private)
        function bounds = regionBounds()
            bounds.orientationX = [0.32 0.72];
            bounds.orientationY = [-0.24 0.18];
            bounds.randomX = [-0.02 0.56];
            bounds.randomY = [-0.76 -0.38];
        end
    end
end
