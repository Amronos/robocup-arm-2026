classdef Phase4Handler
    methods (Static)
        function ctx = runScanPhase(ctx, rgbFrame, depthFrame, camTF)
            queue = competitionController.Perception.perceiveScene(rgbFrame, depthFrame, camTF, ctx);
            queue = competitionController.Phase4Handler.filterQueue(queue);
            competitionController.RuntimeDebug.logQueueSummary(queue, ctx, "random");
            if isempty(queue)
                ctx = competitionController.Phase.handleEmptyPhaseScan(ctx, "no random-bin targets");
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
            bounds = competitionController.Phase4Handler.regionBounds();
            if numel(pos) ~= 3
                zone = "unknown";
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

            bounds = competitionController.Phase4Handler.regionBounds();
            keep = false(numel(queue), 1);
            for idx = 1:numel(queue)
                pos = queue(idx).position;
                keep(idx) = pos(1) >= bounds.randomX(1) && ...
                    pos(1) <= bounds.randomX(2) && ...
                    pos(2) >= bounds.randomY(1) && ...
                    pos(2) <= bounds.randomY(2);
            end
            queue = queue(keep);
        end
    end

    methods (Static, Access = private)
        function bounds = regionBounds()
            bounds.randomX = [-0.02 0.56];
            bounds.randomY = [-0.76 -0.38];
        end
    end
end
