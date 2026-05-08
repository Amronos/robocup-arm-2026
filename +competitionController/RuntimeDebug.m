classdef RuntimeDebug
    methods (Static)
        function logPhaseTarget(ctx)
            fprintf('[TARGET][%s][%s] %s %s score=%.2f pos=[%.3f %.3f %.3f] h=%.3f grasp=[%.3f %.3f %.3f] yaw=%.2f drop=%s source=%s\n', ...
                ctx.phase, competitionController.PhaseRouter.targetZone(ctx.target.position, ctx), ...
                ctx.target.color, ctx.target.label, ctx.target.score, ...
                ctx.target.position(1), ctx.target.position(2), ctx.target.position(3), ctx.target.height, ...
                ctx.target.graspPosition(1), ctx.target.graspPosition(2), ctx.target.graspPosition(3), ctx.target.graspYaw, ...
                ctx.target.dropBin, ctx.target.source);
        end

        function ctx = emitTransitionDebug(ctx, entryPhase, entryState, currentConfig, gripperStatus, stop)
            if ctx.phase ~= entryPhase
                fprintf('[DEBUG] phase=%s step=%d completed=%d empty=%d retries=%d\n', ...
                    ctx.phase, ctx.stepCount, ctx.completedCount, ctx.emptyScanCount, ctx.retryCount);
            end

            if ctx.state ~= entryState
                q = currentConfig(:);
                fprintf('[STATE] %s -> %s @ step=%d phase=%s grip=%d stop=%d q=[%.2f %.2f %.2f %.2f %.2f %.2f] target=%s\n', ...
                    entryState, ctx.state, ctx.stepCount, ctx.phase, gripperStatus, stop, ...
                    q(1), q(2), q(3), q(4), q(5), q(6), competitionController.RuntimeDebug.targetSummary(ctx.target));
            end

            if mod(ctx.stepCount, 100) == 0
                fprintf('[HEARTBEAT] step=%d phase=%s state=%s scanIdx=%d completed=%d target=%s\n', ...
                    ctx.stepCount, ctx.phase, ctx.state, ctx.scanTargetIndex, ...
                    ctx.completedCount, competitionController.RuntimeDebug.targetSummary(ctx.target));
            end
        end

        function logQueueSummary(queue, ctx, tag)
            if isempty(queue)
                fprintf('[QUEUE][%s][%s] count=0\n', ctx.phase, tag);
                return;
            end

            n = min(numel(queue), 3);
            parts = strings(1, n);
            for k = 1:n
                parts(k) = sprintf('%s/%s:%.1f@%s', ...
                    queue(k).color, queue(k).label, queue(k).score, ...
                    competitionController.PhaseRouter.targetZone(queue(k).position, ctx));
            end

            fprintf('[QUEUE][%s][%s] count=%d top=%s\n', ...
                ctx.phase, tag, numel(queue), strjoin(parts, ' | '));
        end

        function summary = targetSummary(target)
            if isempty(target) || (target.label == "unknown" && target.color == "unknown")
                summary = "none";
                return;
            end

            summary = sprintf('%s/%s/%s', target.color, target.label, target.source);
        end
    end
end
