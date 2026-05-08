classdef PhaseRouter
    methods (Static)
        function [ctx, handled] = runScanPhase(ctx, rgbFrame, depthFrame, camTF)
            handled = true;

            switch ctx.phase
                case "PHASE1_FIXED"
                    ctx = competitionController.Phase1Handler.runScanPhase(ctx);

                case "PHASE2_SHAPE"
                    ctx = competitionController.Phase2Handler.runScanPhase(ctx, rgbFrame, depthFrame, camTF);

                case "PHASE3_ORIENTATION"
                    ctx = competitionController.Phase3Handler.runScanPhase(ctx, rgbFrame, depthFrame, camTF);
                case "PHASE4_RANDOM"
                    ctx = competitionController.Phase4Handler.runScanPhase(ctx, rgbFrame, depthFrame, camTF);

                otherwise
                    handled = false;
            end
        end

        function zone = targetZone(position, ctx)
            switch ctx.phase
                case "PHASE1_FIXED"
                    zone = competitionController.Phase1Handler.targetZone(position);
                case "PHASE2_SHAPE"
                    zone = competitionController.Phase2Handler.targetZone(position);
                case "PHASE3_ORIENTATION"
                    zone = competitionController.Phase3Handler.targetZone(position);
                case "PHASE4_RANDOM"
                    zone = competitionController.Phase4Handler.targetZone(position);
                otherwise
                    zone = "unknown";
            end
        end

        function queue = filterQueue(queue, ctx)
            switch ctx.phase
                case "PHASE1_FIXED"
                    queue = competitionController.Phase1Handler.filterQueue(queue);
                case "PHASE2_SHAPE"
                    queue = competitionController.Phase2Handler.filterQueue(queue);
                case "PHASE3_ORIENTATION"
                    queue = competitionController.Phase3Handler.filterQueue(queue);
                case "PHASE4_RANDOM"
                    queue = competitionController.Phase4Handler.filterQueue(queue);
            end
        end
    end
end
