classdef Planning
    methods (Static)
        function [qPre, qGrasp, qLift, graspPosition, yaw] = planTarget(position, objHeight, orientationDeg, label, ctx)
            if label == "marker"
                yaw = deg2rad(-orientationDeg);
                yaw = (pi / 2) * round(yaw / (pi / 2));
            elseif any(label == ["bottle", "can", "spam"]) && objHeight < ctx.P.class.uprightCanHeight
                yaw = deg2rad(90 - orientationDeg);
            else
                yaw = 0;
            end

            graspPosition = position(:);
            effectiveHeight = competitionController.Planning.effectiveHeightForGrasp(objHeight, label, ctx.P);
            switch label
                case {"bottle", "can"}
                    graspPosition(3) = graspPosition(3) - 0.55 * effectiveHeight;
                case "marker"
                    graspPosition(3) = graspPosition(3) - 0.50 * effectiveHeight;
                case "spam"
                    graspPosition(3) = graspPosition(3) - 0.35 * effectiveHeight;
                case "cube"
                    graspPosition(3) = graspPosition(3) - 0.25 * effectiveHeight;
            end

            graspPosition(1:2) = graspPosition(1:2) + ctx.P.motion.graspBiasXY;
            graspPosition(3) = graspPosition(3) + ctx.P.motion.graspBiasZ;
            graspPosition(3) = max(graspPosition(3), ctx.P.motion.minGraspZ);
            graspPosition(3) = min(graspPosition(3), ctx.P.motion.maxGraspZ);
            [qPre, qGrasp, qLift, ok] = competitionController.Planning.planPoseAtYaw( ...
                ctx, graspPosition, yaw, ctx.lastArmQ);
            if ~ok
                [qPre, qGrasp, qLift] = deal(zeros(6, 1));
            end
        end

        function target = buildFixedTargetFromPose(poseRow, label, color, binName, qSeed, ctx)
            target = competitionController.Context.emptyTarget();
            graspPosition = poseRow(1:3)';
            yaw = poseRow(4);
            [qPre, qGrasp, qLift, ok] = competitionController.Planning.planPoseAtYawExact( ...
                ctx, graspPosition, yaw, qSeed);
            if ~ok
                return;
            end

            target.position = graspPosition;
            target.qPre = qPre;
            target.qGrasp = qGrasp;
            target.qLift = qLift;
            target.qRotate = qLift;
            target.qDrop = competitionController.Planning.dropQForBin(binName, ctx);
            target.graspPosition = graspPosition;
            target.graspYaw = yaw;
            target.score = competitionController.Planning.pointValue(label, color) + 100;
            target.label = label;
            target.color = color;
            target.height = competitionController.Planning.nominalHeightForLabel(label, ctx.P);
            target.source = "fixed";
            target.dropBin = binName;
            target.basePosition = graspPosition;
            target.baseYaw = yaw;
            target.variantIndex = 1;
        end

        function target = buildPhase3FixedTargetFromPose(poseRow, label, color, binName, ctx)
            target = competitionController.Context.emptyTarget();
            graspPosition = poseRow(1:3)';
            yaw = poseRow(4);
            [qPre, qGrasp, qLift, ok] = competitionController.Planning.planPhase3FixedPose( ...
                ctx, graspPosition, yaw);
            if ~ok
                return;
            end

            target.position = graspPosition;
            target.qPre = qPre;
            target.qGrasp = qGrasp;
            target.qLift = qLift;
            target.qRotate = qLift;
            target.qDrop = competitionController.Planning.dropQForBin(binName, ctx);
            target.graspPosition = graspPosition;
            target.graspYaw = yaw;
            target.score = competitionController.Planning.pointValue(label, color) + 100;
            target.label = label;
            target.color = color;
            target.height = competitionController.Planning.nominalHeightForLabel(label, ctx.P);
            target.source = "fixed";
            target.dropBin = binName;
            target.basePosition = graspPosition;
            target.baseYaw = yaw;
            target.variantIndex = 1;
        end

        function [qPre, qGrasp, qLift, ok] = planPhase3FixedPose(ctx, graspPosition, yaw)
            qPre = zeros(6, 1);
            qGrasp = zeros(6, 1);
            qLift = zeros(6, 1);
            ok = false;
            bestCost = inf;

            seedSet = [ctx.scanSweepQs, ctx.homeQ];
            for idx = 1:size(seedSet, 2)
                qSeed = seedSet(:, idx);
                [trialPre, trialGrasp, trialLift, trialOk] = competitionController.Planning.planPoseAtYawConservative( ...
                    ctx, graspPosition, yaw, qSeed, ...
                    ctx.P.phase3.pregraspLift, ctx.P.phase3.postLift, ...
                    ctx.P.phase3.maxSeedToPreDelta, ...
                    ctx.P.phase3.maxPreToGraspDelta, ...
                    ctx.P.phase3.maxGraspToLiftDelta);
                if ~trialOk
                    continue;
                end

                cost = competitionController.Planning.jointDeltaNorm(qSeed, trialPre) + ...
                    competitionController.Planning.jointDeltaNorm(trialPre, trialGrasp) + ...
                    competitionController.Planning.jointDeltaNorm(trialGrasp, trialLift);
                if cost < bestCost
                    bestCost = cost;
                    qPre = trialPre;
                    qGrasp = trialGrasp;
                    qLift = trialLift;
                    ok = true;
                end
            end
        end

        function qDrop = dropQForBin(binName, ctx)
            if binName == "blue"
                qDrop = ctx.blueDropQ;
            else
                qDrop = ctx.greenDropQ;
            end
        end

        function binName = binForLabelColor(label, color)
            if label == "bottle" || label == "marker"
                binName = "blue";
            elseif label == "cube"
                if color == "blue" || color == "red"
                    binName = "blue";
                else
                    binName = "green";
                end
            else
                binName = "green";
            end
        end

        function effectiveHeight = effectiveHeightForGrasp(objHeight, label, P)
            nominalHeight = competitionController.Planning.nominalHeightForLabel(label, P);
            effectiveHeight = min(max(objHeight, 0.5 * nominalHeight), 1.15 * nominalHeight);
        end

        function nominalHeight = nominalHeightForLabel(label, P)
            switch label
                case "bottle"
                    nominalHeight = P.class.nominalBottleHeight;
                case "can"
                    nominalHeight = P.class.nominalCanHeight;
                case "marker"
                    nominalHeight = P.class.nominalMarkerHeight;
                case "spam"
                    nominalHeight = P.class.nominalSpamHeight;
                case "cube"
                    nominalHeight = P.class.nominalCubeHeight;
                otherwise
                    nominalHeight = 0.08;
            end
        end

        function value = pointValue(label, color)
            switch label
                case "can"
                    switch color
                        case "green"
                            value = 10;
                        case "yellow"
                            value = 20;
                        case "red"
                            value = 30;
                        otherwise
                            value = 15;
                    end
                case "bottle"
                    switch color
                        case "blue"
                            value = 10;
                        case "yellow"
                            value = 20;
                        case "red"
                            value = 30;
                        otherwise
                            value = 15;
                    end
                case "spam"
                    value = 20;
                case "marker"
                    value = 20;
                case "cube"
                    value = 10;
                otherwise
                    value = 5;
            end
        end

        function [ctx, recovered] = tryAlternateTargetPlan(ctx)
            recovered = false;
            basePos = ctx.target.basePosition(:);

            for variant = (ctx.target.variantIndex + 1):numel(ctx.P.retry.yawOffsets)
                yaw = wrapToPi(ctx.target.baseYaw + ctx.P.retry.yawOffsets(variant));
                graspPosition = basePos;
                yawDelta = wrapToPi(yaw - ctx.target.graspYaw);
                qPre = ctx.target.qPre;
                qGrasp = ctx.target.qGrasp;
                qLift = ctx.target.qLift;
                qPre(6) = wrapToPi(qPre(6) + yawDelta);
                qGrasp(6) = wrapToPi(qGrasp(6) + yawDelta);
                qLift(6) = wrapToPi(qLift(6) + yawDelta);

                ctx.target.qPre = qPre;
                ctx.target.qGrasp = qGrasp;
                ctx.target.qLift = qLift;
                ctx.target.qRotate = qLift;
                ctx.target.graspPosition = graspPosition;
                ctx.target.graspYaw = yaw;
                ctx.target.variantIndex = variant;
                ctx.state = "MOVE_RETRY_ROTATE";
                fprintf('[RECOVER] %s variant=%d yaw=%.2f wrist=%.2f via-lift\n', ...
                    competitionController.RuntimeDebug.targetSummary(ctx.target), ...
                    variant, yaw, qGrasp(6));
                recovered = true;
                return;
            end
        end

        function [qPre, qGrasp, qLift, ok] = planPoseAtYaw(ctx, graspPosition, yaw, qSeed)
            prePosition = graspPosition + [0; 0; ctx.P.motion.pregraspLift];
            liftPosition = graspPosition + [0; 0; ctx.P.motion.postLift];
            prePosition(3) = min(prePosition(3), ctx.P.motion.maxPregraspZ);
            liftPosition(3) = min(liftPosition(3), ctx.P.motion.maxLiftZ);

            Tgrasp = competitionController.Planning.makeT(graspPosition + [0; 0; ctx.P.motion.graspClearance], yaw);
            Tpre = competitionController.Planning.makeT(prePosition, yaw);
            Tlift = competitionController.Planning.makeT(liftPosition, yaw);

            [qPre, okPre] = competitionController.Planning.solveIk(ctx, Tpre, qSeed);
            [qGrasp, okGrasp] = competitionController.Planning.solveIk(ctx, Tgrasp, qPre);
            [qLift, okLift] = competitionController.Planning.solveIk(ctx, Tlift, qGrasp);
            ok = okPre && okGrasp && okLift;
            if ~ok
                [qPre, qGrasp, qLift] = deal(zeros(6, 1));
            end
        end

        function [qPre, qGrasp, qLift, ok] = planPoseAtYawExact(ctx, graspPosition, yaw, qSeed)
            prePosition = graspPosition + [0; 0; ctx.P.motion.pregraspLift];
            liftPosition = graspPosition + [0; 0; ctx.P.motion.postLift];
            prePosition(3) = min(prePosition(3), ctx.P.motion.maxPregraspZ);
            liftPosition(3) = min(liftPosition(3), ctx.P.motion.maxLiftZ);

            Tgrasp = competitionController.Planning.makeT(graspPosition + [0; 0; ctx.P.motion.graspClearance], yaw);
            Tpre = competitionController.Planning.makeT(prePosition, yaw);
            Tlift = competitionController.Planning.makeT(liftPosition, yaw);

            [qPre, okPre] = competitionController.Planning.solveIkExact(ctx, Tpre, qSeed);
            [qGrasp, okGrasp] = competitionController.Planning.solveIkExact(ctx, Tgrasp, qPre);
            [qLift, okLift] = competitionController.Planning.solveIkExact(ctx, Tlift, qGrasp);
            ok = okPre && okGrasp && okLift;
            if ~ok
                [qPre, qGrasp, qLift] = deal(zeros(6, 1));
            end
        end

        function [qPre, qGrasp, qLift, ok] = planPoseAtYawConservative(ctx, graspPosition, yaw, qSeed, pregraspLift, postLift, maxSeedToPreDelta, maxPreToGraspDelta, maxGraspToLiftDelta)
            prePosition = graspPosition + [0; 0; pregraspLift];
            liftPosition = graspPosition + [0; 0; postLift];
            prePosition(3) = min(prePosition(3), ctx.P.motion.maxPregraspZ);
            liftPosition(3) = min(liftPosition(3), ctx.P.motion.maxLiftZ);

            Tgrasp = competitionController.Planning.makeT(graspPosition + [0; 0; ctx.P.motion.graspClearance], yaw);
            Tpre = competitionController.Planning.makeT(prePosition, yaw);
            Tlift = competitionController.Planning.makeT(liftPosition, yaw);

            [qPre, okPre] = competitionController.Planning.solveIkExact(ctx, Tpre, qSeed);
            [qGrasp, okGrasp] = competitionController.Planning.solveIkExact(ctx, Tgrasp, qPre);
            [qLift, okLift] = competitionController.Planning.solveIkExact(ctx, Tlift, qGrasp);
            ok = okPre && okGrasp && okLift;
            if ~ok
                [qPre, qGrasp, qLift] = deal(zeros(6, 1));
                return;
            end

            seedToPre = competitionController.Planning.jointDeltaNorm(qSeed, qPre);
            preToGrasp = competitionController.Planning.jointDeltaNorm(qPre, qGrasp);
            graspToLift = competitionController.Planning.jointDeltaNorm(qGrasp, qLift);
            if seedToPre > maxSeedToPreDelta || preToGrasp > maxPreToGraspDelta || graspToLift > maxGraspToLiftDelta
                [qPre, qGrasp, qLift] = deal(zeros(6, 1));
                ok = false;
            end
        end

        function tf = makeT(position, yaw)
            rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
            rx = [1 0 0; 0 -1 0; 0 0 -1];
            tf = [rz * rx, position(:); 0 0 0 1];
        end

        function [qArm, ok] = solveIk(ctx, targetTform, qSeedArm)
            yawSeeds = [0, pi / 2, -pi / 2];
            xyOffsets = [0 0; 0.015 0; -0.015 0; 0 0.015; 0 -0.015];
            ok = false;
            qArm = zeros(6, 1);
            baseYaw = atan2(targetTform(2, 1), targetTform(1, 1));

            for s = 1:numel(yawSeeds)
                for k = 1:size(xyOffsets, 1)
                    trialPos = targetTform(1:3, 4) + [xyOffsets(k, :) 0]';
                    trial = competitionController.Planning.makeT(trialPos, baseYaw + yawSeeds(s));
                    qSeedFull = [qSeedArm(:)' zeros(1, 6)];
                    [qSol, info] = ctx.ik("tool0", trial, [0.25 0.25 0.25 1 1 1], qSeedFull);
                    if strcmpi(info.Status, "success") || info.PoseErrorNorm < 5e-3
                        qArm = qSol(1:6)';
                        qArm = qArm(:);
                        ok = true;
                        return;
                    end
                end
            end
        end

        function [qArm, ok] = solveIkExact(ctx, targetTform, qSeedArm)
            ok = false;
            qArm = zeros(6, 1);
            qSeedFull = [qSeedArm(:)' zeros(1, 6)];
            [qSol, info] = ctx.ik("tool0", targetTform, [0.25 0.25 0.25 1 1 1], qSeedFull);
            if strcmpi(info.Status, "success") || info.PoseErrorNorm < 5e-3
                qArm = qSol(1:6)';
                qArm = qArm(:);
                ok = true;
            end
        end

        function d = jointDeltaNorm(q1, q2)
            delta = wrapToPi(q2(:) - q1(:));
            d = norm(delta);
        end

        function qGreen = solveMirroredDrop(ikSolver, robot, qBlue)
            qBlueFull = [qBlue(:)' zeros(1, 6)];
            blueTform = getTransform(robot, qBlueFull, "tool0");
            greenTform = blueTform;
            greenTform(2, 4) = -blueTform(2, 4);
            [qSol, info] = ikSolver("tool0", greenTform, [0.25 0.25 0.25 1 1 1], qBlueFull);
            if strcmpi(info.Status, "success") || info.PoseErrorNorm < 5e-3
                qGreen = qSol(1:6)';
                qGreen = qGreen(:);
            else
                qGreen = qBlue(:);
                qGreen(1) = wrapToPi(qGreen(1) + pi / 2);
            end
        end
    end
end
