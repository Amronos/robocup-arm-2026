classdef Perception
    methods (Static)
        function queue = perceiveScene(rgbFrame, depthFrame, camTF, ctx)
            P = ctx.P;
            rgbFrame = uint8(rgbFrame);
            depthFrame = double(depthFrame);

            maskDepth = competitionController.Perception.depthForegroundMask(depthFrame, P);
            maskColor = competitionController.Perception.objectColorMask(rgbFrame, P);

            maskRgb = imdilate(maskColor, strel("disk", 2)) & maskDepth;
            maskRgb = imopen(maskRgb, strel("disk", 2));
            maskRgb = imclose(maskRgb, strel("disk", 4));
            maskRgb = imfill(maskRgb, "holes");
            maskRgb = bwareaopen(maskRgb, P.det.minDepthBlobArea);

            mask = maskDepth & imdilate(maskColor, strel("disk", 3));
            mask = imopen(mask, strel("disk", 2));
            mask = imclose(mask, strel("disk", 4));
            mask = imfill(mask, "holes");
            mask = bwareaopen(mask, P.det.minDepthBlobArea);
            mask = competitionController.Perception.suppressRobotBaseMask(mask, P);
            maskRgb = competitionController.Perception.suppressRobotBaseMask(maskRgb, P);

            cc = bwconncomp(mask, 8);
            props = regionprops(cc, ...
                "BoundingBox", "Centroid", "Area", "PixelIdxList", ...
                "Orientation", "MajorAxisLength", "MinorAxisLength");

            queue = repmat(competitionController.Context.emptyTarget(), 0, 1);
            rawCandidates = 0;
            frameArea = numel(mask);
            rejectPose = 0;
            rejectWorkspace = 0;
            rejectMemory = 0;
            rejectIK = 0;
            rejectSelf = 0;
            softUnsafe = 0;

            for i = 1:numel(props)
                if props(i).Area < P.det.minDepthBlobArea || props(i).Area > P.det.maxDepthBlobArea
                    continue;
                end
                if props(i).Area > 0.85 * frameArea
                    continue;
                end
                rawCandidates = rawCandidates + 1;

                bbox = props(i).BoundingBox;
                [rgbPatch, depthPatch, validDepth] = competitionController.Perception.cropPatches(rgbFrame, depthFrame, bbox);
                if nnz(validDepth) < 20
                    continue;
                end

                [position, height, poseOK] = competitionController.Perception.estimatePose( ...
                    props(i), depthFrame, bbox, camTF, P);
                if ~poseOK
                    rejectPose = rejectPose + 1;
                    continue;
                end

                if competitionController.Context.nearMemory(position, ctx.completedTargets, P.memoryRadius) || ...
                        competitionController.Context.nearMemory(position, ctx.failedTargets, 0.04)
                    rejectMemory = rejectMemory + 1;
                    continue;
                end

                [label, color] = competitionController.Perception.classifyObject( ...
                    rgbPatch, depthPatch, validDepth, bbox, height, props(i), P);
                [position, height, hardReject, qualityPenalty] = ...
                    competitionController.Perception.sanitizeTargetEstimate(position, height, label, P);
                if hardReject
                    rejectWorkspace = rejectWorkspace + 1;
                    continue;
                end
                if competitionController.Perception.isRobotSelfCandidate(position, bbox, size(mask), P)
                    rejectSelf = rejectSelf + 1;
                    continue;
                end

                [graspScore, isSafe] = competitionController.Perception.graspScore( ...
                    props(i), props, position, bbox, height, label, P);
                graspScore = graspScore * qualityPenalty;
                if ~isSafe
                    graspScore = 0.6 * graspScore;
                    softUnsafe = softUnsafe + 1;
                end

                [qPre, qGrasp, qLift, graspPosition, graspYaw] = ...
                    competitionController.Planning.planTarget(position, height, props(i).Orientation, label, ctx);
                if ~all(isfinite([qPre; qGrasp; qLift])) || all(qGrasp == 0)
                    rejectIK = rejectIK + 1;
                    continue;
                end

                entry = competitionController.Context.emptyTarget();
                entry.position = position;
                entry.qPre = qPre;
                entry.qGrasp = qGrasp;
                entry.qLift = qLift;
                entry.qDrop = competitionController.Planning.dropQForBin( ...
                    competitionController.Planning.binForLabelColor(label, color), ctx);
                entry.graspPosition = graspPosition;
                entry.graspYaw = graspYaw;
                entry.score = competitionController.Planning.pointValue(label, color) * graspScore;
                entry.label = label;
                entry.color = color;
                entry.height = height;
                entry.source = "vision";
                entry.dropBin = competitionController.Planning.binForLabelColor(label, color);
                entry.basePosition = graspPosition;
                entry.baseYaw = graspYaw;
                entry.variantIndex = 1;
                entry.qRotate = qLift;
                queue(end + 1, 1) = entry; %#ok<AGROW>
            end

            if isempty(queue)
                if competitionController.RuntimeDebug.isVerbose(P)
                    fprintf('[PERCEPTION] rgbPx=%d depthPx=%d blobs=%d graspable=0 pose=%d ws=%d self=%d mem=%d unsafe=%d ik=%d\n', ...
                        nnz(maskRgb), nnz(maskDepth), rawCandidates, rejectPose, rejectWorkspace, ...
                        rejectSelf, rejectMemory, softUnsafe, rejectIK);
                end
                return;
            end

            [~, order] = sort([queue.score], "descend");
            queue = queue(order);
            if competitionController.RuntimeDebug.isVerbose(P)
                fprintf('[PERCEPTION] rgbPx=%d depthPx=%d blobs=%d graspable=%d pose=%d ws=%d self=%d mem=%d unsafe=%d ik=%d\n', ...
                    nnz(maskRgb), nnz(maskDepth), rawCandidates, numel(queue), rejectPose, ...
                    rejectWorkspace, rejectSelf, rejectMemory, softUnsafe, rejectIK);
            end
        end

        function refinedTarget = selectRefinedTarget(queue, currentTarget, ctx)
            refinedTarget = [];
            if isempty(queue)
                return;
            end

            bestIdx = 0;
            bestScore = -inf;
            for i = 1:numel(queue)
                entry = queue(i);
                posDist = norm(entry.position(:) - currentTarget.position(:));
                if posDist > ctx.P.refineRadius
                    continue;
                end
                if entry.label ~= currentTarget.label
                    continue;
                end
                if currentTarget.color ~= "unknown" && entry.color ~= currentTarget.color
                    continue;
                end

                matchScore = -2.5 * posDist;
                zDist = abs(entry.position(3) - currentTarget.position(3));
                matchScore = matchScore - 3.0 * zDist;
                matchScore = matchScore + 2.5 + 0.05 * entry.score;

                if matchScore > bestScore
                    bestScore = matchScore;
                    bestIdx = i;
                end
            end

            if bestIdx > 0
                refinedTarget = queue(bestIdx);
            end
        end

    end

    methods (Static, Access = private)
        function mask = objectColorMask(rgbFrame, P)
            hsvPatch = rgb2hsv(rgbFrame);
            h = hsvPatch(:, :, 1);
            s = hsvPatch(:, :, 2);
            v = hsvPatch(:, :, 3);

            redMask = ((h <= 0.04) | (h >= 0.96)) & s > 0.30 & v > 0.18;
            yellowMask = (h >= 0.10 & h <= 0.18) & s > 0.28 & v > 0.22;
            greenMask = (h >= 0.22 & h <= 0.42) & s > 0.22 & v > 0.15;
            blueMask = (h >= 0.52 & h <= 0.72) & s > 0.20 & v > 0.15;
            purpleMask = (h >= 0.72 & h <= 0.88) & s > 0.20 & v > 0.15;
            whiteMask = v >= P.det.whiteMinValue & s <= P.det.whiteMaxSat;
            blackMask = v <= P.det.blackMaxValue & s <= P.det.blackMaxSat;
            mask = redMask | yellowMask | greenMask | blueMask | purpleMask | whiteMask | blackMask;
        end

        function mask = suppressRobotBaseMask(mask, P)
            [h, w] = size(mask);
            top = max(1, floor(P.det.baseMaskTopFrac * h));
            left = max(1, floor(P.det.baseMaskLeftFrac * w));
            right = min(w, ceil(P.det.baseMaskRightFrac * w));
            mask(top:h, left:right) = false;
        end

        function [rgbPatch, depthPatch, validDepth] = cropPatches(rgbFrame, depthFrame, bbox)
            [h, w, ~] = size(rgbFrame);
            x1 = max(1, floor(bbox(1)));
            y1 = max(1, floor(bbox(2)));
            x2 = min(w, ceil(bbox(1) + bbox(3)));
            y2 = min(h, ceil(bbox(2) + bbox(4)));
            rgbPatch = rgbFrame(y1:y2, x1:x2, :);
            depthPatch = depthFrame(y1:y2, x1:x2);
            validDepth = isfinite(depthPatch) & depthPatch > 0.05 & depthPatch < 2.50;
        end

        function [position, height, hardReject, qualityPenalty] = sanitizeTargetEstimate(position, height, label, P)
            position = position(:);
            height = max(height, P.det.minHeight);
            hardReject = false;
            qualityPenalty = 1.0;

            if position(1) < P.workspace.x(1) || position(1) > P.workspace.x(2)
                hardReject = true;
                return;
            end
            if position(2) < P.workspace.y(1) || position(2) > P.workspace.y(2)
                hardReject = true;
                return;
            end
            if position(3) < -0.12 || position(3) > P.workspace.z(2)
                hardReject = true;
                return;
            end
            if height > P.class.maxReasonableHeight
                hardReject = true;
                return;
            end

            position(3) = min(max(position(3), P.workspace.z(1)), P.workspace.z(2));

            switch label
                case "bottle"
                    if height > P.class.maxBottleHeight
                        qualityPenalty = 0.70;
                        height = min(height, P.class.maxBottleHeight);
                    end
                case "can"
                    if height > P.class.maxCanHeight
                        qualityPenalty = 0.72;
                        height = min(height, P.class.maxCanHeight);
                    end
                case "marker"
                    if height > P.class.maxMarkerHeight
                        qualityPenalty = 0.65;
                        height = min(height, P.class.maxMarkerHeight);
                    end
                case "spam"
                    if height > P.class.maxSpamHeight
                        qualityPenalty = 0.68;
                        height = min(height, P.class.maxSpamHeight);
                    end
                case "cube"
                    if height > P.class.maxCubeHeight
                        qualityPenalty = 0.65;
                        height = min(height, P.class.maxCubeHeight);
                    end
            end
        end

        function tf = isRobotSelfCandidate(position, bbox, imageSize, P)
            tf = false;
            if position(1) >= P.workspace.baseRejectX(1) && position(1) <= P.workspace.baseRejectX(2) && ...
                    position(2) >= P.workspace.baseRejectY(1) && position(2) <= P.workspace.baseRejectY(2) && ...
                    position(3) <= P.workspace.baseRejectZMax
                tf = true;
                return;
            end

            h = imageSize(1);
            w = imageSize(2);
            bboxCx = bbox(1) + 0.5 * bbox(3);
            bboxBottom = bbox(2) + bbox(4);
            if bboxBottom >= P.det.baseMaskTopFrac * h && ...
                    bboxCx >= P.det.baseMaskLeftFrac * w && ...
                    bboxCx <= P.det.baseMaskRightFrac * w
                tf = true;
            end
        end

        function mask = depthForegroundMask(depthFrame, P)
            valid = isfinite(depthFrame) & depthFrame > P.det.minDepth & depthFrame < P.det.maxDepth;
            if nnz(valid) < 50
                mask = false(size(depthFrame));
                return;
            end

            tableDepth = prctile(depthFrame(valid), 92);
            mask = valid & depthFrame < (tableDepth - P.det.depthBand);
            if nnz(mask) < P.det.minDepthBlobArea
                tableDepth = prctile(depthFrame(valid), 97);
                mask = valid & depthFrame < (tableDepth - 0.03);
            end
            if nnz(mask) > P.det.maxForegroundCoverage * numel(mask)
                depthNorm = mat2gray(depthFrame, [P.det.minDepth P.det.maxDepth]);
                edgeMask = edge(depthNorm, 'Canny');
                edgeMask = imdilate(edgeMask, strel('disk', 2));
                mask = mask & ~edgeMask;
            end

            mask = imopen(mask, strel("disk", 2));
            mask = imclose(mask, strel("disk", 5));
            mask = imfill(mask, "holes");
            mask = bwareafilt(mask, [P.det.minDepthBlobArea max(P.det.maxDepthBlobArea, P.det.minDepthBlobArea)]);
        end

        function [position, objHeight, poseOK] = estimatePose(prop, depthFrame, bbox, camTF, P)
            position = zeros(3, 1);
            objHeight = 0;
            poseOK = false;

            allDepthVals = depthFrame(prop.PixelIdxList);
            allDepthVals = allDepthVals(isfinite(allDepthVals) & ...
                allDepthVals > P.det.minDepth & allDepthVals < P.det.maxDepth);
            depthVals = allDepthVals;
            if numel(depthVals) < 20
                return;
            end

            objDepth = median(depthVals);
            [h, w] = size(depthFrame);
            x1 = max(1, floor(bbox(1)));
            y1 = max(1, floor(bbox(2)));
            x2 = min(w, ceil(bbox(1) + bbox(3)));
            y2 = min(h, ceil(bbox(2) + bbox(4)));

            pad = P.det.ringPad;
            rx1 = max(1, x1 - pad);
            ry1 = max(1, y1 - pad);
            rx2 = min(w, x2 + pad);
            ry2 = min(h, y2 + pad);
            ring = depthFrame(ry1:ry2, rx1:rx2);
            ringMask = true(size(ring));
            ringMask((y1 - ry1 + 1):(y2 - ry1 + 1), (x1 - rx1 + 1):(x2 - rx1 + 1)) = false;
            ringVals = ring(ringMask);
            ringVals = ringVals(isfinite(ringVals) & ringVals > P.det.minDepth & ringVals < P.det.maxDepth);
            if isempty(ringVals)
                tableDepth = prctile(depthVals, 95);
            else
                tableDepth = median(ringVals);
            end

            objHeight = max(tableDepth - objDepth, 0);
            if objHeight < P.det.minHeight
                objHeight = max(prctile(depthVals, 90) - prctile(depthVals, 10), 0.02);
            end

            [rowsObj, colsObj] = ind2sub(size(depthFrame), prop.PixelIdxList);
            objMask = isfinite(depthFrame(prop.PixelIdxList)) & ...
                depthFrame(prop.PixelIdxList) > P.det.minDepth & ...
                depthFrame(prop.PixelIdxList) < P.det.maxDepth;
            rowsObj = rowsObj(objMask);
            colsObj = colsObj(objMask);
            objPixDepth = depthFrame(prop.PixelIdxList);
            objPixDepth = objPixDepth(objMask);

            if isempty(rowsObj)
                u = prop.Centroid(1);
                v = prop.Centroid(2);
            else
                localMask = false(y2 - y1 + 1, x2 - x1 + 1);
                localRows = rowsObj - y1 + 1;
                localCols = colsObj - x1 + 1;
                localIdx = sub2ind(size(localMask), localRows, localCols);
                localMask(localIdx) = true;
                distMap = bwdist(~localMask);

                coreMask = objPixDepth <= prctile(objPixDepth, 45);
                interiorThresh = max(1, 0.55 * max(distMap(:)));
                interiorMask = false(size(rowsObj));
                interiorMask(distMap(localIdx) >= interiorThresh) = true;
                centerMask = coreMask & interiorMask;
                if nnz(centerMask) < 8
                    centerMask = interiorMask;
                end
                if nnz(centerMask) < 8
                    centerMask = coreMask;
                end
                if nnz(centerMask) < 8
                    centerMask = true(size(rowsObj));
                end

                colsCore = double(colsObj(centerMask));
                rowsCore = double(rowsObj(centerMask));
                weightVals = double(distMap(localIdx(centerMask)));
                if isempty(weightVals) || all(weightVals <= 0)
                    weightVals = ones(size(colsCore));
                else
                    weightVals = weightVals .^ 2;
                end
                u = sum(colsCore .* weightVals) / sum(weightVals);
                v = sum(rowsCore .* weightVals) / sum(weightVals);

                zCore = double(objPixDepth(centerMask));
                if isempty(zCore)
                    zCore = double(objPixDepth);
                    cols3d = double(colsObj);
                    rows3d = double(rowsObj);
                    weight3d = ones(size(zCore));
                else
                    cols3d = colsCore;
                    rows3d = rowsCore;
                    weight3d = weightVals;
                end

                xCore = (cols3d - P.cam.cx) .* zCore / P.cam.fx;
                yCore = (rows3d - P.cam.cy) .* zCore / P.cam.fy;
                ptsCam = [xCore(:), yCore(:), zCore(:), ones(numel(zCore), 1)]';
                ptsWorld = camTF * ptsCam;
                if ~isempty(ptsWorld)
                    weight3d = weight3d(:)' / sum(weight3d(:));
                    position = ptsWorld(1:3, :) * weight3d';
                end
            end

            u = u + P.motion.xyNudgeGain * (P.cam.cx - u);
            v = v + 0.5 * P.motion.xyNudgeGain * (P.cam.cy - v);
            ptCam = [(u - P.cam.cx) * objDepth / P.cam.fx; ...
                     (v - P.cam.cy) * objDepth / P.cam.fy; ...
                     objDepth; ...
                     1];
            ptWorld = camTF * ptCam;

            if ~all(isfinite(position))
                position = ptWorld(1:3);
            else
                position = 0.90 * position + 0.10 * ptWorld(1:3);
            end

            poseOK = all(isfinite(position)) && all(abs(position) < 5);
        end

        function [label, color] = classifyObject(rgbPatch, depthPatch, validDepth, bbox, objHeight, prop, P)
            color = competitionController.Perception.dominantColor(rgbPatch);
            metricWidth = bbox(3) * median(depthPatch(validDepth), "omitnan") / P.cam.fx;
            metricHeight = bbox(4) * median(depthPatch(validDepth), "omitnan") / P.cam.fy;
            majorExtent = max(metricWidth, metricHeight);
            minorExtent = min(metricWidth, metricHeight);
            aspect = max(prop.MajorAxisLength / max(prop.MinorAxisLength, 1), bbox(4) / max(bbox(3), 1));

            if color == "white"
                label = "spam";
                return;
            end
            if color == "black"
                if majorExtent > P.class.markerLength || aspect > 2.2
                    label = "marker";
                else
                    label = "spam";
                end
                return;
            end
            if color == "purple" || color == "red_cube" || color == "blue_cube" || color == "green_cube"
                label = "cube";
                return;
            end
            if objHeight < P.class.cubeHeight && abs(metricWidth - metricHeight) < 0.03 && majorExtent < 0.11
                label = "cube";
                return;
            end

            upright = objHeight >= P.class.uprightCanHeight;
            if upright
                if color == "blue"
                    label = "bottle";
                elseif color == "green"
                    label = "can";
                elseif color == "yellow" || color == "red"
                    if objHeight >= P.class.uprightBottleHeight
                        label = "bottle";
                    else
                        label = "can";
                    end
                else
                    label = "can";
                end
                return;
            end

            if majorExtent >= P.class.longObjectLength
                if color == "blue"
                    label = "bottle";
                elseif color == "green"
                    label = "can";
                elseif color == "yellow" || color == "red"
                    if majorExtent > 0.17
                        label = "bottle";
                    else
                        label = "can";
                    end
                else
                    label = "marker";
                end
            else
                if color == "blue"
                    label = "bottle";
                else
                    label = "can";
                end
            end

            if minorExtent < 0.05 && majorExtent < 0.09 && color ~= "blue" && objHeight < 0.08
                label = "cube";
            end
        end

        function color = dominantColor(rgbPatch)
            hsvPatch = rgb2hsv(rgbPatch);
            h = hsvPatch(:, :, 1);
            s = hsvPatch(:, :, 2);
            v = hsvPatch(:, :, 3);
            vivid = s > 0.22 & v > 0.15;

            if nnz(vivid) < 20
                if mean(v(:)) > 0.55
                    color = "white";
                else
                    color = "black";
                end
                return;
            end

            votes = [ ...
                nnz(((h < 0.05) | (h > 0.95)) & vivid), ...
                nnz((h >= 0.10 & h <= 0.18) & vivid), ...
                nnz((h >= 0.24 & h <= 0.43) & vivid), ...
                nnz((h >= 0.53 & h <= 0.72) & vivid), ...
                nnz((h >= 0.72 & h <= 0.86) & vivid)];
            [~, idx] = max(votes);
            names = ["red", "yellow", "green", "blue", "purple"];
            color = names(idx);
        end

        function [score, isSafe] = graspScore(prop, props, position, bbox, objHeight, label, P)
            fillRatio = prop.Area / max(bbox(3) * bbox(4), 1);
            typeBonus = competitionController.Planning.pointValue(label, "unknown") / 30;
            clearance = inf;
            for i = 1:numel(props)
                other = props(i);
                if abs(other.Centroid(1) - prop.Centroid(1)) < 1e-6 && ...
                        abs(other.Centroid(2) - prop.Centroid(2)) < 1e-6
                    continue;
                end
                pxDist = norm(prop.Centroid - other.Centroid);
                clearance = min(clearance, pxDist * max(position(1), 0.2) / P.cam.fx);
            end

            if isinf(clearance)
                clearance = 0.10;
            end

            score = 0.40 * min(fillRatio, 1.0) + ...
                0.25 * min(objHeight / 0.18, 1.0) + ...
                0.20 * min(clearance / 0.08, 1.0) + ...
                0.15 * typeBonus;
            isSafe = clearance >= P.grasp.minClearance;
        end
    end
end
