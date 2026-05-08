function prepareCompetitionController()
clear competitionControllerRuntime;
addpath(pwd);
addpath(fullfile(pwd, "scripts"));
addpath(fullfile(pwd, "modelData"));

armData = load("modelData\arm_data.mat", "q_marker_int1", "q_retBlue1", "q_grip1", "robot", "rt");
assignin("base", "q_marker_int1", armData.q_marker_int1);
assignin("base", "q_retBlue1", armData.q_retBlue1);
assignin("base", "q_grip1", armData.q_grip1);
assignin("base", "robot", armData.robot);
assignin("base", "rt", armData.rt);
gripperData = load("modelData\ur5e_gripper.mat", "robot"); %#ok<NASGU>
open_system("RoboCup_ARM.slx");

delayBlk = "RoboCup_ARM/Camera Tform Delay";
if ~bdIsLoaded("RoboCup_ARM")
    open_system("RoboCup_ARM.slx");
end
if ~any(strcmp(find_system("RoboCup_ARM", "SearchDepth", 1, "Name", "Camera Tform Delay"), delayBlk))
    add_block("simulink/Discrete/Unit Delay", delayBlk, ...
        "InitialCondition", "eye(4)", ...
        "Position", [500 180 560 210]);
end

blk = "RoboCup_ARM/Generate Robot Config";
rt = sfroot;
chart = rt.find("-isa", "Stateflow.EMChart", "Path", blk);
txt = strjoin({ ...
    'function [gripperStatus,currentConfig,stop]= Generate_Robot_Config(rgbFrame,depthFrame,cameraTform,targetGrasped)', ...
    'coder.extrinsic(''competitionControllerRuntime'');', ...
    'gripperStatus = 0;', ...
    'currentConfig = zeros(6,1);', ...
    'stop = false;', ...
    '[gripperStatus,currentConfig,stop] = competitionControllerRuntime(rgbFrame,depthFrame,cameraTform,targetGrasped);', ...
    'end'}, newline);
chart.Script = txt;
save_system("RoboCup_ARM.slx");
close_system("RoboCup_ARM", 0);
open_system("RoboCup_ARM.slx");

ph = get_param(blk, "PortHandles");
for idx = 1:numel(ph.Inport)
    lineHandle = get_param(ph.Inport(idx), "Line");
    if lineHandle ~= -1
        delete_line(lineHandle);
    end
end

add_line("RoboCup_ARM", "camera subsystem/1", "Generate Robot Config/1", "autorouting", "on");
add_line("RoboCup_ARM", "camera subsystem/2", "Generate Robot Config/2", "autorouting", "on");
dph = get_param(delayBlk, "PortHandles");
delayIn = get_param(dph.Inport, "Line");
if delayIn ~= -1
    delete_line(delayIn);
end
delayOut = get_param(dph.Outport, "Line");
if delayOut ~= -1
    delete_line(delayOut);
end
add_line("RoboCup_ARM", "camera subsystem/3", "Camera Tform Delay/1", "autorouting", "on");
add_line("RoboCup_ARM", "Camera Tform Delay/1", "Generate Robot Config/3", "autorouting", "on");
add_line("RoboCup_ARM", "Unit Delay/1", "Generate Robot Config/4", "autorouting", "on");

save_system("RoboCup_ARM.slx");
end
