# RoboCup ARM 2026 Submission

This repository contains our RoboCup ARM challenge submission.

The main model is `RoboCup_ARM.slx`. The runtime controller is implemented in `competitionControllerRuntime.m`, with supporting logic in `+competitionController/`.

## Files

- `RoboCup_ARM.slx` - main Simulink model
- `competitionControllerRuntime.m` - runtime controller entry point
- `scripts/prepareCompetitionController.m` - prepares the model and hooks the runtime controller into the MATLAB Function block
- `scripts/arm_startup.m` - loads the saved model data and opens the model
- `modelData/` - saved robot and motion data used by the controller

## Usage

Open the repository folder in MATLAB, then run:

```matlab
addpath(genpath(pwd))
prepareCompetitionController
open_system("RoboCup_ARM.slx")
```

After that, start the simulation from Simulink.

If you only want to load the saved data and open the model, run:

```matlab
arm_startup
```

## Notes

- This submission currently enables `PHASE1_FIXED` and `PHASE3_ORIENTATION`.
- The controller state is reset when `prepareCompetitionController` runs.
