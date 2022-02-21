# Mower Area Recorder
This package is used to record the navigation and mowing areas for our robotic mower.

Currently, a gamepad is used to drive the robot and to start / stop recordings.

## How to Start
Just use the launch files provided in the open_mower package to launch the recording process.
By default, the existing map will be loaded and you can add additional areas.

## Controls

The following controls are used for the are recorder (default XBox Gamepad):
- A: hold to drive the bot. Use the left analog stick to steer the bot.
- B: Toggle area recording.
- Y: Finish and save current area.
- X: Set docking position.

So, in order to record a new map, do the following:
1. Drive to any point on the outline of your navigation area and press B
2. Drive around the navigation area. Press B and Y to stop recording and to save the area.
3. For each mowing area:
   - Drive to any point on the outline of the mowing Area
   - Press B to start recording
   - Drive around the outline
   - Press B to stop recording
   - Drive to any obstacle
   - Press B to start recording
   - Drive around the obstacle
   - Press B to stop recording
   - Repeat for all obstacles
   - Press Y to save mowing area