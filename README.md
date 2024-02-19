# 2024-robotcode-5970

Welcome to Beavertronics' code for the 2024 FRC Season (Crescendo)!

## Current Branch
### - Drivetrain
Current Developer(s): Ozy, Collin

Purpose: Testing branch for Drivetrain/driving controls code
## How do I run it?

If you want to build the code:
1. Hit Ctrl-Shift-P
2. Start typing `WPILib: Build Robot Code` until it becomes the top element
3. Hit Enter.
It will then pop up a terminal window with a progress bar, and eventually you should see a green `BUILD SUCCESSFUL`.

If you want to build the code and send it to the RoboRIO for testing:
1. Connect to the RoboRIO (You can do this by connecting to the robot's wifi network if the radio is hooked up, if not you have to manually hook up an ethernet cable.)
2. Hit Ctrl-Shift-P
3. Start typing `WPILib: Deploy Robot Code` until it becomes the top element
4. Hit Enter.
It will do the same thing as build, but afterward it will connect to the roboRIO and upload everything- You should still see a green `BUILD SUCCESSFUL` at the end.

## Why isn't kotlin doing all the fancy code completion things?

First, make sure you have the Kotlin extention (by fwcd) *and* the Kotlin for FRC extention (by Brenek Harrison) installed.
Sometimes the extention just freaks out and fails for no reason- Just be calm and close the windows.
Next, go to the settings for Kotlin (not Kotlin for FRC) and make sure `Kotlin > Language Server` is enabled.

Even when it's working, it can glitch out sometimes- if that happens, modifying the line with the glitch on it (ie type asdfkajhsd and then delete it) usually wakes Kotlin up.
If this happens a lot, increasing `Kotlin > Diagnostics: Debounce Time` might help.

## Project Structure

- `src/main/kotlin`: Source code for the robot (we're using kotlin)
- `src/main/deploy`: Files that need to be loaded onto the RoboRIO (ie auto paths)
- `src/ds`: (Arduino) source code for driver station rainbow lights.
- All the `gradle` stuff: Gradle is a build system, which convinces java and kotlin to compile our code. The process involves creating temporary files.
- `.gitignore`: Tells Git which files are important and which are just temporary, to help prevent enormous file sizes and odd junk (ie the `.DS_Store` files MacOS sometimes puts everywhere)
- Everything else: Either self explanatory (ie `.vscode` and `WPILib-License.md`) or temporary files created by gradle.

## Code Structure

Each code file's purpose is outlined briefly near the top of the file- You should read these descriptions now, so that if you need to write some code later, you'll know where that code should go. It has been set up in a way that will hopefully minimize spaghetti, but use your best judgement or else it will become spaghetti anyway.


## Credit

Much of this code is inspired by Team 2898's 2023 season code, which is available on github.
