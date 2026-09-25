@echo off
rem Double-click to start the 8567 robot program for the Synthesis simulation.
rem Leave this window open while you play; close it to stop.
cd /d "%~dp0\.."

rem WPILib installs its Java here, for all users or for just this user.
set "JAVA_HOME="
for %%D in ("C:\Users\Public\wpilib\2026\jdk" "%USERPROFILE%\wpilib\2026\jdk" "C:\Users\Public\wpilib\2027\jdk" "%USERPROFILE%\wpilib\2027\jdk") do (
    if not defined JAVA_HOME if exist "%%~D\bin\java.exe" set "JAVA_HOME=%%~D"
)
if not defined JAVA_HOME (
    echo Could not find Java 17. Install WPILib 2026 ^(see the simulation guide^), then try again.
    pause
    exit /b 1
)

echo Starting the robot program (the first run downloads files and takes a few minutes)...
echo When you see "Robot program startup complete", open your match link in Chrome.
call gradlew.bat simulateJava -Psynthesis %*
echo The robot program stopped.
pause
