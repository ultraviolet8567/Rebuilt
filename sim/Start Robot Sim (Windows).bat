@echo off
rem Double-click to start the 8567 robot program for the Synthesis simulation.
rem Leave this window open while you play; close it to stop.
cd /d "%~dp0\.."

rem Java 17: the one WPILib installs (for all users or for just this user), else a Java 17
rem already on this computer. Every candidate is run and must report version 17: the
rem WPILib 2027 folder, for one, may hold a newer Java.
set "OLD_JAVA_HOME="
if not defined JAVA_HOME goto :java_home_saved
set "OLD_JAVA_HOME=%JAVA_HOME:"=%"
:java_home_saved
set "JAVA_HOME="

for %%D in ("C:\Users\Public\wpilib\2026\jdk" "%USERPROFILE%\wpilib\2026\jdk" "C:\Users\Public\wpilib\2027\jdk" "%USERPROFILE%\wpilib\2027\jdk") do call :try_java "%%~D"
if defined OLD_JAVA_HOME call :try_java "%OLD_JAVA_HOME%"
for /d %%D in ("%ProgramFiles%\Eclipse Adoptium\jdk-17*") do call :try_java "%%~D"
for /d %%D in ("%ProgramFiles%\Java\jdk-17*") do call :try_java "%%~D"
for /d %%D in ("%ProgramFiles%\Microsoft\jdk-17*") do call :try_java "%%~D"
for /d %%D in ("%LOCALAPPDATA%\Programs\Eclipse Adoptium\jdk-17*") do call :try_java "%%~D"
if defined JAVA_HOME goto :java_found

rem Last, the java on PATH: ask it where it is installed (the PATH entry can be a shortcut).
set "PATH_JAVA_HOME="
for /f "tokens=1,* delims==" %%A in ('java -XshowSettings:properties -version 2^>^&1 ^| findstr /c:"java.home ="') do set "PATH_JAVA_HOME=%%B"
if not defined PATH_JAVA_HOME goto :java_missing
rem The value starts with the space that follows the = sign; drop it.
call :try_java "%PATH_JAVA_HOME:~1%"
if defined JAVA_HOME goto :java_found

:java_missing
echo Could not find Java 17. Install Java 17 (Temurin 17) or WPILib 2026 (see the simulation guide), then try again.
pause
exit /b 1

:java_found
echo Using Java 17 in "%JAVA_HOME%"
echo Starting the robot program (the first run downloads files and takes a few minutes)...
echo When you see "Robot program startup complete", open your match link in Chrome.
call gradlew.bat simulateJava -Psynthesis %*
echo The robot program stopped.
pause
exit /b 0

rem Sets JAVA_HOME to the folder it is given if none is chosen yet and its java.exe reports version 17.
rem (java -version prints a line like: openjdk version "17.0.13" 2024-10-15)
:try_java
if defined JAVA_HOME exit /b 0
if not exist "%~1\bin\java.exe" exit /b 0
"%~1\bin\java.exe" -version 2>&1 | findstr /r /c:"version .17[^0-9]" >nul
if not errorlevel 1 set "JAVA_HOME=%~1"
exit /b 0
