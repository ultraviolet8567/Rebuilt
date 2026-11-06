---
description: Compile, check formatting, and run the unit tests for this project
---

Run the project's local verification and report the result concisely.

1. Set the JDK first. It is not on `PATH` in a fresh shell:

    ```powershell
    $env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'
    ```

2. Run:

    ```powershell
    .\gradlew.bat compileJava spotlessCheck test
    ```

3. Report:
    - On success: `verify: PASSED` plus the number of tests that ran.
    - On failure: the failing task, the first real error, and the `file:line` it points at. Do not
      paste the whole log, and do not summarize a failure as a success.

Rules:

- If `spotlessCheck` fails, run `.\gradlew.bat spotlessApply` and re-run this command.
- If a test fails, show the test name and the assertion message before proposing a fix.
- Never run `deploy`. Never start `simulateJava` from here.
