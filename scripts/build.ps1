param([switch]$Test)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location $projectRoot
try {
    cmake -S native -B native/build
    if ($LASTEXITCODE -ne 0) { throw 'C++ configuration failed.' }
    cmake --build native/build --config Release --parallel 4
    if ($LASTEXITCODE -ne 0) { throw 'C++ build failed.' }
    if ($Test) {
        ctest --test-dir native/build -C Release --output-on-failure
        if ($LASTEXITCODE -ne 0) { throw 'C++ tests failed.' }
        mvn -B -C test
    } else { mvn -B -C -DskipTests package }
    if ($LASTEXITCODE -ne 0) { throw 'Java build or tests failed.' }
} finally { Pop-Location }
