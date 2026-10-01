param(
    [string]$Compiler = 'g++',
    [switch]$Zig
)
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$buildRoot = Join-Path $repoRoot 'build\host'
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
$compilerPrefix = @()
if ($Zig) {
    $compilerPrefix = @('c++')
    $env:ZIG_GLOBAL_CACHE_DIR = Join-Path $buildRoot 'zig-cache'
    $env:ZIG_LOCAL_CACHE_DIR = Join-Path $buildRoot 'zig-local'
}
foreach ($name in @('anchor1', 'anchor2', 'tag')) {
    $sourcePath = Join-Path $PSScriptRoot "test_$name.cpp"
    $binaryPath = Join-Path $buildRoot "test_$name.exe"
    $compileArgs = $compilerPrefix + @('-std=c++17', '-Wall', '-Wextra', '-I', (Join-Path $PSScriptRoot 'stubs'), $sourcePath, '-o', $binaryPath)
    & $Compiler @compileArgs
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed for $name" }
    & $binaryPath
    if ($LASTEXITCODE -ne 0) { throw "Test failed for $name" }
}
