param(
    [string]$Compiler = 'g++',
    [switch]$Zig
)
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$buildRoot = Join-Path $repoRoot 'build\host'
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
$firstHeader = Get-FileHash -Algorithm SHA256 -LiteralPath (Join-Path $repoRoot 'firmware\Anchor1\UwbUart.h')
$secondHeader = Get-FileHash -Algorithm SHA256 -LiteralPath (Join-Path $repoRoot 'firmware\Anchor2\UwbUart.h')
if ($firstHeader.Hash -ne $secondHeader.Hash) { throw 'The two UART headers differ.' }
$compilerPrefix = @()
if ($Zig) { $compilerPrefix = @('c++') }
foreach ($name in @('anchor1', 'anchor2', 'tag')) {
    $sourcePath = Join-Path $PSScriptRoot "test_$name.cpp"
    $binaryPath = Join-Path $buildRoot "test_$name.exe"
    $compileArgs = $compilerPrefix + @('-std=c++17', '-Wall', '-Wextra', '-I', (Join-Path $PSScriptRoot 'stubs'), $sourcePath, '-o', $binaryPath)
    & $Compiler @compileArgs
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed for $name" }
    & $binaryPath
    if ($LASTEXITCODE -ne 0) { throw "Test failed for $name" }
}
