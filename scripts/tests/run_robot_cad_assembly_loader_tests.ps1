[CmdletBinding()]
param(
    [string]$StepPath = '',
    [switch]$DetailedPresentation
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$qtRoot = 'E:\workspace\soft\QT\6.7.3\msvc2022_64'
$eigenRoot = 'E:\Eigen3.4\eigen-3.4.0'
$occtRoot = if ($env:VCPKG_ROOT) {
    Join-Path $env:VCPKG_ROOT 'installed\x64-windows'
} else {
    'E:\vcpkg\installed\x64-windows'
}
if ([string]::IsNullOrWhiteSpace($StepPath)) {
    $StepPath = Get-ChildItem -LiteralPath 'E:\WorkFile\新时达' -Recurse -File |
        Where-Object { $_.Extension -ieq '.step' -or $_.Extension -ieq '.stp' } |
        Sort-Object Length -Descending |
        Select-Object -First 1 -ExpandProperty FullName
}

$outputDir = Join-Path $repo 'tmp\RobotCadAssemblyLoaderTests'
$outputExe = Join-Path $outputDir 'RobotCadAssemblyLoaderTests.exe'
foreach ($required in @(
    $vsDevCmd,
    $qtRoot,
    $eigenRoot,
    (Join-Path $occtRoot 'include\opencascade\STEPCAFControl_Reader.hxx'),
    (Join-Path $occtRoot 'lib\TKXCAF.lib'),
    $StepPath
)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "required robot CAD test dependency not found: $required"
    }
}
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null

$qtInclude = Join-Path $qtRoot 'include'
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cd', '/d', ('"{0}"' -f $outputDir), '&&',
    'cl', '/nologo', '/EHsc', '/O2', '/std:c++17', '/Zc:__cplusplus', '/permissive-', '/utf-8',
    '/DUNICODE', '/D_UNICODE', '/DNOMINMAX', '/DWIN32_LEAN_AND_MEAN',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f $eigenRoot),
    ('/I"{0}"' -f (Join-Path $occtRoot 'include\opencascade')),
    ('/I"{0}"' -f $qtInclude),
    ('/I"{0}\QtCore"' -f $qtInclude),
    ('"{0}"' -f (Join-Path $repo 'src\RobotCadAssemblyLoader.cpp')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\robot_cad_assembly_loader_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe),
    '/link', ('/LIBPATH:"{0}"' -f (Join-Path $qtRoot 'lib')),
    ('/LIBPATH:"{0}"' -f (Join-Path $occtRoot 'lib')),
    'Qt6Core.lib',
    'TKDESTEP.lib', 'TKXSBase.lib', 'TKXCAF.lib', 'TKVCAF.lib', 'TKCAF.lib', 'TKLCAF.lib',
    'TKCDF.lib', 'TKService.lib', 'TKMesh.lib', 'TKTopAlgo.lib', 'TKBRep.lib',
    'TKGeomBase.lib', 'TKG3d.lib', 'TKG2d.lib', 'TKMath.lib', 'TKernel.lib'
) -join ' '

& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) {
    throw "RobotCadAssemblyLoaderTests compilation failed: $LASTEXITCODE"
}

$env:PATH = (Join-Path $occtRoot 'bin') + ';' + (Join-Path $qtRoot 'bin') + ';' + $env:PATH
if ($DetailedPresentation) {
    & $outputExe $StepPath
} else {
    & $outputExe $StepPath '--bounds-only'
}
if ($LASTEXITCODE -ne 0) {
    throw "RobotCadAssemblyLoaderTests failed: $LASTEXITCODE"
}
