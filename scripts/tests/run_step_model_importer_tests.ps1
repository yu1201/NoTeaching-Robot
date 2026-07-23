[CmdletBinding()]
param(
    [string]$StepPath = 'E:\WorkFile\bowen\QtWidgetsApplication4\Temp\海星箱体模拟(1).STEP'
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$qtRoot = 'E:\workspace\soft\QT\6.7.3\msvc2022_64'
$eigenRoot = 'E:\Eigen3.4\eigen-3.4.0'
$opencvRoot = 'E:\OpenCV4.6.0\build'
$occtRoot = if ($env:VCPKG_ROOT) {
    Join-Path $env:VCPKG_ROOT 'installed\x64-windows'
} else {
    'E:\vcpkg\installed\x64-windows'
}
$outputDir = Join-Path $repo 'tmp\StepModelImporterTests'
$outputExe = Join-Path $outputDir 'StepModelImporterTests.exe'

foreach ($required in @(
    $vsDevCmd,
    $qtRoot,
    $eigenRoot,
    $opencvRoot,
    (Join-Path $occtRoot 'include\opencascade\STEPControl_Reader.hxx'),
    (Join-Path $occtRoot 'lib\TKDESTEP.lib'),
    $StepPath
)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "required STEP test dependency not found: $required"
    }
}
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null

$qtInclude = Join-Path $qtRoot 'include'
$qtLib = Join-Path $qtRoot 'lib'
$occtInclude = Join-Path $occtRoot 'include\opencascade'
$occtLib = Join-Path $occtRoot 'lib'
$opencvInclude = Join-Path $opencvRoot 'include'
$opencvLib = Join-Path $opencvRoot 'x64\vc15\lib'
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cd', '/d', ('"{0}"' -f $outputDir), '&&',
    'cl', '/nologo', '/EHsc', '/O2', '/std:c++17', '/Zc:__cplusplus', '/permissive-', '/utf-8',
    '/DUNICODE', '/D_UNICODE', '/DNOMINMAX', '/DWIN32_LEAN_AND_MEAN',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f $eigenRoot),
    ('/I"{0}"' -f $occtInclude),
    ('/I"{0}"' -f $opencvInclude),
    ('/I"{0}"' -f $qtInclude),
    ('/I"{0}\QtCore"' -f $qtInclude),
    ('/I"{0}\QtGui"' -f $qtInclude),
    ('"{0}"' -f (Join-Path $repo 'src\StepModelImporter.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\ReferenceModelLibrary.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\WorkpieceMeshBuilder.cpp')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\step_model_importer_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe),
    '/link', ('/LIBPATH:"{0}"' -f $qtLib), ('/LIBPATH:"{0}"' -f $occtLib),
    ('/LIBPATH:"{0}"' -f $opencvLib),
    'Qt6Core.lib', 'Qt6Gui.lib', 'opencv_world460.lib',
    'TKDESTEP.lib', 'TKXSBase.lib', 'TKMesh.lib', 'TKTopAlgo.lib',
    'TKBRep.lib', 'TKPrim.lib', 'TKG3d.lib', 'TKMath.lib', 'TKernel.lib'
) -join ' '

& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) {
    throw "StepModelImporterTests compilation failed: $LASTEXITCODE"
}

$env:PATH = (Join-Path $occtRoot 'bin') + ';' + (Join-Path $qtRoot 'bin') + ';' +
    (Join-Path $opencvRoot 'x64\vc15\bin') + ';' + $env:PATH
& $outputExe $StepPath
if ($LASTEXITCODE -ne 0) {
    throw "StepModelImporterTests failed: $LASTEXITCODE"
}
