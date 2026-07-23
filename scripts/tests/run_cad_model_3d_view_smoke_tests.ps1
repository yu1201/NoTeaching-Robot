[CmdletBinding()]
param(
    [string]$StepPath = 'E:\WorkFile\bowen\QtWidgetsApplication4\Temp\海星箱体模拟(1).STEP',
    [string]$RobotStepPath = ''
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$qtRoot = 'E:\workspace\soft\QT\6.7.3\msvc2022_64'
$occtRoot = if ($env:VCPKG_ROOT) {
    Join-Path $env:VCPKG_ROOT 'installed\x64-windows'
} else {
    'E:\vcpkg\installed\x64-windows'
}
$outputDir = Join-Path $repo 'tmp\CadModel3DViewSmokeTests'
$outputExe = Join-Path $outputDir 'CadModel3DViewSmokeTests.exe'
$screenshot = Join-Path $outputDir 'cad-model-preview.png'

foreach ($required in @(
    $vsDevCmd,
    (Join-Path $qtRoot 'include\QtWidgets\QApplication'),
    (Join-Path $occtRoot 'include\opencascade\AIS_Shape.hxx'),
    (Join-Path $occtRoot 'lib\TKOpenGl.lib'),
    (Join-Path $occtRoot 'lib\TKPrim.lib'),
    $StepPath
)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "required CAD display test dependency not found: $required"
    }
}
if ($RobotStepPath -and -not (Test-Path -LiteralPath $RobotStepPath -PathType Leaf)) {
    throw "required theoretical robot STEP not found: $RobotStepPath"
}
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null

$qtInclude = Join-Path $qtRoot 'include'
$qtLib = Join-Path $qtRoot 'lib'
$occtInclude = Join-Path $occtRoot 'include\opencascade'
$occtLib = Join-Path $occtRoot 'lib'
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cd', '/d', ('"{0}"' -f $outputDir), '&&',
    'cl', '/nologo', '/EHsc', '/O2', '/std:c++17', '/Zc:__cplusplus', '/permissive-', '/utf-8',
    '/DUNICODE', '/D_UNICODE', '/DNOMINMAX', '/DWIN32_LEAN_AND_MEAN',
    '/DCAD_MODEL_3D_VIEW_TESTING',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f $occtInclude),
    '/I"E:\Eigen3.4\eigen-3.4.0"',
    ('/I"{0}"' -f $qtInclude),
    ('/I"{0}\QtCore"' -f $qtInclude),
    ('/I"{0}\QtGui"' -f $qtInclude),
    ('/I"{0}\QtWidgets"' -f $qtInclude),
    ('"{0}"' -f (Join-Path $repo 'src\CadModel3DView.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\AppPaths.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\RobotCollisionEnvelopeStore.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\RobotCadAssemblyLoader.cpp')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\cad_model_3d_view_smoke_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe),
    '/link', ('/LIBPATH:"{0}"' -f $qtLib), ('/LIBPATH:"{0}"' -f $occtLib),
    'Qt6Core.lib', 'Qt6Gui.lib', 'Qt6Widgets.lib',
    'TKDESTEP.lib', 'TKXSBase.lib', 'TKXCAF.lib', 'TKVCAF.lib',
    'TKCAF.lib', 'TKLCAF.lib', 'TKCDF.lib', 'TKMesh.lib', 'TKTopAlgo.lib',
    'TKBRep.lib', 'TKG3d.lib', 'TKMath.lib', 'TKernel.lib', 'TKPrim.lib',
    'TKService.lib', 'TKV3d.lib', 'TKOpenGl.lib', 'shell32.lib'
) -join ' '

& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) {
    throw "CadModel3DViewSmokeTests compilation failed: $LASTEXITCODE"
}

$env:PATH = (Join-Path $occtRoot 'bin') + ';' + (Join-Path $qtRoot 'bin') + ';' + $env:PATH
$testArguments = @($StepPath, $screenshot)
if ($RobotStepPath) { $testArguments += $RobotStepPath }
& $outputExe @testArguments
if ($LASTEXITCODE -ne 0) {
    throw "CadModel3DViewSmokeTests failed: $LASTEXITCODE"
}

Write-Output "CAD screenshot: $screenshot"
