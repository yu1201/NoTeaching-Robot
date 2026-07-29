[CmdletBinding()]
param(
    [string]$StepPath = ''
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vsDevCmd =
    'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$qtRoot = 'E:\workspace\soft\QT\6.7.3\msvc2022_64'
$eigenRoot = 'E:\Eigen3.4\eigen-3.4.0'
$occtRoot = 'E:\vcpkg\installed\x64-windows'
$outputDir = Join-Path $repo 'tmp\RobotModelOriginalPreviewTests'
$outputExe = Join-Path $outputDir 'RobotModelOriginalPreviewTests.exe'
$outputPng = Join-Path $outputDir 'original-step-preview.png'

if ([string]::IsNullOrWhiteSpace($StepPath)) {
    $fixture = Get-ChildItem `
        -LiteralPath (Join-Path $repo 'Data\RobotModels') `
        -Filter '*.step' -File -ErrorAction SilentlyContinue |
        Sort-Object Length -Descending |
        Select-Object -First 1
    if ($null -eq $fixture) {
        throw 'No local robot STEP fixture was found; pass -StepPath explicitly.'
    }
    $StepPath = $fixture.FullName
}
$StepPath = (Resolve-Path -LiteralPath $StepPath).Path

foreach ($required in @(
    $vsDevCmd,
    (Join-Path $qtRoot 'include\QtGui\QGuiApplication'),
    (Join-Path $qtRoot 'lib\Qt6Core.lib'),
    (Join-Path $qtRoot 'lib\Qt6Gui.lib'),
    (Join-Path $eigenRoot 'Eigen\Core'),
    (Join-Path $occtRoot 'include\opencascade\TopoDS_Shape.hxx'),
    (Join-Path $repo 'include\RobotModelOriginalPreview.h'),
    (Join-Path $repo 'src\RobotModelOriginalPreview.cpp'),
    (Join-Path $repo 'src\RobotCadAssemblyLoader.cpp'),
    (Join-Path $repo 'scripts\tests\robot_model_original_preview_tests.cpp'),
    $StepPath
)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "required original preview test dependency not found: $required"
    }
}
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null

$qtInclude = Join-Path $qtRoot 'include'
$qtLib = Join-Path $qtRoot 'lib'
$occtInclude = Join-Path $occtRoot 'include\opencascade'
$occtLib = Join-Path $occtRoot 'lib'
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64',
    '1>nul', '2>nul', '&&',
    'cd', '/d', ('"{0}"' -f $outputDir), '&&',
    'cl', '/nologo', '/EHsc', '/O2', '/std:c++17', '/bigobj',
    '/Zc:__cplusplus', '/permissive-', '/utf-8',
    '/DUNICODE', '/D_UNICODE', '/DNOMINMAX', '/DWIN32_LEAN_AND_MEAN',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f $qtInclude),
    ('/I"{0}\QtCore"' -f $qtInclude),
    ('/I"{0}\QtGui"' -f $qtInclude),
    ('/I"{0}"' -f $eigenRoot),
    ('/I"{0}"' -f $occtInclude),
    ('"{0}"' -f (Join-Path $repo 'src\RobotCadAssemblyLoader.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\RobotModelOriginalPreview.cpp')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\robot_model_original_preview_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe),
    '/link',
    ('/LIBPATH:"{0}"' -f $qtLib),
    ('/LIBPATH:"{0}"' -f $occtLib),
    'Qt6Core.lib', 'Qt6Gui.lib',
    'TKDESTEP.lib', 'TKXSBase.lib', 'TKXCAF.lib', 'TKVCAF.lib',
    'TKCAF.lib', 'TKLCAF.lib', 'TKCDF.lib', 'TKMesh.lib',
    'TKTopAlgo.lib', 'TKBRep.lib', 'TKG3d.lib', 'TKMath.lib',
    'TKernel.lib', 'TKPrim.lib', 'TKService.lib', 'TKV3d.lib'
) -join ' '

& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) {
    throw "RobotModelOriginalPreviewTests compilation failed: $LASTEXITCODE"
}

$env:PATH = (Join-Path $qtRoot 'bin') + ';' +
    (Join-Path $occtRoot 'bin') + ';' + $env:PATH
$env:QT_QPA_PLATFORM_PLUGIN_PATH =
    (Join-Path $qtRoot 'plugins\platforms')
$env:QT_QPA_PLATFORM = 'offscreen'
& $outputExe $StepPath $outputPng
if ($LASTEXITCODE -ne 0) {
    throw "RobotModelOriginalPreviewTests failed: $LASTEXITCODE"
}
