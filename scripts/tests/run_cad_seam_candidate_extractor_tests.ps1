[CmdletBinding()]
param(
    [string]$StepPath = '',
    [double]$MinimumLengthMm = 5.0
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
$outputDir = Join-Path $repo 'tmp\CadSeamCandidateExtractorTests'
$outputExe = Join-Path $outputDir 'CadSeamCandidateExtractorTests.exe'

foreach ($required in @(
    $vsDevCmd,
    (Join-Path $qtRoot 'include\QtCore\QCoreApplication'),
    (Join-Path $eigenRoot 'Eigen\Dense'),
    (Join-Path $occtRoot 'include\opencascade\BRepAlgoAPI_Section.hxx'),
    (Join-Path $occtRoot 'lib\TKBO.lib'),
    (Join-Path $occtRoot 'lib\TKShHealing.lib')
)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "required CAD seam test dependency not found: $required"
    }
}
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null

$qtInclude = Join-Path $qtRoot 'include'
$qtLib = Join-Path $qtRoot 'lib'
$occtInclude = Join-Path $occtRoot 'include\opencascade'
$occtLib = Join-Path $occtRoot 'lib'
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cd', '/d', ('"{0}"' -f $outputDir), '&&',
    'cl', '/nologo', '/EHsc', '/O2', '/std:c++17', '/Zc:__cplusplus', '/permissive-', '/utf-8', '/wd4005',
    '/DUNICODE', '/D_UNICODE', '/DNOMINMAX', '/DWIN32_LEAN_AND_MEAN',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f $eigenRoot),
    ('/I"{0}"' -f $occtInclude),
    ('/I"{0}"' -f $qtInclude),
    ('/I"{0}\QtCore"' -f $qtInclude),
    ('"{0}"' -f (Join-Path $repo 'src\CadSeamCandidateExtractor.cpp')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\cad_seam_candidate_extractor_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe),
    '/link', ('/LIBPATH:"{0}"' -f $qtLib), ('/LIBPATH:"{0}"' -f $occtLib),
    'Qt6Core.lib', 'TKDESTEP.lib', 'TKXSBase.lib', 'TKXCAF.lib', 'TKVCAF.lib',
    'TKCAF.lib', 'TKLCAF.lib', 'TKCDF.lib', 'TKService.lib', 'TKBO.lib', 'TKBool.lib',
    'TKShHealing.lib', 'TKGeomAlgo.lib', 'TKGeomBase.lib', 'TKG2d.lib',
    'TKTopAlgo.lib', 'TKBRep.lib', 'TKPrim.lib', 'TKG3d.lib', 'TKMath.lib', 'TKernel.lib'
) -join ' '

& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) {
    throw "CadSeamCandidateExtractorTests compilation failed: $LASTEXITCODE"
}

$env:PATH = (Join-Path $occtRoot 'bin') + ';' + (Join-Path $qtRoot 'bin') + ';' + $env:PATH
$arguments = @()
if ($StepPath) {
    if (-not (Test-Path -LiteralPath $StepPath -PathType Leaf)) {
        throw "real STEP fixture not found: $StepPath"
    }
    $arguments += $StepPath
    $arguments += $MinimumLengthMm.ToString([System.Globalization.CultureInfo]::InvariantCulture)
}
& $outputExe @arguments
if ($LASTEXITCODE -ne 0) {
    throw "CadSeamCandidateExtractorTests failed: $LASTEXITCODE"
}
