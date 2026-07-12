#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""NoTeaching-Robot OTA 双通道发布工具。

``prepare-dual`` 仅供离线诊断/交付测试，不产生发布权限；外部
``publish-dual --report`` 已永久禁用。唯一可外发入口 ``trusted-release-dual``
在同一父进程内从 clean ``refs/heads/main`` 与 ``refs/heads/hk-pathlynx-corpla``
创建 verifier-owned detached linked worktrees，自行构建/打包/验收唯一候选，全部
通过后才验证 origin 双分支与 GitHub tag/release 前置状态。随后先发布并回读 OTA
双通道，再以同一候选创建 target=main、恰好包含两份安装包的 GitHub Release；
GitHub 资产先隔离在 draft，远端 size/hash 与 main 绑定全部回读通过才公开。
构建失败时不会读取签名密钥、SSH 密码或建立网络连接。

单通道发布、裸 ``payload_hash`` 和无证据的 ``seed_versions`` 已禁用。远端发布先把
两个通道的所有载荷上传到临时名并回读验证，再切换历史、v2 ``latest.json`` 兼容端点
和 v3 ``latest-v3.json`` 当前端点；任一 manifest 切换失败时会回滚已经切换的
manifest/history。
"""

from __future__ import annotations

import argparse
import base64
import contextlib
import ctypes
import datetime as _datetime
import errno
import getpass
import hashlib
import hmac
import importlib.util
import io
import json
import os
import posixpath
import re
import secrets
import shutil
import socket
import stat as stat_module
import subprocess
import sys
import tempfile
import time
import zipfile
from pathlib import Path
from typing import Any, BinaryIO, Iterable


PRODUCTION_HOST = "103.217.203.52"
PRODUCTION_PORT = 48890
PRODUCTION_USER = "root"
REMOTE_ROOT = "/var/www/ota"
EXPECTED_GITHUB_REPOSITORY = "yu1201/NoTeaching-Robot"
PUBLISHED_APP_ID = "A5A7E2A0-8226-40BB-B126-94C5D298B3CF"
REPORT_SCHEMA = "noteaching-ota-dual-candidate-v1"
MANIFEST_SCHEMA_VERSION = 3
LEGACY_SIGNED_MANIFEST_SCHEMA_VERSION = 2
MANIFEST_SIGNATURE_ALGORITHM = "RSA-PKCS1-SHA256"
MANIFEST_VALIDITY_SECONDS = 7 * 24 * 60 * 60
MANIFEST_CLOCK_SKEW_SECONDS = 10 * 60
MANIFEST_RENEWAL_WINDOW_SECONDS = 24 * 60 * 60
LEGACY_MANIFEST_NAME = "latest.json"
CURRENT_MANIFEST_NAME = "latest-v3.json"
CLIENT_OTA_PUBLIC_KEY_FINGERPRINT = "5686a45ad2f6c2d84ba7d911c31cfb4ab972d6afcd9e677c0d9b535629749eda"
MAX_INSTALLER_SIZE_DELTA = 2 * 1024 * 1024
MAX_MANIFEST_JSON_BYTES = 256 * 1024
MAX_REMOTE_JSON_BYTES = 4 * 1024 * 1024
MAX_UPDATE_PAYLOAD_BYTES = 512 * 1024 * 1024
SFTP_IO_TIMEOUT_SECONDS = 30
SFTP_TOTAL_TIMEOUT_SECONDS = 30 * 60
EXPECTED_FANUC_TP_COUNT = 12
EXPECTED_FANUC_PC_COUNT = 9
PUBLISH_ATTESTATION_MAX_AGE_SECONDS = 5 * 60
VERSION_RE = re.compile(r"^[0-9]{4}\.[0-9]{2}\.[0-9]{2}\.[0-9]{4}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
SAFE_REMOTE_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,199}$")
UTC_TIMESTAMP_RE = re.compile(
    r"^[0-9]{4}-(?:0[1-9]|1[0-2])-(?:0[1-9]|[12][0-9]|3[01])T"
    r"(?:[01][0-9]|2[0-3]):[0-5][0-9]:[0-5][0-9]Z$"
)
PAYLOAD_EXCLUDE_FILES = {"build_version.txt", "deploy_notes.txt"}
PAYLOAD_EXCLUDE_TOPDIRS = {"data", "log", "result", "temp"}
CHANNELS = ("neutral", "brand")
CHANNEL_EXE = {
    "neutral": "QtWidgetsApplication4.exe",
    "brand": "HK-Pathlynx-CORPLA.exe",
}

DEFAULT_GIT_EXE = Path(r"C:\Program Files\Git\cmd\git.exe")
DEFAULT_GH_EXE = Path(r"C:\Program Files\GitHub CLI\gh.exe")
DEFAULT_PYTHON_EXE = Path(sys.executable)
DEFAULT_MSBUILD_EXE = Path(
    r"C:\Program Files\Microsoft Visual Studio\2022\Professional\MSBuild\Current\Bin\MSBuild.exe"
)
DEFAULT_WINDEPLOYQT_EXE = Path(
    r"E:\workspace\soft\QT\6.7.3\msvc2022_64\bin\windeployqt.exe"
)
DEFAULT_ISCC_EXE = Path(r"D:\SoftWare\Inno Setup 6\ISCC.exe")
EXPECTED_TOOLCHAIN_CLOSURES: dict[str, dict[str, Any]] = {
    "Qt build/deploy": {
        "sha256": "0396e11d5e2169cefde86fb4dd14f0f4d7f11b3abe4b0127d5920ab4781fc1ec",
        "entries": 10_872,
        "files": 10_084,
        "size": 13_571_041_269,
    },
    "Inno Setup compiler": {
        "sha256": "f9268ed7ec6b349a383d67fcdb23bb826739e899f1ca60f52735731238123e82",
        "entries": 130,
        "files": 122,
        "size": 34_121_271,
    },
}
TRUSTED_RELEASE_FILES = (
    "scripts/upload_ota.py",
    "scripts/ota_manifest_signing.py",
    "scripts/release_gate_common.ps1",
    "scripts/verify_release_pair.ps1",
    "scripts/build_installer.ps1",
    "scripts/build_release_package.ps1",
    "scripts/build_config_migrate.ps1",
)
_SENSITIVE_ENVIRONMENT_NAMES = frozenset({
    "OTA_SSH_PASSWORD",
    "NO_TEACHING_OTA_SSH_PASSWORD",
    "PYTHONHOME",
    "PYTHONPATH",
    "GIT_DIR",
    "GIT_WORK_TREE",
    "GIT_INDEX_FILE",
    "GIT_OBJECT_DIRECTORY",
    "GIT_ALTERNATE_OBJECT_DIRECTORIES",
    "GIT_EXEC_PATH",
    "GIT_CONFIG",
    "GIT_CONFIG_GLOBAL",
    "GIT_CONFIG_SYSTEM",
    "GIT_CONFIG_COUNT",
})
_GITHUB_CREDENTIAL_ENVIRONMENT_NAMES = frozenset({"GH_TOKEN", "GITHUB_TOKEN"})
_BUILD_INJECTION_ENVIRONMENT_NAMES = frozenset({
    "CL", "_CL_", "LINK", "_LINK_", "INCLUDE", "LIB", "LIBPATH", "QTDIR",
    "DEVENVDIR", "VISUALSTUDIOVERSION",
})
_BUILD_INJECTION_ENVIRONMENT_PREFIXES = (
    "QMAKE", "QT_", "MSBUILD", "VS", "VC", "WINDOWSSDK", "UNIVERSALCRT",
    "EXTENSIONSDK", "FRAMEWORK",
)
_CHILD_ENVIRONMENT_ALLOWLIST = frozenset({
    "ALL_PROXY", "APPDATA", "COMMONPROGRAMFILES", "COMMONPROGRAMFILES(X86)",
    "COMMONPROGRAMW6432", "CURL_CA_BUNDLE", "HOME", "HOMEDRIVE", "HOMEPATH",
    "HTTP_PROXY", "HTTPS_PROXY", "LANG", "LC_ALL", "LC_CTYPE", "LOCALAPPDATA",
    "LOGONSERVER", "NO_PROXY", "NUMBER_OF_PROCESSORS", "OS", "PROCESSOR_ARCHITECTURE",
    "PROCESSOR_IDENTIFIER", "PROCESSOR_LEVEL", "PROCESSOR_REVISION", "PROGRAMDATA",
    "PROGRAMFILES", "PROGRAMFILES(X86)", "PROGRAMW6432", "REQUESTS_CA_BUNDLE",
    "SESSIONNAME", "SSL_CERT_DIR", "SSL_CERT_FILE", "SSH_AUTH_SOCK", "SYSTEMDRIVE",
    "TEMP", "TMP", "USERDOMAIN", "USERNAME", "USERPROFILE",
})
_ALLOWED_EXTRA_ENVIRONMENT_NAMES = frozenset({
    "NO_TEACHING_TOOL_PATH", "NO_TEACHING_SECRET_PATH", "NO_TEACHING_ACL_MAX",
})


class ReleaseGateError(RuntimeError):
    """发版硬门禁失败。"""


class _TrustedTool:
    def __init__(self, name: str, path: Path, sha256: str, signer_subject: str) -> None:
        self.name = name
        self.path = path
        self.sha256 = sha256
        self.signer_subject = signer_subject


class _TrustedTreeSnapshot:
    def __init__(
        self,
        *,
        name: str,
        roots: tuple[tuple[str, Path], ...],
        sha256: str,
        entries: int,
        files: int,
        size: int,
        max_entries: int,
        max_bytes: int,
        max_depth: int,
    ) -> None:
        self.name = name
        self.roots = roots
        self.sha256 = sha256
        self.entries = entries
        self.files = files
        self.size = size
        self.max_entries = max_entries
        self.max_bytes = max_bytes
        self.max_depth = max_depth


class _BuiltReleaseContext:
    def __init__(
        self,
        *,
        report: dict[str, Any],
        verifier_root: Path,
        brand_verifier_root: Path,
        main_head: str,
        brand_head: str,
        trusted_file_sha256: dict[str, str],
        git_tool: _TrustedTool,
        gh_tool: _TrustedTool,
        python_tool: _TrustedTool,
        build_tools: tuple[_TrustedTool, ...],
        toolchain_closures: tuple[_TrustedTreeSnapshot, ...],
        protected_tool_roots: tuple[tuple[str, Path], ...],
        python_runtime_root: Path,
        python_runtime_sha256: str,
    ) -> None:
        self.report = report
        self.verifier_root = verifier_root
        self.brand_verifier_root = brand_verifier_root
        self.main_head = main_head
        self.brand_head = brand_head
        self.trusted_file_sha256 = dict(trusted_file_sha256)
        self.git_tool = git_tool
        self.gh_tool = gh_tool
        self.python_tool = python_tool
        self.build_tools = tuple(build_tools)
        self.toolchain_closures = tuple(toolchain_closures)
        self.protected_tool_roots = tuple(protected_tool_roots)
        self.python_runtime_root = python_runtime_root
        self.python_runtime_sha256 = python_runtime_sha256
        self.cleanup_errors: list[str] = []


_ACTIVE_GIT_TOOL: _TrustedTool | None = None
_ACTIVE_GH_TOOL: _TrustedTool | None = None
_ACTIVE_PYTHON_TOOL: _TrustedTool | None = None


_TRUSTED_CANDIDATE_SENTINEL = object()


class _TrustedPublishCandidate(dict[str, Any]):
    """仅能由本进程的新鲜 PowerShell 门禁见证构造的发布能力。"""

    def __init__(
        self,
        report: dict[str, Any],
        attestation: dict[str, Any],
        verified_monotonic: float,
        binding_sha256: str,
        verifier_root: Path,
        trusted_file_sha256: dict[str, str],
        sentinel: object,
    ) -> None:
        if sentinel is not _TRUSTED_CANDIDATE_SENTINEL:
            raise ReleaseGateError("禁止绕过当前受信任验证器构造发布候选。")
        super().__init__(report)
        self._publish_attestation = attestation
        self._verified_monotonic = verified_monotonic
        self._binding_sha256 = binding_sha256
        self._verifier_root = verifier_root
        self._trusted_file_sha256 = dict(trusted_file_sha256)
        self._publish_consumed = False


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ReleaseGateError(message)


def _path_is_reparse_point(path: Path) -> bool:
    try:
        attributes = getattr(os.lstat(path), "st_file_attributes", 0)
    except OSError:
        return True
    reparse_flag = getattr(stat_module, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)
    return bool(attributes & reparse_flag)


def _system_windows_directory() -> Path:
    _require(os.name == "nt", "受信任发布环境只能在 Windows 上构造。")
    buffer = ctypes.create_unicode_buffer(32768)
    length = ctypes.windll.kernel32.GetSystemWindowsDirectoryW(buffer, len(buffer))
    _require(0 < length < len(buffer), "无法通过 Windows API 定位系统目录。")
    try:
        root = Path(buffer.value).resolve(strict=True)
    except OSError as exc:
        raise ReleaseGateError("Windows 系统目录不存在或不可访问。") from exc
    _require(root.is_dir() and not root.is_symlink() and not _path_is_reparse_point(root),
             "Windows 系统目录必须是非链接真实目录。")
    return root


def _sanitized_local_environment(
    extra: dict[str, str] | None = None,
    *,
    allow_github_credentials: bool = False,
) -> dict[str, str]:
    """Construct a minimal allowlisted child environment from scratch."""
    source_environment = {name.upper(): value for name, value in os.environ.items()}
    environment = {
        name: source_environment[name]
        for name in _CHILD_ENVIRONMENT_ALLOWLIST
        if name in source_environment
    }
    github_credentials: dict[str, str] = {}
    for name in _GITHUB_CREDENTIAL_ENVIRONMENT_NAMES:
        if name in source_environment:
            github_credentials[name] = source_environment[name]
    # Git must neither execute environment-selected transports/askpass helpers nor
    # consume user/system URL rewrites. Repository-local origin identity is checked
    # separately and every executable is an absolute, signed, hash-pinned path.
    environment["GIT_TERMINAL_PROMPT"] = "0"
    environment["GCM_INTERACTIVE"] = "Never"
    environment["GIT_CONFIG_NOSYSTEM"] = "1"
    environment["GIT_CONFIG_GLOBAL"] = "NUL" if os.name == "nt" else "/dev/null"
    if allow_github_credentials:
        environment.update(github_credentials)
        environment["GH_HOST"] = "github.com"
        environment["GH_PROMPT_DISABLED"] = "1"
        environment["GH_NO_UPDATE_NOTIFIER"] = "1"
        environment["GH_PAGER"] = ""

    system_root = _system_windows_directory()
    environment["SystemRoot"] = str(system_root)
    environment["WINDIR"] = str(system_root)
    environment["ComSpec"] = str(system_root / "System32" / "cmd.exe")
    environment["PATHEXT"] = ".COM;.EXE;.BAT;.CMD"
    environment["PSModulePath"] = str(
        system_root / "System32" / "WindowsPowerShell" / "v1.0" / "Modules"
    )
    safe_path_entries = [
        system_root / "System32",
        system_root,
        system_root / "System32" / "Wbem",
        system_root / "System32" / "WindowsPowerShell" / "v1.0",
    ]
    if _ACTIVE_GIT_TOOL is not None:
        safe_path_entries.append(_ACTIVE_GIT_TOOL.path.parent)
        environment["NO_TEACHING_RELEASE_GIT_EXE"] = str(_ACTIVE_GIT_TOOL.path)
    else:
        environment.pop("NO_TEACHING_RELEASE_GIT_EXE", None)
    if _ACTIVE_GH_TOOL is not None:
        safe_path_entries.append(_ACTIVE_GH_TOOL.path.parent)
    if _ACTIVE_PYTHON_TOOL is not None:
        safe_path_entries.append(_ACTIVE_PYTHON_TOOL.path.parent)
    environment["PATH"] = os.pathsep.join(
        str(item) for item in safe_path_entries if item.is_dir()
    )
    if extra:
        for name, value in extra.items():
            upper = name.upper()
            _require(upper in _ALLOWED_EXTRA_ENVIRONMENT_NAMES,
                     f"禁止向发布子进程追加未列入白名单的环境变量：{name}")
            environment[name] = value
    return environment


def _resolve_and_verify_trusted_tool(
    path_value: os.PathLike[str] | str,
    *,
    name: str,
    expected_basename: str,
    signer_tokens: tuple[str, ...],
) -> _TrustedTool:
    _require(os.name == "nt", f"{name} 受信工具验证只支持 Windows。")
    candidate = Path(path_value).expanduser()
    _require(candidate.is_absolute(), f"{name} 必须使用显式绝对路径。")
    try:
        path = candidate.resolve(strict=True)
    except OSError as exc:
        raise ReleaseGateError(f"{name} 受信工具不存在或不可访问：{candidate}") from exc
    _require(path.is_file() and not path.is_symlink() and not _path_is_reparse_point(path),
             f"{name} 受信工具必须是非链接普通文件：{path}")
    _require(path.name.casefold() == expected_basename.casefold(),
             f"{name} 受信工具文件名必须是 {expected_basename}。")
    repo_root = Path(__file__).resolve().parents[1]
    try:
        path.relative_to(repo_root)
    except ValueError:
        pass
    else:
        raise ReleaseGateError(f"{name} 受信工具禁止位于项目目录内。")

    powershell = _trusted_windows_powershell()
    script = (
        "$ErrorActionPreference='Stop'; $p=$env:NO_TEACHING_TOOL_PATH; "
        "$s=Get-AuthenticodeSignature -LiteralPath $p; "
        "$subject=if($null -ne $s.SignerCertificate){$s.SignerCertificate.Subject}else{''}; "
        "$o=[ordered]@{status=[string]$s.Status;subject=[string]$subject;"
        "sha256=(Get-FileHash -LiteralPath $p -Algorithm SHA256).Hash.ToLowerInvariant()}; "
        "$o|ConvertTo-Json -Compress"
    )
    try:
        completed = subprocess.run(
            [str(powershell), "-NoLogo", "-NoProfile", "-NonInteractive",
             "-ExecutionPolicy", "Bypass", "-Command", script],
            cwd=str(repo_root),
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=2 * 60,
            check=False,
            env=_sanitized_local_environment({"NO_TEACHING_TOOL_PATH": str(path)}),
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise ReleaseGateError(f"无法验证 {name} Authenticode 来源：{exc}") from exc
    _require(completed.returncode == 0, f"{name} Authenticode 来源验证失败。")
    metadata = _load_json_bytes(completed.stdout.strip().encode("utf-8"), f"{name} 签名元数据")
    _require(isinstance(metadata, dict)
             and metadata.get("status") == "Valid"
             and isinstance(metadata.get("subject"), str)
             and any(token.casefold() in metadata["subject"].casefold() for token in signer_tokens)
             and isinstance(metadata.get("sha256"), str)
             and SHA256_RE.fullmatch(metadata["sha256"]) is not None,
             f"{name} 必须具有受信任发布者的有效 Authenticode 签名。")
    _require(sha256_file(path) == metadata["sha256"], f"{name} 在签名验证期间发生变化。")
    return _TrustedTool(name, path, metadata["sha256"], metadata["subject"])


def _verify_trusted_tool_unchanged(tool: _TrustedTool) -> None:
    _require(tool.path.is_file() and not tool.path.is_symlink()
             and not _path_is_reparse_point(tool.path)
             and sha256_file(tool.path) == tool.sha256,
             f"{tool.name} 在受信验证后发生变化。")


def _assert_windows_tool_tree_not_user_writable(
    name: str,
    root_value: os.PathLike[str] | str,
    *,
    max_entries: int = 25_000,
) -> Path:
    _require(os.name == "nt" and 0 < max_entries <= 100_000,
             f"{name} ACL audit 参数非法。")
    try:
        root = Path(root_value).resolve(strict=True)
    except OSError as exc:
        raise ReleaseGateError(f"{name} ACL audit root 不存在。") from exc
    _require(root.is_dir() and not root.is_symlink() and not _path_is_reparse_point(root),
             f"{name} ACL audit root 必须是非链接目录。")
    script = (
        "$ErrorActionPreference='Stop'; $root=$env:NO_TEACHING_TOOL_PATH; "
        "$limit=[int]$env:NO_TEACHING_ACL_MAX; "
        "$system='S-1-5-18'; $admins='S-1-5-32-544'; "
        "$trusted=([System.Security.Principal.NTAccount]'NT SERVICE\\TrustedInstaller').Translate("
        "[System.Security.Principal.SecurityIdentifier]).Value; "
        "$owners=@($system,$admins,$trusted); $allowed=@($system,$admins,$trusted,'S-1-3-0'); $count=0; $unsafe=@(); "
        "$items=@((Get-Item -LiteralPath $root -Force)); "
        "Get-ChildItem -LiteralPath $root -Force -Recurse | ForEach-Object {"
        "$count++; if($count -gt $limit){throw 'ACL entry limit exceeded'}; $items+=,$_}; "
        "foreach($item in $items){$acl=Get-Acl -LiteralPath $item.FullName; "
        "$owner=$acl.Owner; try{$owner=([System.Security.Principal.NTAccount]$owner).Translate("
        "[System.Security.Principal.SecurityIdentifier]).Value}catch{"
        "$owner=([System.Security.Principal.SecurityIdentifier]$owner).Value}; "
        "if($owners -notcontains $owner){$unsafe+=,[ordered]@{path=$item.FullName;reason='owner';sid=$owner};continue}; "
        "foreach($rule in $acl.Access){$rights=[string]$rule.FileSystemRights; $raw=[int64]$rule.FileSystemRights; "
        "$canWrite=($rights -match 'Write|Modify|FullControl|Delete|ChangePermissions|TakeOwnership|CreateFiles|CreateDirectories|AppendData') "
        "-or (($raw -band 0x40000000) -ne 0) -or (($raw -band 0x10000000) -ne 0); "
        "try{$sid=$rule.IdentityReference.Translate([System.Security.Principal.SecurityIdentifier]).Value}"
        "catch{if($rule.AccessControlType -eq 'Allow' -and $canWrite){"
        "$unsafe+=,[ordered]@{path=$item.FullName;reason='unmapped-write';sid=[string]$rule.IdentityReference}};continue}; "
        "if($rule.AccessControlType -eq 'Allow' -and $canWrite "
        "-and $allowed -notcontains $sid){$unsafe+=,[ordered]@{path=$item.FullName;reason='write';sid=$sid};break}}}; "
        "$o=[ordered]@{entries=$items.Count;unsafe=$unsafe}; $o|ConvertTo-Json -Compress -Depth 5"
    )
    completed = subprocess.run(
        [str(_trusted_windows_powershell()), "-NoLogo", "-NoProfile", "-NonInteractive",
         "-ExecutionPolicy", "Bypass", "-Command", script],
        cwd=str(Path(__file__).resolve().parents[1]),
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=10 * 60,
        check=False,
        env=_sanitized_local_environment({
            "NO_TEACHING_TOOL_PATH": str(root),
            "NO_TEACHING_ACL_MAX": str(max_entries),
        }),
    )
    _require(completed.returncode == 0, f"{name} ACL audit 执行失败。")
    metadata = _load_json_bytes(
        completed.stdout.strip().encode("utf-8"), f"{name} ACL audit metadata"
    )
    _require(isinstance(metadata, dict)
             and set(metadata) == {"entries", "unsafe"}
             and isinstance(metadata.get("entries"), int)
             and 0 < metadata["entries"] <= max_entries
             and metadata.get("unsafe") == [],
             f"{name} 动态依赖目录 owner/ACL 允许普通用户写入。")
    return root


def _snapshot_python_build_runtime(tool: _TrustedTool) -> tuple[Path, str]:
    """Bind the isolated PyInstaller build to the complete immutable Python tree."""
    try:
        root = Path(sys.base_prefix).resolve(strict=True)
        tool.path.relative_to(root)
    except (OSError, ValueError) as exc:
        raise ReleaseGateError("受信 Python 必须位于当前 base runtime 根目录内。") from exc
    _require(root.is_dir() and not root.is_symlink() and not _path_is_reparse_point(root),
             "Python build runtime 根必须是非链接真实目录。")
    digest = hashlib.sha256()
    entries = 0
    total_size = 0
    found_pyinstaller_package = False
    found_pyinstaller_metadata = False
    for directory, child_directories, filenames in os.walk(root, topdown=True):
        directory_path = Path(directory)
        safe_directories: list[str] = []
        for name in sorted(child_directories):
            candidate = directory_path / name
            _require(not candidate.is_symlink() and not _path_is_reparse_point(candidate),
                     f"Python build runtime 含链接目录：{candidate}")
            safe_directories.append(name)
        child_directories[:] = safe_directories
        for name in sorted(filenames):
            candidate = directory_path / name
            info_before = os.lstat(candidate)
            _require(stat_module.S_ISREG(info_before.st_mode)
                     and not candidate.is_symlink()
                     and not _path_is_reparse_point(candidate),
                     f"Python build runtime 含非普通文件：{candidate}")
            relative = candidate.relative_to(root).as_posix()
            lowered = relative.casefold()
            found_pyinstaller_package |= lowered.startswith(
                "lib/site-packages/pyinstaller/"
            )
            found_pyinstaller_metadata |= (
                lowered.startswith("lib/site-packages/pyinstaller-")
                and ".dist-info/" in lowered
            )
            file_sha = sha256_file(candidate)
            info_after = os.lstat(candidate)
            _require(info_before.st_size == info_after.st_size
                     and info_before.st_mtime_ns == info_after.st_mtime_ns,
                     f"Python build runtime 在快照期间变化：{relative}")
            entries += 1
            total_size += info_after.st_size
            _require(entries <= 50_000 and total_size <= 2 * 1024 * 1024 * 1024,
                     "Python build runtime 超过可信快照上限。")
            digest.update(
                f"{relative}\0{info_after.st_size}\0{file_sha}\n".encode("utf-8")
            )
    _require(entries > 0 and found_pyinstaller_package and found_pyinstaller_metadata,
             "Python build runtime 缺少可绑定的 PyInstaller 包或 distribution metadata。")
    return root, digest.hexdigest()


def _verify_and_snapshot_python_build_runtime(tool: _TrustedTool) -> tuple[Path, str]:
    root, before = _snapshot_python_build_runtime(tool)
    probe = (
        "import importlib.metadata,json,pathlib,PyInstaller,sys;"
        "print(json.dumps({'version':importlib.metadata.version('pyinstaller'),"
        "'module':str(pathlib.Path(PyInstaller.__file__).resolve()),"
        "'base':str(pathlib.Path(sys.base_prefix).resolve())}))"
    )
    completed = _run_local_checked(
        [str(tool.path), "-I", "-B", "-c", probe],
        cwd=root,
        label="验证 isolated PyInstaller runtime",
        timeout=2 * 60,
    )
    metadata = _load_json_bytes(
        completed.stdout.strip().encode("utf-8"), "isolated PyInstaller metadata"
    )
    _require(isinstance(metadata, dict)
             and isinstance(metadata.get("version"), str)
             and metadata["version"]
             and _same_local_path(metadata.get("base"), root),
             "isolated PyInstaller runtime identity 非法。")
    try:
        Path(metadata.get("module", "")).resolve(strict=True).relative_to(root)
    except (OSError, ValueError) as exc:
        raise ReleaseGateError("PyInstaller 模块逃逸受信 Python runtime。") from exc
    after_root, after = _snapshot_python_build_runtime(tool)
    _require(after_root == root and after == before,
             "Python/PyInstaller runtime 在 isolated probe 期间发生变化。")
    return root, before


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest().lower()


def sha256_file(path: os.PathLike[str] | str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest().lower()


def _sha256_stream(stream: BinaryIO) -> tuple[str, int]:
    digest = hashlib.sha256()
    size = 0
    for chunk in iter(lambda: stream.read(1 << 20), b""):
        if isinstance(chunk, str):
            chunk = chunk.encode("utf-8")
        digest.update(chunk)
        size += len(chunk)
    return digest.hexdigest().lower(), size


def _snapshot_bounded_tree(
    name: str,
    roots: list[tuple[str, os.PathLike[str] | str]],
    *,
    max_entries: int,
    max_bytes: int,
    max_depth: int = 64,
) -> _TrustedTreeSnapshot:
    """Hash a deterministic, non-link file closure with strict resource bounds."""
    _require(bool(name) and bool(roots)
             and 0 < max_entries <= 100_000
             and 0 < max_bytes <= 32 * 1024 * 1024 * 1024
             and 0 < max_depth <= 128,
             f"{name} closure snapshot 参数非法。")
    canonical_roots: list[tuple[str, Path]] = []
    seen_labels: set[str] = set()
    for label, root_value in roots:
        _require(isinstance(label, str)
                 and re.fullmatch(r"[A-Za-z0-9._/-]+", label) is not None
                 and label not in seen_labels,
                 f"{name} closure root label 非法或重复：{label!r}")
        source = Path(root_value).expanduser()
        _require(source.exists() and not source.is_symlink()
                 and not _path_is_reparse_point(source),
                 f"{name} closure root 缺失或为 link/reparse：{source}")
        try:
            root = source.resolve(strict=True)
        except OSError as exc:
            raise ReleaseGateError(f"{name} closure root 无法解析：{source}") from exc
        seen_labels.add(label)
        canonical_roots.append((label, root))

    inventory: list[str] = []
    entries = 0
    files = 0
    total_size = 0
    stack: list[tuple[str, Path, int, str]] = []
    for label, root in reversed(canonical_roots):
        stack.append((label, root, 0, ""))
    while stack:
        label, candidate, depth, relative = stack.pop()
        _require(depth <= max_depth, f"{name} closure 超过最大深度。")
        try:
            info_before = os.lstat(candidate)
        except OSError as exc:
            raise ReleaseGateError(f"{name} closure 条目无法读取：{candidate}") from exc
        _require(not candidate.is_symlink() and not _path_is_reparse_point(candidate),
                 f"{name} closure 含 link/reparse：{candidate}")
        entries += 1
        _require(entries <= max_entries, f"{name} closure 超过最大条目数。")
        if stat_module.S_ISDIR(info_before.st_mode):
            logical_directory = f"{label}/{relative}" if relative else label
            inventory.append(f"D\0{logical_directory}\n")
            try:
                with os.scandir(candidate) as iterator:
                    children = []
                    for child in iterator:
                        child_relative = f"{relative}/{child.name}" if relative else child.name
                        children.append((label, Path(child.path), depth + 1, child_relative))
                        _require(entries + len(children) <= max_entries,
                                 f"{name} closure 目录枚举超过最大条目数。")
            except OSError as exc:
                raise ReleaseGateError(f"{name} closure 目录无法枚举：{candidate}") from exc
            stack.extend(reversed(sorted(children, key=lambda item: item[3])))
            continue
        _require(stat_module.S_ISREG(info_before.st_mode)
                 and getattr(info_before, "st_nlink", 1) in {0, 1},
                 f"{name} closure 含非普通文件或 hardlink：{candidate}")
        declared_size = int(info_before.st_size)
        _require(declared_size >= 0 and total_size + declared_size <= max_bytes,
                 f"{name} closure 超过最大字节数。")
        digest = hashlib.sha256()
        read_size = 0
        try:
            with open(candidate, "rb") as stream:
                while True:
                    remaining = declared_size - read_size
                    chunk = stream.read(min(1 << 20, remaining + 1))
                    if not chunk:
                        break
                    read_size += len(chunk)
                    _require(read_size <= declared_size,
                             f"{name} closure 文件在读取期间增长：{candidate}")
                    digest.update(chunk)
        except OSError as exc:
            raise ReleaseGateError(f"{name} closure 文件无法读取：{candidate}") from exc
        try:
            info_after = os.lstat(candidate)
        except OSError as exc:
            raise ReleaseGateError(f"{name} closure 文件在读取后消失：{candidate}") from exc
        _require(read_size == declared_size
                 and info_after.st_size == info_before.st_size
                 and info_after.st_mtime_ns == info_before.st_mtime_ns
                 and not candidate.is_symlink()
                 and not _path_is_reparse_point(candidate),
                 f"{name} closure 文件在快照期间变化：{candidate}")
        files += 1
        total_size += declared_size
        logical_path = f"{label}/{relative}" if relative else label
        inventory.append(
            f"{logical_path}\0{declared_size}\0{digest.hexdigest()}\n"
        )
    _require(files > 0, f"{name} closure 不含普通文件。")
    closure_digest = hashlib.sha256()
    for line in sorted(inventory):
        closure_digest.update(line.encode("utf-8"))
    return _TrustedTreeSnapshot(
        name=name,
        roots=tuple(canonical_roots),
        sha256=closure_digest.hexdigest(),
        entries=entries,
        files=files,
        size=total_size,
        max_entries=max_entries,
        max_bytes=max_bytes,
        max_depth=max_depth,
    )


def _verify_bounded_tree_snapshot(snapshot: _TrustedTreeSnapshot) -> None:
    current = _snapshot_bounded_tree(
        snapshot.name,
        list(snapshot.roots),
        max_entries=snapshot.max_entries,
        max_bytes=snapshot.max_bytes,
        max_depth=snapshot.max_depth,
    )
    _require(current.sha256 == snapshot.sha256
             and current.entries == snapshot.entries
             and current.files == snapshot.files
             and current.size == snapshot.size,
             f"{snapshot.name} toolchain closure 在可信快照后发生变化。")


def _snapshot_release_toolchain_closures(
    windeployqt_tool: _TrustedTool,
    iscc_tool: _TrustedTool,
) -> tuple[_TrustedTreeSnapshot, ...]:
    qt_root = windeployqt_tool.path.parent.parent
    qt_library_names = (
        "Qt6Network.lib", "Qt6Sql.lib", "Qt6OpenGL.lib", "Qt6OpenGLWidgets.lib",
    )
    qt_roots: list[tuple[str, Path]] = [
        ("bin", qt_root / "bin"),
        ("plugins", qt_root / "plugins"),
        ("translations", qt_root / "translations"),
        ("include", qt_root / "include"),
        ("mkspecs", qt_root / "mkspecs"),
    ]
    qt_roots.extend(
        (f"lib/{name}", qt_root / "lib" / name) for name in qt_library_names
    )
    qt_snapshot = _snapshot_bounded_tree(
        "Qt build/deploy",
        qt_roots,
        max_entries=20_000,
        max_bytes=20 * 1024 * 1024 * 1024,
    )
    inno_snapshot = _snapshot_bounded_tree(
        "Inno Setup compiler",
        [("install", iscc_tool.path.parent)],
        max_entries=2_000,
        max_bytes=512 * 1024 * 1024,
    )
    return qt_snapshot, inno_snapshot


def _require_expected_toolchain_closures(
    snapshots: tuple[_TrustedTreeSnapshot, ...],
    expectations: dict[str, dict[str, Any]] | None = None,
) -> None:
    expected = EXPECTED_TOOLCHAIN_CLOSURES if expectations is None else expectations
    actual = {snapshot.name: snapshot for snapshot in snapshots}
    _require(set(actual) == set(expected),
             "工具链 closure 集合不等于受信发布代码固定集合。")
    for name, evidence in expected.items():
        _require(isinstance(evidence, dict)
                 and set(evidence) == {"sha256", "entries", "files", "size"}
                 and SHA256_RE.fullmatch(str(evidence.get("sha256", ""))) is not None,
                 f"受信发布代码中的 {name} closure expectation 非法。")
        snapshot = actual[name]
        _require(snapshot.sha256 == evidence["sha256"]
                 and snapshot.entries == evidence["entries"]
                 and snapshot.files == evidence["files"]
                 and snapshot.size == evidence["size"],
                 f"{name} 初始 closure 不匹配受审固定摘要；禁止学习当前磁盘为新基线。")


def _verify_release_toolchain_dependencies(
    tools: tuple[_TrustedTool, ...],
    closures: tuple[_TrustedTreeSnapshot, ...],
    protected_roots: tuple[tuple[str, Path], ...],
) -> None:
    for tool in tools:
        _verify_trusted_tool_unchanged(tool)
    for snapshot in closures:
        _verify_bounded_tree_snapshot(snapshot)
    for name, root in protected_roots:
        _assert_windows_tool_tree_not_user_writable(name, root)


def parse_version(value: str) -> tuple[int, int, int, int]:
    _require(isinstance(value, str) and VERSION_RE.fullmatch(value) is not None,
             f"版本号格式非法：{value!r}；必须是 yyyy.MM.dd.HHmm。")
    year_text, month_text, day_text, hm_text = value.split(".")
    year, month, day = int(year_text), int(month_text), int(day_text)
    hour, minute = int(hm_text[:2]), int(hm_text[2:])
    _require(2000 <= year <= 9999, f"版本年份超出范围：{value}")
    try:
        _datetime.datetime(year, month, day, hour, minute)
    except ValueError as exc:
        raise ReleaseGateError(f"版本号不是有效日期时间：{value} ({exc})") from exc
    return year, month, day, hour * 100 + minute


def compare_versions(lhs: str, rhs: str) -> int:
    left = parse_version(lhs)
    right = parse_version(rhs)
    return (left > right) - (left < right)


def _expected_installer_name(channel: str, version: str) -> str:
    parse_version(version)
    if channel == "neutral":
        return f"NoTeaching-Robot-Setup-v{version}.exe"
    if channel == "brand":
        return f"HK-Pathlynx-CORPLA-Setup-v{version}.exe"
    raise ReleaseGateError(f"未知 OTA 通道：{channel!r}")


def _expected_patch_name(channel: str, version: str) -> str:
    _require(channel == "brand", "中性通道禁止发布增量补丁。")
    parse_version(version)
    return f"HK-Pathlynx-CORPLA-Patch-v{version}.zip"


def _validate_remote_name(name: str) -> None:
    _require(isinstance(name, str) and SAFE_REMOTE_NAME_RE.fullmatch(name) is not None,
             f"远端文件名不安全：{name!r}")
    _require(name not in {".", ".."} and "/" not in name and "\\" not in name,
             f"远端文件名必须是单一文件名：{name!r}")


def _contains_target_version(path: Path, version: str) -> bool:
    target_ascii = version.encode("ascii")
    target_utf16 = version.encode("utf-16le")
    overlap = max(len(target_ascii), len(target_utf16)) - 1
    tail = b""
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            data = tail + chunk
            if target_ascii in data or target_utf16 in data:
                return True
            tail = data[-overlap:] if overlap > 0 else b""
    return False


def _inventory_digest(entries: Iterable[dict[str, Any]]) -> str:
    digest = hashlib.sha256()
    for entry in entries:
        digest.update(
            f"{entry['path']}:{entry['size']}:{entry['sha256']}\n".encode("utf-8")
        )
    return digest.hexdigest().lower()


def inspect_dist(dist_dir: os.PathLike[str] | str, channel: str, version: str) -> dict[str, Any]:
    """校验 dist 并返回完整 inventory 与非 exe 载荷哈希。"""
    parse_version(version)
    _require(channel in CHANNELS, f"未知 OTA 通道：{channel!r}")
    dist = Path(dist_dir).expanduser()
    _require(dist.exists(), f"dist 目录不存在：{dist}")
    _require(dist.is_dir(), f"dist 路径不是目录：{dist}")
    dist = dist.resolve()

    entries: list[dict[str, Any]] = []
    casefold_paths: set[str] = set()
    for path in sorted(dist.rglob("*"), key=lambda item: item.as_posix().casefold()):
        _require(not path.is_symlink(), f"dist 中禁止符号链接：{path}")
        if not path.is_file():
            continue
        relative = path.relative_to(dist).as_posix()
        folded = relative.casefold()
        _require(folded not in casefold_paths, f"dist 中存在大小写冲突路径：{relative}")
        casefold_paths.add(folded)
        size = path.stat().st_size
        entries.append({"path": relative, "size": size, "sha256": sha256_file(path)})

    _require(entries, f"dist 目录为空：{dist}")
    expected_exe = CHANNEL_EXE[channel]
    root_executables = [
        entry["path"] for entry in entries
        if "/" not in entry["path"] and Path(entry["path"]).suffix.casefold() == ".exe"
    ]
    _require(root_executables == [expected_exe],
             f"{channel} dist 根目录必须且只能有 {expected_exe}；实际={root_executables}")
    exe_path = dist / expected_exe
    _require(_contains_target_version(exe_path, version),
             f"主程序未内嵌目标版本 {version}：{exe_path}")
    fanuc_tp = [entry for entry in entries if entry["path"].casefold().startswith("sdk/fanuc/")
                and entry["path"].casefold().endswith(".tp")]
    fanuc_pc = [entry for entry in entries if entry["path"].casefold().startswith("sdk/fanuc/")
                and entry["path"].casefold().endswith(".pc")]
    _require(len(fanuc_tp) == EXPECTED_FANUC_TP_COUNT and len(fanuc_pc) == EXPECTED_FANUC_PC_COUNT,
             f"{channel} dist 的 SDK/FANUC 运行产物必须是 "
             f"tp={EXPECTED_FANUC_TP_COUNT}/pc={EXPECTED_FANUC_PC_COUNT}；"
             f"实际 tp={len(fanuc_tp)} pc={len(fanuc_pc)}。")
    branding_entries = [entry for entry in entries if entry["path"].casefold().startswith("branding/")]
    if channel == "brand":
        _require(any(entry["path"].casefold() == "branding/branding.ini" for entry in branding_entries),
                 "brand dist 缺 branding/branding.ini，运行时会误入 neutral OTA 通道。")
    else:
        _require(not branding_entries, "neutral dist 混入 branding/，会误入 brand OTA 通道。")

    payload_entries: list[dict[str, Any]] = []
    for entry in entries:
        relative = entry["path"]
        folded = relative.casefold()
        top = folded.split("/", 1)[0]
        if folded == expected_exe.casefold():
            continue
        if folded in PAYLOAD_EXCLUDE_FILES or top in PAYLOAD_EXCLUDE_TOPDIRS:
            continue
        payload_entries.append(entry)
    _require(payload_entries,
             f"{channel} dist 除主程序/运行目录外没有可校验载荷，拒绝空载荷哈希。")

    payload_digest = hashlib.sha256()
    for entry in payload_entries:
        payload_digest.update(
            f"{entry['path']}:{entry['sha256']}\n".encode("utf-8")
        )
    exe_entry = next(entry for entry in entries if entry["path"] == expected_exe)
    return {
        "distDir": str(dist),
        "mainExe": expected_exe,
        "mainExeSize": exe_entry["size"],
        "mainExeSha256": exe_entry["sha256"],
        "payloadHash": payload_digest.hexdigest().lower(),
        "payloadFileCount": len(payload_entries),
        "inventoryHash": _inventory_digest(entries),
        "inventory": entries,
    }


def _cross_channel_evidence(channels: dict[str, Any]) -> dict[str, Any]:
    neutral = channels["neutral"]
    brand = channels["brand"]
    neutral_size = neutral["installer"]["size"]
    brand_size = brand["installer"]["size"]
    size_delta = abs(neutral_size - brand_size)
    _require(size_delta <= MAX_INSTALLER_SIZE_DELTA,
             f"中性/品牌安装包体积相差 {size_delta} 字节（超过 2 MiB），疑似漏打运行文件。")

    def comparable_inventory(channel_data: dict[str, Any]) -> list[dict[str, Any]]:
        main_exe = channel_data["dist"]["mainExe"].casefold()
        result = []
        for entry in channel_data["dist"]["inventory"]:
            folded = entry["path"].casefold()
            if folded == main_exe or folded in PAYLOAD_EXCLUDE_FILES:
                continue
            if folded.startswith("branding/") or folded == "icons/app.ico":
                continue
            result.append(entry)
        return result

    neutral_runtime = comparable_inventory(neutral)
    brand_runtime = comparable_inventory(brand)
    _require(neutral_runtime == brand_runtime,
             "中性/品牌 dist 的非品牌运行文件清单或哈希不一致，拒绝发布。")
    return {
        "installerSizeDelta": size_delta,
        "maxInstallerSizeDelta": MAX_INSTALLER_SIZE_DELTA,
        "sharedRuntimeFileCount": len(neutral_runtime),
        "sharedRuntimeInventoryHash": _inventory_digest(neutral_runtime),
    }


def _resolve_gate_file(path_value: Any, label: str) -> Path:
    _require(isinstance(path_value, str) and path_value.strip(), f"{label} 路径缺失。")
    try:
        path = Path(path_value).expanduser().resolve(strict=True)
    except OSError as exc:
        raise ReleaseGateError(f"{label} 不存在或不可访问。") from exc
    _require(path.is_file(), f"{label} 不是文件。")
    return path


def _same_local_path(left: Any, right: Any) -> bool:
    try:
        return os.path.normcase(str(Path(str(left)).expanduser().resolve(strict=True))) == \
            os.path.normcase(str(Path(str(right)).expanduser().resolve(strict=True)))
    except OSError:
        return False


def _normalize_gate_inventory(value: Any, label: str) -> list[dict[str, Any]]:
    _require(isinstance(value, list), f"{label} 必须是文件清单数组。")
    result: list[dict[str, Any]] = []
    seen: set[str] = set()
    for item in value:
        _require(isinstance(item, dict), f"{label} 含非对象条目。")
        path = item.get("path")
        size = item.get("size")
        digest = item.get("sha256")
        _require(isinstance(path, str) and path and "\\" not in path and ":" not in path
                 and not path.startswith("/") and all(part not in {"", ".", ".."} for part in path.split("/")),
                 f"{label} 含非法相对路径：{path!r}")
        key = path.casefold()
        _require(key not in seen, f"{label} 含大小写冲突路径：{path}")
        seen.add(key)
        _require(isinstance(size, int) and not isinstance(size, bool) and size >= 0,
                 f"{label} 中 {path} 的 size 非法。")
        _require(isinstance(digest, str) and SHA256_RE.fullmatch(digest) is not None,
                 f"{label} 中 {path} 的 sha256 非法。")
        result.append({"path": path, "size": size, "sha256": digest})
    return sorted(result, key=lambda item: item["path"].casefold())


def _inspect_pair_gate(
    pair_gate_path_value: os.PathLike[str] | str,
    version: str,
    channels: dict[str, Any],
) -> dict[str, Any]:
    """把 PowerShell package/installer/pair PASS 链绑定到 OTA 候选。"""
    pair_path = _resolve_gate_file(str(pair_gate_path_value), "双通道 pair gate report")
    pair_bytes = pair_path.read_bytes()
    pair = _load_json_bytes(pair_bytes, f"pair gate {pair_path}")
    _require(isinstance(pair, dict), "pair gate 根节点必须是对象。")
    _require(pair.get("schemaVersion") == 1 and pair.get("kind") == "release-pair"
             and pair.get("status") == "pass" and isinstance(pair.get("runId"), str)
             and bool(pair.get("runId").strip()), "pair gate 不是有效 PASS 报告。")
    _require(pair.get("appId") == PUBLISHED_APP_ID, "pair gate AppId 不是已发布固定值。")
    _require(pair.get("version") == version, "pair gate 版本与 OTA 候选不一致。")
    neutral_pair_node = pair.get("neutral")
    _require(isinstance(neutral_pair_node, dict)
             and isinstance(neutral_pair_node.get("repoRoot"), str),
             "pair gate 缺 neutral verifier worktree 根目录。")
    try:
        verifier_root = Path(neutral_pair_node["repoRoot"]).resolve(strict=True)
        pair_path.relative_to(verifier_root)
    except (OSError, ValueError) as exc:
        raise ReleaseGateError("pair gate 必须位于 neutral verifier worktree 内。") from exc
    current_gate_script = verifier_root / "scripts" / "release_gate_common.ps1"
    _require(current_gate_script.is_file(), "缺少当前 release_gate_common.ps1。")
    current_gate_sha = sha256_file(current_gate_script)
    _require(pair.get("gateScriptSha256") == current_gate_sha,
             "pair gate 由不同版本的 release_gate_common.ps1 生成。")
    pair_producer = verifier_root / "scripts" / "verify_release_pair.ps1"
    _require(pair_producer.is_file() and pair.get("producerScriptSha256") == sha256_file(pair_producer),
             "pair gate 由不同版本的 verify_release_pair.ps1 生成。")
    _require(pair.get("maxInstallerSizeDifferenceBytes") == MAX_INSTALLER_SIZE_DELTA,
             "pair gate 的双通道安装包体积阈值不一致。")
    actual_delta = abs(channels["neutral"]["installer"]["size"]
                       - channels["brand"]["installer"]["size"])
    _require(pair.get("installerSizeDifferenceBytes") == actual_delta,
             "pair gate 的安装包体积差证据已失效。")

    pair_fanuc_sha = pair.get("fanucManifestSha256")
    pair_migrator_sha = pair.get("configMigrateSha256")
    _require(isinstance(pair_fanuc_sha, str) and SHA256_RE.fullmatch(pair_fanuc_sha) is not None,
             "pair gate FANUC manifest 哈希非法。")
    _require(isinstance(pair_migrator_sha, str) and SHA256_RE.fullmatch(pair_migrator_sha) is not None,
             "pair gate ConfigMigrate 哈希非法。")

    channel_evidence: dict[str, Any] = {}
    package_reports: dict[str, dict[str, Any]] = {}
    for channel in CHANNELS:
        node = pair.get(channel)
        _require(isinstance(node, dict), f"pair gate 缺 {channel} 节点。")
        head = node.get("head")
        _require(isinstance(head, str) and re.fullmatch(r"[0-9a-f]{40}", head) is not None,
                 f"pair gate {channel}.head 非法。")
        installer_gate_path = _resolve_gate_file(
            node.get("installerGateReport"), f"{channel} installer gate report"
        )
        installer_gate_sha = sha256_file(installer_gate_path)
        _require(node.get("installerGateSha256") == installer_gate_sha,
                 f"{channel} installer gate report 在 pair gate 后发生变化。")
        installer_gate = _load_json_bytes(
            installer_gate_path.read_bytes(), f"{channel} installer gate report"
        )
        _require(isinstance(installer_gate, dict)
                 and installer_gate.get("schemaVersion") == 1
                 and installer_gate.get("kind") == "installer"
                 and installer_gate.get("status") == "pass"
                 and isinstance(installer_gate.get("runId"), str)
                 and bool(installer_gate.get("runId").strip()),
                 f"{channel} installer gate 不是有效 PASS 报告。")
        _require(installer_gate.get("gateScriptSha256") == current_gate_sha,
                 f"{channel} installer gate 脚本哈希已过期。")
        installer_producer = _resolve_gate_file(
            str(Path(str(installer_gate.get("repoRoot"))) / "scripts" / "build_installer.ps1"),
            f"{channel} build_installer.ps1",
        )
        _require(installer_gate.get("producerScriptSha256") == sha256_file(installer_producer),
                 f"{channel} installer gate producer 脚本已变化。")
        _require(installer_gate.get("appId") == PUBLISHED_APP_ID
                 and installer_gate.get("version") == version
                 and installer_gate.get("channel") == channel
                 and installer_gate.get("head") == head,
                 f"{channel} installer gate 身份/version/HEAD 不匹配。")

        installer_node = installer_gate.get("installer")
        candidate_installer = channels[channel]["installer"]
        _require(isinstance(installer_node, dict)
                 and _same_local_path(installer_node.get("path"), candidate_installer["path"])
                 and installer_node.get("name") == candidate_installer["file"]
                 and installer_node.get("size") == candidate_installer["size"]
                 and installer_node.get("sha256") == candidate_installer["sha256"]
                 and installer_node.get("productVersion") == version,
                 f"{channel} installer bytes/version 不再匹配 installer gate。")
        _require(node.get("installerSha256") == candidate_installer["sha256"]
                 and node.get("installerSize") == candidate_installer["size"],
                 f"{channel} installer 不再匹配 pair gate。")

        package_gate_path = _resolve_gate_file(
            installer_gate.get("packageGateReport"), f"{channel} package gate report"
        )
        package_gate_sha = sha256_file(package_gate_path)
        _require(installer_gate.get("packageGateReportSha256") == package_gate_sha,
                 f"{channel} package gate 在 installer gate 后发生变化。")
        package_gate = _load_json_bytes(package_gate_path.read_bytes(), f"{channel} package gate report")
        _require(isinstance(package_gate, dict)
                 and package_gate.get("schemaVersion") == 1
                 and package_gate.get("kind") == "package"
                 and package_gate.get("status") == "pass"
                 and isinstance(package_gate.get("runId"), str)
                 and bool(package_gate.get("runId").strip()),
                 f"{channel} package gate 不是有效 PASS 报告。")
        _require(package_gate.get("gateScriptSha256") == current_gate_sha,
                 f"{channel} package gate 脚本哈希已过期。")
        package_producer = _resolve_gate_file(
            str(Path(str(package_gate.get("repoRoot"))) / "scripts" / "build_release_package.ps1"),
            f"{channel} build_release_package.ps1",
        )
        _require(package_gate.get("producerScriptSha256") == sha256_file(package_producer),
                 f"{channel} package gate producer 脚本已变化。")
        _require(package_gate.get("appId") == PUBLISHED_APP_ID
                 and package_gate.get("version") == version
                 and package_gate.get("channel") == channel
                 and package_gate.get("head") == head,
                 f"{channel} package gate 身份/version/HEAD 不匹配。")
        _require(_same_local_path(package_gate.get("packageDir"), channels[channel]["dist"]["distDir"]),
                 f"{channel} package gate 未绑定当前候选 dist。")
        _require(package_gate.get("expectedExe") == channels[channel]["dist"]["mainExe"]
                 and package_gate.get("executableSha256") == channels[channel]["dist"]["mainExeSha256"],
                 f"{channel} package gate 主程序证据不匹配。")
        gate_inventory = _normalize_gate_inventory(
            package_gate.get("packageInventory"), f"{channel} package inventory"
        )
        candidate_inventory = _normalize_gate_inventory(
            channels[channel]["dist"]["inventory"], f"{channel} candidate inventory"
        )
        _require(gate_inventory == candidate_inventory,
                 f"{channel} dist inventory 在 package gate 后发生变化。")

        fanuc = package_gate.get("fanucManifest")
        migrator = package_gate.get("configMigrate")
        migrate_run = package_gate.get("configMigrateRun")
        _require(isinstance(fanuc, dict) and fanuc.get("sha256") == pair_fanuc_sha
                 and fanuc.get("fileCount") == EXPECTED_FANUC_TP_COUNT + EXPECTED_FANUC_PC_COUNT,
                 f"{channel} FANUC gate 证据不匹配。")
        _require(isinstance(migrator, dict) and migrator.get("sha256") == pair_migrator_sha
                 and isinstance(migrator.get("sourceSha256"), str)
                 and SHA256_RE.fullmatch(migrator["sourceSha256"]) is not None,
                 f"{channel} ConfigMigrate gate 证据不匹配。")
        _require(isinstance(migrate_run, dict)
                 and isinstance(migrate_run.get("sha256"), str)
                 and SHA256_RE.fullmatch(migrate_run["sha256"]) is not None,
                 f"{channel} ConfigMigrate_Run gate 证据不匹配。")
        package_reports[channel] = package_gate
        channel_evidence[channel] = {
            "head": head,
            "installerGateReport": str(installer_gate_path),
            "installerGateSha256": installer_gate_sha,
            "packageGateReport": str(package_gate_path),
            "packageGateSha256": package_gate_sha,
        }

    _require(package_reports["neutral"]["configMigrate"]["sourceSha256"]
             == package_reports["brand"]["configMigrate"]["sourceSha256"],
             "双通道 ConfigMigrate source provenance 不一致。")
    _require(package_reports["neutral"]["configMigrateRun"]["sha256"]
             == package_reports["brand"]["configMigrateRun"]["sha256"],
             "双通道 ConfigMigrate_Run.cmd 不一致。")
    return {
        "path": str(pair_path),
        "verifierRoot": str(verifier_root),
        "sha256": sha256_bytes(pair_bytes),
        "runId": pair["runId"],
        "gateScriptSha256": current_gate_sha,
        "producerScriptSha256": sha256_file(pair_producer),
        "appId": PUBLISHED_APP_ID,
        "version": version,
        "neutral": channel_evidence["neutral"],
        "brand": channel_evidence["brand"],
    }


def _inspect_installer(path_value: os.PathLike[str] | str, channel: str, version: str) -> dict[str, Any]:
    path = Path(path_value).expanduser()
    expected_name = _expected_installer_name(channel, version)
    _require(path.exists() and path.is_file(), f"找不到安装包：{path}")
    _require(not path.is_symlink(), f"安装包禁止符号链接：{path}")
    _require(path.name == expected_name,
             f"{channel} 安装包命名错误：期望 {expected_name}，实际 {path.name}")
    _validate_remote_name(path.name)
    path = path.resolve()
    size = path.stat().st_size
    _require(0 < size <= MAX_UPDATE_PAYLOAD_BYTES,
             f"安装包大小必须在 1..512 MiB：{path} ({size} bytes)")
    return {"path": str(path), "file": path.name, "size": size, "sha256": sha256_file(path)}


def _zip_member_is_symlink(info: zipfile.ZipInfo) -> bool:
    mode = (info.external_attr >> 16) & 0xFFFF
    return stat_module.S_ISLNK(mode)


def _inspect_patch(
    path_value: os.PathLike[str] | str,
    channel: str,
    version: str,
    dist_info: dict[str, Any],
) -> dict[str, Any]:
    path = Path(path_value).expanduser()
    expected_name = _expected_patch_name(channel, version)
    _require(path.exists() and path.is_file(), f"找不到增量补丁：{path}")
    _require(not path.is_symlink(), f"增量补丁禁止符号链接：{path}")
    _require(path.name == expected_name,
             f"增量补丁命名错误：期望 {expected_name}，实际 {path.name}")
    _validate_remote_name(path.name)
    path = path.resolve()
    _require(0 < path.stat().st_size <= MAX_UPDATE_PAYLOAD_BYTES,
             f"增量补丁大小必须在 1..512 MiB：{path} ({path.stat().st_size} bytes)")

    try:
        with zipfile.ZipFile(path, "r") as archive:
            members = archive.infolist()
            _require(len(members) == 1, f"增量补丁必须只含一个主程序；实际条目数={len(members)}")
            member = members[0]
            expected_exe = CHANNEL_EXE[channel]
            _require(not member.is_dir() and member.filename == expected_exe,
                     f"增量补丁只能包含根目录 {expected_exe}；实际={member.filename!r}")
            _require(not _zip_member_is_symlink(member), "增量补丁主程序不能是符号链接。")
            _require((member.flag_bits & 0x1) == 0, "增量补丁不能使用加密 ZIP 条目。")
            _require(member.file_size == dist_info["mainExeSize"],
                     "增量补丁内主程序大小与候选 dist 不一致。")
            with archive.open(member, "r") as stream:
                member_sha, member_size = _sha256_stream(stream)
    except (OSError, zipfile.BadZipFile, RuntimeError) as exc:
        if isinstance(exc, ReleaseGateError):
            raise
        raise ReleaseGateError(f"无法校验增量补丁 {path}: {exc}") from exc

    _require(member_size == dist_info["mainExeSize"], "增量补丁解压大小与候选主程序不一致。")
    _require(member_sha == dist_info["mainExeSha256"],
             "增量补丁内主程序哈希与候选 dist 主程序不一致。")
    return {
        "path": str(path),
        "file": path.name,
        "size": path.stat().st_size,
        "sha256": sha256_file(path),
        "embeddedExe": CHANNEL_EXE[channel],
        "embeddedExeSize": member_size,
        "embeddedExeSha256": member_sha,
    }


def _validate_notes(notes: str) -> str:
    _require(isinstance(notes, str), "更新说明必须是字符串。")
    _require(len(notes) <= 4000, "更新说明不能超过 4000 个字符。")
    _require(all(ch == "\n" or ch == "\r" or ord(ch) >= 0x20 for ch in notes),
             "更新说明包含非法控制字符。")
    return notes


def _candidate_digest(report_without_digest: dict[str, Any]) -> str:
    encoded = json.dumps(
        report_without_digest, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return sha256_bytes(encoded)


def prepare_dual_candidate(
    version: str,
    neutral_dist_dir: os.PathLike[str] | str,
    neutral_installer: os.PathLike[str] | str,
    brand_dist_dir: os.PathLike[str] | str,
    brand_installer: os.PathLike[str] | str,
    brand_patch: os.PathLike[str] | str,
    pair_gate_report: os.PathLike[str] | str,
    notes: str = "",
) -> dict[str, Any]:
    parse_version(version)
    notes = _validate_notes(notes)
    channels: dict[str, Any] = {}
    for channel, dist_dir, installer_path in (
        ("neutral", neutral_dist_dir, neutral_installer),
        ("brand", brand_dist_dir, brand_installer),
    ):
        dist_info = inspect_dist(dist_dir, channel, version)
        channel_info: dict[str, Any] = {
            "channel": channel,
            "dist": dist_info,
            "installer": _inspect_installer(installer_path, channel, version),
            "patch": None,
        }
        if channel == "brand":
            channel_info["patch"] = _inspect_patch(brand_patch, channel, version, dist_info)
        channels[channel] = channel_info

    report: dict[str, Any] = {
        "schema": REPORT_SCHEMA,
        "version": version,
        "notes": notes,
        "generatedAtUtc": _datetime.datetime.now(_datetime.timezone.utc).isoformat(),
        "channels": channels,
        "crossChannel": _cross_channel_evidence(channels),
        "pairGate": _inspect_pair_gate(pair_gate_report, version, channels),
    }
    digest_source = {key: value for key, value in report.items() if key != "generatedAtUtc"}
    report["candidateSha256"] = _candidate_digest(digest_source)
    return report


def _write_json_atomic(path_value: os.PathLike[str] | str, value: dict[str, Any], overwrite: bool) -> Path:
    path = Path(path_value).expanduser().resolve()
    _require(overwrite or not path.exists(), f"候选报告已存在，拒绝覆盖：{path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    data = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8")
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, path)
    except Exception:
        try:
            os.unlink(temp_name)
        except OSError:
            pass
        raise
    return path


def _reject_duplicate_json_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ReleaseGateError(f"JSON 含重复字段：{key}")
        result[key] = value
    return result


def _load_json_bytes(data: bytes, label: str) -> Any:
    try:
        return json.loads(data.decode("utf-8"), object_pairs_hook=_reject_duplicate_json_keys)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ReleaseGateError(f"{label} 不是有效 UTF-8 JSON：{exc}") from exc


def _revalidate_loaded_report(report: Any) -> dict[str, Any]:
    _require(isinstance(report, dict), "候选报告根节点必须是对象。")
    _require(report.get("schema") == REPORT_SCHEMA, "候选报告 schema 不受支持。")
    version = report.get("version")
    parse_version(version)
    notes = _validate_notes(report.get("notes"))
    channels = report.get("channels")
    _require(isinstance(channels, dict) and set(channels) == set(CHANNELS),
             "候选报告必须同时且仅包含 neutral/brand。")
    supplied_digest = report.get("candidateSha256")
    _require(isinstance(supplied_digest, str) and SHA256_RE.fullmatch(supplied_digest),
             "候选报告 candidateSha256 非法。")
    digest_source = {
        key: value for key, value in report.items()
        if key not in {"generatedAtUtc", "candidateSha256"}
    }
    _require(_candidate_digest(digest_source) == supplied_digest, "候选报告内容摘要不匹配。")

    recomputed: dict[str, Any] = {}
    for channel in CHANNELS:
        channel_data = channels[channel]
        _require(isinstance(channel_data, dict), f"{channel} 候选节点非法。")
        dist_data = channel_data.get("dist")
        installer_data = channel_data.get("installer")
        _require(isinstance(dist_data, dict) and isinstance(installer_data, dict),
                 f"{channel} 候选缺 dist/installer。")
        current_dist = inspect_dist(dist_data.get("distDir"), channel, version)
        current_installer = _inspect_installer(installer_data.get("path"), channel, version)
        current_patch = None
        if channel == "brand":
            patch_data = channel_data.get("patch")
            _require(isinstance(patch_data, dict), "brand 候选必须包含单 exe 增量补丁。")
            current_patch = _inspect_patch(patch_data.get("path"), channel, version, current_dist)
        else:
            _require(channel_data.get("patch") is None, "neutral 候选禁止补丁。")
        recomputed[channel] = {
            "channel": channel,
            "dist": current_dist,
            "installer": current_installer,
            "patch": current_patch,
        }
        _require(recomputed[channel] == channel_data,
                 f"{channel} 候选文件在 prepare 后发生变化，拒绝发布。")

    cross_channel = _cross_channel_evidence(recomputed)
    _require(report.get("crossChannel") == cross_channel,
             "候选报告的双通道体积/运行文件对照证据不一致。")
    pair_gate_node = report.get("pairGate")
    _require(isinstance(pair_gate_node, dict), "候选报告缺 pairGate 证据。")
    pair_gate = _inspect_pair_gate(pair_gate_node.get("path"), version, recomputed)
    _require(pair_gate_node == pair_gate, "pair gate 或其引用报告在 prepare 后发生变化。")

    return {
        "schema": REPORT_SCHEMA,
        "version": version,
        "notes": notes,
        "channels": recomputed,
        "crossChannel": cross_channel,
        "pairGate": pair_gate,
        "candidateSha256": supplied_digest,
    }


def load_and_revalidate_report(path_value: os.PathLike[str] | str) -> dict[str, Any]:
    path = Path(path_value).expanduser()
    _require(path.exists() and path.is_file(), f"候选报告不存在：{path}")
    report = _load_json_bytes(path.read_bytes(), f"候选报告 {path}")
    return _revalidate_loaded_report(report)


def _trusted_windows_powershell() -> Path:
    _require(os.name == "nt", "受信任发布门禁必须在 Windows 上通过系统 PowerShell 执行。")
    try:
        system_root = _system_windows_directory()
        powershell = (
            system_root / "System32" / "WindowsPowerShell" / "v1.0" / "powershell.exe"
        ).resolve(strict=True)
    except OSError as exc:
        raise ReleaseGateError("无法定位系统 PowerShell。") from exc
    _require(powershell.is_file(), f"系统 PowerShell 不存在：{powershell}")
    return powershell


def _validate_fresh_publish_attestation(
    report: dict[str, Any],
    attestation: Any,
    challenge: str,
    verifier_root: Path,
    trusted_file_sha256: dict[str, str],
) -> dict[str, Any]:
    _require(isinstance(attestation, dict), "受信任发布见证根节点必须是对象。")
    expected_top = {
        "schemaVersion", "kind", "status", "challenge", "candidateSha256",
        "generatedAtUtc", "verifierProcessId", "verifierScriptSha256",
        "gateScriptSha256", "appId", "version", "pairGate", "neutral", "brand",
    }
    _require(set(attestation) == expected_top, "受信任发布见证字段集与当前 schema 不一致。")
    _require(attestation.get("schemaVersion") == 1
             and attestation.get("kind") == "publish-attestation"
             and attestation.get("status") == "pass",
             "受信任发布见证不是受支持的 PASS 证据。")
    _require(isinstance(attestation.get("challenge"), str)
             and hmac.compare_digest(attestation["challenge"], challenge),
             "受信任发布见证 challenge 不匹配，拒绝预制/重放证据。")
    _require(isinstance(attestation.get("verifierProcessId"), int)
             and not isinstance(attestation["verifierProcessId"], bool)
             and attestation["verifierProcessId"] > 0,
             "受信任发布见证缺有效验证进程 ID。")

    generated_text = attestation.get("generatedAtUtc")
    _require(isinstance(generated_text, str), "受信任发布见证缺 generatedAtUtc。")
    try:
        generated = _datetime.datetime.fromisoformat(generated_text.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ReleaseGateError("受信任发布见证时间格式非法。") from exc
    _require(generated.tzinfo is not None, "受信任发布见证时间必须带时区。")
    age_seconds = abs((
        _datetime.datetime.now(_datetime.timezone.utc) - generated.astimezone(_datetime.timezone.utc)
    ).total_seconds())
    _require(age_seconds <= PUBLISH_ATTESTATION_MAX_AGE_SECONDS,
             "受信任发布见证已过期或本机时钟异常。")

    pair_evidence = report.get("pairGate")
    _require(isinstance(pair_evidence, dict), "候选报告缺 pairGate 绑定。")
    current_verifier = verifier_root / "scripts" / "verify_release_pair.ps1"
    current_common = verifier_root / "scripts" / "release_gate_common.ps1"
    _require(attestation.get("verifierScriptSha256")
             == trusted_file_sha256.get("scripts/verify_release_pair.ps1")
             == sha256_file(current_verifier)
             and attestation.get("gateScriptSha256")
             == trusted_file_sha256.get("scripts/release_gate_common.ps1")
             == sha256_file(current_common),
             "受信任发布见证不是由当前验证代码产生。")
    _require(attestation.get("appId") == PUBLISHED_APP_ID
             and attestation.get("version") == report.get("version")
             and attestation.get("candidateSha256") == report.get("candidateSha256"),
             "受信任发布见证未绑定本次候选/version/AppId。")

    pair_node = attestation.get("pairGate")
    _require(isinstance(pair_node, dict)
             and set(pair_node) == {"path", "sha256", "runId"}
             and _same_local_path(pair_node.get("path"), pair_evidence.get("path"))
             and pair_node.get("sha256") == pair_evidence.get("sha256")
             and pair_node.get("runId") == pair_evidence.get("runId"),
             "受信任发布见证未绑定当前 release-pair 字节。")

    expected_channel_fields = {
        "head", "installerGateReport", "installerGateSha256", "packageGateReport",
        "packageGateSha256", "installerSha256", "installerSize",
    }
    channels = report.get("channels")
    _require(isinstance(channels, dict), "候选报告缺双通道。")
    for channel in CHANNELS:
        attested = attestation.get(channel)
        pair_channel = pair_evidence.get(channel)
        candidate_channel = channels.get(channel)
        _require(isinstance(attested, dict) and set(attested) == expected_channel_fields,
                 f"受信任发布见证 {channel} 节点 schema 非法。")
        _require(isinstance(pair_channel, dict) and isinstance(candidate_channel, dict),
                 f"候选报告 {channel} 绑定缺失。")
        installer = candidate_channel.get("installer")
        _require(isinstance(installer, dict)
                 and attested.get("head") == pair_channel.get("head")
                 and _same_local_path(attested.get("installerGateReport"),
                                      pair_channel.get("installerGateReport"))
                 and attested.get("installerGateSha256") == pair_channel.get("installerGateSha256")
                 and _same_local_path(attested.get("packageGateReport"),
                                      pair_channel.get("packageGateReport"))
                 and attested.get("packageGateSha256") == pair_channel.get("packageGateSha256")
                 and attested.get("installerSha256") == installer.get("sha256")
                 and attested.get("installerSize") == installer.get("size"),
                 f"受信任发布见证未绑定当前 {channel} 候选。")
    return attestation


def trust_candidate_for_publish(
    report: dict[str, Any],
    *,
    verifier_root: os.PathLike[str] | str | None = None,
    trusted_file_sha256: dict[str, str] | None = None,
    _runner: Any = None,
) -> _TrustedPublishCandidate:
    """用当前 PowerShell 门禁对 live 候选做一次 challenge-bound 复验。

    ``_runner`` 只用于无网单元测试；CLI 不暴露任何替代验证器入口。
    """
    normalized = _revalidate_loaded_report(report)
    pair_evidence = normalized["pairGate"]
    root_value = pair_evidence.get("verifierRoot") if verifier_root is None else verifier_root
    _require(isinstance(root_value, (str, os.PathLike)), "候选缺 verifier-owned 根目录。")
    try:
        trusted_root = Path(root_value).resolve(strict=True)
    except OSError as exc:
        raise ReleaseGateError("verifier-owned 根目录不存在或不可访问。") from exc
    if trusted_file_sha256 is None:
        trusted_file_sha256 = {
            relative: sha256_file(trusted_root / Path(relative))
            for relative in TRUSTED_RELEASE_FILES
        }
    _require(set(trusted_file_sha256) == set(TRUSTED_RELEASE_FILES),
             "受信任发布文件哈希集合不完整。")
    for relative, expected_sha in trusted_file_sha256.items():
        _require(SHA256_RE.fullmatch(expected_sha) is not None
                 and sha256_file(trusted_root / Path(relative)) == expected_sha,
                 f"受信任发布文件已漂移：{relative}")
    challenge = secrets.token_hex(32)
    verifier = trusted_root / "scripts" / "verify_release_pair.ps1"
    _require(verifier.is_file(), "缺少当前 verify_release_pair.ps1。")
    powershell = _trusted_windows_powershell()
    runner = subprocess.run if _runner is None else _runner
    if _ACTIVE_PYTHON_TOOL is None:
        if _runner is None:
            fallback_python = _resolve_and_verify_trusted_tool(
                DEFAULT_PYTHON_EXE,
                name="Python runtime",
                expected_basename="python.exe",
                signer_tokens=("Python Software Foundation",),
            )
            python_path = fallback_python.path
            python_sha256 = fallback_python.sha256
        else:
            python_path = DEFAULT_PYTHON_EXE.resolve(strict=True)
            python_sha256 = sha256_file(python_path)
    else:
        _verify_trusted_tool_unchanged(_ACTIVE_PYTHON_TOOL)
        python_path = _ACTIVE_PYTHON_TOOL.path
        python_sha256 = _ACTIVE_PYTHON_TOOL.sha256

    with tempfile.TemporaryDirectory(prefix="noteaching-publish-attest-") as temp_dir:
        attestation_path = Path(temp_dir) / "fresh-publish-attestation.json"
        command = [
            str(powershell),
            "-NoLogo",
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(verifier),
            "-AttestExistingPair",
            str(pair_evidence["path"]),
            "-PublishChallenge",
            challenge,
            "-ExpectedCandidateSha256",
            str(normalized["candidateSha256"]),
            "-ExpectedPairGateSha256",
            str(pair_evidence["sha256"]),
            "-AttestationOutputPath",
            str(attestation_path),
            "-PythonExecutable",
            str(python_path),
            "-PythonSha256",
            python_sha256,
        ]
        try:
            completed = runner(
                command,
                cwd=str(Path(__file__).resolve().parents[1]),
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=15 * 60,
                check=False,
                stdin=subprocess.DEVNULL,
                env=_sanitized_local_environment(),
            )
        except (OSError, subprocess.SubprocessError) as exc:
            raise ReleaseGateError(f"无法启动当前受信任发布验证器：{exc}") from exc
        if completed.returncode != 0:
            detail = "\n".join(
                item.strip() for item in (completed.stdout, completed.stderr)
                if isinstance(item, str) and item.strip()
            )
            if len(detail) > 4000:
                detail = detail[-4000:]
            raise ReleaseGateError(
                "当前受信任发布验证器拒绝候选；"
                "package/installer/pair PASS JSON 不能代替 live 复验。"
                + (f"\n{detail}" if detail else "")
            )
        _require(attestation_path.is_file(),
                 "受信任发布验证器未产生一次性见证。")
        attestation = _load_json_bytes(
            attestation_path.read_bytes(), "受信任发布见证"
        )
        validated = _validate_fresh_publish_attestation(
            normalized, attestation, challenge, trusted_root, trusted_file_sha256
        )

    verified_monotonic = time.monotonic()
    binding_sha256 = sha256_bytes(_json_bytes(normalized))
    return _TrustedPublishCandidate(
        normalized,
        validated,
        verified_monotonic,
        binding_sha256,
        trusted_root,
        trusted_file_sha256,
        _TRUSTED_CANDIDATE_SENTINEL,
    )


def load_and_trust_publish_report(
    path_value: os.PathLike[str] | str,
    *,
    _runner: Any = None,
) -> _TrustedPublishCandidate:
    return trust_candidate_for_publish(load_and_revalidate_report(path_value), _runner=_runner)


def _require_trusted_publish_candidate(report: dict[str, Any]) -> None:
    _require(isinstance(report, _TrustedPublishCandidate),
             "发布拒绝普通/手写候选 JSON；必须先由当前受信任验证器新鲜复验。")
    _require(not report._publish_consumed,
             "受信任发布见证已使用；每次发布/重试都必须重新执行门禁。")
    _require(time.monotonic() - report._verified_monotonic
             <= PUBLISH_ATTESTATION_MAX_AGE_SECONDS,
             "受信任发布见证已超过 5 分钟，必须重新执行门禁。")
    _require(sha256_bytes(_json_bytes(dict(report))) == report._binding_sha256,
             "候选内存对象在受信任复验后发生变化。")
    normalized = _revalidate_loaded_report(dict(report))
    _require(normalized == dict(report), "候选文件在受信任复验后发生变化。")
    for relative, expected_sha in report._trusted_file_sha256.items():
        _require(sha256_file(report._verifier_root / Path(relative)) == expected_sha,
                 f"受信任验证代码在复验后发生变化：{relative}")
    _require(report._publish_attestation.get("verifierScriptSha256")
             == report._trusted_file_sha256["scripts/verify_release_pair.ps1"]
             and report._publish_attestation.get("gateScriptSha256")
             == report._trusted_file_sha256["scripts/release_gate_common.ps1"],
             "受信任验证代码在复验后发生变化，拒绝发布。")
    report._publish_consumed = True


def compute_base_min_version(history: dict[str, str], current_version: str, current_hash: str) -> tuple[str, dict[str, str]]:
    parse_version(current_version)
    _require(SHA256_RE.fullmatch(current_hash) is not None, "当前 payloadHash 非法。")
    normalized: dict[str, str] = {}
    for version, payload_digest in history.items():
        parse_version(version)
        _require(isinstance(payload_digest, str) and SHA256_RE.fullmatch(payload_digest) is not None,
                 f"payload_history 中 {version} 的哈希非法。")
        normalized[version] = payload_digest

    if current_version in normalized:
        _require(normalized[current_version] == current_hash,
                 f"远端历史中版本 {current_version} 已绑定不同 payloadHash，禁止改写同版本载荷。")
    if normalized:
        highest = max(normalized, key=parse_version)
        _require(compare_versions(current_version, highest) >= 0,
                 f"目标版本 {current_version} 低于远端历史最高版本 {highest}。")
    normalized[current_version] = current_hash
    ordered_versions = sorted(normalized, key=parse_version)
    index = ordered_versions.index(current_version)
    base = current_version
    for pos in range(index, -1, -1):
        version = ordered_versions[pos]
        if normalized[version] != current_hash:
            break
        base = version
    return base, normalized


def _json_bytes(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _load_signing_module(
    module_path_value: os.PathLike[str] | str | None = None,
    *,
    expected_sha256: str | None = None,
) -> Any:
    module_path = (
        Path(__file__).resolve().with_name("ota_manifest_signing.py")
        if module_path_value is None else Path(module_path_value).resolve(strict=True)
    )
    _require(module_path.is_file(), "缺少 OTA manifest 签名模块。")
    if expected_sha256 is not None:
        _require(SHA256_RE.fullmatch(expected_sha256) is not None
                 and sha256_file(module_path) == expected_sha256,
                 "OTA manifest 签名模块不再匹配初始 main HEAD 字节。")
    spec = importlib.util.spec_from_file_location("noteaching_ota_manifest_signing", module_path)
    _require(spec is not None and spec.loader is not None, "无法加载 OTA manifest 签名模块。")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _require(module.SCHEMA_VERSION == MANIFEST_SCHEMA_VERSION
             and module.LEGACY_SIGNED_SCHEMA_VERSION == LEGACY_SIGNED_MANIFEST_SCHEMA_VERSION
             and module.SIGNATURE_ALGORITHM == MANIFEST_SIGNATURE_ALGORITHM
             and module.MANIFEST_VALIDITY_SECONDS == MANIFEST_VALIDITY_SECONDS
             and module.MANIFEST_CLOCK_SKEW_SECONDS == MANIFEST_CLOCK_SKEW_SECONDS,
             "OTA manifest 签名模块的 schema/时间合约/算法与发布工具不一致。")
    return module


def _resolve_signing_key(path_value: os.PathLike[str] | str) -> Path:
    try:
        path = Path(path_value).expanduser().resolve(strict=True)
    except OSError as exc:
        raise ReleaseGateError("OTA 签名密钥不存在或不可访问。") from exc
    _require(path.is_file(), "OTA 签名密钥路径不是文件。")
    repo_root = Path(__file__).resolve().parents[1]
    try:
        path.relative_to(repo_root)
    except ValueError:
        return path
    raise ReleaseGateError("OTA 签名私钥必须位于项目目录之外。")


def make_manifest_signer(
    key_path_value: os.PathLike[str] | str,
    *,
    signing_module_path: os.PathLike[str] | str | None = None,
    expected_module_sha256: str | None = None,
):
    """返回只输出签名文本的闭包；私钥内容不会进入 report/日志。"""
    key_path = _resolve_signing_key(key_path_value)
    signing = _load_signing_module(
        signing_module_path, expected_sha256=expected_module_sha256
    )
    try:
        public_key = signing.load_private_key(key_path).public_key()
    except Exception as exc:
        raise ReleaseGateError(f"无法加载 OTA manifest 签名密钥：{exc}") from exc
    actual_public_fingerprint = sha256_bytes(signing.cng_public_blob(public_key))
    _require(hmac.compare_digest(actual_public_fingerprint, CLIENT_OTA_PUBLIC_KEY_FINGERPRINT),
             "OTA 签名私钥与客户端内置公钥不匹配；已在联网前拒绝。")

    def signer(manifest: dict[str, Any]) -> str:
        try:
            signature = signing.sign_manifest(manifest, key_path)
            _require(signing.verify_manifest(manifest, signature, public_key),
                     "OTA manifest 本地签名自检失败。")
            return signature
        except ReleaseGateError:
            raise
        except Exception as exc:
            raise ReleaseGateError(f"OTA manifest 签名失败：{exc}") from exc

    def verify(manifest: dict[str, Any]) -> bool:
        return signing.verify_manifest(manifest, manifest.get("signature", ""), public_key)

    def sign_legacy(manifest: dict[str, Any]) -> str:
        try:
            signature = signing.sign_legacy_manifest_v2(manifest, key_path)
            _require(signing.verify_manifest(manifest, signature, public_key),
                     "OTA v2 compatibility manifest 本地签名自检失败。")
            return signature
        except ReleaseGateError:
            raise
        except Exception as exc:
            raise ReleaseGateError(f"OTA v2 compatibility manifest 签名失败：{exc}") from exc

    signer.verify = verify  # type: ignore[attr-defined]
    signer.sign_legacy = sign_legacy  # type: ignore[attr-defined]
    return signer


def _is_missing_error(exc: BaseException) -> bool:
    return isinstance(exc, (FileNotFoundError,)) or getattr(exc, "errno", None) == errno.ENOENT


def _read_remote_bytes(
    sftp: Any,
    remote_path: str,
    *,
    missing_ok: bool = False,
    max_bytes: int = MAX_REMOTE_JSON_BYTES,
) -> bytes | None:
    remote_stat = _remote_stat(sftp, remote_path)
    if remote_stat is None:
        if missing_ok:
            return None
        raise ReleaseGateError(f"远端文件不存在：{remote_path}")
    _require(0 < int(remote_stat.st_size) <= max_bytes,
             f"远端 JSON 大小非法：{remote_path} ({remote_stat.st_size} bytes)")
    try:
        with sftp.open(remote_path, "rb") as stream:
            data = stream.read(max_bytes + 1)
            result = data.encode("utf-8") if isinstance(data, str) else data
            _require(len(result) <= max_bytes,
                     f"远端 JSON 在 stat 后增长并超过上限：{remote_path}")
            final_stat = _remote_stat(sftp, remote_path)
            _require(final_stat is not None
                     and len(result) == int(remote_stat.st_size)
                     and len(result) == int(final_stat.st_size),
                     f"远端 JSON 回读大小变化：{remote_path}")
            return result
    except OSError as exc:
        raise ReleaseGateError(f"读取远端文件失败：{remote_path}: {exc}") from exc


def _remote_stat(sftp: Any, remote_path: str) -> Any | None:
    try:
        return sftp.stat(remote_path)
    except OSError as exc:
        if _is_missing_error(exc):
            return None
        raise ReleaseGateError(f"读取远端状态失败：{remote_path}: {exc}") from exc


def _remote_sha_size(
    sftp: Any,
    remote_path: str,
    *,
    expected_size: int | None = None,
    max_bytes: int = MAX_UPDATE_PAYLOAD_BYTES,
) -> tuple[str, int]:
    _require(isinstance(max_bytes, int) and 0 < max_bytes <= MAX_UPDATE_PAYLOAD_BYTES,
             "远端回读大小上限非法。")
    if expected_size is not None:
        _require(isinstance(expected_size, int) and 0 < expected_size <= max_bytes,
                 "远端 expected_size 非法。")
    initial_stat = _remote_stat(sftp, remote_path)
    _require(initial_stat is not None
             and 0 < int(initial_stat.st_size) <= max_bytes
             and (expected_size is None or int(initial_stat.st_size) == expected_size),
             f"远端文件 stat 大小超限或不匹配：{remote_path}")
    try:
        with sftp.open(remote_path, "rb") as stream:
            digest = hashlib.sha256()
            size = 0
            while True:
                remaining = max_bytes - size
                chunk = stream.read(min(1 << 20, remaining + 1))
                if isinstance(chunk, str):
                    chunk = chunk.encode("utf-8")
                if not chunk:
                    break
                size += len(chunk)
                _require(size <= max_bytes
                         and (expected_size is None or size <= expected_size),
                         f"远端文件在 stat 后增长或超过上限：{remote_path}")
                digest.update(chunk)
    except OSError as exc:
        raise ReleaseGateError(f"回读远端文件失败：{remote_path}: {exc}") from exc
    remote_stat = _remote_stat(sftp, remote_path)
    _require(remote_stat is not None
             and int(initial_stat.st_size) == size
             and int(remote_stat.st_size) == size
             and (expected_size is None or size == expected_size),
             f"远端 stat/回读大小不一致：{remote_path}")
    return digest.hexdigest(), size


def _verify_remote(sftp: Any, remote_path: str, expected_sha: str, expected_size: int) -> None:
    actual_sha, actual_size = _remote_sha_size(
        sftp,
        remote_path,
        expected_size=expected_size,
        max_bytes=MAX_UPDATE_PAYLOAD_BYTES,
    )
    _require(actual_size == expected_size,
             f"远端文件大小不匹配：{remote_path}，期望 {expected_size}，实际 {actual_size}")
    _require(actual_sha == expected_sha,
             f"远端文件 SHA256 不匹配：{remote_path}，期望 {expected_sha}，实际 {actual_sha}")


def _validate_history_bytes(data: bytes | None, label: str) -> dict[str, str]:
    if data is None:
        return {}
    value = _load_json_bytes(data, label)
    _require(isinstance(value, dict), f"{label} 根节点必须是对象。")
    result: dict[str, str] = {}
    for version, digest in value.items():
        parse_version(version)
        _require(isinstance(digest, str) and SHA256_RE.fullmatch(digest) is not None,
                 f"{label} 中 {version} 的哈希非法。")
        result[version] = digest
    return result


def _format_manifest_utc(value: _datetime.datetime) -> str:
    _require(isinstance(value, _datetime.datetime) and value.tzinfo is not None,
             "manifest 时间必须是带时区的 datetime。")
    return value.astimezone(_datetime.timezone.utc).replace(microsecond=0).strftime(
        "%Y-%m-%dT%H:%M:%SZ"
    )


def _parse_manifest_utc(value: Any, field: str) -> _datetime.datetime:
    _require(isinstance(value, str) and UTC_TIMESTAMP_RE.fullmatch(value) is not None,
             f"manifest {field} 必须是严格 UTC YYYY-MM-DDTHH:MM:SSZ。")
    try:
        return _datetime.datetime.strptime(value, "%Y-%m-%dT%H:%M:%SZ").replace(
            tzinfo=_datetime.timezone.utc
        )
    except ValueError as exc:
        raise ReleaseGateError(f"manifest {field} 不是有效 UTC 日历时间。") from exc


def _manifest_freshness_times(
    value: dict[str, Any],
) -> tuple[_datetime.datetime, _datetime.datetime]:
    published = _parse_manifest_utc(value.get("publishedAtUtc"), "publishedAtUtc")
    expires = _parse_manifest_utc(value.get("expiresAtUtc"), "expiresAtUtc")
    _require(int((expires - published).total_seconds()) == MANIFEST_VALIDITY_SECONDS,
             "manifest expiresAtUtc 必须精确等于 publishedAtUtc + 7 天。")
    return published, expires


def _normalized_utc_now(now: _datetime.datetime | None = None) -> _datetime.datetime:
    current = _datetime.datetime.now(_datetime.timezone.utc) if now is None else now
    _require(isinstance(current, _datetime.datetime) and current.tzinfo is not None,
             "manifest 新鲜度校验的当前时间必须带时区。")
    return current.astimezone(_datetime.timezone.utc)


def _new_manifest_freshness(
    now: _datetime.datetime | None = None,
) -> tuple[str, str, _datetime.datetime]:
    published = _normalized_utc_now(now).replace(microsecond=0)
    expires = published + _datetime.timedelta(seconds=MANIFEST_VALIDITY_SECONDS)
    return _format_manifest_utc(published), _format_manifest_utc(expires), published


def _require_manifest_fresh_now(
    value: dict[str, Any],
    *,
    now: _datetime.datetime | None = None,
) -> tuple[_datetime.datetime, _datetime.datetime]:
    published, expires = _manifest_freshness_times(value)
    current = _normalized_utc_now(now)
    skew = _datetime.timedelta(seconds=MANIFEST_CLOCK_SKEW_SECONDS)
    _require(published <= current + skew,
             "manifest publishedAtUtc 超过本机 UTC 10 分钟容差，拒绝过度未来清单。")
    _require(current <= expires + skew,
             "manifest expiresAtUtc 已过期（含 10 分钟时钟容差）。")
    return published, expires


def _validate_manifest(
    value: Any,
    channel: str,
    *,
    allow_legacy: bool = False,
    require_fresh: bool = False,
    now: _datetime.datetime | None = None,
) -> dict[str, Any]:
    _require(isinstance(value, dict), f"{channel} latest.json 根节点必须是对象。")
    schema_version = value.get("schemaVersion")
    unsigned_legacy = schema_version is None
    signed_legacy = schema_version == LEGACY_SIGNED_MANIFEST_SCHEMA_VERSION
    current_schema = schema_version == MANIFEST_SCHEMA_VERSION
    if unsigned_legacy:
        _require(allow_legacy and not require_fresh,
                 f"{channel} latest.json 缺 schemaVersion={MANIFEST_SCHEMA_VERSION}。")
        allowed_top = {"version", "file", "sha256", "size", "notes"}
        if channel == "brand" and value.get("patch") is not None:
            allowed_top.add("patch")
        _require(set(value) == allowed_top,
                  f"{channel} 旧 latest.json 含缺失/未知字段：{sorted(set(value) ^ allowed_top)}")
    else:
        _require(current_schema or (allow_legacy and signed_legacy and not require_fresh),
                 f"{channel} latest.json schemaVersion 非法：{schema_version!r}")
        _require(value.get("channel") == channel, f"{channel} latest.json channel 不匹配。")
        _require(value.get("signatureAlgorithm") == MANIFEST_SIGNATURE_ALGORITHM,
                 f"{channel} latest.json signatureAlgorithm 不匹配。")
        signature = value.get("signature")
        _require(isinstance(signature, str), f"{channel} latest.json signature 缺失。")
        try:
            signature_bytes = base64.b64decode(signature, validate=True)
        except Exception as exc:
            raise ReleaseGateError(f"{channel} latest.json signature 不是合法 Base64。") from exc
        _require(len(signature_bytes) == 384,
                 f"{channel} latest.json signature 不是 RSA-3072 签名。")
        allowed_top = {
            "schemaVersion", "channel", "version", "file", "sha256", "size", "notes",
            "signatureAlgorithm", "signature",
        }
        if current_schema:
            allowed_top.update({"publishedAtUtc", "expiresAtUtc"})
        if channel == "brand" and value.get("patch") is not None:
            allowed_top.add("patch")
        _require(set(value) == allowed_top,
                 f"{channel} v{schema_version} latest.json 含缺失/未知字段："
                 f"{sorted(set(value) ^ allowed_top)}")
        if current_schema:
            _manifest_freshness_times(value)
            if require_fresh:
                _require_manifest_fresh_now(value, now=now)
    required = {"version", "file", "sha256", "size", "notes"}
    _require(required.issubset(value), f"{channel} latest.json 缺少字段：{sorted(required - set(value))}")
    version = value["version"]
    parse_version(version)
    expected_file = _expected_installer_name(channel, version)
    _require(value["file"] == expected_file, f"{channel} latest 安装包名与版本不一致。")
    _require(isinstance(value["sha256"], str) and SHA256_RE.fullmatch(value["sha256"]) is not None,
             f"{channel} latest sha256 非法。")
    _require(isinstance(value["size"], int) and not isinstance(value["size"], bool)
             and 0 < value["size"] <= MAX_UPDATE_PAYLOAD_BYTES,
             f"{channel} latest size 非法。")
    _validate_notes(value["notes"])
    patch = value.get("patch")
    if patch is not None:
        _require(channel == "brand", "远端 neutral latest 不应含 patch。")
        _require(isinstance(patch, dict), "远端 patch 必须是对象。")
        patch_required = {"file", "sha256", "size", "baseMinVersion"}
        _require(set(patch) == patch_required,
                 f"远端 patch 含缺失/未知字段：{sorted(set(patch) ^ patch_required)}")
        _require(patch["file"] == _expected_patch_name(channel, version), "远端 patch 文件名与版本不一致。")
        _require(isinstance(patch["sha256"], str) and SHA256_RE.fullmatch(patch["sha256"]) is not None,
                 "远端 patch sha256 非法。")
        _require(isinstance(patch["size"], int) and not isinstance(patch["size"], bool)
                 and 0 < patch["size"] <= MAX_UPDATE_PAYLOAD_BYTES,
                 "远端 patch size 非法。")
        parse_version(patch["baseMinVersion"])
        _require(compare_versions(patch["baseMinVersion"], version) <= 0,
                 "远端 patch.baseMinVersion 高于目标版本。")
    return value


def _payload_identity(manifest: dict[str, Any]) -> dict[str, Any]:
    identity: dict[str, Any] = {
        "version": manifest["version"],
        "file": manifest["file"],
        "sha256": manifest["sha256"],
        "size": manifest["size"],
    }
    if manifest.get("patch") is not None:
        identity["patch"] = manifest["patch"]
    return identity


def _release_semantics(manifest: dict[str, Any]) -> dict[str, Any]:
    """Fields that v2 and v3 must keep byte-semantically aligned for one release."""
    result = {
        "channel": manifest["channel"],
        "version": manifest["version"],
        "file": manifest["file"],
        "sha256": manifest["sha256"],
        "size": manifest["size"],
        "notes": manifest["notes"],
    }
    if manifest.get("patch") is not None:
        result["patch"] = dict(manifest["patch"])
    return result


def _manifest_from_release_semantics(
    source: dict[str, Any],
    schema_version: int,
    *,
    published_at_utc: str | None = None,
    expires_at_utc: str | None = None,
) -> dict[str, Any]:
    manifest: dict[str, Any] = {
        "schemaVersion": schema_version,
        **_release_semantics(source),
        "signatureAlgorithm": MANIFEST_SIGNATURE_ALGORITHM,
    }
    if schema_version == MANIFEST_SCHEMA_VERSION:
        _require(isinstance(published_at_utc, str) and isinstance(expires_at_utc, str),
                 "v3 manifest conversion requires a fresh UTC window.")
        manifest["publishedAtUtc"] = published_at_utc
        manifest["expiresAtUtc"] = expires_at_utc
        # Keep the canonical signature field order stable in the JSON emitted by humans/tools.
        ordered = {
            "schemaVersion": manifest["schemaVersion"],
            "channel": manifest["channel"],
            "version": manifest["version"],
            "publishedAtUtc": manifest["publishedAtUtc"],
            "expiresAtUtc": manifest["expiresAtUtc"],
            "file": manifest["file"],
            "sha256": manifest["sha256"],
            "size": manifest["size"],
            "notes": manifest["notes"],
        }
        if "patch" in manifest:
            ordered["patch"] = manifest["patch"]
        ordered["signatureAlgorithm"] = MANIFEST_SIGNATURE_ALGORITHM
        return ordered
    _require(schema_version == LEGACY_SIGNED_MANIFEST_SCHEMA_VERSION,
             f"unsupported manifest conversion schema: {schema_version}")
    return manifest


def _sign_and_validate_current_manifest(
    manifest: dict[str, Any],
    channel: str,
    manifest_signer: Any,
    *,
    now: _datetime.datetime,
) -> bytes:
    unsigned = dict(manifest)
    unsigned.pop("signature", None)
    try:
        unsigned["signature"] = manifest_signer(unsigned)
    except Exception as exc:
        if isinstance(exc, ReleaseGateError):
            raise
        raise ReleaseGateError(f"{channel} manifest 签名失败：{exc}") from exc
    _validate_manifest(unsigned, channel, require_fresh=True, now=now)
    verifier = getattr(manifest_signer, "verify", None)
    _require(callable(verifier) and verifier(unsigned),
             f"{channel} manifest 签名生成后验签失败。")
    manifest.clear()
    manifest.update(unsigned)
    manifest_bytes = _json_bytes(manifest)
    _require(len(manifest_bytes) <= MAX_MANIFEST_JSON_BYTES,
             f"{channel} latest.json 超过客户端 256 KiB 接收上限。")
    return manifest_bytes


def _sign_and_validate_legacy_manifest(
    manifest: dict[str, Any],
    channel: str,
    manifest_signer: Any,
) -> bytes:
    unsigned = dict(manifest)
    unsigned.pop("signature", None)
    legacy_signer = getattr(manifest_signer, "sign_legacy", None)
    _require(callable(legacy_signer),
             "发布签名器缺少 v2 compatibility manifest 签名能力。")
    try:
        unsigned["signature"] = legacy_signer(unsigned)
    except Exception as exc:
        if isinstance(exc, ReleaseGateError):
            raise
        raise ReleaseGateError(f"{channel} v2 compatibility manifest 签名失败：{exc}") from exc
    _validate_manifest(unsigned, channel, allow_legacy=True)
    verifier = getattr(manifest_signer, "verify", None)
    _require(callable(verifier) and verifier(unsigned),
             f"{channel} v2 compatibility manifest 签名生成后验签失败。")
    manifest.clear()
    manifest.update(unsigned)
    manifest_bytes = _json_bytes(manifest)
    _require(len(manifest_bytes) <= MAX_MANIFEST_JSON_BYTES,
             f"{channel} v2 compatibility latest.json 超过 256 KiB 上限。")
    return manifest_bytes


def _freshen_signed_manifest_without_changing_release(
    old_manifest: dict[str, Any],
    published_at_utc: str,
    expires_at_utc: str,
) -> dict[str, Any]:
    refreshed: dict[str, Any] = {
        "schemaVersion": MANIFEST_SCHEMA_VERSION,
        "channel": old_manifest["channel"],
        "version": old_manifest["version"],
        "publishedAtUtc": published_at_utc,
        "expiresAtUtc": expires_at_utc,
        "file": old_manifest["file"],
        "sha256": old_manifest["sha256"],
        "size": old_manifest["size"],
        "notes": old_manifest["notes"],
        "signatureAlgorithm": MANIFEST_SIGNATURE_ALGORITHM,
    }
    if old_manifest.get("patch") is not None:
        refreshed["patch"] = dict(old_manifest["patch"])
    return refreshed


def _manifest_has_reusable_freshness(
    manifest: dict[str, Any],
    *,
    now: _datetime.datetime,
) -> bool:
    if manifest.get("schemaVersion") != MANIFEST_SCHEMA_VERSION:
        return False
    try:
        published, expires = _require_manifest_fresh_now(manifest, now=now)
    except ReleaseGateError:
        return False
    current = _normalized_utc_now(now)
    return (expires - current).total_seconds() >= MANIFEST_RENEWAL_WINDOW_SECONDS \
        and published <= current + _datetime.timedelta(seconds=MANIFEST_CLOCK_SKEW_SECONDS)


def _stage_local_file(
    sftp: Any,
    local_path: str,
    final_path: str,
    txn: str,
    expected_sha: str,
    expected_size: int,
) -> dict[str, Any]:
    actual_sha = sha256_file(local_path)
    actual_size = os.path.getsize(local_path)
    _require(actual_sha == expected_sha and actual_size == expected_size,
             f"候选文件在联网前后发生变化，拒绝上传：{local_path}")
    existing = _remote_stat(sftp, final_path)
    if existing is not None:
        _verify_remote(sftp, final_path, expected_sha, expected_size)
        return {"final": final_path, "temp": None, "sha256": expected_sha, "size": expected_size}
    temp_path = f"{final_path}.tmp.{txn}"
    try:
        sftp.remove(temp_path)
    except OSError as exc:
        if not _is_missing_error(exc):
            raise ReleaseGateError(f"清理远端临时文件失败：{temp_path}: {exc}") from exc
    try:
        sftp.put(local_path, temp_path)
        _verify_remote(sftp, temp_path, expected_sha, expected_size)
    except BaseException as exc:
        try:
            remover = getattr(sftp, "cleanup_remove", sftp.remove)
            remover(temp_path)
        except OSError as cleanup_exc:
            if not _is_missing_error(cleanup_exc) and hasattr(exc, "add_note"):
                exc.add_note(f"上传失败后的远端临时文件清理同时失败：{temp_path}")
        raise
    return {"final": final_path, "temp": temp_path, "sha256": expected_sha, "size": expected_size}


def _stage_bytes(sftp: Any, data: bytes, final_path: str, txn: str, tag: str) -> dict[str, Any]:
    expected_sha = sha256_bytes(data)
    expected_size = len(data)
    temp_path = f"{final_path}.tmp.{txn}.{tag}"
    try:
        sftp.remove(temp_path)
    except OSError as exc:
        if not _is_missing_error(exc):
            raise ReleaseGateError(f"清理远端临时文件失败：{temp_path}: {exc}") from exc
    try:
        if hasattr(sftp, "putfo"):
            sftp.putfo(io.BytesIO(data), temp_path, file_size=len(data))
        else:
            with sftp.open(temp_path, "wb") as stream:
                stream.write(data)
        _verify_remote(sftp, temp_path, expected_sha, expected_size)
    except BaseException as exc:
        try:
            remover = getattr(sftp, "cleanup_remove", sftp.remove)
            remover(temp_path)
        except OSError as cleanup_exc:
            if not _is_missing_error(cleanup_exc) and hasattr(exc, "add_note"):
                exc.add_note(f"上传失败后的远端临时文件清理同时失败：{temp_path}")
        raise
    return {"final": final_path, "temp": temp_path, "sha256": expected_sha, "size": expected_size}


def _atomic_commit(sftp: Any, staged: dict[str, Any]) -> None:
    temp_path = staged.get("temp")
    if temp_path is None:
        _verify_remote(sftp, staged["final"], staged["sha256"], staged["size"])
        return
    try:
        sftp.posix_rename(temp_path, staged["final"])
    except Exception as exc:
        raise ReleaseGateError(
            f"服务器不支持/未完成原子重命名 {temp_path} -> {staged['final']}：{exc}"
        ) from exc
    _verify_remote(sftp, staged["final"], staged["sha256"], staged["size"])
    staged["temp"] = None


def _restore_remote_bytes(sftp: Any, final_path: str, old_data: bytes | None, txn: str, tag: str) -> None:
    if old_data is None:
        try:
            sftp.remove(final_path)
        except OSError as exc:
            if not _is_missing_error(exc):
                raise
        return
    staged = _stage_bytes(sftp, old_data, final_path, txn, f"rollback-{tag}")
    _atomic_commit(sftp, staged)


def _cleanup_temps(sftp: Any, staged_items: Iterable[dict[str, Any]]) -> None:
    for staged in staged_items:
        temp_path = staged.get("temp")
        if not temp_path:
            continue
        try:
            remover = getattr(sftp, "cleanup_remove", sftp.remove)
            remover(temp_path)
        except OSError:
            pass


def _build_remote_plan(
    sftp: Any,
    report: dict[str, Any],
    manifest_signer: Any,
    *,
    now: _datetime.datetime | None = None,
) -> dict[str, dict[str, Any]]:
    version = report["version"]
    plans: dict[str, dict[str, Any]] = {}
    published_at_utc, expires_at_utc, publish_now = _new_manifest_freshness(now)
    verifier = getattr(manifest_signer, "verify", None)
    _require(callable(verifier), "发布签名器缺少 manifest 验签能力。")
    for channel in CHANNELS:
        candidate = report["channels"][channel]
        remote_dir = posixpath.join(REMOTE_ROOT, channel)
        history_path = posixpath.join(remote_dir, "payload_history.json")
        old_history_bytes = _read_remote_bytes(sftp, history_path, missing_ok=True)
        history = _validate_history_bytes(old_history_bytes, f"{channel} payload_history.json")
        base_min, new_history = compute_base_min_version(
            history, version, candidate["dist"]["payloadHash"]
        )

        installer = candidate["installer"]
        candidate_semantics: dict[str, Any] = {
            "schemaVersion": MANIFEST_SCHEMA_VERSION,
            "channel": channel,
            "version": version,
            "file": installer["file"],
            "sha256": installer["sha256"],
            "size": installer["size"],
            "notes": report["notes"],
            "signatureAlgorithm": MANIFEST_SIGNATURE_ALGORITHM,
        }
        patch = candidate.get("patch")
        if patch is not None:
            candidate_semantics["patch"] = {
                "file": patch["file"],
                "sha256": patch["sha256"],
                "size": patch["size"],
                "baseMinVersion": base_min,
            }
        history_bytes = _json_bytes(new_history)

        endpoint_specs = {
            "legacy": {
                "path": posixpath.join(remote_dir, LEGACY_MANIFEST_NAME),
                "schemaVersion": LEGACY_SIGNED_MANIFEST_SCHEMA_VERSION,
            },
            "current": {
                "path": posixpath.join(remote_dir, CURRENT_MANIFEST_NAME),
                "schemaVersion": MANIFEST_SCHEMA_VERSION,
            },
        }
        same_version_manifests: list[dict[str, Any]] = []
        for endpoint_name, endpoint in endpoint_specs.items():
            old_bytes = _read_remote_bytes(
                sftp, endpoint["path"], missing_ok=True, max_bytes=MAX_MANIFEST_JSON_BYTES
            )
            endpoint["oldBytes"] = old_bytes
            endpoint["oldManifest"] = None
            if old_bytes is None:
                continue
            old_manifest = _validate_manifest(
                _load_json_bytes(old_bytes, f"{channel} {endpoint_name} manifest"),
                channel,
                allow_legacy=True,
            )
            endpoint["oldManifest"] = old_manifest
            old_schema = old_manifest.get("schemaVersion")
            if old_schema in {
                LEGACY_SIGNED_MANIFEST_SCHEMA_VERSION,
                MANIFEST_SCHEMA_VERSION,
            }:
                _require(verifier(old_manifest),
                         f"{channel} 远端 {endpoint_name} manifest 签名校验失败。")
            comparison = compare_versions(version, old_manifest["version"])
            _require(comparison >= 0,
                     f"{channel} 目标版本 {version} 低于远端 {endpoint_name} "
                     f"manifest {old_manifest['version']}。")
            if comparison != 0:
                continue
            _require(old_schema in {
                LEGACY_SIGNED_MANIFEST_SCHEMA_VERSION,
                MANIFEST_SCHEMA_VERSION,
            }, f"{channel} 远端同版本仍是未签名旧清单；必须升新版本，禁止原版本补签。")
            _require(_payload_identity(old_manifest) == _payload_identity(candidate_semantics),
                     f"{channel} 远端同版本已绑定不同安装包/补丁，禁止覆盖。")
            _verify_remote(
                sftp,
                posixpath.join(remote_dir, old_manifest["file"]),
                old_manifest["sha256"],
                old_manifest["size"],
            )
            if old_manifest.get("patch") is not None:
                old_patch = old_manifest["patch"]
                _verify_remote(
                    sftp,
                    posixpath.join(remote_dir, old_patch["file"]),
                    old_patch["sha256"],
                    old_patch["size"],
                )
            same_version_manifests.append(old_manifest)

        semantic_source = candidate_semantics
        if same_version_manifests:
            semantic_source = same_version_manifests[0]
            expected_semantics = _release_semantics(semantic_source)
            for old_manifest in same_version_manifests[1:]:
                _require(_release_semantics(old_manifest) == expected_semantics,
                         f"{channel} v2/v3 同版本端点的已签发语义冲突，拒绝自动改写。")

        desired_legacy = _manifest_from_release_semantics(
            semantic_source, LEGACY_SIGNED_MANIFEST_SCHEMA_VERSION
        )
        desired_legacy_bytes = _sign_and_validate_legacy_manifest(
            desired_legacy, channel, manifest_signer
        )
        desired_current = _manifest_from_release_semantics(
            semantic_source,
            MANIFEST_SCHEMA_VERSION,
            published_at_utc=published_at_utc,
            expires_at_utc=expires_at_utc,
        )
        desired_current_bytes = _sign_and_validate_current_manifest(
            desired_current, channel, manifest_signer, now=publish_now
        )

        for endpoint_name, endpoint in endpoint_specs.items():
            old_manifest = endpoint["oldManifest"]
            old_bytes = endpoint["oldBytes"]
            desired_schema = endpoint["schemaVersion"]
            already_visible = False
            if (old_manifest is not None
                    and compare_versions(version, old_manifest["version"]) == 0
                    and old_manifest.get("schemaVersion") == desired_schema):
                if desired_schema == LEGACY_SIGNED_MANIFEST_SCHEMA_VERSION:
                    # The v2 compatibility bytes are immutable for a same-version retry.
                    endpoint["manifest"] = old_manifest
                    endpoint["manifestBytes"] = old_bytes
                    already_visible = True
                elif _manifest_has_reusable_freshness(old_manifest, now=publish_now):
                    endpoint["manifest"] = old_manifest
                    endpoint["manifestBytes"] = old_bytes
                    already_visible = True
                else:
                    endpoint["manifest"] = desired_current
                    endpoint["manifestBytes"] = desired_current_bytes
            elif desired_schema == LEGACY_SIGNED_MANIFEST_SCHEMA_VERSION:
                endpoint["manifest"] = desired_legacy
                endpoint["manifestBytes"] = desired_legacy_bytes
            else:
                endpoint["manifest"] = desired_current
                endpoint["manifestBytes"] = desired_current_bytes
            endpoint["alreadyVisible"] = already_visible

        plans[channel] = {
            "channel": channel,
            "remoteDir": remote_dir,
            "historyPath": history_path,
            "oldHistory": old_history_bytes,
            "manifestEndpoints": endpoint_specs,
            "historyBytes": history_bytes,
            "candidate": candidate,
        }
    return plans


def _publish_dual_locked(sftp: Any, report: dict[str, Any], manifest_signer: Any) -> dict[str, Any]:
    """调用方持有远端发布锁时执行双通道发布。"""
    _require(report.get("schema") == REPORT_SCHEMA, "发布报告 schema 非法。")
    parse_version(report.get("version"))
    _require(set(report.get("channels", {})) == set(CHANNELS), "发布必须同时包含双通道。")
    _require(callable(manifest_signer), "发布必须提供 OTA manifest 签名器。")
    plans = _build_remote_plan(sftp, report, manifest_signer)
    txn = f"{report['version'].replace('.', '')}-{secrets.token_hex(6)}"
    all_staged: list[dict[str, Any]] = []
    history_committed: list[str] = []
    manifest_committed: list[tuple[str, str]] = []

    try:
        # 1) 两通道所有载荷先以临时名上传并回读 size+sha；此时 latest/history 均未变化。
        for channel in CHANNELS:
            plan = plans[channel]
            candidate = plan["candidate"]
            payload_stages = [
                _stage_local_file(
                    sftp,
                    candidate["installer"]["path"],
                    posixpath.join(plan["remoteDir"], candidate["installer"]["file"]),
                    txn,
                    candidate["installer"]["sha256"],
                    candidate["installer"]["size"],
                )
            ]
            if candidate.get("patch") is not None:
                payload_stages.append(
                    _stage_local_file(
                        sftp,
                        candidate["patch"]["path"],
                        posixpath.join(plan["remoteDir"], candidate["patch"]["file"]),
                        txn,
                        candidate["patch"]["sha256"],
                        candidate["patch"]["size"],
                    )
                )
            plan["payloadStages"] = payload_stages
            all_staged.extend(payload_stages)
            plan["historyStage"] = _stage_bytes(
                sftp, plan["historyBytes"], plan["historyPath"], txn, f"{channel}-history"
            )
            all_staged.append(plan["historyStage"])
            for endpoint_name, endpoint in plan["manifestEndpoints"].items():
                if endpoint["alreadyVisible"]:
                    endpoint["stage"] = None
                    continue
                endpoint["stage"] = _stage_bytes(
                    sftp,
                    endpoint["manifestBytes"],
                    endpoint["path"],
                    txn,
                    f"{channel}-{endpoint_name}-manifest",
                )
                all_staged.append(endpoint["stage"])

        # 2) 两通道载荷都验证完后才原子落最终文件名，再逐份回读。
        for channel in CHANNELS:
            for staged in plans[channel]["payloadStages"]:
                _atomic_commit(sftp, staged)

        # 3) payload_history 先原子切换；任一失败先回滚已切历史，latest 仍未变化。
        try:
            for channel in CHANNELS:
                history_committed.append(channel)
                _atomic_commit(sftp, plans[channel]["historyStage"])
        except Exception:
            for committed_channel in reversed(history_committed):
                committed_plan = plans[committed_channel]
                _restore_remote_bytes(
                    sftp,
                    committed_plan["historyPath"],
                    committed_plan["oldHistory"],
                    txn,
                    f"{committed_channel}-history",
                )
            raise

        # 4) 两个通道的 v2 compatibility + v3 current manifests 永远最后切换。
        # 任一份失败时回滚此前切换的全部 manifest 与双通道历史。
        try:
            for channel in CHANNELS:
                for endpoint_name, endpoint in plans[channel]["manifestEndpoints"].items():
                    stage = endpoint["stage"]
                    if stage is None:
                        continue
                    # 先登记再切换：即使 rename 已成功但随后的回读失败，也必须回滚。
                    manifest_committed.append((channel, endpoint_name))
                    _atomic_commit(sftp, stage)

            # 最终远端回读：四份 manifest 字节、引用载荷及 history 必须全部一致。
            for channel in CHANNELS:
                plan = plans[channel]
                for endpoint_name, endpoint in plan["manifestEndpoints"].items():
                    manifest_bytes = _read_remote_bytes(
                        sftp, endpoint["path"], max_bytes=MAX_MANIFEST_JSON_BYTES
                    )
                    _require(manifest_bytes == endpoint["manifestBytes"],
                             f"{channel} {endpoint_name} manifest 最终回读内容不一致。")
                history_bytes = _read_remote_bytes(sftp, plan["historyPath"])
                _require(history_bytes == plan["historyBytes"],
                         f"{channel} payload_history.json 最终回读内容不一致。")
                for staged in plan["payloadStages"]:
                    _verify_remote(sftp, staged["final"], staged["sha256"], staged["size"])
        except Exception as publish_exc:
            rollback_errors: list[str] = []
            for committed_channel, endpoint_name in reversed(manifest_committed):
                endpoint = plans[committed_channel]["manifestEndpoints"][endpoint_name]
                try:
                    _restore_remote_bytes(
                        sftp,
                        endpoint["path"],
                        endpoint["oldBytes"],
                        txn,
                        f"{committed_channel}-{endpoint_name}-manifest",
                    )
                except Exception as exc:
                    rollback_errors.append(f"{committed_channel} {endpoint_name}: {exc}")
            for committed_channel in reversed(history_committed):
                committed_plan = plans[committed_channel]
                try:
                    _restore_remote_bytes(
                        sftp,
                        committed_plan["historyPath"],
                        committed_plan["oldHistory"],
                        txn,
                        f"{committed_channel}-history",
                    )
                except Exception as exc:
                    rollback_errors.append(f"{committed_channel} history: {exc}")
            if rollback_errors:
                raise ReleaseGateError(
                    f"发布失败且回滚不完整：{publish_exc}；{'；'.join(rollback_errors)}"
                ) from publish_exc
            raise

        return {
            "version": report["version"],
            "channels": {
                channel: {
                    "legacyLatest": plans[channel]["manifestEndpoints"]["legacy"]["manifest"],
                    "currentLatest": plans[channel]["manifestEndpoints"]["current"]["manifest"],
                    "alreadyVisible": all(
                        endpoint["alreadyVisible"]
                        for endpoint in plans[channel]["manifestEndpoints"].values()
                    ),
                }
                for channel in CHANNELS
            },
        }
    finally:
        _cleanup_temps(sftp, all_staged)


def publish_dual_with_sftp(sftp: Any, report: dict[str, Any], manifest_signer: Any) -> dict[str, Any]:
    """通过已连接的 SFTP 发布；供 CLI 与离线假 SFTP 测试共同调用。"""
    # This check intentionally precedes even the remote lock acquisition. A self-consistent
    # package/installer/pair JSON chain has no authority until the current PowerShell verifier
    # freshly re-runs all live gates for this exact candidate.
    _require_trusted_publish_candidate(report)
    lock_path = posixpath.join(REMOTE_ROOT, ".dual-publish.lock")
    try:
        sftp.mkdir(lock_path)
    except Exception as exc:
        raise ReleaseGateError(
            f"无法取得 OTA 双通道发布锁 {lock_path}；可能有另一发布正在进行或上次异常中止：{exc}"
        ) from exc
    publish_result: dict[str, Any] | None = None
    publish_error: BaseException | None = None
    publish_traceback = None
    try:
        publish_result = _publish_dual_locked(sftp, report, manifest_signer)
    except BaseException as exc:
        publish_error = exc
        publish_traceback = exc.__traceback__
    finally:
        lock_cleanup_error: BaseException | None = None
        try:
            sftp.rmdir(lock_path)
        except Exception as exc:
            lock_cleanup_error = exc
        if publish_error is not None:
            if lock_cleanup_error is not None and hasattr(publish_error, "add_note"):
                publish_error.add_note(
                    f"OTA 发布主异常保留；发布锁清理同时失败，需人工检查：{lock_path}"
                )
            raise publish_error.with_traceback(publish_traceback)
        assert publish_result is not None
        publish_result["committed"] = True
        publish_result["lockCleanup"] = (
            {"status": "passed"}
            if lock_cleanup_error is None
            else {
                "status": "failed-after-commit",
                "path": lock_path,
                "errorType": type(lock_cleanup_error).__name__,
            }
        )
    return publish_result


def _known_host_lookup_name(host: str, port: int) -> str:
    return host if port == 22 else f"[{host}]:{port}"


def _validate_known_hosts_path(path_value: str) -> str:
    try:
        path = Path(path_value).expanduser().resolve(strict=True)
    except OSError as exc:
        raise ReleaseGateError("known_hosts 不存在或不可访问。") from exc
    _require(path.is_file(), f"known_hosts 不是文件：{path}")
    repo_root = Path(__file__).resolve().parents[1]
    try:
        path.relative_to(repo_root)
    except ValueError:
        return str(path)
    raise ReleaseGateError("known_hosts 必须放在项目目录之外，禁止随仓库分发信任根。")


def _normalize_host_fingerprint(value: str) -> str:
    _require(isinstance(value, str) and value.startswith("SHA256:"),
             "主机指纹必须使用 OpenSSH SHA256:<base64> 格式。")
    body = value[len("SHA256:"):].rstrip("=")
    _require(re.fullmatch(r"[A-Za-z0-9+/]{43}", body) is not None,
             "主机 SHA256 指纹格式非法。")
    return f"SHA256:{body}"


def make_ssh(
    host: str,
    port: int,
    user: str,
    password: str,
    *,
    known_hosts: str | None = None,
    host_key_sha256: str | None = None,
    timeout: int = 30,
) -> Any:
    """只信任项目外 known_hosts 或显式 SHA256 主机指纹；绝不自动接受新主机。"""
    _require(bool(known_hosts) ^ bool(host_key_sha256),
             "必须且只能提供 --known-hosts 或 --host-key-sha256 之一。")
    import paramiko

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    if known_hosts:
        client.load_host_keys(_validate_known_hosts_path(known_hosts))
    else:
        expected = _normalize_host_fingerprint(host_key_sha256 or "")
        transport = None
        raw_socket = None
        try:
            raw_socket = socket.create_connection((host, port), timeout=timeout)
            transport = paramiko.Transport(raw_socket)
            transport.start_client(timeout=timeout)
            server_key = transport.get_remote_server_key()
            actual_body = base64.b64encode(hashlib.sha256(server_key.asbytes()).digest()).decode("ascii").rstrip("=")
            actual = f"SHA256:{actual_body}"
            _require(hmac.compare_digest(actual, expected),
                     f"SSH 主机指纹不匹配：期望 {expected}，实际 {actual}")
            client.get_host_keys().add(_known_host_lookup_name(host, port), server_key.get_name(), server_key)
        finally:
            if transport is not None:
                transport.close()
            elif raw_socket is not None:
                raw_socket.close()

    client.connect(
        host,
        port=port,
        username=user,
        password=password,
        timeout=timeout,
        banner_timeout=timeout,
        auth_timeout=timeout,
        allow_agent=False,
        look_for_keys=False,
    )
    return client


class _DeadlineSftpFile:
    def __init__(self, inner: Any, owner: "_DeadlineSftpClient") -> None:
        self._inner = inner
        self._owner = owner

    def __enter__(self) -> "_DeadlineSftpFile":
        self._owner._arm_timeout()
        self._inner.__enter__()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> Any:
        try:
            self._owner._arm_timeout()
        except ReleaseGateError:
            # Preserve the read/write deadline error while still closing the
            # remote handle under the channel's already-armed finite timeout.
            return self._inner.__exit__(exc_type, exc, traceback)
        result = self._inner.__exit__(exc_type, exc, traceback)
        if exc_type is None:
            self._owner._check_deadline()
        return result

    def read(self, *args: Any, **kwargs: Any) -> Any:
        self._owner._arm_timeout()
        result = self._inner.read(*args, **kwargs)
        self._owner._check_deadline()
        return result

    def write(self, *args: Any, **kwargs: Any) -> Any:
        self._owner._arm_timeout()
        result = self._inner.write(*args, **kwargs)
        self._owner._check_deadline()
        return result

    def __getattr__(self, name: str) -> Any:
        return getattr(self._inner, name)


class _DeadlineSftpClient:
    """Apply one monotonic total budget plus a bounded Paramiko channel timeout."""

    def __init__(
        self,
        inner: Any,
        *,
        total_timeout: float = SFTP_TOTAL_TIMEOUT_SECONDS,
        io_timeout: float = SFTP_IO_TIMEOUT_SECONDS,
        _clock: Any = time.monotonic,
    ) -> None:
        _require(total_timeout > 0 and io_timeout > 0,
                 "SFTP timeout 配置必须为正数。")
        self._inner = inner
        self._clock = _clock
        self._expires_at = _clock() + total_timeout
        self._io_timeout = io_timeout
        self._channel = inner.get_channel()
        self._arm_timeout()

    def _remaining(self) -> float:
        remaining = self._expires_at - self._clock()
        if remaining <= 0:
            raise ReleaseGateError("SFTP 发布总 deadline 已耗尽。")
        return remaining

    def _arm_timeout(self) -> None:
        self._channel.settimeout(max(0.001, min(self._io_timeout, self._remaining())))

    def _check_deadline(self) -> None:
        self._remaining()

    def open(self, *args: Any, **kwargs: Any) -> _DeadlineSftpFile:
        self._arm_timeout()
        result = self._inner.open(*args, **kwargs)
        self._check_deadline()
        return _DeadlineSftpFile(result, self)

    def close(self) -> Any:
        return self._inner.close()

    def _wrap_progress_callback(self, callback: Any = None) -> Any:
        def checked(transferred: int, total: int) -> None:
            self._arm_timeout()
            if callback is not None:
                callback(transferred, total)
            self._check_deadline()
        return checked

    def put(
        self,
        localpath: str,
        remotepath: str,
        callback: Any = None,
        confirm: bool = True,
    ) -> Any:
        self._arm_timeout()
        result = self._inner.put(
            localpath,
            remotepath,
            callback=self._wrap_progress_callback(callback),
            confirm=confirm,
        )
        self._check_deadline()
        return result

    def putfo(
        self,
        fileobj: Any,
        remotepath: str,
        file_size: int = 0,
        callback: Any = None,
        confirm: bool = True,
    ) -> Any:
        self._arm_timeout()
        result = self._inner.putfo(
            fileobj,
            remotepath,
            file_size=file_size,
            callback=self._wrap_progress_callback(callback),
            confirm=confirm,
        )
        self._check_deadline()
        return result

    def cleanup_remove(self, path: str) -> Any:
        # Cleanup has one fresh finite I/O window even when the publication's
        # total deadline has elapsed; it cannot reopen the publish transaction.
        self._channel.settimeout(self._io_timeout)
        return self._inner.remove(path)

    def __getattr__(self, name: str) -> Any:
        attribute = getattr(self._inner, name)
        if not callable(attribute):
            return attribute

        def checked(*args: Any, **kwargs: Any) -> Any:
            self._arm_timeout()
            result = attribute(*args, **kwargs)
            self._check_deadline()
            return result

        return checked


def _run_local_checked(
    command: list[str],
    *,
    cwd: os.PathLike[str] | str,
    label: str,
    timeout: int = 30 * 60,
    env: dict[str, str] | None = None,
    allow_github_credentials: bool = False,
) -> subprocess.CompletedProcess[str]:
    _require(bool(command) and Path(command[0]).is_absolute(),
             f"{label} 必须使用受信任绝对可执行文件路径。")
    executable = Path(command[0])
    for tool in (_ACTIVE_GIT_TOOL, _ACTIVE_GH_TOOL, _ACTIVE_PYTHON_TOOL):
        if tool is not None and executable == tool.path:
            _verify_trusted_tool_unchanged(tool)
    child_environment = _sanitized_local_environment(
        env, allow_github_credentials=allow_github_credentials
    )
    try:
        completed = subprocess.run(
            command,
            cwd=str(cwd),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
            check=False,
            env=child_environment,
            stdin=subprocess.DEVNULL,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise ReleaseGateError(f"{label} 无法执行：{exc}") from exc
    if completed.returncode != 0:
        detail = "\n".join(
            value.strip() for value in (completed.stdout, completed.stderr) if value.strip()
        )
        if len(detail) > 6000:
            detail = detail[-6000:]
        raise ReleaseGateError(f"{label} 失败（exit={completed.returncode}）"
                               + (f"\n{detail}" if detail else ""))
    return completed


def _git_text(repo_root: Path, *arguments: str) -> str:
    git_path = (_ACTIVE_GIT_TOOL.path if _ACTIVE_GIT_TOOL is not None else DEFAULT_GIT_EXE)
    completed = _run_local_checked(
        [str(git_path), *arguments],
        cwd=repo_root,
        label=f"git {' '.join(arguments)}",
        timeout=5 * 60,
    )
    return completed.stdout.strip()


def _require_release_launcher_is_clean_main(repo_root: Path) -> tuple[str, str]:
    main_head = _git_text(repo_root, "rev-parse", "refs/heads/main")
    brand_head = _git_text(repo_root, "rev-parse", "refs/heads/hk-pathlynx-corpla")
    _require(re.fullmatch(r"[0-9a-f]{40}", main_head) is not None
             and re.fullmatch(r"[0-9a-f]{40}", brand_head) is not None,
             "release refs did not resolve to full commit IDs.")
    current_head = _git_text(repo_root, "rev-parse", "HEAD")
    _require(current_head == main_head,
             "trusted-release-dual 必须从 refs/heads/main 当前 HEAD 的受信发布代码启动。")
    tracked_dirty = _git_text(repo_root, "status", "--porcelain=v1", "--untracked-files=no")
    _require(not tracked_dirty,
             "trusted-release-dual 启动仓库含 tracked/unmerged 变更，拒绝使用未提交发布代码。")
    git_path = (_ACTIVE_GIT_TOOL.path if _ACTIVE_GIT_TOOL is not None else DEFAULT_GIT_EXE)
    _run_local_checked(
        [str(git_path), "merge-base", "--is-ancestor", main_head, brand_head],
        cwd=repo_root,
        label="验证 main 是品牌分支祖先",
        timeout=5 * 60,
    )
    return main_head, brand_head


def _safe_verifier_worktree(root: Path, name: str) -> Path:
    _require(name in {"neutral", "brand"}, "非法 verifier worktree 名称。")
    root = root.resolve(strict=True)
    child = (root / name).resolve()
    _require(child.parent == root and child.name == name,
             "verifier worktree 路径逃逸临时根目录。")
    return child


_ALLOWED_BRAND_TRACKED_DELTA = frozenset({
    ".gitignore",
    "QtWidgetsApplication4.vcxproj",
    "installer/QtWidgetsApplication4.iss",
    "icons/app.ico",
    "branding/app_color.ico",
    "branding/app_color.png",
    "branding/app_nobg.ico",
    "branding/app_nobg.png",
    "branding/branding.ini",
})


def _decode_normalized_text(path: Path) -> str:
    try:
        return path.read_bytes().decode("utf-8-sig").replace("\r\n", "\n").replace("\r", "\n")
    except (OSError, UnicodeDecodeError) as exc:
        raise ReleaseGateError(f"无法按 UTF-8 读取品牌边界文件：{path}") from exc


def _normalize_brand_vcxproj(main_text: str, brand_text: str) -> tuple[str, str]:
    target_line = re.compile(
        r"(?m)^[ \t]*<TargetName>HK-Pathlynx-CORPLA</TargetName>[ \t]*\n?"
    )
    _require(len(target_line.findall(brand_text)) == 2
             and "<TargetName>" not in target_line.sub("", brand_text),
             "brand vcxproj 必须且只能增加两个固定 HK-Pathlynx-CORPLA TargetName。")
    for configuration in ("Debug", "Release"):
        groups = list(re.finditer(
            rf"<PropertyGroup[^>]*Condition=\"[^\"]*{configuration}\|x64[^\"]*\"[^>]*>"
            rf"(.*?)</PropertyGroup>",
            brand_text,
            re.S,
        ))
        _require(sum(
            group.group(1).count("<TargetName>HK-Pathlynx-CORPLA</TargetName>")
            for group in groups
        ) == 1,
                 f"brand vcxproj 的 {configuration}|x64 TargetName 边界非法。")
    return main_text, target_line.sub("", brand_text)


def _functional_iss_lines(text: str, *, brand: bool) -> list[str]:
    lines: list[str] = []
    replacement_counts = {
        "name": 0,
        "output": 0,
        "publisher": 0,
        "exe": 0,
        "icon": 0,
    }
    for raw_line in text.split("\n"):
        line = raw_line.strip()
        if not line or line.startswith(";"):
            continue
        if brand:
            mappings = (
                ("name", '#define MyAppName "HK-Pathlynx-CORPLA"',
                 '#define MyAppName "NoTeaching-Robot"'),
                ("publisher", '#define MyAppPublisher "海瞰智焊"',
                 '#define MyAppPublisher "yu1201"'),
                ("exe", '#define MyAppExeName "HK-Pathlynx-CORPLA.exe"',
                 '#define MyAppExeName "QtWidgetsApplication4.exe"'),
            )
            for key, source, target in mappings:
                if line == source:
                    line = target
                    replacement_counts[key] += 1
                    break
            output_match = re.fullmatch(
                r'#define MyOutputBaseFilename "HK-Pathlynx-CORPLA-Setup-v(.+)"', line
            )
            if output_match is not None:
                line = f'#define MyOutputBaseFilename "NoTeaching-Robot-Setup-v{output_match.group(1)}"'
                replacement_counts["output"] += 1
            icon_suffix = '; IconFilename: "{app}\\branding\\app_nobg.ico"'
            if icon_suffix in line:
                _require(line.count(icon_suffix) == 1,
                         "brand iss IconFilename 后缀重复。")
                line = line.replace(icon_suffix, "")
                replacement_counts["icon"] += 1
        lines.append(line)
    if brand:
        _require(replacement_counts == {
            "name": 1, "output": 1, "publisher": 1, "exe": 1, "icon": 2,
        }, f"brand iss 允许的品牌字段数量漂移：{replacement_counts}")
    return lines


def _assert_brand_source_boundary(
    repo_root: Path,
    neutral_root: Path,
    brand_root: Path,
    main_head: str,
    brand_head: str,
) -> None:
    changed_output = _git_text(
        repo_root, "diff", "--name-only", "--no-renames", main_head, brand_head
    )
    changed = frozenset(line.strip().replace("\\", "/")
                        for line in changed_output.splitlines() if line.strip())
    _require(changed == _ALLOWED_BRAND_TRACKED_DELTA,
             "brand 相对 main 的 tracked 差异超出精确 allowlist："
             f"extra={sorted(changed - _ALLOWED_BRAND_TRACKED_DELTA)} "
             f"missing={sorted(_ALLOWED_BRAND_TRACKED_DELTA - changed)}")

    main_vcx, normalized_brand_vcx = _normalize_brand_vcxproj(
        _decode_normalized_text(neutral_root / "QtWidgetsApplication4.vcxproj"),
        _decode_normalized_text(brand_root / "QtWidgetsApplication4.vcxproj"),
    )
    _require(main_vcx == normalized_brand_vcx,
             "brand vcxproj 去除两个固定 TargetName 后仍与 main 不同。")

    main_iss = _functional_iss_lines(
        _decode_normalized_text(neutral_root / "installer" / "QtWidgetsApplication4.iss"),
        brand=False,
    )
    brand_iss = _functional_iss_lines(
        _decode_normalized_text(brand_root / "installer" / "QtWidgetsApplication4.iss"),
        brand=True,
    )
    immutable_app_id_line = '#define MyAppGuid "A5A7E2A0-8226-40BB-B126-94C5D298B3CF"'
    _require(main_iss.count(immutable_app_id_line) == 1
             and brand_iss.count(immutable_app_id_line) == 1
             and main_iss == brand_iss,
             "brand iss 规范化后功能行/AppId 与 main 不同。")

    def ignore_entries(root: Path) -> list[str]:
        return [line.strip() for line in _decode_normalized_text(root / ".gitignore").split("\n")
                if line.strip() and not line.lstrip().startswith("#")]

    main_ignore = ignore_entries(neutral_root)
    brand_ignore = ignore_entries(brand_root)
    _require(main_ignore.count("branding/") == 1
             and [entry for entry in main_ignore if entry != "branding/"] == brand_ignore,
             "brand .gitignore 功能项只能放开 branding/。")

    for relative in sorted(_ALLOWED_BRAND_TRACKED_DELTA):
        if not relative.startswith("branding/"):
            continue
        path = brand_root / Path(relative)
        _require(path.is_file() and not path.is_symlink() and path.stat().st_size > 0,
                 f"brand allowlist 资产缺失/为空/链接：{relative}")
    _require(sha256_file(brand_root / "icons" / "app.ico")
             == sha256_file(brand_root / "branding" / "app_color.ico"),
             "brand icons/app.ico 必须与 branding/app_color.ico 完全一致。")


def _copy_verified_fanuc_runtime(runtime_source_value: str, targets: Iterable[Path]) -> None:
    try:
        runtime_source = Path(runtime_source_value).expanduser().resolve(strict=True)
    except OSError as exc:
        raise ReleaseGateError("显式 runtime-source 不存在或不可访问。") from exc
    _require(runtime_source.is_dir(), "显式 runtime-source 必须是目录。")
    source_manifest_path = runtime_source / "installer" / "fanuc-runtime-manifest.json"
    _require(source_manifest_path.is_file() and not source_manifest_path.is_symlink()
             and source_manifest_path.resolve(strict=True).is_relative_to(runtime_source),
             "runtime-source 缺 FANUC 权威 manifest 或 manifest 是逃逸链接。")
    source_manifest_bytes = source_manifest_path.read_bytes()
    source_manifest = _load_json_bytes(source_manifest_bytes, "runtime-source FANUC manifest")
    _require(isinstance(source_manifest, dict)
             and source_manifest.get("schemaVersion") == 1
             and source_manifest.get("expectedFileCount") == EXPECTED_FANUC_TP_COUNT + EXPECTED_FANUC_PC_COUNT
             and isinstance(source_manifest.get("files"), list)
             and len(source_manifest["files"]) == source_manifest["expectedFileCount"],
             "runtime-source FANUC manifest schema/count 非法。")

    validated_files: list[tuple[Path, str]] = []
    seen: set[str] = set()
    tp_count = 0
    pc_count = 0
    for item in source_manifest["files"]:
        _require(isinstance(item, dict) and set(item) == {"path", "size", "sha256"},
                 "FANUC manifest 文件条目 schema 非法。")
        relative = item["path"]
        _require(isinstance(relative, str) and relative.startswith("SDK/FANUC/")
                 and "\\" not in relative and ":" not in relative
                 and all(part not in {"", ".", ".."} for part in relative.split("/")),
                 f"FANUC manifest 路径非法：{relative!r}")
        suffix = Path(relative).suffix.casefold()
        _require(suffix in {".tp", ".pc"}, f"FANUC manifest 含非 tp/pc：{relative}")
        folded = relative.casefold()
        _require(folded not in seen, f"FANUC manifest 路径重复：{relative}")
        seen.add(folded)
        source_candidate = runtime_source / Path(relative)
        _require(not source_candidate.is_symlink(),
                 f"FANUC runtime 文件禁止符号链接：{relative}")
        source_file = source_candidate.resolve(strict=True)
        _require(source_file.is_file()
                 and source_file.is_relative_to(runtime_source),
                 f"FANUC runtime 文件逃逸/链接/缺失：{relative}")
        _require(isinstance(item["size"], int) and not isinstance(item["size"], bool)
                 and source_file.stat().st_size == item["size"]
                 and isinstance(item["sha256"], str)
                 and SHA256_RE.fullmatch(item["sha256"]) is not None
                 and sha256_file(source_file) == item["sha256"],
                 f"FANUC runtime 文件 size/hash 不匹配：{relative}")
        tp_count += suffix == ".tp"
        pc_count += suffix == ".pc"
        validated_files.append((source_file, relative))
    _require(tp_count == EXPECTED_FANUC_TP_COUNT and pc_count == EXPECTED_FANUC_PC_COUNT,
             "FANUC runtime tp/pc 数量不符合硬门禁。")

    for target in targets:
        target_manifest = target / "installer" / "fanuc-runtime-manifest.json"
        _require(target_manifest.is_file()
                 and target_manifest.read_bytes() == source_manifest_bytes,
                 f"{target.name} clean HEAD 的 FANUC manifest 与 runtime-source 不一致。")
        for source_file, relative in validated_files:
            destination = target / Path(relative)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source_file, destination)
            _require(destination.stat().st_size == source_file.stat().st_size
                     and sha256_file(destination) == sha256_file(source_file),
                     f"复制 FANUC runtime 后回读失败：{target.name}/{relative}")


def _create_single_exe_patch(brand_root: Path, version: str) -> Path:
    exe = brand_root / "dist" / "QtWidgetsApplication4" / CHANNEL_EXE["brand"]
    _require(exe.is_file(), "brand clean build 未生成补丁源 exe。")
    patch = brand_root / "dist" / "installer" / _expected_patch_name("brand", version)
    patch.parent.mkdir(parents=True, exist_ok=True)
    info = zipfile.ZipInfo(exe.name, date_time=(1980, 1, 1, 0, 0, 0))
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o100644 << 16
    with zipfile.ZipFile(patch, "w") as archive:
        archive.writestr(info, exe.read_bytes())
    return patch


@contextlib.contextmanager
def _build_trusted_release_candidate(args: argparse.Namespace):
    """Build the only publishable candidate in verifier-owned linked worktrees."""
    global _ACTIVE_GIT_TOOL, _ACTIVE_GH_TOOL, _ACTIVE_PYTHON_TOOL
    repo_root = Path(__file__).resolve().parents[1]
    git_tool = _resolve_and_verify_trusted_tool(
        getattr(args, "git_exe", DEFAULT_GIT_EXE),
        name="Git for Windows",
        expected_basename="git.exe",
        signer_tokens=("Johannes Schindelin", "Git for Windows"),
    )
    gh_tool = _resolve_and_verify_trusted_tool(
        getattr(args, "gh_exe", DEFAULT_GH_EXE),
        name="GitHub CLI",
        expected_basename="gh.exe",
        signer_tokens=("GitHub, Inc.",),
    )
    python_tool = _resolve_and_verify_trusted_tool(
        DEFAULT_PYTHON_EXE,
        name="Python runtime",
        expected_basename="python.exe",
        signer_tokens=("Python Software Foundation",),
    )
    msbuild_tool = _resolve_and_verify_trusted_tool(
        getattr(args, "msbuild_exe", DEFAULT_MSBUILD_EXE),
        name="MSBuild",
        expected_basename="MSBuild.exe",
        signer_tokens=("Microsoft Corporation",),
    )
    windeployqt_tool = _resolve_and_verify_trusted_tool(
        getattr(args, "windeployqt_exe", DEFAULT_WINDEPLOYQT_EXE),
        name="windeployqt",
        expected_basename="windeployqt.exe",
        signer_tokens=("The Qt Company",),
    )
    iscc_tool = _resolve_and_verify_trusted_tool(
        getattr(args, "iscc_exe", DEFAULT_ISCC_EXE),
        name="Inno Setup Compiler",
        expected_basename="ISCC.exe",
        signer_tokens=("Pyrsys B.V.",),
    )
    qt_core_tool = _resolve_and_verify_trusted_tool(
        windeployqt_tool.path.parent / "Qt6Core.dll",
        name="Qt6Core runtime",
        expected_basename="Qt6Core.dll",
        signer_tokens=("The Qt Company",),
    )
    inno_critical_tools = tuple(
        _resolve_and_verify_trusted_tool(
            iscc_tool.path.parent / filename,
            name=f"Inno critical dependency {filename}",
            expected_basename=filename,
            signer_tokens=("Pyrsys B.V.",),
        )
        for filename in ("ISCmplr.dll", "ISPP.dll", "is7z.dll", "islzma.dll")
    )
    build_tools = (
        msbuild_tool, windeployqt_tool, iscc_tool, qt_core_tool, *inno_critical_tools
    )
    previous_git, previous_gh, previous_python = (
        _ACTIVE_GIT_TOOL, _ACTIVE_GH_TOOL, _ACTIVE_PYTHON_TOOL
    )
    _ACTIVE_GIT_TOOL, _ACTIVE_GH_TOOL, _ACTIVE_PYTHON_TOOL = (
        git_tool, gh_tool, python_tool
    )
    msbuild_acl_root = msbuild_tool.path.parents[2]
    protected_tool_roots = ((
        "MSBuild dependency tree",
        _assert_windows_tool_tree_not_user_writable(
            "MSBuild dependency tree", msbuild_acl_root
        ),
    ),)
    toolchain_closures = _snapshot_release_toolchain_closures(
        windeployqt_tool, iscc_tool
    )
    _require_expected_toolchain_closures(toolchain_closures)
    python_runtime_root, python_runtime_sha256 = (
        _verify_and_snapshot_python_build_runtime(python_tool)
    )
    worktrees_added: list[Path] = []
    context: _BuiltReleaseContext | None = None
    yielded = False
    temporary_root = Path(tempfile.mkdtemp(prefix="noteaching-trusted-release-")).resolve(strict=True)
    neutral_root = _safe_verifier_worktree(temporary_root, "neutral")
    brand_root = _safe_verifier_worktree(temporary_root, "brand")
    powershell = _trusted_windows_powershell()
    git_path = git_tool.path
    try:
        main_head, brand_head = _require_release_launcher_is_clean_main(repo_root)
        try:
            for root, head, channel in (
                (neutral_root, main_head, "neutral"),
                (brand_root, brand_head, "brand"),
            ):
                _run_local_checked(
                    [str(git_path), "worktree", "add", "--detach", str(root), head],
                    cwd=repo_root,
                    label=f"创建 verifier-owned {channel} detached worktree",
                    timeout=5 * 60,
                )
                worktrees_added.append(root)
                _require(root.resolve(strict=True).parent == temporary_root,
                         "git worktree 创建后路径逃逸 verifier 临时根。")

            trusted_file_sha256 = {
                relative: sha256_file(neutral_root / Path(relative))
                for relative in TRUSTED_RELEASE_FILES
            }
            for relative, expected_sha in trusted_file_sha256.items():
                _require(sha256_file(repo_root / Path(relative)) == expected_sha,
                         f"启动器发布代码不等于初始 main HEAD：{relative}")
            _assert_brand_source_boundary(
                repo_root, neutral_root, brand_root, main_head, brand_head
            )
            _copy_verified_fanuc_runtime(args.runtime_source, (neutral_root, brand_root))

            installer_gates: dict[str, Path] = {}
            for channel, root in (("neutral", neutral_root), ("brand", brand_root)):
                _verify_release_toolchain_dependencies(
                    build_tools, toolchain_closures, protected_tool_roots
                )
                _run_local_checked(
                    [
                        str(powershell), "-NoLogo", "-NoProfile", "-NonInteractive",
                        "-ExecutionPolicy", "Bypass", "-File",
                        str(root / "scripts" / "build_installer.ps1"),
                        "-AppVersion", args.version, "-Channel", channel,
                        "-MSBuildExecutable", str(msbuild_tool.path),
                        "-MSBuildSha256", msbuild_tool.sha256,
                        "-WinDeployQtExecutable", str(windeployqt_tool.path),
                        "-WinDeployQtSha256", windeployqt_tool.sha256,
                        "-InnoCompilerExecutable", str(iscc_tool.path),
                        "-InnoCompilerSha256", iscc_tool.sha256,
                        "-PythonExecutable", str(python_tool.path),
                        "-PythonSha256", python_tool.sha256,
                    ],
                    cwd=root,
                    label=f"{channel} clean-HEAD Rebuild + Inno",
                    timeout=90 * 60,
                )
                installer_gates[channel] = (
                    root / "dist" / "release-gates" / f"installer-{channel}-{args.version}.json"
                )
                _require(installer_gates[channel].is_file(),
                         f"{channel} build 未产生 canonical installer gate。")
                _verify_release_toolchain_dependencies(
                    build_tools, toolchain_closures, protected_tool_roots
                )

            brand_patch = _create_single_exe_patch(brand_root, args.version)
            pair_path = (
                neutral_root / "dist" / "release-gates" / f"release-pair-{args.version}.json"
            )
            _run_local_checked(
                [
                    str(powershell), "-NoLogo", "-NoProfile", "-NonInteractive",
                    "-ExecutionPolicy", "Bypass", "-File",
                    str(neutral_root / "scripts" / "verify_release_pair.ps1"),
                    "-NeutralInstallerGateReport", str(installer_gates["neutral"]),
                    "-BrandInstallerGateReport", str(installer_gates["brand"]),
                    "-OutputPath", str(pair_path),
                    "-PythonExecutable", str(python_tool.path),
                    "-PythonSha256", python_tool.sha256,
                ],
                cwd=neutral_root,
                label="创建 clean-HEAD 双通道 pair gate",
                timeout=30 * 60,
            )

            report = prepare_dual_candidate(
                args.version,
                neutral_root / "dist" / "QtWidgetsApplication4",
                neutral_root / "dist" / "installer" / _expected_installer_name("neutral", args.version),
                brand_root / "dist" / "QtWidgetsApplication4",
                brand_root / "dist" / "installer" / _expected_installer_name("brand", args.version),
                brand_patch,
                pair_path,
                args.notes,
            )

            # Regressions are intentionally inside the same parent process phase and before
            # any signing-key read, password read, socket, or remote publish lock.
            for command, cwd, label, timeout in (
                ([str(python_tool.path), "-I", "-B",
                  str(neutral_root / "scripts" / "tests" / "verify_ota_release_gate.py")],
                 neutral_root, "OTA client/release static regression", 10 * 60),
                ([str(python_tool.path), "-I", "-B",
                  str(neutral_root / "scripts" / "tests" / "test_upload_ota_offline.py")],
                 neutral_root, "OTA offline publish regression", 15 * 60),
                ([str(powershell), "-NoLogo", "-NoProfile", "-NonInteractive",
                  "-ExecutionPolicy", "Bypass", "-File",
                  str(neutral_root / "scripts" / "tests" / "test_release_packaging_gates.ps1")],
                 neutral_root, "release packaging gate regression", 30 * 60),
            ):
                _run_local_checked(command, cwd=cwd, label=label, timeout=timeout)

            context = _BuiltReleaseContext(
                report=report,
                verifier_root=neutral_root,
                brand_verifier_root=brand_root,
                main_head=main_head,
                brand_head=brand_head,
                trusted_file_sha256=trusted_file_sha256,
                git_tool=git_tool,
                gh_tool=gh_tool,
                python_tool=python_tool,
                build_tools=build_tools,
                toolchain_closures=toolchain_closures,
                protected_tool_roots=protected_tool_roots,
                python_runtime_root=python_runtime_root,
                python_runtime_sha256=python_runtime_sha256,
            )
            _revalidate_built_release_context(repo_root, context)
            yielded = True
            yield context
        finally:
            cleanup_errors = context.cleanup_errors if context is not None else []
            for root in reversed(worktrees_added):
                try:
                    _require(root.parent == temporary_root and root.name in {"neutral", "brand"},
                             "拒绝清理不在 verifier 临时根下的 worktree。")
                    _run_local_checked(
                        [str(git_path), "worktree", "remove", "--force", str(root)],
                        cwd=repo_root,
                        label=f"清理 verifier worktree {root.name}",
                        timeout=10 * 60,
                    )
                except Exception as exc:
                    cleanup_errors.append(f"worktree {root.name}: {type(exc).__name__}")
            try:
                if temporary_root.exists():
                    shutil.rmtree(temporary_root)
            except Exception as exc:
                cleanup_errors.append(f"temporary root: {type(exc).__name__}")
            active_error = sys.exc_info()[1]
            if cleanup_errors and active_error is not None and hasattr(active_error, "add_note"):
                active_error.add_note(
                    "verifier 本地清理同时失败（主异常保持不变）："
                    + "；".join(cleanup_errors)
                )
            elif cleanup_errors and not yielded:
                raise ReleaseGateError("verifier 本地清理失败：" + "；".join(cleanup_errors))
    finally:
        _ACTIVE_GIT_TOOL, _ACTIVE_GH_TOOL, _ACTIVE_PYTHON_TOOL = (
            previous_git, previous_gh, previous_python
        )


def _revalidate_built_release_context(repo_root: Path, context: _BuiltReleaseContext) -> None:
    """Re-bind refs, trusted bytes, and tools immediately before every external phase."""
    _verify_trusted_tool_unchanged(context.git_tool)
    _verify_trusted_tool_unchanged(context.gh_tool)
    _verify_trusted_tool_unchanged(context.python_tool)
    _verify_release_toolchain_dependencies(
        context.build_tools,
        context.toolchain_closures,
        context.protected_tool_roots,
    )
    runtime_root, runtime_sha256 = _snapshot_python_build_runtime(context.python_tool)
    _require(runtime_root == context.python_runtime_root
             and runtime_sha256 == context.python_runtime_sha256,
             "Python/PyInstaller build runtime 在候选构建后发生变化。")
    _require(_git_text(repo_root, "rev-parse", "refs/heads/main") == context.main_head
             and _git_text(repo_root, "rev-parse", "refs/heads/hk-pathlynx-corpla")
             == context.brand_head,
             "本地 release refs 在 clean build 后发生变化。")
    _require(_git_text(context.verifier_root, "rev-parse", "HEAD") == context.main_head,
             "neutral verifier worktree HEAD 已漂移。")
    _require(_git_text(context.brand_verifier_root, "rev-parse", "HEAD")
             == context.brand_head,
             "brand verifier worktree HEAD 已漂移。")
    for channel, root in (
        ("neutral", context.verifier_root),
        ("brand", context.brand_verifier_root),
    ):
        tracked_dirty = _git_text(
            root, "status", "--porcelain=v1", "--untracked-files=no"
        )
        _require(not tracked_dirty, f"{channel} verifier worktree 的 tracked 字节已漂移。")
        for relative, expected_sha in context.trusted_file_sha256.items():
            _require(sha256_file(root / Path(relative)) == expected_sha,
                     f"外发前 {channel} 受信任发布文件已漂移：{relative}")
    pair = context.report.get("pairGate", {})
    _require(isinstance(pair, dict)
             and pair.get("neutral", {}).get("head") == context.main_head
             and pair.get("brand", {}).get("head") == context.brand_head,
             "候选 pair gate 未绑定初始双分支 HEAD。")


def _normalize_github_remote_url(value: str) -> str | None:
    text = value.strip()
    patterns = (
        r"https://github\.com/([^/]+/[^/]+?)(?:\.git)?/?$",
        r"git@github\.com:([^/]+/[^/]+?)(?:\.git)?$",
        r"git@github\.com-443:([^/]+/[^/]+?)(?:\.git)?$",
        r"ssh://git@github\.com/([^/]+/[^/]+?)(?:\.git)?/?$",
        r"ssh://git@ssh\.github\.com:443/([^/]+/[^/]+?)(?:\.git)?/?$",
    )
    for pattern in patterns:
        match = re.fullmatch(pattern, text, re.IGNORECASE)
        if match is not None:
            return match.group(1)
    return None


def _verify_expected_origin_identity(repo_root: Path) -> None:
    expected = EXPECTED_GITHUB_REPOSITORY.casefold()
    for arguments, label in (
        (("remote", "get-url", "--all", "origin"), "fetch"),
        (("remote", "get-url", "--push", "--all", "origin"), "push"),
    ):
        values = [line.strip() for line in _git_text(repo_root, *arguments).splitlines()
                  if line.strip()]
        _require(len(values) == 1
                 and _normalize_github_remote_url(values[0]) is not None
                 and _normalize_github_remote_url(values[0]).casefold() == expected,
                 f"origin {label} URL 必须唯一且固定为 {EXPECTED_GITHUB_REPOSITORY}。")


def _verify_pushed_origin_heads(repo_root: Path, report: dict[str, Any]) -> None:
    _verify_expected_origin_identity(repo_root)
    pair = report.get("pairGate")
    _require(isinstance(pair, dict), "trusted candidate 缺 pairGate，无法确认已 push refs。")
    expected: dict[str, str] = {}
    for ref, channel in (
        ("refs/heads/main", "neutral"),
        ("refs/heads/hk-pathlynx-corpla", "brand"),
    ):
        node = pair.get(channel)
        _require(isinstance(node, dict)
                 and isinstance(node.get("head"), str)
                 and re.fullmatch(r"[0-9a-f]{40}", node["head"]) is not None,
                 f"trusted candidate 的 {channel} head 非法。")
        expected[ref] = node["head"]
    output = _git_text(
        repo_root,
        "ls-remote",
        "--heads",
        "origin",
        "refs/heads/main",
        "refs/heads/hk-pathlynx-corpla",
    )
    actual: dict[str, str] = {}
    for line in output.splitlines():
        fields = line.split()
        if len(fields) == 2:
            actual[fields[1]] = fields[0]
    _require(actual == expected,
             f"正式外发前 origin refs 未与本次 clean build 对齐：expected={expected}, actual={actual}")


def _run_gh_checked(
    gh: str,
    arguments: list[str],
    *,
    cwd: Path,
    label: str,
    timeout: int = 10 * 60,
) -> subprocess.CompletedProcess[str]:
    return _run_local_checked(
        [gh, *arguments],
        cwd=cwd,
        label=label,
        timeout=timeout,
        allow_github_credentials=True,
    )


def _run_gh_api_expect_not_found(
    gh: str,
    endpoint: str,
    *,
    cwd: Path,
    label: str,
) -> None:
    if _ACTIVE_GH_TOOL is not None:
        _verify_trusted_tool_unchanged(_ACTIVE_GH_TOOL)
    try:
        completed = subprocess.run(
            [gh, "api", endpoint],
            cwd=str(cwd),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=5 * 60,
            check=False,
            stdin=subprocess.DEVNULL,
            env=_sanitized_local_environment(allow_github_credentials=True),
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise ReleaseGateError(f"{label} 查询失败：{exc}") from exc
    _require(completed.returncode != 0, f"{label} 已存在，拒绝继续外发。")
    not_found_text = (completed.stdout + "\n" + completed.stderr).lower()
    _require("404" in not_found_text or "not found" in not_found_text,
             f"无法确认 {label} 不存在；可能是鉴权或网络错误。")


def _read_authenticated_github_ref(
    gh: str,
    repo_root: Path,
    repo_name: str,
    ref_path: str,
    *,
    label: str,
) -> dict[str, Any]:
    payload = _run_gh_checked(
        gh,
        ["api", f"repos/{repo_name}/git/ref/{ref_path}"],
        cwd=repo_root,
        label=label,
    ).stdout
    value = _load_json_bytes(payload.encode("utf-8"), label)
    expected_ref = f"refs/{ref_path}"
    _require(isinstance(value, dict)
             and value.get("ref") == expected_ref
             and isinstance(value.get("object"), dict)
             and value["object"].get("type") in {"commit", "tag"}
             and isinstance(value["object"].get("sha"), str)
             and re.fullmatch(r"[0-9a-f]{40}", value["object"]["sha"]) is not None,
             f"{label} 返回的 GitHub ref 身份非法。")
    return value


def _resolve_authenticated_github_tag_commit(
    gh: str,
    repo_root: Path,
    repo_name: str,
    tag: str,
) -> str:
    ref = _read_authenticated_github_ref(
        gh,
        repo_root,
        repo_name,
        f"tags/{tag}",
        label="GitHub 公开 tag ref 回读",
    )
    target = ref["object"]
    for depth in range(5):
        object_type = target.get("type")
        sha = target.get("sha")
        _require(isinstance(sha, str) and re.fullmatch(r"[0-9a-f]{40}", sha) is not None,
                 "GitHub tag object SHA 非法。")
        if object_type == "commit":
            return sha
        _require(object_type == "tag" and depth < 4,
                 "GitHub annotated tag 嵌套过深或目标类型非法。")
        tag_payload = _run_gh_checked(
            gh,
            ["api", f"repos/{repo_name}/git/tags/{sha}"],
            cwd=repo_root,
            label="GitHub annotated tag 目标回读",
        ).stdout
        tag_object = _load_json_bytes(
            tag_payload.encode("utf-8"), "GitHub annotated tag metadata"
        )
        _require(isinstance(tag_object, dict)
                 and isinstance(tag_object.get("object"), dict),
                 "GitHub annotated tag metadata 非法。")
        target = tag_object["object"]
    raise ReleaseGateError("GitHub annotated tag 解析未收敛。")


def _github_release_preflight(
    repo_root: Path,
    args: argparse.Namespace,
    report: dict[str, Any],
) -> dict[str, str]:
    gh_path = (
        _ACTIVE_GH_TOOL.path
        if _ACTIVE_GH_TOOL is not None
        else Path(getattr(args, "gh_exe", DEFAULT_GH_EXE)).resolve(strict=True)
    )
    gh = str(gh_path)
    _run_gh_checked(gh, ["auth", "status"], cwd=repo_root, label="GitHub auth preflight")
    repo_metadata_text = _run_gh_checked(
        gh,
        ["repo", "view", EXPECTED_GITHUB_REPOSITORY,
         "--json", "nameWithOwner,viewerPermission"],
        cwd=repo_root,
        label="读取 GitHub repository identity",
    ).stdout.strip()
    repo_metadata = _load_json_bytes(
        repo_metadata_text.encode("utf-8"), "GitHub repository identity"
    )
    _require(isinstance(repo_metadata, dict)
             and repo_metadata.get("nameWithOwner") == EXPECTED_GITHUB_REPOSITORY
             and repo_metadata.get("viewerPermission") in {"ADMIN", "MAINTAIN", "WRITE"},
             f"GitHub repository 必须精确为 {EXPECTED_GITHUB_REPOSITORY} 且当前身份可写。")
    repo_name = EXPECTED_GITHUB_REPOSITORY
    pair = report.get("pairGate")
    _require(isinstance(pair, dict), "GitHub preflight 缺少双分支 pairGate。")
    for branch, channel in (("main", "neutral"), ("hk-pathlynx-corpla", "brand")):
        node = pair.get(channel)
        _require(isinstance(node, dict)
                 and isinstance(node.get("head"), str)
                 and re.fullmatch(r"[0-9a-f]{40}", node["head"]) is not None,
                 f"GitHub preflight 的 {channel} HEAD 非法。")
        remote_ref = _read_authenticated_github_ref(
            gh,
            repo_root,
            repo_name,
            f"heads/{branch}",
            label=f"GitHub API 回读 {branch} ref",
        )
        _require(remote_ref["object"].get("type") == "commit"
                 and remote_ref["object"].get("sha") == node["head"],
                 f"GitHub API 的 {branch} ref 未与本次 clean build 对齐。")
    tag = f"v{args.version}"
    # A pre-existing release makes ownership/asset identity ambiguous. Refuse before OTA;
    # recovery from a prior partial external publish is an explicit manual operation.
    _run_gh_api_expect_not_found(
        gh,
        f"repos/{repo_name}/releases/tags/{tag}",
        cwd=repo_root,
        label=f"GitHub Release {tag}",
    )
    _require(report.get("version") == args.version,
             "GitHub preflight 的版本与 trusted candidate 不一致。")
    _run_gh_api_expect_not_found(
        gh,
        f"repos/{repo_name}/git/ref/tags/{tag}",
        cwd=repo_root,
        label=f"GitHub tag ref {tag}",
    )
    return {"gh": gh, "repo": repo_name, "tag": tag}


def _publish_github_release(
    repo_root: Path,
    args: argparse.Namespace,
    report: dict[str, Any],
    preflight: dict[str, str],
) -> dict[str, Any]:
    gh = preflight["gh"]
    repo_name = preflight["repo"]
    tag = preflight["tag"]
    channels = report["channels"]
    assets = [Path(channels[channel]["installer"]["path"]) for channel in CHANNELS]
    for channel, asset in zip(CHANNELS, assets):
        node = channels[channel]["installer"]
        _require(asset.is_file() and sha256_file(asset) == node["sha256"]
                 and asset.stat().st_size == node["size"],
                 f"GitHub 上传前 {channel} installer 已变化。")
    release_notes = args.notes.strip() or f"NoTeaching-Robot {args.version} 双通道正式发布"
    _run_gh_checked(
        gh,
        [
            "release", "create", tag,
            str(assets[0]), str(assets[1]),
            "--repo", repo_name,
            "--target", "main",
            "--title", f"NoTeaching-Robot v{args.version}",
            "--notes", release_notes,
            "--draft",
        ],
        cwd=repo_root,
        label="创建 main-target GitHub draft 并上传双通道安装包",
        timeout=30 * 60,
    )

    def read_and_validate_release(*, expected_draft: bool, label: str) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
        release_json = _run_gh_checked(
            gh,
            ["api", f"repos/{repo_name}/releases/tags/{tag}"],
            cwd=repo_root,
            label=label,
        ).stdout
        metadata = _load_json_bytes(release_json.encode("utf-8"), label)
        _require(isinstance(metadata, dict)
                 and metadata.get("tag_name") == tag
                 and metadata.get("target_commitish") == "main"
                 and metadata.get("draft") is expected_draft
                 and metadata.get("prerelease") is False,
                 "GitHub Release tag/target/draft 回读不一致（target 必须为 main）。")
        remote_assets = metadata.get("assets")
        _require(isinstance(remote_assets, list)
                 and len(remote_assets) == len(assets)
                 and all(isinstance(item, dict) for item in remote_assets)
                 and {item.get("name") for item in remote_assets}
                 == {asset.name for asset in assets},
                 "GitHub Release 远端资产集合不是恰好两份受信安装包。")
        remote_assets_by_name = {item["name"]: item for item in remote_assets}
        for channel, asset in zip(CHANNELS, assets):
            expected = channels[channel]["installer"]
            remote = remote_assets_by_name[asset.name]
            _require(remote.get("state") == "uploaded"
                     and remote.get("size") == expected["size"],
                     f"GitHub Release {channel} 资产 metadata 状态/大小不一致。")
        return metadata, remote_assets_by_name

    _draft_metadata, draft_assets = read_and_validate_release(
        expected_draft=True,
        label="回读 GitHub draft metadata",
    )
    with tempfile.TemporaryDirectory(prefix="noteaching-gh-readback-") as readback_dir:
        _run_gh_checked(
            gh,
            ["release", "download", tag, "--repo", repo_name, "--dir", readback_dir],
            cwd=repo_root,
            label="认证下载回读 GitHub draft 双通道资产",
            timeout=30 * 60,
        )
        for channel, asset in zip(CHANNELS, assets):
            downloaded = Path(readback_dir) / asset.name
            expected = channels[channel]["installer"]
            _require(downloaded.is_file()
                     and downloaded.stat().st_size == expected["size"]
                     and sha256_file(downloaded) == expected["sha256"],
                     f"GitHub draft {channel} 资产回读 size/hash 不一致。")
    expected_main_head = report["pairGate"]["neutral"]["head"]
    main_ref_json = _run_gh_checked(
        gh,
        ["api", f"repos/{repo_name}/git/ref/heads/main"],
        cwd=repo_root,
        label="GitHub draft 公开前回读 main ref",
    ).stdout
    main_ref = _load_json_bytes(
        main_ref_json.encode("utf-8"), "GitHub main ref metadata"
    )
    _require(isinstance(main_ref, dict)
             and main_ref.get("ref") == "refs/heads/main"
             and isinstance(main_ref.get("object"), dict)
             and main_ref["object"].get("type") == "commit"
             and main_ref["object"].get("sha") == expected_main_head,
             "GitHub draft 的 target=main 未绑定本次已验证并推送的 main commit。")

    # The draft is the quarantine boundary: nothing becomes public until metadata,
    # the exact asset set/sizes, authenticated downloaded hashes, and main binding
    # have all passed.  A failure above intentionally leaves a non-public draft for
    # explicit inspection/cleanup instead of exposing bad assets.
    _run_gh_checked(
        gh,
        ["release", "edit", tag, "--repo", repo_name, "--draft=false"],
        cwd=repo_root,
        label="公开已完整验证的 GitHub Release",
    )
    metadata, published_assets = read_and_validate_release(
        expected_draft=False,
        label="公开后回读 GitHub Release metadata",
    )
    _require(
        {
            name: (item.get("id"), item.get("size"), item.get("state"))
            for name, item in published_assets.items()
        }
        == {
            name: (item.get("id"), item.get("size"), item.get("state"))
            for name, item in draft_assets.items()
        },
        "GitHub Release 公开后资产身份/大小/状态与已验 draft 不一致。",
    )
    tag_commit = _resolve_authenticated_github_tag_commit(
        gh, repo_root, repo_name, tag
    )
    _require(tag_commit == expected_main_head,
             "公开后的 GitHub Release tag 未绑定本次 main commit。")
    return {
        "tag": tag,
        "target": "main",
        "commit": tag_commit,
        "url": metadata.get("html_url", ""),
        "assets": {
            channel: {
                "name": channels[channel]["installer"]["file"],
                "sha256": channels[channel]["installer"]["sha256"],
                "size": channels[channel]["installer"]["size"],
            }
            for channel in CHANNELS
        },
    }


def _validate_windows_private_acl_metadata(metadata: Any) -> None:
    _require(isinstance(metadata, dict)
             and set(metadata) == {
                 "ownerSid", "currentSid", "protected", "currentCanRead", "rules", "sha256"
             },
             "SSH 密码文件 ACL 元数据 schema 非法。")
    current_sid = metadata.get("currentSid")
    owner_sid = metadata.get("ownerSid")
    rules = metadata.get("rules")
    _require(isinstance(current_sid, str) and re.fullmatch(r"S-1(?:-[0-9]+)+", current_sid)
             and isinstance(owner_sid, str)
             and metadata.get("protected") is True
             and metadata.get("currentCanRead") is True
             and isinstance(rules, list) and rules
             and isinstance(metadata.get("sha256"), str)
             and SHA256_RE.fullmatch(metadata["sha256"]) is not None,
             "SSH 密码文件 ACL 必须关闭继承并允许当前用户读取。")
    allowed_sids = {current_sid, "S-1-5-18", "S-1-5-32-544"}
    _require(owner_sid in allowed_sids,
             "SSH 密码文件 owner 必须是当前用户、SYSTEM 或 Administrators。")
    for rule in rules:
        _require(isinstance(rule, dict)
                 and set(rule) == {"sid", "type", "inherited"}
                 and rule.get("sid") in allowed_sids
                 and rule.get("type") == "Allow"
                 and rule.get("inherited") is False,
                 "SSH 密码文件 ACL 含其他用户/组、Deny 或继承规则。")


def _validate_private_password_file(
    path_value: os.PathLike[str] | str,
) -> tuple[Path, str]:
    try:
        path = Path(path_value).expanduser().resolve(strict=True)
    except OSError as exc:
        raise ReleaseGateError("SSH 密码文件不存在或不可访问。") from exc
    _require(path.is_file() and not path.is_symlink() and not _path_is_reparse_point(path),
             "SSH 密码文件必须是非链接普通文件。")
    _require(path.stat().st_size <= 4096, "SSH 密码文件过大。")
    repo_root = Path(__file__).resolve().parents[1]
    try:
        path.relative_to(repo_root)
    except ValueError:
        pass
    else:
        raise ReleaseGateError("SSH 密码文件必须位于项目目录之外。")
    if os.name == "posix":
        _require(stat_module.S_IMODE(path.stat().st_mode) & 0o077 == 0,
                 "SSH 密码文件权限必须是 0600 或更严格。")
        expected_sha256 = sha256_file(path)
    elif os.name == "nt":
        powershell = _trusted_windows_powershell()
        script = (
            "$ErrorActionPreference='Stop'; $p=$env:NO_TEACHING_SECRET_PATH; $acl=Get-Acl -LiteralPath $p; "
            "$current=[System.Security.Principal.WindowsIdentity]::GetCurrent().User.Value; "
            "$owner=$acl.Owner; try{$owner=([System.Security.Principal.NTAccount]$owner).Translate("
            "[System.Security.Principal.SecurityIdentifier]).Value}catch{"
            "$owner=([System.Security.Principal.SecurityIdentifier]$owner).Value}; "
            "$rules=@(); $canRead=$false; foreach($rule in $acl.Access){"
            "$sid=$rule.IdentityReference.Translate([System.Security.Principal.SecurityIdentifier]).Value; "
            "$kind=[string]$rule.AccessControlType; if($sid -eq $current -and $kind -eq 'Allow' -and "
            "(($rule.FileSystemRights -band [System.Security.AccessControl.FileSystemRights]::ReadData) -ne 0)){"
            "$canRead=$true}; $rules+=,[ordered]@{sid=$sid;type=$kind;inherited=[bool]$rule.IsInherited}}; "
            "$o=[ordered]@{ownerSid=$owner;currentSid=$current;protected=[bool]$acl.AreAccessRulesProtected;"
            "currentCanRead=$canRead;rules=$rules;sha256=(Get-FileHash -LiteralPath $p -Algorithm SHA256).Hash.ToLowerInvariant()};"
            "$o|ConvertTo-Json -Compress -Depth 5"
        )
        completed = subprocess.run(
            [str(powershell), "-NoLogo", "-NoProfile", "-NonInteractive",
             "-ExecutionPolicy", "Bypass", "-Command", script],
            cwd=str(repo_root),
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=2 * 60,
            check=False,
            env=_sanitized_local_environment({"NO_TEACHING_SECRET_PATH": str(path)}),
        )
        _require(completed.returncode == 0,
                 "SSH 密码文件 owner/ACL 无法验证。")
        metadata = _load_json_bytes(
            completed.stdout.strip().encode("utf-8"), "SSH 密码文件 ACL 元数据"
        )
        _validate_windows_private_acl_metadata(metadata)
        expected_sha256 = metadata["sha256"]
    else:
        raise ReleaseGateError("当前平台不支持安全读取 SSH 密码文件。")
    _require(sha256_file(path) == expected_sha256,
             "SSH 密码文件在权限验证后发生变化。")
    return path, expected_sha256


def _read_ssh_password_after_gates(args: argparse.Namespace) -> str:
    _require("OTA_SSH_PASSWORD" not in os.environ,
             "已永久禁用 OTA_SSH_PASSWORD 启动环境；请使用受限文件或门禁后 stdin。")
    password_file = getattr(args, "ssh_password_file", None)
    password_stdin = bool(getattr(args, "ssh_password_stdin", False))
    _require(bool(password_file) ^ password_stdin,
             "必须且只能选择 --ssh-password-file 或 --ssh-password-stdin。")
    if password_file:
        password_path, expected_sha256 = _validate_private_password_file(password_file)
        raw_bytes = password_path.read_bytes()
        _require(sha256_bytes(raw_bytes) == expected_sha256
                 and not password_path.is_symlink()
                 and not _path_is_reparse_point(password_path),
                 "SSH 密码文件在读取期间发生变化。")
        raw = raw_bytes.decode("utf-8", errors="strict")
        if raw.endswith("\r\n"):
            password = raw[:-2]
        elif raw.endswith("\n"):
            password = raw[:-1]
        else:
            password = raw
    elif sys.stdin.isatty():
        password = getpass.getpass("OTA SSH password (read only after all local gates): ")
    else:
        password = sys.stdin.readline(4097)
        _require(len(password) <= 4096, "stdin SSH 密码超过上限。")
        password = password.rstrip("\r\n")
    _require(1 <= len(password) <= 512
             and "\r" not in password and "\n" not in password
             and all(ord(ch) >= 0x20 and ord(ch) != 0x7F for ch in password),
             "SSH 密码为空、过长或含控制字符。")
    return password


def _publish_ota_production(
    args: argparse.Namespace,
    trusted_report: _TrustedPublishCandidate,
    *,
    signer_factory: Any,
    ssh_factory: Any,
) -> dict[str, Any]:
    signing_relative = "scripts/ota_manifest_signing.py"
    manifest_signer = signer_factory(
        args.signing_key,
        signing_module_path=trusted_report._verifier_root / signing_relative,
        expected_module_sha256=trusted_report._trusted_file_sha256[signing_relative],
    )
    password = _read_ssh_password_after_gates(args)
    ssh = ssh_factory(
        args.server,
        args.port,
        args.user,
        password,
        known_hosts=args.known_hosts,
        host_key_sha256=args.host_key_sha256,
    )
    publish_result: dict[str, Any] | None = None
    primary_error: BaseException | None = None
    primary_traceback = None
    close_errors: list[str] = []
    sftp = None
    try:
        transport = ssh.get_transport()
        _require(transport is not None and transport.is_active(),
                 "SSH transport 在打开 SFTP 前已失效。")
        transport.set_keepalive(15)
        sftp = _DeadlineSftpClient(
            ssh.open_sftp(),
            total_timeout=SFTP_TOTAL_TIMEOUT_SECONDS,
            io_timeout=SFTP_IO_TIMEOUT_SECONDS,
        )
        publish_result = publish_dual_with_sftp(sftp, trusted_report, manifest_signer)
    except BaseException as exc:
        primary_error = exc
        primary_traceback = exc.__traceback__
    finally:
        if sftp is not None:
            try:
                sftp.close()
            except Exception as exc:
                close_errors.append(f"sftp:{type(exc).__name__}")
        try:
            ssh.close()
        except Exception as exc:
            close_errors.append(f"ssh:{type(exc).__name__}")
    if primary_error is not None:
        if close_errors and hasattr(primary_error, "add_note"):
            primary_error.add_note("SSH/SFTP 清理同时失败：" + ",".join(close_errors))
        raise primary_error.with_traceback(primary_traceback)
    assert publish_result is not None
    if close_errors:
        publish_result["sessionCleanup"] = {
            "status": "failed-after-commit",
            "errors": close_errors,
        }
    else:
        publish_result["sessionCleanup"] = {"status": "passed"}
    return publish_result


def trusted_release_dual(
    args: argparse.Namespace,
    *,
    _candidate_builder: Any = None,
    _signer_factory: Any = None,
    _ssh_factory: Any = None,
    _trust_factory: Any = None,
    _origin_verifier: Any = None,
    _github_preflight_factory: Any = None,
    _ota_publisher: Any = None,
    _github_publisher: Any = None,
) -> dict[str, Any]:
    """Build and publish both OTA and GitHub from one verifier-owned candidate.

    The build context remains alive through both external publications.  OTA is
    deliberately first; a failed OTA publication must never create a GitHub
    Release.  A GitHub failure after OTA is reported as a partial-publish
    ambiguity and is never disguised as an ordinary all-or-nothing failure.
    """
    parse_version(args.version)
    _validate_notes(args.notes)
    _require(args.server == PRODUCTION_HOST and args.port == PRODUCTION_PORT
             and args.user == PRODUCTION_USER,
             f"当前发版规则只允许生产端 {PRODUCTION_USER}@{PRODUCTION_HOST}:{PRODUCTION_PORT}。")
    _require(not getattr(args, "known_hosts", None)
             and isinstance(getattr(args, "host_key_sha256", None), str),
             "生产发版强制使用显式 --host-key-sha256；禁止依赖可漂移 known_hosts。")
    _normalize_host_fingerprint(args.host_key_sha256)
    _require("OTA_SSH_PASSWORD" not in os.environ,
             "已禁用启动环境 OTA_SSH_PASSWORD；密码只能在门禁后从受限文件或 stdin 读取。")
    repo_root = Path(__file__).resolve().parents[1]
    builder = _build_trusted_release_candidate if _candidate_builder is None else _candidate_builder
    signer_factory = make_manifest_signer if _signer_factory is None else _signer_factory
    ssh_factory = make_ssh if _ssh_factory is None else _ssh_factory
    trust_factory = trust_candidate_for_publish if _trust_factory is None else _trust_factory
    origin_verifier = _verify_pushed_origin_heads if _origin_verifier is None else _origin_verifier
    github_preflight_factory = (
        _github_release_preflight
        if _github_preflight_factory is None
        else _github_preflight_factory
    )
    ota_publisher = _publish_ota_production if _ota_publisher is None else _ota_publisher
    github_publisher = _publish_github_release if _github_publisher is None else _github_publisher
    release_context: _BuiltReleaseContext | None = None
    completed_result: dict[str, Any] | None = None
    with builder(args) as built_value:
        if isinstance(built_value, _BuiltReleaseContext):
            release_context = built_value
            _revalidate_built_release_context(repo_root, release_context)
            if _trust_factory is None:
                trusted_report = trust_factory(
                    release_context.report,
                    verifier_root=release_context.verifier_root,
                    trusted_file_sha256=release_context.trusted_file_sha256,
                )
            else:
                trusted_report = trust_factory(release_context.report)
        else:
            # Only dependency-injected offline tests may use the legacy report-shaped builder.
            _require(_candidate_builder is not None,
                     "生产 candidate builder 必须返回 verifier-owned release context。")
            trusted_report = trust_factory(built_value)
        # Build/regressions and the fresh in-process trust gate are complete before
        # any network preflight, signing-key read, password read, or remote lock.
        if release_context is not None:
            _revalidate_built_release_context(repo_root, release_context)
        origin_verifier(repo_root, trusted_report)
        github_preflight = github_preflight_factory(repo_root, args, trusted_report)
        if release_context is not None:
            _revalidate_built_release_context(repo_root, release_context)
        ota_result = ota_publisher(
            args,
            trusted_report,
            signer_factory=signer_factory,
            ssh_factory=ssh_factory,
        )
        try:
            if release_context is not None:
                _revalidate_built_release_context(repo_root, release_context)
            github_result = github_publisher(
                repo_root, args, trusted_report, github_preflight
            )
        except Exception as exc:
            raise ReleaseGateError(
                "OTA 双通道已成功外发，但 GitHub Release 发布或远端回读失败，"
                "当前处于部分发布/外部状态有歧义；禁止自动重试 OTA 或覆盖 Release，"
                "必须人工核对远端状态。"
            ) from exc
        completed_result = {
            "version": args.version,
            "ota": ota_result,
            "github": github_result,
        }
    assert completed_result is not None
    if release_context is not None and release_context.cleanup_errors:
        completed_result["localCleanup"] = {
            "status": "failed-after-external-publication",
            "errors": list(release_context.cleanup_errors),
        }
    return completed_result


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="NoTeaching-Robot OTA 双通道硬门禁发布工具")
    subparsers = parser.add_subparsers(dest="command", required=True)

    compute = subparsers.add_parser("compute", help="只校验一个 dist 并输出 inventory/payloadHash，不联网")
    compute.add_argument("--version", required=True)
    compute.add_argument("--channel", choices=CHANNELS, required=True)
    compute.add_argument("--dist-dir", required=True)

    prepare = subparsers.add_parser("prepare-dual", help="离线校验双通道并生成候选报告")
    prepare.add_argument("--version", required=True)
    prepare.add_argument("--neutral-dist-dir", required=True)
    prepare.add_argument("--neutral-installer", required=True)
    prepare.add_argument("--brand-dist-dir", required=True)
    prepare.add_argument("--brand-installer", required=True)
    prepare.add_argument("--brand-patch", required=True)
    prepare.add_argument("--pair-gate", required=True,
                         help="verify_release_pair.ps1 生成的 release-pair PASS 报告")
    prepare.add_argument("--notes", default="")
    prepare.add_argument("--report", required=True)
    prepare.add_argument("--force-report", action="store_true")

    def add_remote_arguments(command_parser: argparse.ArgumentParser) -> None:
        command_parser.add_argument("--server", default=PRODUCTION_HOST)
        command_parser.add_argument("--port", type=int, default=PRODUCTION_PORT)
        command_parser.add_argument("--user", default=PRODUCTION_USER)
        command_parser.add_argument(
            "--signing-key",
            default=str(Path.home() / ".codex" / "secrets" / "ota-release-signing-key.dpapi"),
            help="项目外 CurrentUser-DPAPI 私钥",
        )
        trust_group = command_parser.add_mutually_exclusive_group(required=True)
        trust_group.add_argument("--known-hosts", help="项目目录外的专用 known_hosts 文件")
        trust_group.add_argument("--host-key-sha256", help="OpenSSH SHA256:<base64> 主机指纹")

    publish = subparsers.add_parser(
        "publish-dual",
        help="已禁用：外部 candidate/report 永远没有发布权限",
    )
    publish.add_argument("--report", required=True)
    add_remote_arguments(publish)

    trusted = subparsers.add_parser(
        "trusted-release-dual",
        help="在 verifier-owned clean detached worktrees 内构建唯一候选并直接发布",
    )
    trusted.add_argument("--version", required=True)
    trusted.add_argument("--runtime-source", required=True,
                         help="仅作为权威 FANUC manifest 所列 tp/pc 的显式只读来源")
    trusted.add_argument("--notes", default="")
    trusted.add_argument("--git-exe", default=str(DEFAULT_GIT_EXE),
                         help="受信任且带有效 Authenticode 的 Git for Windows 绝对路径")
    trusted.add_argument("--gh-exe", default=str(DEFAULT_GH_EXE),
                         help="受信任且带有效 Authenticode 的 GitHub CLI 绝对路径")
    trusted.add_argument("--msbuild-exe", default=str(DEFAULT_MSBUILD_EXE),
                         help="受信任 Microsoft MSBuild.exe 绝对路径")
    trusted.add_argument("--windeployqt-exe", default=str(DEFAULT_WINDEPLOYQT_EXE),
                         help="受信任 Qt windeployqt.exe 绝对路径")
    trusted.add_argument("--iscc-exe", default=str(DEFAULT_ISCC_EXE),
                         help="受信任 Inno Setup ISCC.exe 绝对路径")
    password_group = trusted.add_mutually_exclusive_group(required=True)
    password_group.add_argument("--ssh-password-file",
                                help="门禁全过后才读取的项目外受限 UTF-8 单行密码文件")
    password_group.add_argument("--ssh-password-stdin", action="store_true",
                                help="门禁全过后才从 stdin/终端安全读取密码")
    add_remote_arguments(trusted)
    return parser


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    legacy_flags = {"--compute-only", "--payload-hash", "--seed-versions", "--channel", "--installer", "--patch"}
    if argv and argv[0].startswith("-") and argv[0] not in {"-h", "--help"}:
        used = sorted(set(argv) & legacy_flags)
        raise ReleaseGateError(
            "旧的单通道/裸哈希调用已禁用。请先运行 `upload_ota.py prepare-dual --help` "
            f"生成双通道候选报告，再运行 publish-dual。旧参数={used}"
        )

    args = _build_parser().parse_args(argv)
    if args.command == "compute":
        result = inspect_dist(args.dist_dir, args.channel, args.version)
        print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
        return 0
    if args.command == "prepare-dual":
        report = prepare_dual_candidate(
            args.version,
            args.neutral_dist_dir,
            args.neutral_installer,
            args.brand_dist_dir,
            args.brand_installer,
            args.brand_patch,
            args.pair_gate,
            args.notes,
        )
        report_path = _write_json_atomic(args.report, report, args.force_report)
        print(f"双通道候选报告已生成：{report_path}")
        print(f"version={report['version']} candidateSha256={report['candidateSha256']}")
        for channel in CHANNELS:
            channel_data = report["channels"][channel]
            print(
                f"{channel}: files={len(channel_data['dist']['inventory'])} "
                f"inventory={channel_data['dist']['inventoryHash']} "
                f"payload={channel_data['dist']['payloadHash']}"
            )
        return 0
    if args.command == "publish-dual":
        raise ReleaseGateError(
            "publish-dual 外部 candidate/report 模式已永久禁用，不能外发。"
            "请使用 trusted-release-dual；它不接受任何 dist/installer/gate/candidate 路径。"
        )
    if args.command == "trusted-release-dual":
        result = trusted_release_dual(args)
        print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
        return 0
    raise ReleaseGateError(f"未知命令：{args.command}")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ReleaseGateError as exc:
        print(f"发布门禁失败：{exc}", file=sys.stderr)
        raise SystemExit(2)
