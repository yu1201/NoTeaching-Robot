#!/usr/bin/python3 -I
# -*- coding: utf-8 -*-
"""OTA server administration API.

The service is deliberately small and fail-closed.  It runs as root, listens
only on loopback, and is intended to be exposed through the existing nginx
``/admin/`` reverse proxy.  Every request must carry ``X-Admin-Token``; the
token file is root-owned and must not be readable by group or other users.

FTP account mutations follow the deployed vsftpd layout without ever deleting
uploaded device data:

* accounts are members of ``ftpdata`` and use ``/srv/devicedata`` as chroot;
* ``/etc/vsftpd.userlist`` is the publication/allow-list boundary;
* every ordinary account is upload-only.  Its FTP chroot stays at the
  root-owned, non-writable ``/srv/devicedata``; Unix directory ownership binds
  it to the writable host path ``/srv/devicedata/data/<account>``;
* that directory is owned by the account and grouped to the private primary
  group of ``devicedata``.  Mode 2770 lets the administrator access it without
  granting another member of the shared ``ftpdata`` group any filesystem
  access;
* ``uploader`` is retired and may never be allow-listed.  ``devicedata`` is the
  only full-access account and cannot be deleted.

All file publication uses same-directory atomic replacement.  Account changes
are serialized in-process and with a deployment-shared ``flock`` because
``ThreadingHTTPServer`` or overlapping service processes otherwise permit lost
updates.  Each mutation and its rollback share one monotonic deadline.
"""

import errno
import hmac
import json
import logging
import os
import re
import shutil
import stat
import subprocess
import tempfile
import threading
import time
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

try:  # Linux-only modules; keeping import side effects out aids offline tests.
    import fcntl
    import grp
    import pwd
except ImportError:  # pragma: no cover - exercised implicitly by Windows imports
    fcntl = None
    grp = None
    pwd = None


LISTEN = ("127.0.0.1", 8091)
TOKEN_FILE = "/opt/ota-admin/token"
DATA_DIR = "/srv/devicedata/data"
USERLIST = "/etc/vsftpd.userlist"
USER_CONF_DIR = "/etc/vsftpd_user_conf"
FTP_GROUP = "ftpdata"
FTP_HOME = "/srv/devicedata"
NOLOGIN_SHELL = "/usr/sbin/nologin"
# Ubuntu's default login.defs UID_MIN is 1000.  Never let a root/system
# identity become mutable through the FTP account administration API.
MIN_MANAGED_UID = 1000

USERADD = "/usr/sbin/useradd"
USERDEL = "/usr/sbin/userdel"
GROUPDEL = "/usr/sbin/groupdel"
CHPASSWD = "/usr/sbin/chpasswd"
ACCOUNT_LOCK_FILE = "/run/no-teaching-ota/ota-accounts.lock"
REQUIRED_LOCK_OWNER_UID = 0

FIXED_ACCOUNT_PERMISSIONS = {"devicedata": "full"}
RETIRED_ACCOUNTS = frozenset({"uploader"})
PROTECTED = frozenset((*FIXED_ACCOUNT_PERMISSIONS, *RETIRED_ACCOUNTS))
NAME_RE = re.compile(r"^[a-z][a-z0-9_-]{2,31}$")
MAX_BODY_BYTES = 16 * 1024
MAX_CONFIG_BYTES = 64 * 1024
COMMAND_TIMEOUT_SECONDS = 10
OPERATION_TIMEOUT_SECONDS = 12
ROLLBACK_RESERVE_SECONDS = 2
LOCK_POLL_SECONDS = 0.05
MAX_DIRECTORY_SCAN_ENTRIES = 250_000
MAX_DIRECTORY_SCAN_DEPTH = 64
MIN_PASSWORD_CHARS = 8
MAX_PASSWORD_CHARS = 256
MIN_TOKEN_CHARS = 32
MAX_TOKEN_CHARS = 512
RETRYABLE_TIMEOUT_MESSAGE = "操作超时，结果可能不确定，请使用相同参数重试确认"
UPLOAD_ONLY_CONFIG_PREFIX = (
    "download_enable=NO\n"
    "chmod_enable=NO\n"
    "file_open_mode=0440\n"
    "local_umask=007\n"
    "cmds_denied=DELE,RMD,RNFR,RNTO,APPE,REST\n"
)
SAFE_EXTRA_UPLOAD_DIRECTIVES = {"hide_ids": frozenset({"YES"})}
DEVICE_DIRECTORY_MODE = 0o2770
ENFORCE_POSIX_OWNERSHIP = os.name == "posix"

_account_lock = threading.RLock()
_stats_lock = threading.Lock()
_stats_cache = {"at": 0.0, "data": None}

LOG = logging.getLogger("ota-admin")


class ApiError(Exception):
    """An expected failure whose deliberately-safe message may reach a client."""

    status = 400

    def __init__(self, public_message):
        super().__init__(public_message)
        self.public_message = public_message


class ConflictError(ApiError):
    status = 409


class NotFoundError(ApiError):
    status = 404


class LengthRequiredError(ApiError):
    status = 411


class PayloadTooLargeError(ApiError):
    status = 413


class UnsupportedMediaTypeError(ApiError):
    status = 415


class MethodNotAllowedError(ApiError):
    status = 405


class IntegrityError(RuntimeError):
    """A local configuration invariant is broken; details stay server-side."""


class CommandError(RuntimeError):
    """A system command failed without retaining stdout, stderr, or stdin."""

    def __init__(self, executable, returncode):
        super().__init__("system command failed")
        self.executable = os.path.basename(executable)
        self.returncode = returncode


class CommandTimeoutError(CommandError):
    """A command deadline expired; its external side effect may have committed."""

    def __init__(self, executable):
        super().__init__(executable, -1)


class OperationTimeoutError(RuntimeError):
    """The bounded account-mutation window has been exhausted."""


class OperationDeadline:
    """One monotonic budget shared by a mutation and all of its rollback work."""

    def __init__(self, seconds=OPERATION_TIMEOUT_SECONDS):
        self.expires_at = time.monotonic() + seconds

    def remaining(self):
        remaining = self.expires_at - time.monotonic()
        if remaining <= 0:
            raise OperationTimeoutError("account operation deadline exceeded")
        return remaining

    def command_timeout(self, reserve_seconds=0):
        usable = self.remaining() - reserve_seconds
        if usable <= 0:
            raise OperationTimeoutError("account operation deadline exceeded")
        return min(COMMAND_TIMEOUT_SECONDS, usable)


class DirectoryScanBudget:
    """One entry/depth budget shared by every directory scanned for one request."""

    def __init__(self, max_entries=None, max_depth=None):
        self.max_entries = (
            MAX_DIRECTORY_SCAN_ENTRIES if max_entries is None else max_entries
        )
        self.max_depth = MAX_DIRECTORY_SCAN_DEPTH if max_depth is None else max_depth
        self.entries = 0

    def consume(self, depth, deadline):
        deadline.remaining()
        if depth > self.max_depth:
            raise IntegrityError("managed device tree exceeds the maximum scan depth")
        self.entries += 1
        if self.entries > self.max_entries:
            raise IntegrityError("managed device tree exceeds the maximum entry budget")


def run(cmd, input_text=None, deadline=None, reserve_seconds=0):
    """Run one fixed argv command with a minimal environment and no shell."""
    timeout = COMMAND_TIMEOUT_SECONDS
    if deadline is not None:
        timeout = deadline.command_timeout(reserve_seconds=reserve_seconds)
    try:
        result = subprocess.run(
            cmd,
            input=input_text,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=timeout,
            check=False,
            close_fds=True,
            env={"PATH": "/usr/sbin:/usr/bin:/sbin:/bin", "LANG": "C", "LC_ALL": "C"},
        )
    except subprocess.TimeoutExpired as exc:
        raise CommandTimeoutError(cmd[0]) from exc
    except OSError as exc:
        raise CommandError(cmd[0], -1) from exc
    if result.returncode != 0:
        raise CommandError(cmd[0], result.returncode)


def _open_account_lock_file():
    if fcntl is None:
        raise RuntimeError("POSIX file locking is unavailable")
    if os.name == "posix" and not hasattr(os, "O_NOFOLLOW"):
        raise RuntimeError("secure lock-file opening is unavailable")
    parent = os.path.dirname(os.path.abspath(ACCOUNT_LOCK_FILE)) or "."
    _assert_directory(parent)
    parent_info = os.lstat(parent)
    if os.name == "posix" and parent_info.st_uid != REQUIRED_LOCK_OWNER_UID:
        raise IntegrityError("account lock directory must be owned by root")
    if os.name == "posix" and stat.S_IMODE(parent_info.st_mode) & 0o077:
        raise IntegrityError("account lock directory permissions must be 0700 or stricter")
    flags = os.O_RDWR | os.O_CREAT
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    fd = os.open(ACCOUNT_LOCK_FILE, flags, 0o600)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode):
            raise IntegrityError("account lock must be a regular file")
        if os.name == "posix" and info.st_uid != REQUIRED_LOCK_OWNER_UID:
            raise IntegrityError("account lock must be owned by root")
        if os.name == "posix" and stat.S_IMODE(info.st_mode) & 0o077:
            raise IntegrityError("account lock permissions must be 0600 or stricter")
        return fd
    except Exception:
        os.close(fd)
        raise


@contextmanager
def _account_file_lock(deadline, exclusive):
    """Synchronize with deploy and any second ota-admin process via flock."""
    fd = _open_account_lock_file()
    lock_kind = fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH
    acquired = False
    try:
        while True:
            deadline.remaining()
            try:
                fcntl.flock(fd, lock_kind | fcntl.LOCK_NB)
                acquired = True
                break
            except OSError as exc:
                if exc.errno not in (errno.EACCES, errno.EAGAIN):
                    raise
                time.sleep(min(LOCK_POLL_SECONDS, deadline.remaining()))
        yield
    finally:
        if acquired:
            try:
                fcntl.flock(fd, fcntl.LOCK_UN)
            except OSError:
                LOG.critical("failed to release account coordination lock")
        os.close(fd)


@contextmanager
def _account_operation_scope(exclusive):
    """Bound local/cross-process lock wait, commands, and rollback to one budget."""
    deadline = OperationDeadline()
    acquired = _account_lock.acquire(timeout=deadline.remaining())
    if not acquired:
        raise OperationTimeoutError("account operation deadline exceeded")
    try:
        with _account_file_lock(deadline, exclusive=exclusive):
            yield deadline
    finally:
        _account_lock.release()


def _validate_name(name):
    if not isinstance(name, str) or NAME_RE.fullmatch(name) is None:
        raise ApiError("账号名需小写字母开头、3-32位（小写字母/数字/_-）")
    if name in RETIRED_ACCOUNTS:
        raise ApiError("共享 uploader 账号已退役；请使用与设备名完全一致的独立账号")
    return name


def _validate_stored_name(name):
    """Validate an on-disk account name, including retired-name detection."""
    if not isinstance(name, str) or NAME_RE.fullmatch(name) is None:
        raise IntegrityError("invalid account name in managed state")
    if name in RETIRED_ACCOUNTS:
        raise IntegrityError("retired shared uploader account is still published")
    return name


def _validate_password(password):
    if not isinstance(password, str):
        raise ApiError("password 必须是字符串")
    if not MIN_PASSWORD_CHARS <= len(password) <= MAX_PASSWORD_CHARS:
        raise ApiError("密码长度须为 8-256 位")
    # chpasswd consumes ``name:password`` records one per line.  A colon or any
    # ASCII control character would make the input ambiguous or inject records.
    if ":" in password or any(ord(ch) < 0x20 or ord(ch) == 0x7F for ch in password):
        raise ApiError("密码不得包含冒号或控制字符")
    return password


def _validate_permission(permission):
    if permission not in ("upload", "full"):
        raise ApiError("permission 须为 upload 或 full")
    return permission


def _validate_fixed_account_permission(name, permission):
    expected = FIXED_ACCOUNT_PERMISSIONS.get(name)
    if expected is not None and permission != expected:
        raise ApiError("系统账号权限固定，禁止修改")
    if expected is None and permission != "upload":
        raise ApiError("普通设备账号必须保持 upload-only 独立目录权限")
    return permission


def _assert_fixed_account_permission(name, permission):
    expected = FIXED_ACCOUNT_PERMISSIONS.get(name)
    if expected is not None and permission != expected:
        raise IntegrityError("fixed system account permission invariant is broken")
    if expected is None and permission != "upload":
        raise IntegrityError("ordinary device account is not upload-only")


def _assert_regular_file(path, max_bytes=None):
    try:
        info = os.lstat(path)
    except FileNotFoundError:
        return None
    if not stat.S_ISREG(info.st_mode):
        raise IntegrityError("expected a regular configuration file")
    if max_bytes is not None and info.st_size > max_bytes:
        raise IntegrityError("configuration file exceeds its size limit")
    return info


def _assert_directory(path, create=False):
    if create:
        os.makedirs(path, mode=0o755, exist_ok=True)
    try:
        info = os.lstat(path)
    except FileNotFoundError as exc:
        raise IntegrityError("required configuration directory is missing") from exc
    if not stat.S_ISDIR(info.st_mode):
        raise IntegrityError("expected a real configuration directory")


def _fsync_directory(path):
    """Best-effort directory fsync (not available on every offline test host)."""
    flags = os.O_RDONLY
    if hasattr(os, "O_DIRECTORY"):
        flags |= os.O_DIRECTORY
    try:
        fd = os.open(path, flags)
    except OSError:
        return
    try:
        os.fsync(fd)
    except OSError:
        pass
    finally:
        os.close(fd)


def _atomic_write_bytes(path, payload, mode=0o600):
    if len(payload) > MAX_CONFIG_BYTES:
        raise IntegrityError("configuration payload exceeds its size limit")
    parent = os.path.dirname(path) or "."
    _assert_directory(parent, create=True)
    _assert_regular_file(path, MAX_CONFIG_BYTES)
    fd = None
    temporary = None
    try:
        fd, temporary = tempfile.mkstemp(prefix=".%s." % os.path.basename(path), dir=parent)
        os.fchmod(fd, mode)
        with os.fdopen(fd, "wb") as stream:
            fd = None
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
        _fsync_directory(parent)
    finally:
        if fd is not None:
            os.close(fd)
        if temporary is not None:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def _atomic_write_text(path, text, mode=0o600):
    _atomic_write_bytes(path, text.encode("utf-8"), mode=mode)


def _snapshot_file(path):
    info = _assert_regular_file(path, MAX_CONFIG_BYTES)
    if info is None:
        return None
    with open(path, "rb") as stream:
        return stream.read(), stat.S_IMODE(info.st_mode)


def _remove_regular_file(path):
    info = _assert_regular_file(path, MAX_CONFIG_BYTES)
    if info is not None:
        os.unlink(path)
        _fsync_directory(os.path.dirname(path) or ".")


def _restore_file(path, snapshot):
    if snapshot is None:
        _remove_regular_file(path)
    else:
        payload, mode = snapshot
        _atomic_write_bytes(path, payload, mode=mode)


def _safe_conf_path(name):
    _validate_name(name)
    base = os.path.abspath(USER_CONF_DIR)
    _assert_directory(base)
    candidate = os.path.abspath(os.path.join(base, name))
    try:
        inside = os.path.commonpath((base, candidate)) == base
    except ValueError:
        inside = False
    if not inside:
        raise IntegrityError("account configuration escaped its root")
    return candidate


def _safe_device_path(name):
    _validate_name(name)
    base = os.path.abspath(DATA_DIR)
    _assert_directory(base)
    _assert_ftp_directory_boundary()
    candidate = os.path.abspath(os.path.join(base, name))
    try:
        inside = os.path.commonpath((base, candidate)) == base
    except ValueError:
        inside = False
    if not inside or candidate == base:
        raise IntegrityError("device directory escaped its root")
    return candidate


def _assert_ftp_directory_boundary():
    if not ENFORCE_POSIX_OWNERSHIP:
        return
    admin = _get_device_admin_record()
    for path, expected_mode, expected_gid in (
        (FTP_HOME, 0o755, 0),
        (DATA_DIR, 0o2771, admin.pw_gid),
    ):
        try:
            info = os.lstat(path)
        except FileNotFoundError as exc:
            raise IntegrityError("managed FTP directory boundary is missing") from exc
        if not stat.S_ISDIR(info.st_mode) or stat.S_ISLNK(info.st_mode):
            raise IntegrityError("managed FTP directory boundary is not a real directory")
        if info.st_uid != 0 or info.st_gid != expected_gid or stat.S_IMODE(info.st_mode) != expected_mode:
            raise IntegrityError("managed FTP directory boundary owner/mode has drifted")


def _device_local_root(name):
    _validate_name(name)
    # With chroot_local_user=YES, vsftpd treats local_root as the chroot root.
    # Pointing it at the user-owned device directory would make that root
    # writable and login would fail while allow_writeable_chroot=NO.  Keep the
    # root fixed/root-owned; the 2771 parent + account-owned 2770 child is the
    # actual per-device authorization boundary.
    return FTP_HOME


def _upload_only_config(name):
    return UPLOAD_ONLY_CONFIG_PREFIX + "local_root=%s\n" % _device_local_root(name)


def _get_device_admin_record():
    record = _get_system_account("devicedata")
    if record is None:
        raise IntegrityError("devicedata administrator account is missing")
    if record.pw_uid < MIN_MANAGED_UID or record.pw_gid <= 0:
        raise IntegrityError("devicedata administrator identity is privileged")
    if record.pw_dir != FTP_HOME or record.pw_shell != NOLOGIN_SHELL:
        raise IntegrityError("devicedata administrator is outside the managed FTP profile")
    try:
        same_uid = [entry for entry in pwd.getpwall() if entry.pw_uid == record.pw_uid]
        same_primary_gid = [entry for entry in pwd.getpwall() if entry.pw_gid == record.pw_gid]
        admin_group = grp.getgrgid(record.pw_gid)
    except (KeyError, OSError) as exc:
        raise IntegrityError("devicedata private group cannot be resolved") from exc
    if len(same_uid) != 1 or same_uid[0].pw_name != "devicedata":
        raise IntegrityError("devicedata UID is shared or aliased")
    if len(same_primary_gid) != 1 or same_primary_gid[0].pw_name != "devicedata":
        raise IntegrityError("devicedata primary GID is shared")
    if any(member != "devicedata" for member in admin_group.gr_mem):
        raise IntegrityError("devicedata private group has an unexpected member")
    try:
        ftp_group = grp.getgrnam(FTP_GROUP)
    except KeyError as exc:
        raise IntegrityError("managed FTP group is missing") from exc
    if record.pw_gid == ftp_group.gr_gid:
        raise IntegrityError("devicedata primary group must be private, not ftpdata")
    return record


def _walk_device_tree(path, deadline, budget):
    """Yield bounded lstat records without letting os.walk preallocate an unbounded directory."""
    stack = [(os.path.abspath(path), 0)]
    while stack:
        directory, depth = stack.pop()
        deadline.remaining()
        try:
            iterator = os.scandir(directory)
        except OSError as exc:
            raise IntegrityError("managed device directory cannot be scanned") from exc
        child_directories = []
        try:
            for entry in iterator:
                child_depth = depth + 1
                budget.consume(child_depth, deadline)
                try:
                    # DirEntry.stat().st_nlink is reported as zero by some Windows
                    # Python builds used by offline CI; lstat is the portable identity.
                    info = os.lstat(entry.path)
                except OSError as exc:
                    raise IntegrityError("managed device entry changed during scan") from exc
                yield entry.path, info, child_depth
                if stat.S_ISDIR(info.st_mode):
                    child_directories.append((entry.path, child_depth))
        finally:
            iterator.close()
        stack.extend(reversed(child_directories))


def _assert_device_tree_has_no_links(path, deadline, budget):
    """Reject link-based crossings under one shared request deadline and entry budget."""
    for _candidate, info, _depth in _walk_device_tree(path, deadline, budget):
        if stat.S_ISLNK(info.st_mode):
            raise IntegrityError("device directory contains a symbolic link")
        if stat.S_ISREG(info.st_mode) and info.st_nlink != 1:
            raise IntegrityError("device directory contains a hard-linked file")


def _assert_device_directory(
    name,
    account_record=None,
    scan_links=False,
    deadline=None,
    scan_budget=None,
):
    path = _safe_device_path(name)
    try:
        info = os.lstat(path)
    except FileNotFoundError as exc:
        raise IntegrityError("device account is missing its bound data directory") from exc
    if not stat.S_ISDIR(info.st_mode) or stat.S_ISLNK(info.st_mode):
        raise IntegrityError("bound device data path is not a real directory")
    if account_record is None:
        account_record = _assert_managed_system_account(name)
    admin = _get_device_admin_record()
    if ENFORCE_POSIX_OWNERSHIP:
        if info.st_uid != account_record.pw_uid or info.st_gid != admin.pw_gid:
            raise IntegrityError("bound device directory owner/group is not isolated")
        if stat.S_IMODE(info.st_mode) != DEVICE_DIRECTORY_MODE:
            raise IntegrityError("bound device directory mode must be 2770")
    if scan_links:
        if deadline is None:
            deadline = OperationDeadline()
        if scan_budget is None:
            scan_budget = DirectoryScanBudget()
        _assert_device_tree_has_no_links(path, deadline, scan_budget)
    return path


def _create_device_directory(name, account_record):
    path = _safe_device_path(name)
    if os.path.lexists(path):
        raise ConflictError("同名设备数据目录已存在（可能是保留数据），禁止重建账号接管")
    admin = _get_device_admin_record()
    os.mkdir(path, 0o700)
    try:
        if ENFORCE_POSIX_OWNERSHIP:
            os.chown(path, account_record.pw_uid, admin.pw_gid)
        os.chmod(path, DEVICE_DIRECTORY_MODE)
        _assert_device_directory(name, account_record=account_record)
    except Exception:
        try:
            os.rmdir(path)
        except OSError:
            LOG.critical("rollback failed for new device directory")
        raise
    _fsync_directory(DATA_DIR)
    return path


def _retire_device_directory(name):
    """Preserve data while preventing a recycled UID from traversing it."""
    path = _safe_device_path(name)
    info = os.lstat(path)
    if not stat.S_ISDIR(info.st_mode) or stat.S_ISLNK(info.st_mode):
        raise IntegrityError("cannot retire a non-directory device data path")
    admin = _get_device_admin_record()
    if ENFORCE_POSIX_OWNERSHIP:
        os.chown(path, 0, admin.pw_gid)
    os.chmod(path, DEVICE_DIRECTORY_MODE)
    _fsync_directory(DATA_DIR)


def _restore_device_directory_metadata(name, metadata):
    path = _safe_device_path(name)
    info = os.lstat(path)
    if not stat.S_ISDIR(info.st_mode) or stat.S_ISLNK(info.st_mode):
        raise IntegrityError("cannot restore a non-directory device data path")
    if ENFORCE_POSIX_OWNERSHIP:
        os.chown(path, metadata.st_uid, metadata.st_gid)
    os.chmod(path, stat.S_IMODE(metadata.st_mode))
    _fsync_directory(DATA_DIR)


def _rollback_new_device_directory(name):
    path = _safe_device_path(name)
    if not os.path.lexists(path):
        return
    try:
        os.rmdir(path)
        _fsync_directory(DATA_DIR)
    except OSError:
        # Unexpected content must remain inaccessible even if rollback cannot
        # remove it.  The retained path also blocks same-name recreation.
        try:
            _retire_device_directory(name)
        except Exception:
            LOG.critical("rollback failed to quarantine new device directory")


def _parse_userlist_document(content):
    """Return account names while leaving comments/blank lines opaque.

    Deployment owns the human-maintained comments in this file.  Admin
    mutations therefore preserve them byte-for-byte and only interpret actual
    non-comment account lines.
    """
    names = []
    seen = set()
    for line in content.splitlines(keepends=True):
        logical = line.rstrip("\r\n")
        stripped = logical.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if logical != stripped or NAME_RE.fullmatch(logical) is None or logical in seen:
            raise IntegrityError("invalid or duplicate account in vsftpd allow-list")
        _validate_stored_name(logical)
        seen.add(logical)
        names.append(logical)
    return names


def _read_userlist_document_unlocked():
    _assert_directory(os.path.dirname(os.path.abspath(USERLIST)) or ".")
    info = _assert_regular_file(USERLIST, MAX_CONFIG_BYTES)
    if info is None:
        return "", []
    with open(USERLIST, "r", encoding="utf-8", newline="") as stream:
        content = stream.read()
    return content, _parse_userlist_document(content)


def _read_userlist_unlocked():
    return _read_userlist_document_unlocked()[1]


def _publish_userlist_document_unlocked(content):
    _parse_userlist_document(content)
    _atomic_write_text(USERLIST, content, mode=0o600)


def _append_account_line(content, name):
    _validate_name(name)
    if content and not content.endswith(("\n", "\r")):
        content += "\n"
    return content + name + "\n"


def _remove_account_line(content, name):
    _validate_name(name)
    kept = []
    removed = False
    for line in content.splitlines(keepends=True):
        logical = line.rstrip("\r\n")
        stripped = logical.strip()
        if logical == stripped == name:
            removed = True
            continue
        kept.append(line)
    if not removed:
        raise IntegrityError("account line disappeared during mutation")
    return "".join(kept)


def _replace_account_lines(content, names):
    checked = []
    seen = set()
    for name in names:
        _validate_name(name)
        if name in seen:
            raise IntegrityError("duplicate account in new vsftpd allow-list")
        seen.add(name)
        checked.append(name)
    kept = []
    for line in content.splitlines(keepends=True):
        logical = line.rstrip("\r\n")
        stripped = logical.strip()
        if stripped and not stripped.startswith("#"):
            continue
        kept.append(line)
    rebuilt = "".join(kept)
    for name in checked:
        rebuilt = _append_account_line(rebuilt, name)
    return rebuilt


def _write_userlist_unlocked(names):
    content, _existing = _read_userlist_document_unlocked()
    _publish_userlist_document_unlocked(_replace_account_lines(content, names))


def write_userlist(names):
    with _account_operation_scope(exclusive=True):
        _write_userlist_unlocked(names)


def _read_permission_unlocked(name):
    conf = _safe_conf_path(name)
    info = _assert_regular_file(conf, MAX_CONFIG_BYTES)
    if info is None:
        return "full"
    with open(conf, "r", encoding="utf-8") as stream:
        content = stream.read()
    directives = {}
    for raw_line in content.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise IntegrityError("invalid per-account vsftpd configuration")
        key, value = (part.strip() for part in line.split("=", 1))
        if not key or key in directives:
            raise IntegrityError("duplicate per-account vsftpd directive")
        directives[key] = value

    if directives.get("download_enable") != "NO" or directives.get("chmod_enable") != "NO":
        raise IntegrityError("upload-only account is missing a required restriction")
    if directives.get("file_open_mode") != "0440":
        raise IntegrityError("upload-only account is missing write-once file mode")
    if directives.get("local_umask") != "007":
        raise IntegrityError("upload-only account is missing its private directory umask")
    if directives.get("local_root") != _device_local_root(name):
        raise IntegrityError("upload-only account escaped the root-owned FTP chroot")
    denied_items = directives.get("cmds_denied", "").split(",")
    if any(not item or item != item.strip() or item != item.upper() for item in denied_items):
        raise IntegrityError("invalid cmds_denied restriction")
    required_denied = {"DELE", "RMD", "RNFR", "RNTO", "APPE", "REST"}
    if not required_denied.issubset(set(denied_items)):
        raise IntegrityError("upload-only account permits destructive FTP commands")

    # Existing deployment-managed files may add comments, deny more commands,
    # or disable additional capabilities.  Accept only additions that cannot
    # weaken this permission profile; unknown positive/enabling directives fail.
    for key, value in directives.items():
        if key in ("download_enable", "chmod_enable", "cmds_denied"):
            continue
        if key == "file_open_mode" and value == "0440":
            continue
        if key == "local_umask" and value == "007":
            continue
        if key == "local_root" and value == _device_local_root(name):
            continue
        allowed_values = SAFE_EXTRA_UPLOAD_DIRECTIVES.get(key)
        if allowed_values is not None and value in allowed_values:
            continue
        if not key.endswith("_enable") or value != "NO":
            raise IntegrityError("unsupported per-account vsftpd directive")
    return "upload"


def _set_permission_unlocked(name, permission):
    _validate_permission(permission)
    _validate_fixed_account_permission(name, permission)
    conf = _safe_conf_path(name)
    if permission == "upload":
        _atomic_write_text(conf, _upload_only_config(name), mode=0o600)
    else:
        _remove_regular_file(conf)


def set_permission(name, permission):
    with _account_operation_scope(exclusive=True):
        _set_permission_unlocked(name, permission)


def list_accounts():
    with _account_operation_scope(exclusive=False) as deadline:
        accounts = []
        scan_budget = DirectoryScanBudget()
        for name in _read_userlist_unlocked():
            deadline.remaining()
            permission = _read_permission_unlocked(name)
            _assert_fixed_account_permission(name, permission)
            record = _assert_managed_system_account(name)
            if name != "devicedata":
                _assert_device_directory(
                    name,
                    account_record=record,
                    scan_links=True,
                    deadline=deadline,
                    scan_budget=scan_budget,
                )
            accounts.append(
                {
                    "name": name,
                    "permission": permission,
                    "protected": name in PROTECTED,
                }
            )
        return accounts


def _get_system_account(name):
    if pwd is None:
        raise RuntimeError("POSIX account database is unavailable")
    try:
        return pwd.getpwnam(name)
    except KeyError:
        return None


def _get_system_group(name):
    if grp is None:
        raise RuntimeError("POSIX group database is unavailable")
    try:
        return grp.getgrnam(name)
    except KeyError:
        return None


def _assert_managed_system_account(name):
    record = _get_system_account(name)
    if record is None:
        raise IntegrityError("allow-listed account is missing from the system account database")
    if record.pw_uid < MIN_MANAGED_UID:
        raise IntegrityError("refusing to mutate a root or system identity")
    try:
        canonical_identity = pwd.getpwuid(record.pw_uid)
        uid_identities = [entry for entry in pwd.getpwall() if entry.pw_uid == record.pw_uid]
    except (KeyError, OSError) as exc:
        raise IntegrityError("managed account UID cannot be resolved canonically") from exc
    if canonical_identity.pw_name != name or len(uid_identities) != 1 or uid_identities[0].pw_name != name:
        raise IntegrityError("managed account UID is shared or aliased")
    if record.pw_dir != FTP_HOME or record.pw_shell != NOLOGIN_SHELL:
        raise IntegrityError("refusing to mutate a system account outside the managed FTP profile")
    if grp is None:
        raise RuntimeError("POSIX group database is unavailable")
    try:
        group = grp.getgrnam(FTP_GROUP)
    except KeyError as exc:
        raise IntegrityError("managed FTP group is missing") from exc
    if group.gr_gid <= 0:
        raise IntegrityError("managed FTP group uses a privileged GID")
    try:
        canonical_group = grp.getgrgid(group.gr_gid)
        groups = grp.getgrall()
    except KeyError as exc:
        raise IntegrityError("managed FTP group GID cannot be resolved") from exc
    same_gid = [candidate for candidate in groups if candidate.gr_gid == group.gr_gid]
    if canonical_group.gr_name != FTP_GROUP or len(same_gid) != 1 or same_gid[0].gr_name != FTP_GROUP:
        raise IntegrityError("managed FTP group GID is ambiguous")
    if record.pw_gid != group.gr_gid and name not in group.gr_mem:
        raise IntegrityError("allow-listed account is not a member of the managed FTP group")
    if name != "devicedata":
        admin_gid = _get_device_admin_record().pw_gid
        if record.pw_gid in (group.gr_gid, admin_gid):
            raise IntegrityError("ordinary device account does not use a private primary group")
        try:
            private_group = grp.getgrgid(record.pw_gid)
            same_gid_groups = [entry for entry in grp.getgrall() if entry.gr_gid == record.pw_gid]
            same_primary_gid = [entry for entry in pwd.getpwall() if entry.pw_gid == record.pw_gid]
        except (KeyError, OSError) as exc:
            raise IntegrityError("device account private primary group cannot be resolved") from exc
        if (
            private_group.gr_name != name
            or len(same_gid_groups) != 1
            or same_gid_groups[0].gr_name != name
            or len(same_primary_gid) != 1
            or same_primary_gid[0].pw_name != name
            or any(member != name for member in private_group.gr_mem)
        ):
            raise IntegrityError("device account primary group is shared or aliased")
    return record


def _rollback_file(path, snapshot, label):
    try:
        _restore_file(path, snapshot)
    except Exception:  # rollback diagnostics intentionally exclude paths and contents
        LOG.critical("rollback failed for %s", label)


def account_create(name, password, permission):
    name = _validate_name(name)
    password = _validate_password(password)
    permission = _validate_permission(permission)
    _validate_fixed_account_permission(name, permission)
    with _account_operation_scope(exclusive=True) as deadline:
        userlist_document, names = _read_userlist_document_unlocked()
        if name in names:
            raise ConflictError("账号已存在")
        if _get_system_account(name) is not None:
            raise ConflictError("同名系统账号已存在，拒绝接管")
        if name != "devicedata" and _get_system_group(name) is not None:
            raise ConflictError("同名系统组已存在，拒绝接管或复用旧 GID")
        conf = _safe_conf_path(name)
        if _snapshot_file(conf) is not None:
            raise ConflictError("账号存在残留权限配置，拒绝覆盖")
        if name != "devicedata" and os.path.lexists(_safe_device_path(name)):
            raise ConflictError("同名设备数据已保留，禁止新 UID 重建账号接管")

        user_added = False
        device_directory_created = False
        try:
            try:
                run(
                    [USERADD, "-U", "-M", "-d", FTP_HOME, "-s", NOLOGIN_SHELL, "-G", FTP_GROUP, name],
                    deadline=deadline,
                    reserve_seconds=ROLLBACK_RESERVE_SECONDS,
                )
                user_added = True
            except Exception:
                # A timeout can be reported after useradd crossed its commit
                # point.  Detect that case so the outer rollback still removes
                # the hidden partial account.
                user_added = _get_system_account(name) is not None
                raise
            _assert_managed_system_account(name)
            run(
                [CHPASSWD],
                input_text="%s:%s\n" % (name, password),
                deadline=deadline,
                reserve_seconds=ROLLBACK_RESERVE_SECONDS,
            )
            deadline.remaining()
            if name != "devicedata":
                _create_device_directory(name, _assert_managed_system_account(name))
                device_directory_created = True
            _set_permission_unlocked(name, permission)
            _publish_userlist_document_unlocked(_append_account_line(userlist_document, name))
        except Exception:
            _rollback_file(conf, None, "new account permission")
            if device_directory_created or (name != "devicedata" and os.path.lexists(_safe_device_path(name))):
                _rollback_new_device_directory(name)
            if user_added:
                try:
                    run([USERDEL, name], deadline=deadline)
                except Exception:
                    LOG.critical("rollback failed for newly-created system account")
            if name != "devicedata" and _get_system_account(name) is None:
                try:
                    run([GROUPDEL, name], deadline=deadline)
                except Exception:
                    LOG.critical("rollback left a private device group")
            raise


def account_update(name, password=None, permission=None):
    name = _validate_name(name)
    if password is None and permission is None:
        raise ApiError("至少提供 password 或 permission 之一")
    if password is not None:
        password = _validate_password(password)
    if permission is not None:
        permission = _validate_permission(permission)
        _validate_fixed_account_permission(name, permission)

    with _account_operation_scope(exclusive=True) as deadline:
        names = _read_userlist_unlocked()
        if name not in names:
            raise NotFoundError("账号不存在")
        record = _assert_managed_system_account(name)
        if name != "devicedata":
            _assert_device_directory(
                name,
                account_record=record,
                scan_links=True,
                deadline=deadline,
                scan_budget=DirectoryScanBudget(),
            )
        conf = _safe_conf_path(name)
        permission_snapshot = _snapshot_file(conf)
        current_permission = _read_permission_unlocked(name)
        _assert_fixed_account_permission(name, current_permission)
        try:
            # Apply the reversible file mutation first.  If chpasswd fails, restore it.
            if permission is not None:
                _set_permission_unlocked(name, permission)
            if password is not None:
                run(
                    [CHPASSWD],
                    input_text="%s:%s\n" % (name, password),
                    deadline=deadline,
                )
        except Exception:
            _rollback_file(conf, permission_snapshot, "account permission")
            raise


def account_delete(name):
    name = _validate_name(name)
    if name in PROTECTED:
        raise ApiError("受保护账号，禁止删除")
    with _account_operation_scope(exclusive=True) as deadline:
        userlist_document, names = _read_userlist_document_unlocked()
        if name not in names:
            raise NotFoundError("账号不存在")
        record = _assert_managed_system_account(name)
        device_path = _assert_device_directory(
            name,
            account_record=record,
            scan_links=True,
            deadline=deadline,
            scan_budget=DirectoryScanBudget(),
        )
        device_metadata = os.lstat(device_path)
        conf = _safe_conf_path(name)
        permission_snapshot = _snapshot_file(conf)

        try:
            # Reversible visibility changes happen before the irreversible userdel.
            _remove_regular_file(conf)
            _publish_userlist_document_unlocked(_remove_account_line(userlist_document, name))
            # Retire only the directory entry, not its contents.  Changing its
            # owner to root before userdel closes the UID-reuse window while the
            # private devicedata group preserves administrator access.
            _retire_device_directory(name)
            try:
                run([USERDEL, name], deadline=deadline)  # deliberately never passes -r
            except Exception:
                # If userdel committed before a timeout/error was observed, the
                # desired deletion is already complete; do not republish a
                # now-nonexistent account into the allow-list.
                if _get_system_account(name) is None:
                    try:
                        run([GROUPDEL, name], deadline=deadline)
                    except Exception:
                        LOG.critical("deleted account left a private device group")
                    return
                raise
            try:
                run([GROUPDEL, name], deadline=deadline)
            except Exception:
                # userdel may already remove user-private groups.  A stale empty
                # group blocks unsafe same-name recreation but does not expose data.
                LOG.warning("deleted account private group was already absent or retained")
        except Exception:
            if _get_system_account(name) is not None:
                try:
                    _restore_device_directory_metadata(name, device_metadata)
                except Exception:
                    LOG.critical("rollback failed for device directory metadata")
            try:
                _publish_userlist_document_unlocked(userlist_document)
            except Exception:
                LOG.critical("rollback failed for account allow-list")
            _rollback_file(conf, permission_snapshot, "deleted account permission")
            raise


def _scan_device_directory(path, deadline, scan_budget):
    size = 0
    files = 0
    newest = 0.0
    for _candidate, info, _depth in _walk_device_tree(path, deadline, scan_budget):
        if stat.S_ISLNK(info.st_mode):
            continue
        if stat.S_ISREG(info.st_mode):
            if info.st_nlink != 1:
                raise IntegrityError("device stats tree contains a hard-linked file")
            size += info.st_size
            files += 1
            newest = max(newest, info.st_mtime)
    return size, files, newest


def build_stats():
    deadline = OperationDeadline()
    now = time.time()
    acquired = _stats_lock.acquire(timeout=deadline.remaining())
    if not acquired:
        raise OperationTimeoutError("stats operation deadline exceeded")
    try:
        if _stats_cache["data"] is not None and now - _stats_cache["at"] < 30:
            return _stats_cache["data"]
        deadline.remaining()
        _assert_directory(DATA_DIR)
        disk = shutil.disk_usage(DATA_DIR)
        devices = []
        total = 0
        scan_budget = DirectoryScanBudget()
        if os.path.isdir(DATA_DIR) and not os.path.islink(DATA_DIR):
            device_entries = []
            with os.scandir(DATA_DIR) as entries:
                for entry in entries:
                    scan_budget.consume(1, deadline)
                    try:
                        info = os.lstat(entry.path)
                    except OSError as exc:
                        raise IntegrityError("device stats root changed during scan") from exc
                    if stat.S_ISLNK(info.st_mode):
                        continue
                    if stat.S_ISDIR(info.st_mode):
                        device_entries.append(entry)
            device_entries.sort(key=lambda entry: entry.name)
            for entry in device_entries:
                size, files, newest = _scan_device_directory(
                    entry.path, deadline, scan_budget
                )
                total += size
                devices.append(
                    {
                        "name": entry.name,
                        "bytes": size,
                        "files": files,
                        "lastUploadEpoch": int(newest),
                    }
                )
        data = {
            "disk": {
                "totalBytes": disk.total,
                "usedBytes": disk.used,
                "freeBytes": disk.free,
            },
            "dataBytes": total,
            "devices": devices,
            "serverTimeEpoch": int(now),
        }
        _stats_cache["at"] = now
        _stats_cache["data"] = data
        return data
    finally:
        _stats_lock.release()


def load_admin_token(path=TOKEN_FILE):
    _assert_directory(os.path.dirname(os.path.abspath(path)) or ".")
    info = os.lstat(path)
    if not stat.S_ISREG(info.st_mode) or stat.S_ISLNK(info.st_mode):
        raise RuntimeError("admin token must be a regular file")
    if os.name == "posix" and info.st_uid != 0:
        raise RuntimeError("admin token must be owned by root")
    if os.name == "posix" and stat.S_IMODE(info.st_mode) & 0o077:
        raise RuntimeError("admin token permissions must be 0600 or stricter")
    if info.st_size > MAX_TOKEN_CHARS + 2:
        raise RuntimeError("admin token is too large")
    with open(path, "r", encoding="utf-8", newline="") as stream:
        raw_token = stream.read()
    if raw_token.endswith("\r\n"):
        token = raw_token[:-2]
    elif raw_token.endswith("\n"):
        token = raw_token[:-1]
    else:
        token = raw_token
    if not MIN_TOKEN_CHARS <= len(token) <= MAX_TOKEN_CHARS:
        raise RuntimeError("admin token length is invalid")
    if any(ord(ch) < 0x21 or ord(ch) > 0x7E for ch in token):
        raise RuntimeError("admin token contains invalid characters")
    return token


def _reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ApiError("JSON 不得包含重复字段")
        result[key] = value
    return result


class Handler(BaseHTTPRequestHandler):
    server_version = "OtaAdmin"
    sys_version = ""

    def _reply(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _auth(self):
        values = self.headers.get_all("X-Admin-Token", [])
        supplied = values[0] if len(values) == 1 else ""
        expected = getattr(self.server, "admin_token", "")
        supplied_is_ascii = all(0x21 <= ord(ch) <= 0x7E for ch in supplied)
        if (
            len(values) != 1
            or len(supplied) > MAX_TOKEN_CHARS
            or not supplied_is_ascii
            or not expected
            or not hmac.compare_digest(supplied, expected)
        ):
            self.close_connection = True
            self._reply(401, {"ok": False, "error": "认证失败"})
            return False
        return True

    def _canonical_path(self):
        try:
            parsed = urlsplit(self.path)
        except ValueError as exc:
            raise ApiError("请求路径必须使用规范格式") from exc
        if (
            parsed.scheme
            or parsed.netloc
            or parsed.query
            or parsed.fragment
            or "%" in parsed.path
            or "\\" in parsed.path
        ):
            raise ApiError("请求路径必须使用规范格式")
        if any(ord(ch) < 0x20 or ord(ch) >= 0x7F for ch in parsed.path):
            raise ApiError("请求路径包含非法字符")
        return parsed.path

    def _require_no_body(self):
        if self.headers.get_all("Transfer-Encoding", []):
            raise ApiError("该接口不接受请求体")
        lengths = self.headers.get_all("Content-Length", [])
        if not lengths:
            return
        if (
            len(lengths) != 1
            or not lengths[0].isdigit()
            or len(lengths[0]) > 10
            or int(lengths[0]) != 0
        ):
            raise ApiError("该接口不接受请求体")

    def _body_json(self, allowed, required=()):
        transfer_encodings = self.headers.get_all("Transfer-Encoding", [])
        if transfer_encodings:
            self.close_connection = True
            raise ApiError("不支持 Transfer-Encoding")
        content_types = self.headers.get_all("Content-Type", [])
        if len(content_types) != 1:
            raise UnsupportedMediaTypeError("请求必须包含一个 Content-Type")
        media_type = content_types[0].split(";", 1)[0].strip().lower()
        if media_type != "application/json":
            raise UnsupportedMediaTypeError("Content-Type 必须为 application/json")
        lengths = self.headers.get_all("Content-Length", [])
        if len(lengths) != 1:
            raise LengthRequiredError("请求必须包含一个 Content-Length")
        raw_length = lengths[0]
        if not raw_length.isdigit():
            self.close_connection = True
            raise ApiError("Content-Length 无效")
        if len(raw_length) > 10:
            self.close_connection = True
            raise PayloadTooLargeError("请求体过大")
        length = int(raw_length)
        if length > MAX_BODY_BYTES:
            self.close_connection = True
            raise PayloadTooLargeError("请求体过大")
        if length <= 0:
            raise ApiError("请求体不能为空")
        payload = self.rfile.read(length)
        if len(payload) != length:
            self.close_connection = True
            raise ApiError("请求体不完整")
        try:
            document = json.loads(
                payload.decode("utf-8", errors="strict"),
                object_pairs_hook=_reject_duplicate_keys,
            )
        except ApiError:
            raise
        except (UnicodeDecodeError, ValueError, RecursionError) as exc:
            raise ApiError("JSON 格式无效") from exc
        if not isinstance(document, dict):
            raise ApiError("JSON 顶层必须是对象")
        unknown = set(document) - set(allowed)
        missing = set(required) - set(document)
        if unknown:
            raise ApiError("JSON 包含未知字段")
        if missing:
            raise ApiError("JSON 缺少必填字段")
        return document

    def _dispatch(self, action):
        try:
            if self._auth():
                action()
        except (CommandTimeoutError, OperationTimeoutError):
            self.close_connection = True
            self._reply(503, {"ok": False, "error": RETRYABLE_TIMEOUT_MESSAGE})
        except ApiError as exc:
            self.close_connection = True
            self._reply(exc.status, {"ok": False, "error": exc.public_message})
        except Exception as exc:
            # Deliberately omit headers, body, raw path, command output, and exception text.
            LOG.error("request failed: method=%s error=%s", self.command, type(exc).__name__)
            self.close_connection = True
            self._reply(500, {"ok": False, "error": "服务器内部错误"})

    def do_GET(self):
        def action():
            self._require_no_body()
            path = self._canonical_path()
            if path == "/admin/api/ping":
                self._reply(200, {"ok": True})
            elif path == "/admin/api/stats":
                self._reply(200, {"ok": True, **build_stats()})
            elif path == "/admin/api/accounts":
                self._reply(200, {"ok": True, "accounts": list_accounts()})
            else:
                raise NotFoundError("未知接口")

        self._dispatch(action)

    def do_POST(self):
        def action():
            if self._canonical_path() != "/admin/api/accounts":
                raise NotFoundError("未知接口")
            body = self._body_json(
                allowed=("name", "password", "permission"),
                required=("name", "password"),
            )
            name = body.get("name")
            password = body.get("password")
            permission = body.get("permission", "upload")
            account_create(name, password, permission)
            self._reply(201, {"ok": True})

        self._dispatch(action)

    def do_PATCH(self):
        def action():
            path = self._canonical_path()
            match = re.fullmatch(r"/admin/api/accounts/([^/]+)", path)
            if match is None:
                raise NotFoundError("未知接口")
            name = _validate_name(match.group(1))
            body = self._body_json(allowed=("password", "permission"))
            if "password" in body and not isinstance(body["password"], str):
                raise ApiError("password 必须是字符串")
            if "permission" in body and not isinstance(body["permission"], str):
                raise ApiError("permission 必须是字符串")
            account_update(name, password=body.get("password"), permission=body.get("permission"))
            self._reply(200, {"ok": True})

        self._dispatch(action)

    def do_DELETE(self):
        def action():
            self._require_no_body()
            path = self._canonical_path()
            match = re.fullmatch(r"/admin/api/accounts/([^/]+)", path)
            if match is None:
                raise NotFoundError("未知接口")
            account_delete(_validate_name(match.group(1)))
            self._reply(200, {"ok": True})

        self._dispatch(action)

    def log_message(self, fmt, *args):
        # BaseHTTPRequestHandler's default log includes the raw URL.  Keep only
        # fixed protocol metadata so query strings can never leak credentials.
        address = getattr(self, "client_address", ())
        client = address[0] if isinstance(address, (tuple, list)) and address else "-"
        method = getattr(self, "command", "-")
        candidate = args[1] if len(args) > 1 else None
        status = str(candidate) if isinstance(candidate, int) or str(candidate).isdigit() else "-"
        LOG.info("http response: client=%s method=%s status=%s", client, method, status)

    def _method_not_allowed(self):
        def action():
            raise MethodNotAllowedError("不支持的 HTTP 方法")

        self._dispatch(action)

    do_PUT = _method_not_allowed
    do_HEAD = _method_not_allowed
    do_OPTIONS = _method_not_allowed
    do_TRACE = _method_not_allowed
    do_CONNECT = _method_not_allowed


class SecureThreadingHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 32

    def get_request(self):
        request, client_address = super().get_request()
        request.settimeout(15)
        return request, client_address


def main():
    if os.name != "posix" or os.geteuid() != 0:
        raise SystemExit("ota-admin must run as root on a POSIX host")
    if LISTEN[0] not in ("127.0.0.1", "::1"):
        raise SystemExit("ota-admin refuses to bind a non-loopback address")
    os.umask(0o077)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    admin_token = load_admin_token()
    server = SecureThreadingHTTPServer(LISTEN, Handler)
    server.admin_token = admin_token
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
