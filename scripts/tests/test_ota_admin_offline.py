#!/usr/bin/env python3
"""Offline negative and transaction tests for scripts/server/ota_admin.py."""

import importlib.util
import io
import json
import os
import shutil
import stat
import tempfile
import threading
import types
import unittest
from contextlib import contextmanager
from email.message import Message
from pathlib import Path
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPO_ROOT / "scripts" / "server" / "ota_admin.py"
SPEC = importlib.util.spec_from_file_location("ota_admin_under_test", MODULE_PATH)
ota = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ota)
REAL_RUN = ota.run

TEST_TOKEN = "T" * 40


class CommandRunnerTests(unittest.TestCase):
    def test_system_command_timeout_finishes_before_client_deadline(self):
        completed = types.SimpleNamespace(returncode=0)
        with mock.patch.object(ota.subprocess, "run", return_value=completed) as runner:
            ota.run([ota.CHPASSWD], input_text="alpha:valid-pass\n")
        self.assertEqual(10, ota.COMMAND_TIMEOUT_SECONDS)
        self.assertLess(ota.COMMAND_TIMEOUT_SECONDS, 20)
        self.assertEqual(ota.COMMAND_TIMEOUT_SECONDS, runner.call_args.kwargs["timeout"])

    def test_shared_deadline_reduces_later_command_budget_and_stops_new_commands(self):
        clock = [100.0]
        observed_timeouts = []

        def delayed_timeout(command, **kwargs):
            observed_timeouts.append(kwargs["timeout"])
            clock[0] += kwargs["timeout"]
            raise ota.subprocess.TimeoutExpired(command, kwargs["timeout"])

        with (
            mock.patch.object(ota.time, "monotonic", side_effect=lambda: clock[0]),
            mock.patch.object(ota.subprocess, "run", side_effect=delayed_timeout) as runner,
        ):
            deadline = ota.OperationDeadline()
            with self.assertRaises(ota.CommandTimeoutError):
                ota.run([ota.CHPASSWD], deadline=deadline)
            with self.assertRaises(ota.CommandTimeoutError):
                ota.run([ota.CHPASSWD], deadline=deadline)
            with self.assertRaises(ota.OperationTimeoutError):
                ota.run([ota.CHPASSWD], deadline=deadline)

        self.assertEqual([10, 2], observed_timeouts)
        self.assertEqual(2, runner.call_count)
        self.assertEqual(112.0, clock[0])


class FakePwdDatabase:
    def __init__(self):
        self.records = {}

    def getpwnam(self, name):
        if name not in self.records:
            raise KeyError(name)
        return types.SimpleNamespace(pw_name=name, **vars(self.records[name]))

    def getpwuid(self, uid):
        for name, record in self.records.items():
            if record.pw_uid == uid:
                return types.SimpleNamespace(pw_name=name, **vars(record))
        raise KeyError(uid)

    def getpwall(self):
        return [
            types.SimpleNamespace(pw_name=name, **vars(record))
            for name, record in self.records.items()
        ]

    def allocate_uid(self):
        used = {record.pw_uid for record in self.records.values()}
        uid = ota.MIN_MANAGED_UID
        while uid in used:
            uid += 1
        return uid


class FakeGrpDatabase:
    def __init__(self, pwd_database):
        self.pwd_database = pwd_database
        self.ftp_gid = 2400
        self.admin_gid = 2500
        self.extra_records = []

    def _ftp_record(self):
        return types.SimpleNamespace(
            gr_name=ota.FTP_GROUP,
            gr_gid=self.ftp_gid,
            gr_mem=sorted(self.pwd_database.records),
        )

    def getgrnam(self, name):
        if name == ota.FTP_GROUP:
            return self._ftp_record()
        if name == "devicedata":
            return types.SimpleNamespace(gr_name="devicedata", gr_gid=self.admin_gid, gr_mem=[])
        record = self.pwd_database.records.get(name)
        if record is not None:
            return types.SimpleNamespace(gr_name=name, gr_gid=record.pw_gid, gr_mem=[])
        raise KeyError(name)

    def getgrgid(self, gid):
        for record in self.getgrall():
            if record.gr_gid == gid:
                return record
        raise KeyError(gid)

    def getgrall(self):
        dynamic = [
            types.SimpleNamespace(gr_name=name, gr_gid=record.pw_gid, gr_mem=[])
            for name, record in self.pwd_database.records.items()
            if name != "devicedata"
        ]
        return [
            self._ftp_record(),
            types.SimpleNamespace(gr_name="devicedata", gr_gid=self.admin_gid, gr_mem=[]),
            *dynamic,
            *self.extra_records,
        ]


class FakeFcntl:
    LOCK_SH = 1
    LOCK_EX = 2
    LOCK_NB = 4
    LOCK_UN = 8

    def __init__(self):
        self.operations = []
        self.contended = False

    def flock(self, fd, operation):
        self.operations.append(operation)
        if self.contended and operation != self.LOCK_UN:
            raise BlockingIOError(ota.errno.EAGAIN, "offline contention")


class AdminFixture(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.userlist = self.root / "vsftpd.userlist"
        self.conf_dir = self.root / "vsftpd_user_conf"
        self.data_dir = self.root / "data"
        self.conf_dir.mkdir()
        self.data_dir.mkdir()
        self.userlist.write_text("# managed fixture\n\n", encoding="utf-8")
        os.chmod(self.userlist, 0o600)

        self.fake_pwd = FakePwdDatabase()
        self.fake_grp = FakeGrpDatabase(self.fake_pwd)
        self.fake_pwd.records["devicedata"] = types.SimpleNamespace(
            pw_uid=2000,
            pw_dir=ota.FTP_HOME,
            pw_shell=ota.NOLOGIN_SHELL,
            pw_gid=self.fake_grp.admin_gid,
        )
        self.fake_fcntl = FakeFcntl()
        self.account_lock = self.root / "account.lock"
        self.commands = []
        self.fail_executable = None

        self.patch = mock.patch.multiple(
            ota,
            USERLIST=str(self.userlist),
            USER_CONF_DIR=str(self.conf_dir),
            DATA_DIR=str(self.data_dir),
            ACCOUNT_LOCK_FILE=str(self.account_lock),
            REQUIRED_LOCK_OWNER_UID=getattr(os, "getuid", lambda: 0)(),
            pwd=self.fake_pwd,
            grp=self.fake_grp,
            fcntl=self.fake_fcntl,
            run=self.fake_run,
            ENFORCE_POSIX_OWNERSHIP=False,
        )
        self.patch.start()
        ota._stats_cache["at"] = 0.0
        ota._stats_cache["data"] = None

    def tearDown(self):
        self.patch.stop()
        self.temporary.cleanup()

    def fake_run(self, command, input_text=None, deadline=None, reserve_seconds=0):
        executable = command[0]
        self.commands.append(tuple(command))
        if executable == self.fail_executable:
            raise ota.CommandError(executable, 1)
        if executable == ota.USERADD:
            name = command[-1]
            uid = self.fake_pwd.allocate_uid()
            self.fake_pwd.records[name] = types.SimpleNamespace(
                pw_uid=uid,
                pw_dir=ota.FTP_HOME,
                pw_shell=ota.NOLOGIN_SHELL,
                pw_gid=uid + 4000,
            )
        elif executable == ota.USERDEL:
            self.fake_pwd.records.pop(command[-1], None)

    def seed_account(self, name, permission="upload"):
        _content, names = ota._read_userlist_document_unlocked()
        if name != "devicedata":
            uid = self.fake_pwd.allocate_uid()
            self.fake_pwd.records[name] = types.SimpleNamespace(
                pw_uid=uid,
                pw_dir=ota.FTP_HOME,
                pw_shell=ota.NOLOGIN_SHELL,
                pw_gid=uid + 4000,
            )
            (self.data_dir / name).mkdir()
            os.chmod(self.data_dir / name, ota.DEVICE_DIRECTORY_MODE)
        ota.set_permission(name, permission)
        ota.write_userlist(names + [name])


class AccountMutationTests(AdminFixture):
    def test_password_record_injection_and_oversize_are_rejected_before_commands(self):
        malicious = (
            "abcdefgh\nroot:changed",
            "abcdefgh\rroot:changed",
            "abcdefgh:root",
            "abcdefgh\x00root",
            "abcdefgh\troot",
            "x" * (ota.MAX_PASSWORD_CHARS + 1),
        )
        for password in malicious:
            with self.subTest(repr=repr(password)):
                with self.assertRaises(ota.ApiError):
                    ota.account_create("alpha", password, "upload")
        self.assertEqual([], self.commands)
        self.assertEqual(["# managed fixture", ""], self.userlist.read_text(encoding="utf-8").splitlines())

    def test_path_traversal_and_system_account_takeover_are_rejected(self):
        for name in ("../root", "..", "/root", "UPPER", "a", "abc%2froot", "abc\\root"):
            with self.subTest(name=name), self.assertRaises(ota.ApiError):
                ota.account_create(name, "valid-pass", "upload")
        self.fake_pwd.records["existing"] = types.SimpleNamespace(
            pw_uid=ota.MIN_MANAGED_UID,
            pw_dir="/home/existing",
            pw_shell="/bin/bash",
            pw_gid=1000,
        )
        with self.assertRaises(ota.ConflictError):
            ota.account_create("existing", "valid-pass", "upload")
        self.assertEqual([], self.commands)

    def test_create_rolls_back_system_user_and_permission_when_password_change_fails(self):
        self.fail_executable = ota.CHPASSWD
        with self.assertRaises(ota.CommandError):
            ota.account_create("alpha", "valid-pass", "upload")
        self.assertNotIn("alpha", self.fake_pwd.records)
        self.assertFalse((self.conf_dir / "alpha").exists())
        self.assertNotIn("alpha", self.userlist.read_text(encoding="utf-8"))
        self.assertIn((ota.USERDEL, "alpha"), self.commands)

    def test_useradd_timeout_after_commit_still_removes_partial_system_account(self):
        def ambiguous_run(command, input_text=None, deadline=None, reserve_seconds=0):
            if command[0] == ota.USERADD:
                name = command[-1]
                self.fake_pwd.records[name] = types.SimpleNamespace(
                    pw_uid=ota.MIN_MANAGED_UID,
                    pw_dir=ota.FTP_HOME,
                    pw_shell=ota.NOLOGIN_SHELL,
                    pw_gid=1000,
                )
                raise ota.CommandError(command[0], -1)
            if command[0] == ota.USERDEL:
                self.fake_pwd.records.pop(command[-1], None)

        with mock.patch.object(ota, "run", side_effect=ambiguous_run):
            with self.assertRaises(ota.CommandError):
                ota.account_create("alpha", "valid-pass", "upload")
        self.assertNotIn("alpha", self.fake_pwd.records)
        self.assertNotIn("alpha", self.userlist.read_text(encoding="utf-8"))

    def test_create_and_delete_preserve_comments_and_blank_lines(self):
        original = "# first\r\n\r\n  # indented comment\n# last without newline"
        self.userlist.write_bytes(original.encode("utf-8"))
        ota.account_create("alpha", "valid-pass", "upload")
        created = self.userlist.read_bytes().decode("utf-8")
        self.assertTrue(created.startswith(original + "\n"))
        self.assertTrue(created.endswith("alpha\n"))
        self.assertEqual(ota._upload_only_config("alpha"), (self.conf_dir / "alpha").read_text(encoding="utf-8"))

        ota.account_delete("alpha")
        self.assertEqual(original + "\n", self.userlist.read_bytes().decode("utf-8"))
        self.assertFalse((self.conf_dir / "alpha").exists())

    def test_device_account_uses_private_group_and_retained_data_blocks_uid_reuse(self):
        ota.account_create("alpha", "valid-pass", "upload")
        useradd = next(command for command in self.commands if command[0] == ota.USERADD)
        self.assertIn("-U", useradd)
        payload = self.data_dir / "alpha" / "retained.bin"
        payload.write_bytes(b"retained")

        ota.account_delete("alpha")
        self.assertEqual(b"retained", payload.read_bytes())
        self.assertIn((ota.GROUPDEL, "alpha"), self.commands)
        before = list(self.commands)
        with self.assertRaises(ota.ConflictError):
            ota.account_create("alpha", "valid-pass", "upload")
        self.assertEqual(before, self.commands)

    def test_cross_device_local_root_and_shared_primary_gid_fail_closed(self):
        self.seed_account("alpha")
        self.seed_account("bravo")
        alpha_conf = self.conf_dir / "alpha"
        alpha_conf.write_text(
            ota._upload_only_config("alpha").replace(
                "local_root=/srv/devicedata", "local_root=/srv/devicedata/data/bravo"
            ),
            encoding="utf-8",
        )
        with self.assertRaises(ota.IntegrityError):
            ota.list_accounts()

        alpha_conf.write_text(ota._upload_only_config("alpha"), encoding="utf-8")
        self.fake_pwd.records["bravo"].pw_gid = self.fake_pwd.records["alpha"].pw_gid
        with self.assertRaises(ota.IntegrityError):
            ota.list_accounts()

    def test_symbolic_and_hard_links_inside_device_tree_fail_closed(self):
        self.seed_account("alpha")
        outside = self.root / "outside.bin"
        outside.write_bytes(b"outside")
        linked = self.data_dir / "alpha" / "hard-linked.bin"
        try:
            os.link(outside, linked)
        except OSError:
            self.skipTest("host filesystem does not permit hard links")
        with self.assertRaises(ota.IntegrityError):
            ota.list_accounts()

    def test_upload_permission_is_exact_and_forbids_destructive_commands_only(self):
        self.seed_account("alpha", "upload")
        content = (self.conf_dir / "alpha").read_text(encoding="utf-8")
        self.assertEqual(
            "download_enable=NO\n"
            "chmod_enable=NO\n"
            "file_open_mode=0440\n"
            "local_umask=007\n"
            "cmds_denied=DELE,RMD,RNFR,RNTO,APPE,REST\n"
            "local_root=/srv/devicedata\n",
            content,
        )
        self.assertNotIn("STOR", content)
        self.assertNotIn("MKD", content)
        self.assertEqual("upload", ota.list_accounts()[0]["permission"])

        # This is the exact shape produced when deploy_online_services.sh
        # merges its offline fixture's pre-existing uploader restrictions.
        (self.conf_dir / "alpha").write_text(
            "# deployment-owned hardening\n"
            "hide_ids=YES\n"
            "download_enable=NO\n"
            "chmod_enable=NO\n"
            "file_open_mode=0440\n"
            "local_umask=007\n"
            "cmds_denied=SITE_CHMOD,DELE,RMD,RNFR,RNTO,APPE,REST\n"
            "local_root=/srv/devicedata\n"
            "dirlist_enable=NO\n",
            encoding="utf-8",
        )
        self.assertEqual("upload", ota.list_accounts()[0]["permission"])

        (self.conf_dir / "alpha").write_text(
            "download_enable=NO\n"
            "chmod_enable=NO\n"
            "file_open_mode=0440\n"
            "local_umask=007\n"
            "cmds_denied=DELE,RMD,RNFR,RNTO,APPE\n"
            "local_root=/srv/devicedata\n",
            encoding="utf-8",
        )
        with self.assertRaises(ota.IntegrityError):
            ota.list_accounts()

        (self.conf_dir / "alpha").write_text(
            ota._upload_only_config("alpha").replace("file_open_mode=0440", "file_open_mode=0666"),
            encoding="utf-8",
        )
        with self.assertRaises(ota.IntegrityError):
            ota.list_accounts()

        (self.conf_dir / "alpha").write_text(
            ota._upload_only_config("alpha").replace("file_open_mode=0440\n", ""),
            encoding="utf-8",
        )
        with self.assertRaises(ota.IntegrityError):
            ota.list_accounts()

        (self.conf_dir / "alpha").write_text(
            ota._upload_only_config("alpha") + "download_enable=NO\n",
            encoding="utf-8",
        )
        with self.assertRaises(ota.IntegrityError):
            ota.list_accounts()

    def test_update_restores_permission_when_chpasswd_fails(self):
        self.seed_account("alpha", "upload")
        before = (self.conf_dir / "alpha").read_bytes()
        self.fail_executable = ota.CHPASSWD
        with self.assertRaises(ota.CommandError):
            ota.account_update("alpha", password="valid-pass", permission="upload")
        self.assertEqual(before, (self.conf_dir / "alpha").read_bytes())

    def test_delete_restores_allowlist_and_permission_when_userdel_fails(self):
        self.seed_account("alpha", "upload")
        before = self.userlist.read_bytes()
        permission_before = (self.conf_dir / "alpha").read_bytes()
        self.fail_executable = ota.USERDEL
        with self.assertRaises(ota.CommandError):
            ota.account_delete("alpha")
        self.assertEqual(before, self.userlist.read_bytes())
        self.assertEqual(permission_before, (self.conf_dir / "alpha").read_bytes())
        self.assertIn("alpha", self.fake_pwd.records)

    def test_userdel_timeout_after_commit_does_not_republish_missing_account(self):
        self.seed_account("alpha", "upload")

        def committed_then_failed(command, input_text=None, deadline=None, reserve_seconds=0):
            if command[0] == ota.USERDEL:
                self.fake_pwd.records.pop(command[-1], None)
                raise ota.CommandError(command[0], -1)

        with mock.patch.object(ota, "run", side_effect=committed_then_failed):
            ota.account_delete("alpha")
        self.assertNotIn("alpha", self.userlist.read_text(encoding="utf-8"))
        self.assertFalse((self.conf_dir / "alpha").exists())

    def test_protected_accounts_cannot_be_deleted(self):
        for name in sorted(ota.PROTECTED):
            with self.subTest(name=name), self.assertRaises(ota.ApiError):
                ota.account_delete(name)
        self.assertEqual([], self.commands)

    def test_fixed_system_accounts_reject_reverse_permissions_but_allow_password_updates(self):
        self.seed_account("devicedata", "full")
        before_commands = list(self.commands)
        with self.assertRaises(ota.ApiError):
            ota.account_update("devicedata", permission="upload")
        self.assertEqual(before_commands, self.commands)
        self.assertEqual("full", ota.list_accounts()[-1]["permission"])

        ota.account_update("devicedata", password="new-valid-pass")
        self.assertEqual((ota.CHPASSWD,), self.commands[-1])

    def test_corrupt_fixed_permission_fails_closed_for_list_and_password_update(self):
        self.seed_account("devicedata", "full")
        (self.conf_dir / "devicedata").write_text(
            ota._upload_only_config("devicedata"), encoding="utf-8"
        )

        with self.assertRaises(ota.IntegrityError):
            ota.list_accounts()
        before_commands = list(self.commands)
        with self.assertRaises(ota.IntegrityError):
            ota.account_update("devicedata", password="new-valid-pass")
        self.assertEqual(before_commands, self.commands)

    def test_retired_shared_uploader_is_never_publishable(self):
        for operation in (
            lambda: ota.account_create("uploader", "valid-pass", "upload"),
            lambda: ota.account_update("uploader", password="valid-pass"),
            lambda: ota.write_userlist(["uploader"]),
        ):
            with self.assertRaises(ota.ApiError):
                operation()
        self.userlist.write_text("uploader\n", encoding="utf-8")
        with self.assertRaises(ota.IntegrityError):
            ota.list_accounts()

    def test_create_transaction_shares_budget_with_rollback(self):
        clock = [100.0]
        subprocess_calls = []

        def delayed_subprocess(command, **kwargs):
            executable = command[0]
            timeout = kwargs["timeout"]
            subprocess_calls.append((executable, timeout))
            self.assertEqual(
                self.fake_fcntl.LOCK_EX | self.fake_fcntl.LOCK_NB,
                self.fake_fcntl.operations[-1],
            )
            if executable == ota.USERADD:
                clock[0] += 7
                name = command[-1]
                self.fake_pwd.records[name] = types.SimpleNamespace(
                    pw_uid=ota.MIN_MANAGED_UID,
                    pw_dir=ota.FTP_HOME,
                    pw_shell=ota.NOLOGIN_SHELL,
                    pw_gid=1000,
                )
                return types.SimpleNamespace(returncode=0)
            if executable == ota.CHPASSWD:
                # The normal command may use only the non-reserved remainder.
                clock[0] += timeout
                raise ota.subprocess.TimeoutExpired(command, timeout)
            if executable == ota.USERDEL:
                # Rollback gets the reserved tail of the same 12-second budget.
                clock[0] += 1
                self.fake_pwd.records.pop(command[-1], None)
                return types.SimpleNamespace(returncode=0)
            if executable == ota.GROUPDEL:
                return types.SimpleNamespace(returncode=0)
            self.fail("unexpected subprocess")

        def bounded_real_run(command, input_text=None, deadline=None, reserve_seconds=0):
            return REAL_RUN(
                command,
                input_text=input_text,
                deadline=deadline,
                reserve_seconds=reserve_seconds,
            )

        with (
            mock.patch.object(ota.time, "monotonic", side_effect=lambda: clock[0]),
            mock.patch.object(ota.subprocess, "run", side_effect=delayed_subprocess),
            mock.patch.object(ota, "run", side_effect=bounded_real_run),
        ):
            with self.assertRaises(ota.CommandError):
                ota.account_create("alpha", "valid-pass", "upload")

        self.assertEqual(
            [ota.USERADD, ota.CHPASSWD, ota.USERDEL, ota.GROUPDEL],
            [item[0] for item in subprocess_calls],
        )
        self.assertEqual([10, 3, 2, 1], [item[1] for item in subprocess_calls])
        self.assertLessEqual(clock[0] - 100.0, ota.OPERATION_TIMEOUT_SECONDS)
        self.assertNotIn("alpha", self.fake_pwd.records)
        self.assertNotIn("alpha", self.userlist.read_text(encoding="utf-8"))

    def test_mutations_take_exclusive_flock_and_list_takes_shared_flock(self):
        self.fake_fcntl.operations.clear()
        ota.list_accounts()
        self.assertEqual(
            [self.fake_fcntl.LOCK_SH | self.fake_fcntl.LOCK_NB, self.fake_fcntl.LOCK_UN],
            self.fake_fcntl.operations,
        )

        self.fake_fcntl.operations.clear()
        ota.account_create("alpha", "valid-pass", "upload")
        self.assertEqual(
            [self.fake_fcntl.LOCK_EX | self.fake_fcntl.LOCK_NB, self.fake_fcntl.LOCK_UN],
            self.fake_fcntl.operations,
        )

    def test_flock_contention_consumes_operation_budget_without_starting_commands(self):
        clock = [100.0]
        self.fake_fcntl.contended = True

        def advance(delay):
            clock[0] += delay

        with (
            mock.patch.object(ota.time, "monotonic", side_effect=lambda: clock[0]),
            mock.patch.object(ota.time, "sleep", side_effect=advance),
            mock.patch.object(ota, "LOCK_POLL_SECONDS", 1),
        ):
            with self.assertRaises(ota.OperationTimeoutError):
                ota.account_create("alpha", "valid-pass", "upload")

        self.assertAlmostEqual(ota.OPERATION_TIMEOUT_SECONDS, clock[0] - 100.0, places=6)
        self.assertEqual([], self.commands)
        self.assertNotIn(self.fake_fcntl.LOCK_UN, self.fake_fcntl.operations)

    def test_posix_account_lock_rejects_insecure_parent_and_file(self):
        if os.name != "posix":
            self.skipTest("POSIX lock-file invariants are exercised on the deployment CI host")

        os.chmod(self.root, 0o770)
        try:
            with self.assertRaises(ota.IntegrityError):
                ota._open_account_lock_file()
        finally:
            os.chmod(self.root, 0o700)

        with (
            mock.patch.object(ota, "ACCOUNT_LOCK_FILE", str(self.root / "owner-check.lock")),
            mock.patch.object(ota, "REQUIRED_LOCK_OWNER_UID", os.getuid() + 1),
        ):
            with self.assertRaises(ota.IntegrityError):
                ota._open_account_lock_file()

        real_parent = self.root / "real-lock-parent"
        linked_parent = self.root / "linked-lock-parent"
        real_parent.mkdir(mode=0o700)
        linked_parent.symlink_to(real_parent, target_is_directory=True)
        with mock.patch.object(ota, "ACCOUNT_LOCK_FILE", str(linked_parent / "lock")):
            with self.assertRaises(ota.IntegrityError):
                ota._open_account_lock_file()

        self.account_lock.write_text("", encoding="utf-8")
        os.chmod(self.account_lock, 0o640)
        with self.assertRaises(ota.IntegrityError):
            ota._open_account_lock_file()

        os.chmod(self.account_lock, 0o600)
        with mock.patch.object(ota, "REQUIRED_LOCK_OWNER_UID", os.getuid() + 1):
            with self.assertRaises(ota.IntegrityError):
                ota._open_account_lock_file()

        self.account_lock.unlink()
        target = self.root / "lock-target"
        target.write_text("", encoding="utf-8")
        self.account_lock.symlink_to(target)
        with self.assertRaises(OSError):
            ota._open_account_lock_file()

    def test_poisoned_allowlist_is_rejected_without_path_access(self):
        self.userlist.write_text("# keep\n../escape\n", encoding="utf-8")
        with self.assertRaises(ota.IntegrityError):
            ota.list_accounts()
        self.assertEqual([], self.commands)

    def test_atomic_replace_failure_preserves_previous_allowlist_and_cleans_temp(self):
        before = self.userlist.read_bytes()
        with mock.patch.object(ota.os, "replace", side_effect=OSError("offline fault")):
            with self.assertRaises(OSError):
                ota.write_userlist(["alpha"])
        self.assertEqual(before, self.userlist.read_bytes())
        self.assertEqual([], [item for item in self.root.iterdir() if item.name.startswith(".vsftpd.userlist.")])

    def test_concurrent_creates_do_not_lose_allowlist_entries(self):
        errors = []

        def create(index):
            try:
                ota.account_create("user%02d" % index, "valid-pass", "upload")
            except Exception as exc:  # captured for the main test thread
                errors.append(exc)

        threads = [threading.Thread(target=create, args=(index,)) for index in range(20)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=5)
        self.assertFalse(errors)
        self.assertTrue(all(not thread.is_alive() for thread in threads))
        self.assertEqual(20, len(ota.list_accounts()))
        self.assertEqual(20, len(set(account["name"] for account in ota.list_accounts())))

    def test_managed_profile_check_blocks_unrelated_allowlisted_system_account(self):
        ota.write_userlist(["alpha"])
        self.fake_pwd.records["alpha"] = types.SimpleNamespace(
            pw_uid=ota.MIN_MANAGED_UID,
            pw_dir="/root",
            pw_shell="/bin/bash",
            pw_gid=2400,
        )
        with self.assertRaises(ota.IntegrityError):
            ota.account_update("alpha", permission="upload")
        with self.assertRaises(ota.IntegrityError):
            ota.account_delete("alpha")
        self.assertEqual([], self.commands)

    def test_managed_profile_rejects_root_and_system_uids_even_with_matching_home_and_shell(self):
        ota.write_userlist(["alpha"])
        for uid in (0, ota.MIN_MANAGED_UID - 1):
            with self.subTest(uid=uid):
                self.fake_pwd.records["alpha"] = types.SimpleNamespace(
                    pw_uid=uid,
                    pw_dir=ota.FTP_HOME,
                    pw_shell=ota.NOLOGIN_SHELL,
                    pw_gid=self.fake_grp.ftp_gid,
                )
                with self.assertRaises(ota.IntegrityError):
                    ota.account_update("alpha", permission="upload")
                with self.assertRaises(ota.IntegrityError):
                    ota.account_delete("alpha")
        self.assertEqual([], self.commands)

    def test_managed_profile_rejects_shared_uid_alias(self):
        ota.write_userlist(["alpha"])
        shared_uid = ota.MIN_MANAGED_UID + 50
        for name in ("alpha", "alias"):
            self.fake_pwd.records[name] = types.SimpleNamespace(
                pw_uid=shared_uid,
                pw_dir=ota.FTP_HOME,
                pw_shell=ota.NOLOGIN_SHELL,
                pw_gid=self.fake_grp.ftp_gid,
            )
        with self.assertRaises(ota.IntegrityError):
            ota.account_update("alpha", password="valid-pass")
        with self.assertRaises(ota.IntegrityError):
            ota.account_delete("alpha")
        self.assertEqual([], self.commands)

    def test_managed_profile_rejects_privileged_or_aliased_ftp_group_gid(self):
        ota.write_userlist(["alpha"])
        self.fake_pwd.records["alpha"] = types.SimpleNamespace(
            pw_uid=ota.MIN_MANAGED_UID,
            pw_dir=ota.FTP_HOME,
            pw_shell=ota.NOLOGIN_SHELL,
            pw_gid=2400,
        )

        self.fake_grp.ftp_gid = 0
        with self.assertRaises(ota.IntegrityError):
            ota.account_update("alpha", password="valid-pass")

        self.fake_grp.ftp_gid = 2400
        self.fake_grp.extra_records.append(
            types.SimpleNamespace(gr_name="privileged-alias", gr_gid=2400, gr_mem=[])
        )
        with self.assertRaises(ota.IntegrityError):
            ota.account_update("alpha", password="valid-pass")
        with self.assertRaises(ota.IntegrityError):
            ota.account_delete("alpha")
        self.assertEqual([], self.commands)

    def test_files_are_published_with_no_group_or_other_permissions(self):
        if os.name != "posix":
            self.skipTest("POSIX file mode enforcement is exercised on the deployment host")
        ota.account_create("alpha", "valid-pass", "upload")
        self.assertEqual(0, stat.S_IMODE(os.stat(self.userlist).st_mode) & 0o077)
        self.assertEqual(0, stat.S_IMODE(os.stat(self.conf_dir / "alpha").st_mode) & 0o077)


class TokenFileTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def write_token(self, value, mode=0o600):
        path = self.root / "token"
        path.write_bytes(value.encode("utf-8"))
        os.chmod(path, mode)
        return path

    @contextmanager
    def token_owner(self, path, uid):
        """Make only the token's lstat owner deterministic on unprivileged CI."""
        original_lstat = os.lstat
        expected = os.path.normcase(os.path.abspath(str(path)))

        def controlled_lstat(candidate):
            info = original_lstat(candidate)
            actual = os.path.normcase(os.path.abspath(str(candidate)))
            if actual != expected:
                return info
            return types.SimpleNamespace(
                st_mode=info.st_mode,
                st_uid=uid,
                st_size=info.st_size,
            )

        with mock.patch.object(ota.os, "lstat", side_effect=controlled_lstat):
            yield

    def test_secure_token_loads_but_empty_short_whitespace_and_oversize_fail(self):
        for suffix in ("", "\n", "\r\n"):
            with self.subTest(valid_suffix=repr(suffix)):
                valid = self.write_token(TEST_TOKEN + suffix)
                with self.token_owner(valid, 0):
                    self.assertEqual(TEST_TOKEN, ota.load_admin_token(str(valid)))
        for value in (
            "",
            "short",
            "A" * 31,
            " " + TEST_TOKEN + " ",
            TEST_TOKEN + "\n\n",
            TEST_TOKEN + "\r\n\r\n",
            TEST_TOKEN + "\r",
            "A" * 20 + " " + "B" * 20,
            "A" * 32 + "非",
            "A" * 513,
        ):
            with self.subTest(length=len(value)):
                path = self.write_token(value)
                with self.token_owner(path, 0):
                    with self.assertRaises(RuntimeError):
                        ota.load_admin_token(str(path))

    def test_group_readable_and_symlink_tokens_fail(self):
        path = self.write_token(TEST_TOKEN, mode=0o640)
        if stat.S_IMODE(os.stat(path).st_mode) & 0o040:
            with self.token_owner(path, 0):
                with mock.patch.object(ota.os, "name", "posix"):
                    with self.assertRaises(RuntimeError):
                        ota.load_admin_token(str(path))
        else:
            self.skipTest("host filesystem does not expose POSIX permission bits")

        target = self.write_token(TEST_TOKEN)
        link = self.root / "token-link"
        try:
            link.symlink_to(target)
        except OSError:
            return
        with self.assertRaises(RuntimeError):
            ota.load_admin_token(str(link))

    def test_posix_token_must_be_owned_by_root(self):
        path = self.write_token(TEST_TOKEN)
        with self.token_owner(path, 1234):
            with mock.patch.object(ota.os, "name", "posix"):
                with self.assertRaises(RuntimeError):
                    ota.load_admin_token(str(path))


class HttpApiTests(AdminFixture):
    def request(self, method, path, body=None, token=TEST_TOKEN, headers=None):
        raw = None if body is None else json.dumps(body).encode("utf-8")
        merged = list((headers or {}).items())
        if token is not None:
            merged.append(("X-Admin-Token", token))
        if raw is not None:
            if not any(name.lower() == "content-type" for name, _value in merged):
                merged.append(("Content-Type", "application/json"))
            if not any(name.lower() == "content-length" for name, _value in merged):
                merged.append(("Content-Length", str(len(raw))))
        status, response_headers, response_body = self.raw_request(
            method, path, merged, payload=raw or b""
        )
        return status, response_headers, response_body

    def raw_request(self, method, path, headers, payload=b""):
        handler = object.__new__(ota.Handler)
        handler.command = method
        handler.path = path
        handler.request_version = "HTTP/1.1"
        handler.requestline = "%s %s HTTP/1.1" % (method, path)
        handler.client_address = ("127.0.0.1", 12345)
        handler.server = types.SimpleNamespace(admin_token=TEST_TOKEN)
        handler.headers = Message()
        for name, value in headers:
            handler.headers.add_header(name, value)
        handler.rfile = io.BytesIO(payload)
        handler.wfile = io.BytesIO()
        handler.close_connection = False
        getattr(handler, "do_%s" % method)()
        wire = handler.wfile.getvalue()
        head, raw_body = wire.split(b"\r\n\r\n", 1)
        header_lines = head.decode("iso-8859-1").split("\r\n")
        status = int(header_lines[0].split(" ", 2)[1])
        response_headers = []
        for line in header_lines[1:]:
            name, value = line.split(":", 1)
            response_headers.append((name, value.strip()))
        return status, response_headers, json.loads(raw_body.decode("utf-8"))

    def test_auth_is_required_and_duplicate_or_oversize_tokens_fail(self):
        status, headers, body = self.request("GET", "/admin/api/ping", token=None)
        self.assertEqual(401, status)
        self.assertFalse(body["ok"])
        self.assertIn(("Cache-Control", "no-store"), headers)

        status, _headers, body = self.raw_request(
            "GET",
            "/admin/api/ping",
            [("X-Admin-Token", TEST_TOKEN), ("X-Admin-Token", TEST_TOKEN)],
        )
        self.assertEqual(401, status)
        self.assertFalse(body["ok"])

        status, _, _ = self.request("GET", "/admin/api/ping", token="A" * 513)
        self.assertEqual(401, status)
        status, _, _ = self.request("GET", "/admin/api/ping", token="A" * 32 + "非")
        self.assertEqual(401, status)

    def test_content_length_transfer_encoding_and_media_type_fail_closed(self):
        base = [("X-Admin-Token", TEST_TOKEN), ("Content-Type", "application/json")]
        cases = (
            (base + [("Content-Length", "-1")], 400),
            (base + [("Content-Length", "abc")], 400),
            (base + [("Content-Length", str(ota.MAX_BODY_BYTES + 1))], 413),
            (base + [("Content-Length", "9" * 1000)], 413),
            (base + [("Content-Length", "2"), ("Content-Length", "2")], 411),
            (base + [("Transfer-Encoding", "chunked"), ("Content-Length", "2")], 400),
            (
                base
                + [("Content-Type", "application/json"), ("Content-Length", "2")],
                415,
            ),
        )
        for headers, expected in cases:
            with self.subTest(headers=headers):
                status, _response_headers, body = self.raw_request("POST", "/admin/api/accounts", headers)
                self.assertEqual(expected, status)
                self.assertFalse(body["ok"])

        status, _response_headers, body = self.raw_request(
            "POST",
            "/admin/api/accounts",
            [("X-Admin-Token", TEST_TOKEN), ("Content-Type", "text/plain"), ("Content-Length", "2")],
            b"{}",
        )
        self.assertEqual(415, status)

        for method, path in (("GET", "/admin/api/ping"), ("DELETE", "/admin/api/accounts/alpha")):
            status, _headers, body = self.raw_request(
                method,
                path,
                [("X-Admin-Token", TEST_TOKEN), ("Content-Length", "2")],
                b"{}",
            )
            self.assertEqual(400, status)
            self.assertFalse(body["ok"])

    def test_json_schema_rejects_duplicates_unknowns_arrays_and_wrong_types(self):
        malformed = (
            b'{"name":"alpha","name":"beta","password":"valid-pass"}',
            b'{"name":"alpha","password":"valid-pass","extra":true}',
            b'[]',
            b'{"name":123,"password":"valid-pass"}',
            b'{"name":"alpha","password":["valid-pass"]}',
        )
        for payload in malformed:
            with self.subTest(payload=payload):
                status, _response_headers, body = self.raw_request(
                    "POST",
                    "/admin/api/accounts",
                    [
                        ("X-Admin-Token", TEST_TOKEN),
                        ("Content-Type", "application/json"),
                        ("Content-Length", str(len(payload))),
                    ],
                    payload,
                )
                self.assertEqual(400, status)
                self.assertFalse(body["ok"])
        self.assertEqual([], self.commands)

        self.seed_account("alpha")
        for body in ({"password": None}, {"permission": None}, {"permission": 1}):
            with self.subTest(body=body):
                status, _, result = self.request("PATCH", "/admin/api/accounts/alpha", body)
                self.assertEqual(400, status)
                self.assertFalse(result["ok"])

    def test_canonical_paths_reject_query_percent_encoding_and_backslash(self):
        for path in (
            "/admin/api/accounts/alpha?password=secret",
            "/admin/api/accounts/%2e%2e",
            "/admin/api/accounts/alpha%2froot",
            "/admin/api/accounts/alpha\\root",
            "http://example.invalid/admin/api/accounts/alpha",
        ):
            with self.subTest(path=path):
                status, _, body = self.request("DELETE", path)
                self.assertEqual(400, status)
                self.assertFalse(body["ok"])

    def test_subprocess_failure_returns_generic_error_without_password_or_details(self):
        self.fail_executable = ota.CHPASSWD
        password = "never-return-this"
        with self.assertLogs(ota.LOG, level="ERROR") as captured:
            status, _, body = self.request(
                "POST",
                "/admin/api/accounts",
                {"name": "alpha", "password": password, "permission": "upload"},
            )
        encoded = json.dumps(body, ensure_ascii=False)
        logs = "\n".join(captured.output)
        self.assertEqual(500, status)
        self.assertNotIn(password, encoded)
        self.assertNotIn("chpasswd", encoded)
        self.assertNotIn(password, logs)
        self.assertNotIn("chpasswd", logs)
        self.assertEqual("服务器内部错误", body["error"])

    def test_command_timeout_returns_fixed_safe_503_retry_message(self):
        self.seed_account("alpha")
        password = "never-return-this"
        with mock.patch.object(
            ota, "run", side_effect=ota.CommandTimeoutError(ota.CHPASSWD)
        ):
            status, _, body = self.request(
                "PATCH", "/admin/api/accounts/alpha", {"password": password}
            )

        encoded = json.dumps(body, ensure_ascii=False)
        self.assertEqual(503, status)
        self.assertEqual(ota.RETRYABLE_TIMEOUT_MESSAGE, body["error"])
        for secret in (password, "alpha", "chpasswd"):
            self.assertNotIn(secret, encoded)

    def test_lock_deadline_returns_same_fixed_safe_503_without_mutation(self):
        clock = [100.0]
        self.fake_fcntl.contended = True

        def advance(delay):
            clock[0] += delay

        with (
            mock.patch.object(ota.time, "monotonic", side_effect=lambda: clock[0]),
            mock.patch.object(ota.time, "sleep", side_effect=advance),
            mock.patch.object(ota, "LOCK_POLL_SECONDS", 1),
        ):
            status, _, body = self.request(
                "POST",
                "/admin/api/accounts",
                {"name": "alpha", "password": "never-return-this", "permission": "upload"},
            )

        encoded = json.dumps(body, ensure_ascii=False)
        self.assertEqual(503, status)
        self.assertEqual(ota.RETRYABLE_TIMEOUT_MESSAGE, body["error"])
        for secret in ("never-return-this", "alpha", "useradd"):
            self.assertNotIn(secret, encoded)
        self.assertEqual([], self.commands)

    def test_unsupported_method_still_requires_auth_and_returns_json(self):
        status, _, body = self.request("PUT", "/admin/api/accounts", token=None)
        self.assertEqual(401, status)
        self.assertFalse(body["ok"])
        status, _, body = self.request("PUT", "/admin/api/accounts")
        self.assertEqual(405, status)
        self.assertEqual("不支持的 HTTP 方法", body["error"])

    def test_log_message_handles_parser_errors_without_leaking_raw_arguments(self):
        handler = object.__new__(ota.Handler)
        handler.client_address = ("127.0.0.1", 12345)
        with self.assertLogs(ota.LOG, level="INFO") as captured:
            handler.log_message("timeout while parsing %r", "secret-in-request")
        logs = "\n".join(captured.output)
        self.assertIn("client=127.0.0.1 method=- status=-", logs)
        self.assertNotIn("secret-in-request", logs)

    def test_successful_create_patch_list_delete_and_status_codes(self):
        status, _, _ = self.request(
            "POST",
            "/admin/api/accounts",
            {"name": "alpha", "password": "valid-pass", "permission": "upload"},
        )
        self.assertEqual(201, status)
        status, _, body = self.request("GET", "/admin/api/accounts")
        self.assertEqual(200, status)
        self.assertEqual("upload", body["accounts"][0]["permission"])
        status, _, _ = self.request("PATCH", "/admin/api/accounts/alpha", {"permission": "full"})
        self.assertEqual(400, status)
        status, _, _ = self.request(
            "PATCH", "/admin/api/accounts/alpha", {"password": "new-valid-pass"}
        )
        self.assertEqual(200, status)
        status, _, _ = self.request("DELETE", "/admin/api/accounts/alpha")
        self.assertEqual(200, status)
        self.assertTrue((self.data_dir / "alpha").is_dir())
        with self.assertRaises(ota.ConflictError):
            ota.account_create("alpha", "valid-pass", "upload")

    def test_http_rejects_retired_shared_uploader(self):
        for method, path, payload in (
            ("POST", "/admin/api/accounts", {"name": "uploader", "password": "valid-pass", "permission": "upload"}),
            ("PATCH", "/admin/api/accounts/uploader", {"password": "new-valid-pass"}),
        ):
            status, _, body = self.request(method, path, payload)
            self.assertEqual(400, status)
            self.assertFalse(body["ok"])


class StatsTests(AdminFixture):
    def test_stats_skip_symlinked_files_and_device_directories(self):
        device = self.data_dir / "device-a"
        device.mkdir()
        (device / "payload.bin").write_bytes(b"1234")
        outside = self.root / "outside.bin"
        outside.write_bytes(b"secret-outside-data")
        try:
            (device / "outside-link").symlink_to(outside)
            (self.data_dir / "device-link").symlink_to(self.root, target_is_directory=True)
        except OSError:
            self.skipTest("host does not permit symlink creation")
        stats = ota.build_stats()
        self.assertEqual(4, stats["dataBytes"])
        self.assertEqual(1, stats["devices"][0]["files"])
        self.assertEqual(["device-a"], [device["name"] for device in stats["devices"]])

    def test_stats_share_one_entry_budget_across_all_devices(self):
        for device_name in ("device-a", "device-b"):
            device = self.data_dir / device_name
            device.mkdir()
            (device / "one.bin").write_bytes(b"1")
            (device / "two.bin").write_bytes(b"2")
        with mock.patch.object(ota, "MAX_DIRECTORY_SCAN_ENTRIES", 4), \
                self.assertRaisesRegex(ota.IntegrityError, "entry budget"):
            ota.build_stats()


class DirectoryScanLimitTests(AdminFixture):
    def test_account_list_rejects_excessive_depth_and_entry_count(self):
        self.seed_account("alpha")
        root = self.data_dir / "alpha"
        current = root
        for index in range(4):
            current = current / f"depth-{index}"
            current.mkdir()
        with mock.patch.object(ota, "MAX_DIRECTORY_SCAN_DEPTH", 2), \
                self.assertRaisesRegex(ota.IntegrityError, "maximum scan depth"):
            ota.list_accounts()

        shutil.rmtree(root / "depth-0")
        for index in range(5):
            (root / f"entry-{index}.bin").write_bytes(b"x")
        with mock.patch.object(ota, "MAX_DIRECTORY_SCAN_ENTRIES", 3), \
                self.assertRaisesRegex(ota.IntegrityError, "entry budget"):
            ota.list_accounts()

    def test_scan_checks_shared_deadline_while_enumerating(self):
        root = self.data_dir / "deadline"
        root.mkdir()
        for index in range(10):
            (root / f"entry-{index}.bin").write_bytes(b"x")

        class ExpiringDeadline:
            def __init__(self):
                self.calls = 0

            def remaining(self):
                self.calls += 1
                if self.calls > 4:
                    raise ota.OperationTimeoutError("scan deadline")
                return 1.0

        deadline = ExpiringDeadline()
        with self.assertRaises(ota.OperationTimeoutError):
            list(ota._walk_device_tree(root, deadline, ota.DirectoryScanBudget()))
        self.assertGreater(deadline.calls, 4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
