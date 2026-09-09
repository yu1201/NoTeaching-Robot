"""Brand backend for the standalone READ-ONLY kinematics identification experiment.

Not an application business entry or production-driver replacement. Every socket
request is whitelisted. No login, permit, Set, motion, reset, STOR or DELE exists.
The common fitting workflow uses only snapshot/forward/inverse, not vendor commands.
"""
from __future__ import annotations

import ftplib
import hashlib
import io
import json
import re
import socket
import time
from contextlib import AbstractContextManager

import numpy as np
from kinematics_model_fit import dh, pose_matrix


NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
FIXED_GETTERS = {"Get_RobotType", "Get_FwVersion", "Get_MotionSts", "Get_RobJPHere",
                 "Get_ToolCNum", "Get_WobjNum", "Get_StrPara", "Get_StrParaComp",
                 "Get_SupplementaryStrParamComp"}


def allowed_request(command):
    if command in FIXED_GETTERS:
        return True
    if re.fullmatch(r"Get_Axis[NP]Lim J[1-6]", command):
        return True
    if re.fullmatch(r"Get_(?:ToolData|WobjData) (?:[0-9]|1[0-5])", command):
        return True
    # Strict grammar: no arbitrary Get_* passthrough and no injected framing.
    if command.startswith("Get_RobJToRobP "):
        data = command[len("Get_RobJToRobP "):]
        pattern = rf"{NUMBER}(?:,{NUMBER}){{7}};{NUMBER}(?:,{NUMBER}){{5}} \d+,\d+,\d+"
        return re.fullmatch(pattern, data) is not None
    if command.startswith("Get_RobPToRobJ "):
        data = command[len("Get_RobPToRobJ "):]
        pattern = rf"{NUMBER}(?:,{NUMBER}){{5}};[-+]?\d+(?:,[-+]?\d+){{3}};{NUMBER}(?:,{NUMBER}){{5}} \d+,\d+,\d+"
        return re.fullmatch(pattern, data) is not None
    return False


def numbers(reply, count):
    if not reply.startswith("="):
        raise ValueError("Missing numeric reply prefix")
    fields = re.split(r"[,;]", reply[1:].rstrip(";"))
    if len(fields) != count or any(re.fullmatch(NUMBER, field) is None for field in fields):
        raise ValueError(f"Malformed numeric reply (expected {count} fields)")
    values = np.array([float(v) for v in fields])
    if not np.isfinite(values).all():
        raise ValueError("Nonfinite controller reply")
    return values


class ControllerRejection(RuntimeError):
    pass


class InovanceReference(AbstractContextManager):
    def __init__(self, host, port=2222, ftp_port=7777, ftp_user="robot", ftp_password="", rate=5):
        if not 0 < rate <= 10:
            raise ValueError("Read-only polling rate must be in (0,10] requests/s")
        self.host, self.port = host, port
        self.ftp_port, self.ftp_user, self.ftp_password = ftp_port, ftp_user, ftp_password
        self.interval = 1 / rate; self.last = 0.; self.socket = None; self.audit = None

    def __enter__(self):
        self.socket = socket.create_connection((self.host, self.port), timeout=4)
        self.socket.settimeout(4)
        return self

    def __exit__(self, *_):
        if self.socket is not None:
            self.socket.close(); self.socket = None

    def query(self, command):
        if not allowed_request(command):
            raise ValueError("Blocked: command is not in the read-only calculation whitelist")
        time.sleep(max(0, self.last + self.interval - time.monotonic()))
        self.last = time.monotonic()
        self.socket.sendall(("@@" + command + "$$").encode("ascii"))
        frame = b""
        while b"$$" not in frame:
            part = self.socket.recv(4096)
            if not part:
                raise ConnectionError("Controller closed connection")
            frame += part
            if len(frame) > 32768:
                raise ValueError("Oversize controller reply")
        if not frame.startswith(b"##") or not frame.endswith(b"$$") or frame.count(b"$$") != 1:
            raise ValueError("Malformed controller frame")
        reply = frame[2:-2].decode("ascii").strip()
        if self.audit:
            self.audit(command, reply)
        if reply.lower().startswith("e"):
            raise ControllerRejection(command.split()[0] + ": " + reply)
        return reply

    def assert_stopped(self):
        if numbers(self.query("Get_MotionSts"), 1)[0] != 0:
            raise RuntimeError("Robot is moving: experiment stopped without sending STOP or any write")

    def snapshot(self, profiles):
        self.assert_stopped()
        model = self.query("Get_RobotType").removeprefix("=")
        firmware = self.query("Get_FwVersion").removeprefix("=")
        with ftplib.FTP() as ftp:
            ftp.connect(self.host, self.ftp_port, timeout=5)
            ftp.login(self.ftp_user, self.ftp_password)
            data = io.BytesIO()
            def receive(block):
                if data.tell() + len(block) > 2 * 1024 * 1024:
                    raise ValueError("Machine parameter file exceeds 2 MiB")
                data.write(block)
            ftp.retrbinary("RETR /RobotParams/MachineParams.json", receive)
        machine = json.loads(data.getvalue())
        body = machine["stRobotBody"]
        if body["cRobotName"] != model or body["RobotType"] != 6 or body["stBase"]["i32AxisNum"] != 6:
            raise ValueError("Unsupported model or TCP/FTP identity mismatch")
        if not model.startswith("IR-R"):
            raise ValueError("Nominal mapping is scoped to this IR-R six-axis experiment")
        structure = body["stKinematics"]["dRobotStructureParam"][:6]
        install = machine["stMotion"]["stSpace"]["stInstall"]["stInstallMode"]
        angles = [install[k] for k in ("alpha1", "alpha2", "alpha3", "alpha4", "alpha5", "beta2")]
        lengths = [install[k] for k in ("d3", "d5", "a4", "a5")]
        for command, expected in [("Get_StrPara", structure), ("Get_StrParaComp", angles),
                                  ("Get_SupplementaryStrParamComp", lengths)]:
            actual = numbers(self.query(command), len(expected))
            if np.max(np.abs(actual - expected)) > .002:
                raise ValueError("TCP/FTP mechanical parameter mismatch: " + command)
        joint = machine["stJoint"]
        limits = np.array([joint["dNegLimit"][:6], joint["dPosLimit"][:6]]).T
        if not np.isfinite(limits).all() or np.any(limits[:, 1] - limits[:, 0] < 20):
            raise ValueError("Invalid joint limits")
        for i in range(6):
            for j, kind in enumerate(["N", "P"]):
                if abs(numbers(self.query(f"Get_Axis{kind}Lim J{i+1}"), 1)[0] - limits[i, j]) > .002:
                    raise ValueError("TCP/FTP limit mismatch")
        a = [structure[0], structure[1], structure[2], install["a4"], install["a5"], 0]
        d = [structure[5], 0, install["d3"], structure[3], install["d5"], structure[4]]
        offsets = [0, 90, 0, 0, 0, 0]
        # Reproduce CURRENT application's baseline, not a claim beta2 is alpha6.
        nominal = [dh(a[i], angles[i], d[i], offsets[i]).tolist() for i in range(6)]
        contexts = {}
        for profile in profiles:
            tool_no, wobj_no, load_no = profile
            if any(not 0 <= x <= 15 for x in profile):
                raise ValueError("Profile indices must be in 0..15")
            tr = self.query(f"Get_ToolData {tool_no}"); wr = self.query(f"Get_WobjData {wobj_no}")
            tv, wv = numbers(tr, 17), numbers(wr, 14)
            if tv[0] != 1 or tuple(wv[:2]) != (0, 1):
                raise ValueError("Only robot-held tools and fixed work objects are supported")
            tool = pose_matrix(tv[1:7])
            work = pose_matrix(wv[2:8]) @ pose_matrix(wv[8:14])
            contexts[','.join(map(str, profile))] = {"tool_no": tool_no, "wobj_no": wobj_no,
                "load_no": load_no, "tool_reply": tr, "wobj_reply": wr,
                "tool_matrix": tool.tolist(), "work_matrix": work.tolist()}
        return {"schema": "kinematic-reference-v1", "host": self.host, "port": self.port,
                "model": model, "firmware": firmware,
                "machine_sha256": hashlib.sha256(data.getvalue()).hexdigest(),
                "nominal_links": nominal, "limits_deg": limits.tolist(),
                "contexts": contexts, "source": "TCP2222+FTP/RobotParams/MachineParams.json",
                "current_joints": numbers(self.query("Get_RobJPHere"), 14)[:6].tolist(),
                "active_tool": int(numbers(self.query("Get_ToolCNum"), 1)[0]),
                "active_wobj": int(numbers(self.query("Get_WobjNum"), 1)[0]),
                "structure": structure, "install": install,
                "absolute_accuracy": body.get("stAccuracy", {})}

    def forward(self, q, profile):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("Expected finite six joint angles")
        q = np.round(q, 3)  # Store and fit exactly the values sent to the controller.
        point = ','.join(f"{v:.3f}" for v in q) + ",0.000,0.000;" + ','.join(["0.000"]*6)
        reply = self.query("Get_RobJToRobP " + point + " " + profile)
        value = numbers(reply, 16)
        if np.any(value[6:10] != np.round(value[6:10])) or np.any(value[10:] != 0):
            raise ValueError("Invalid arm flags or unexpected external axes")
        return {"joint_deg": q.tolist(), "pose": value[:6].tolist(),
                "arm": value[6:10].astype(int).tolist(), "raw_reply": reply}

    def inverse(self, sample, profile):
        # Match vendor SDK's three-decimal input serialization. Full six-decimal
        # FK reply passthrough was rejected by this firmware; record rounded target.
        target = np.round(sample["pose"], 3)
        point = ','.join(f"{v:.3f}" for v in target) + ';' + ','.join(map(str, sample["arm"]))
        point += ';' + ','.join(["0.000"]*6)
        reply = self.query("Get_RobPToRobJ " + point + " " + profile)
        values = numbers(reply, 14)
        if np.any(values[6:] != 0):
            raise ValueError("Unexpected external axes in IK")
        return {"target_sent": target.tolist(), "joint_deg": values[:6].tolist(), "raw_reply": reply}
