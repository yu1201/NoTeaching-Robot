"""Observe the vendor SDK against a loopback-only fake controller; never a robot."""
import argparse
import ctypes
import hashlib
import os
from pathlib import Path
import socket
import threading


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk", required=True)
    args = parser.parse_args()
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    listener.settimeout(6)
    port = listener.getsockname()[1]
    accepted = threading.Event()
    finished = threading.Event()
    login_commands = []
    failures = []

    def serve():
        try:
            with listener.accept()[0] as client:
                accepted.set()
                client.settimeout(5)
                pending = b""
                while not finished.is_set():
                    part = client.recv(2048)
                    if not part:
                        break
                    pending += part
                    while b"$$" in pending:
                        frame, pending = pending.split(b"$$", 1)
                        command = frame.removeprefix(b"@@")
                        operation = command.split(b" ", 1)[0]
                        print("Mock received:", operation.decode("ascii", errors="replace"))
                        if operation == b"UserLogin":
                            login_commands.append(command.split(b" "))
                            reply = b"ok"
                        elif operation == b"Get_ConnectState":
                            reply = b"=1"
                        elif operation in (b"CurCtrlDev", b"CurUserType"):
                            reply = b"=2"
                        elif operation == b"CurPermit":
                            reply = b"=1 Ip:127.0.0.1 Port:0"
                        elif operation.startswith(b"Get_"):
                            reply = b"=0"
                        else:
                            reply = b"ok"
                        client.sendall(b"##" + reply + b"$$")
        except (OSError, RuntimeError) as error:
            if not finished.is_set():
                failures.append(type(error).__name__)

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    dependency_dir = Path(args.sdk).resolve().parent.parent / "KineExtApi" / "x64"
    dependency_handle = os.add_dll_directory(str(dependency_dir))
    dll = ctypes.WinDLL(args.sdk)
    dll.IMC100_Init_ETH.argtypes = [ctypes.c_uint, ctypes.c_ushort, ctypes.c_int, ctypes.c_int]
    dll.IMC100_Init_ETH.restype = ctypes.c_int
    dll.IMC100_UserLogin.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_char), ctypes.c_int]
    dll.IMC100_UserLogin.restype = ctypes.c_int
    dll.IMC100_Exit_ETH.argtypes = [ctypes.c_int]
    dll.IMC100_Exit_ETH.restype = ctypes.c_int
    try:
        # SDK manual's DWORD representation: 0xc0a81719 = 192.168.23.25.
        result = dll.IMC100_Init_ETH(0x7F000001, port, 2, 0)
        print("SDK loopback connect result:", result)
        if result != 0 or not accepted.wait(1):
            raise RuntimeError("SDK did not connect to the loopback fake; login not called")
        password = b"000000"
        result = dll.IMC100_UserLogin(2, ctypes.create_string_buffer(password, 8), 0)
        print("SDK loopback login result:", result)
        if result != 0:
            raise RuntimeError("SDK login failed against the loopback fake")
        if len(login_commands) != 1:
            raise RuntimeError("Expected exactly one SDK login command")
        fields = login_commands[0]
        if fields != [b"UserLogin", b"2", password]:
            raise RuntimeError("Vendor SDK login format differs from the brand implementation (credentials redacted)")
        print("Login parameter count:", len(fields) - 1)
        print("Login level:", fields[1].decode() if len(fields) > 1 else "missing")
        if len(fields) == 3:
            token = fields[2]
            print("Wire password length:", len(token))
            print("Wire equals plaintext:", token == password)
            print("Wire equals lowercase MD5:", token == hashlib.md5(password).hexdigest().encode())
            print("Wire equals uppercase MD5:", token == hashlib.md5(password).hexdigest().upper().encode())
            print("Wire equals uppercase ASCII hex:", token == password.hex().upper().encode())
        if failures:
            raise RuntimeError("Mock transport failed: " + ",".join(failures))
    finally:
        dll.IMC100_Exit_ETH(0)
        finished.set()
        listener.close()
        thread.join(1)
        dependency_handle.close()


if __name__ == "__main__":
    main()
