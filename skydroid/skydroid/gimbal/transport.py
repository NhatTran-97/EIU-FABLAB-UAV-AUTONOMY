import socket

class UdpTransport:
    def __init__(self, local_addr, remote_addr, timeout=1.0):
        self.remote = remote_addr
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(local_addr)
        self.sock.settimeout(timeout)

    def send(self, packet: bytes):
        self.sock.sendto(packet, self.remote)

    def recv(self, timeout: float | None = None) -> str | None:
        previous_timeout = self.sock.gettimeout()
        if timeout is not None:
            self.sock.settimeout(timeout)
        try:
            data, _ = self.sock.recvfrom(1024)
            return data.decode("ascii", errors="ignore")
        except socket.timeout:
            return None
        finally:
            if timeout is not None:
                self.sock.settimeout(previous_timeout)

    def close(self):
        self.sock.close()

    def __enter__(self): return self
    def __exit__(self, *_): self.close()
