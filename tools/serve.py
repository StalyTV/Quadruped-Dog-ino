#!/usr/bin/env python3
"""Serve the browser tools over HTTPS, reachable from a phone on the same network.

    python3 tools/serve.py

Plain HTTP is enough on the machine itself, because browsers treat localhost as
a secure context whether or not it is encrypted. Nothing else gets that
exemption: open the same page at http://192.168.x.x and the camera is refused,
with getUserMedia simply absent. So reaching it from a phone means TLS, and TLS
on a local network means a self-signed certificate.

This generates one on first run, with the machine's current LAN address in the
subjectAltName, and serves this directory with it. The phone will warn that the
certificate is not trusted, which is true and expected: nothing signed it. Accept
it once and the origin counts as secure, which is all the camera needs.

Regenerate by deleting tools/.cert if the machine's address changes.
"""

import http.server
import pathlib
import socket
import ssl
import subprocess
import sys
import tempfile

PORT = 8443
HERE = pathlib.Path(__file__).resolve().parent
CERTDIR = HERE / ".cert"
CERT = CERTDIR / "dev.crt"
KEY = CERTDIR / "dev.key"


def lan_ip() -> str:
    """The address this machine uses to reach the outside world.

    Connecting a UDP socket sends nothing; it only asks the routing table which
    local interface would be used, which is the address a phone on the same
    network can reach.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def ensure_cert(ip: str) -> None:
    if CERT.exists() and KEY.exists():
        return
    CERTDIR.mkdir(exist_ok=True)

    # Written as a config file rather than passed with -addext, because the
    # openssl that ships with macOS is LibreSSL and has not always taken that
    # flag. A config file works on both.
    conf = f"""
[req]
distinguished_name = dn
x509_extensions    = v3
prompt             = no
[dn]
CN = {ip}
[v3]
subjectAltName   = IP:{ip}, DNS:localhost, IP:127.0.0.1
basicConstraints = critical, CA:FALSE
keyUsage         = digitalSignature, keyEncipherment
extendedKeyUsage = serverAuth
"""
    with tempfile.NamedTemporaryFile("w", suffix=".cnf", delete=False) as f:
        f.write(conf)
        path = f.name

    print(f"generating a self-signed certificate for {ip}")
    try:
        subprocess.run(
            ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-sha256",
             "-days", "825", "-nodes",
             "-keyout", str(KEY), "-out", str(CERT), "-config", path],
            check=True, capture_output=True,
        )
    except FileNotFoundError:
        sys.exit("openssl not found, so a certificate cannot be generated")
    except subprocess.CalledProcessError as e:
        sys.exit("openssl failed:\n" + e.stderr.decode(errors="replace"))


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=str(HERE), **kw)

    def log_message(self, fmt, *args):
        pass        # the default log is noise while a camera page polls


def main() -> None:
    ip = lan_ip()
    ensure_cert(ip)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(CERT, KEY)

    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)

    print()
    print(f"  on this machine   https://localhost:{PORT}/angle-meter/")
    print(f"  on your phone     https://{ip}:{PORT}/angle-meter/")
    print()
    print("  The phone will warn that the certificate is not trusted. It is not,")
    print("  and that is expected. Proceed anyway; the origin then counts as")
    print("  secure and the camera works.")
    print()
    print("  Reachable by anything on this network while it runs. Ctrl-C to stop.")
    print()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("stopped")


if __name__ == "__main__":
    main()
