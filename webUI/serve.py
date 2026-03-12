#!/usr/bin/env python3
import http.server
import socketserver
import os

PORT = 8080
DIRECTORY = os.path.dirname(os.path.abspath(__file__))

class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print(f"👉 请在浏览器中访问: http://localhost:{PORT}")
    print(f"👉 若在局域网内，请访问: http://<你的Ubuntu服务器IP>:{PORT}")
    print("按 Ctrl+C 停止服务...")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n已停止服务。")
        pass
    httpd.server_close()
