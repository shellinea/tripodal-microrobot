#!/usr/bin/env python3
"""
机器人模拟服务器（Mock Robot Server，测试用）

模拟机器人固件的 HTTP 接口行为：接收上位机指令并返回应答，
用于在没有真机的情况下联调上位机与通信协议。
"""
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import json
import time


class RobotHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        params = parse_qs(parsed.query)

        print(f"[{time.strftime('%H:%M:%S')}] {path} {params}")

        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps({"status": "ok"}).encode())

    def log_message(self, format, *args):
        pass


if __name__ == '__main__':
    PORT = 80
    server = HTTPServer(('0.0.0.0', PORT), RobotHandler)
    print(f"服务器启动，端口 {PORT}")
    print("按 Ctrl+C 停止")
    server.serve_forever()
