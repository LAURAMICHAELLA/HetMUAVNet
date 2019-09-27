import json
import http.server
import socketserver
import sys
import threading

_status = 'WAITING'
_uav_info = None
_network_info = None

class _Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/info':
            self._send_text(json.dumps({
                'status': _status,
                'uav_info': _uav_info,
                'network_info': _network_info
            }))

    def _send_text(self, text):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

        self.wfile.write(text.encode('utf-8'))

def start(port, uav_info, network_info):
    global _uav_info, _network_info
    _uav_info = uav_info
    _network_info = network_info

    socketserver.TCPServer.allow_reuse_address = True
    httpd = socketserver.TCPServer(("0.0.0.0", port), _Handler)
    print("StatusServer: Listening on {}/tcp".format(port), file=sys.stderr)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

def set_status_running():
    global _status
    _status = 'RUNNING'
