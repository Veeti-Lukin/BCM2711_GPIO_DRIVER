from http.server import HTTPServer, BaseHTTPRequestHandler
import json


HTML_FILE = "frontend/index.html"
IP = "localhost"
PORT = 80


class HttpServer(BaseHTTPRequestHandler):
    """def __init__(self):
        super(HTTPServer, self).__init__()"""

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()

        self.wfile.write(self.html_)

    def do_POST(self):
        if self.path == '/api/control':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode('utf-8'))
                action = data.get('action')
                if action:
                    # Process the action here and control your RC car
                    # accordingly
                    # Example: execute_action(action)
                    print(action)
                    if action == "forward":
                        print("asd")


                    self.send_response(200)
                    self.end_headers()
                    self.wfile.write(
                        "Action executed successfully.".encode('utf-8'))
                else:
                    self.send_response(400)
                    self.end_headers()
                    self.wfile.write("Invalid request data.".encode('utf-8'))
            except json.JSONDecodeError as e:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(
                    f"Invalid JSON data: {str(e)}".encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write("Not Found".encode('utf-8'))
    html_ = bytes(open(HTML_FILE).read(), "UTF-8")


def main():
    server = HTTPServer((IP, PORT), HttpServer)
    server.serve_forever()
    server.server_close()


if __name__ == '__main__':
    main()
