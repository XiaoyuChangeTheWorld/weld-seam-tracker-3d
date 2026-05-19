import http.server
import socketserver
import webbrowser
import os
import sys

PORT = 8080
DIRECTORY = os.path.dirname(os.path.abspath(__file__))

class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)
        
    def log_message(self, format, *args):
        # 保持控制台清洁，静默请求日志
        pass

def main():
    print("==================================================")
    print(f" 3D 焊缝可视化本地服务器正在启动...")
    print(f" 托管目录: {DIRECTORY}")
    print(f" 本地网址: http://localhost:{PORT}/index.html")
    print("==================================================")
    
    # 自动在默认浏览器中打开页面
    try:
        webbrowser.open(f"http://localhost:{PORT}/index.html")
    except Exception as e:
        print(f" 提示: 自动打开浏览器失败，请手动在浏览器输入 http://localhost:{PORT}/index.html")
        
    print(" 按 Ctrl + C 组合键可停止服务器。")
    
    # 启用端口复用，防止频繁启动报错
    socketserver.TCPServer.allow_reuse_address = True
    try:
        with socketserver.TCPServer(("", PORT), QuietHandler) as httpd:
            httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n 服务器已停止。")
        sys.exit(0)
    except Exception as e:
        print(f"\n 启动服务器失败: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
