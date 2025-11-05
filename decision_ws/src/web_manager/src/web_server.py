#!/usr/bin/env python3
import http.server
import socketserver
import os
from rclpy.node import Node
import rclpy

class WebServer(Node):
    def __init__(self):
        super().__init__('web_server')
        port = 8080

        # Trouver le chemin vers le dossier du package installé
        from ament_index_python.packages import get_package_share_directory

        package_share = get_package_share_directory('web_manager')
        web_dir = os.path.join(package_share, 'web')
        os.chdir(web_dir)

        handler = http.server.SimpleHTTPRequestHandler
        self.httpd = socketserver.TCPServer(("", port), handler)

        self.get_logger().info(f"🌐 Serveur web lancé sur http://localhost:{port}/web-coflot.html")
        self.get_logger().info(f"📁 Dossier servi : {web_dir}")

    def spin_forever(self):
        try:
            self.httpd.serve_forever()
        except KeyboardInterrupt:
            self.get_logger().info("🛑 Arrêt du serveur web")
        finally:
            self.httpd.server_close()


def main(args=None):
    rclpy.init(args=args)
    node = WebServer()
    try:
        node.spin_forever()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
