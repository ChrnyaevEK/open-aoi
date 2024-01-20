"""
    This script is a web interface server.
"""

import roslibpy
from nicegui import ui

ros = roslibpy.Ros(host="localhost", port=9090)
ros.run()

ui.label("Hello NiceGUI!")
ui.run()

