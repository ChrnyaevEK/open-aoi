# import roslibpy
# ros = roslibpy.Ros(host="localhost", port=9090)
# ros.run()

from nicegui import ui

from ros_docker_ws.web_app.views.login import view as view_login

ui.page("/login")(view_login)
ui.run(port=8000)
