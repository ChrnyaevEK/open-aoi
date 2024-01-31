# import roslibpy
# ros = roslibpy.Ros(host="localhost", port=9090)
# ros.run()

from nicegui import ui, __version__

from ros_docker_ws.core.settings import STORAGE_SECRET
from ros_docker_ws.web_app.views.view_access import view as view_access
from ros_docker_ws.web_app.views.view_home import view as view_home

print(__version__)
ui.page("/")(view_home)
ui.page("/access")(view_access)

ui.run(port=8000, storage_secret=STORAGE_SECRET)
