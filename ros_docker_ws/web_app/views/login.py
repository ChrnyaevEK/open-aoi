from nicegui import ui
from ros_docker_ws.web_app.views import colors


def view():
    ui.colors(**colors)
    with ui.card().classes("absolute-center"):
        with ui.row(wrap=False).classes('w-full justify-between items-center'):
            ui.markdown("**Login**")
            ui.button(icon='question_mark').props('flat round size=xs')
        ui.input(placeholder="Password", password=True).classes("w-full")
        ui.button("Continue as operator", color="primary").classes("w-full")
        ui.button("Continue as administrator", color="primary").classes("w-full")

    with ui.header(fixed=True).classes("py-1"):
        ui.markdown("**AOI** portal | Powered by ROS")
    with ui.footer(fixed=True).classes("py-2"):
        ui.label("Created by @Cherniaev.public")
