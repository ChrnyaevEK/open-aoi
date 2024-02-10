from nicegui import ui
from src.settings import STORAGE_SECRET

from src.services.interface_web.views.view_access import view as view_access
from src.services.interface_web.views.view_home import view as view_home

ui.page("/")(view_home)
ui.page("/access")(view_access)

ui.run(port=8000, storage_secret=STORAGE_SECRET)
