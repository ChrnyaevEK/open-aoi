from nicegui import ui
from open_aoi.settings import STORAGE_SECRET, WEB_INTERFACE_PORT

from open_aoi_web_interface.views.view_access import view as view_access
from open_aoi_web_interface.views.view_home import view as view_home
from open_aoi_web_interface.views.view_devices import view as view_devices

ui.page("/", title='Home | AOI Portal')(view_home)
ui.page("/access", title='Access | AOI Portal')(view_access)
ui.page("/devices", title='Devices | AOI Portal')(view_devices)

ui.run(port=WEB_INTERFACE_PORT, storage_secret=STORAGE_SECRET)
