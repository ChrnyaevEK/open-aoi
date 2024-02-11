import os
from dotenv import load_dotenv

load_dotenv()

MYSQL_DATABASE = os.environ["MYSQL_DATABASE"]
assert MYSQL_DATABASE

MYSQL_USER = os.environ["MYSQL_USER"]
assert MYSQL_USER

MYSQL_PASSWORD = os.environ["MYSQL_PASSWORD"]
assert MYSQL_PASSWORD

MYSQL_PORT = os.environ["MYSQL_PORT"]
assert MYSQL_PORT
try:
    MYSQL_PORT = int(MYSQL_PORT)
except ValueError:
    raise RuntimeError('Failed to parse mysql port from environment')


WEB_INTERFACE_PORT = os.environ["WEB_INTERFACE_PORT"]
assert WEB_INTERFACE_PORT
try:
    WEB_INTERFACE_PORT = int(WEB_INTERFACE_PORT)
except ValueError:
    raise RuntimeError('Failed to parse web port from environment')

ALLOW_SIMULATION_NODE = os.environ["SIMULATION"] == "1"

AOI_OPERATOR_INITIAL_PASSWORD = os.environ["AOI_OPERATOR_INITIAL_PASSWORD"]
assert AOI_OPERATOR_INITIAL_PASSWORD

AOI_ADMINISTRATOR_INITIAL_PASSWORD = os.environ["AOI_ADMINISTRATOR_INITIAL_PASSWORD"]
assert AOI_ADMINISTRATOR_INITIAL_PASSWORD

STORAGE_SECRET = os.environ["STORAGE_SECRET"]
assert STORAGE_SECRET and len(STORAGE_SECRET) > 10
