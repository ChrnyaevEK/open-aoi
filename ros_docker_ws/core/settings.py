import os
from dotenv import load_dotenv

load_dotenv()

MYSQL_DATABASE = os.environ["MYSQL_DATABASE"]

MYSQL_USER = os.environ["MYSQL_USER"]
MYSQL_PASSWORD = os.environ["MYSQL_PASSWORD"]

ALLOW_SIMULATION_NODE = os.environ["ALLOW_SIMULATION_NODE"] == "1"
SIMULATION_DATASET_PATH = os.environ["SIMULATION_DATASET_PATH"]
assert os.path.exists(SIMULATION_DATASET_PATH)
