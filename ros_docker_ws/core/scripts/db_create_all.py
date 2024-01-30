"""Script create DB tables"""

from sqlalchemy import create_engine

from ros_docker_ws.core.models import *
from ros_docker_ws.core.settings import MYSQL_DATABASE, MYSQL_PASSWORD, MYSQL_USER

if __name__ == "__main__":
    engine = create_engine(
        f"mysql+pymysql://{MYSQL_USER}:{MYSQL_PASSWORD}@localhost/{MYSQL_DATABASE}"
    )
    metadata_obj.create_all(engine)