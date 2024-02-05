"""Script create DB tables"""


from ros_docker_ws.core.models import *

if __name__ == "__main__":
    metadata_obj.create_all(engine)
