"""Script create DB tables"""


from ..models import *

if __name__ == "__main__":
    metadata_obj.create_all(engine)
