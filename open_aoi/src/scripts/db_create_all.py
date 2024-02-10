"""Script create DB tables"""


from src.models import *

if __name__ == "__main__":
    metadata_obj.create_all(engine)
