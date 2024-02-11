"""Script create DB tables"""


from open_aoi.models import *

if __name__ == "__main__":
    metadata_obj.create_all(engine)
