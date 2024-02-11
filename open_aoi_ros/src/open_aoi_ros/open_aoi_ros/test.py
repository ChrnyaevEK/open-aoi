from open_aoi.models import Role
from sqlalchemy.orm import Session
from sqlalchemy import select

from open_aoi.models import Role, engine


def main():
    print('Test start')
    with Session(engine) as session:
        q = select(Role)
        roles = session.execute(q)
        print(list(roles))


if __name__ == '__main__':
    main()
