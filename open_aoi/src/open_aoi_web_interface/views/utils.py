import logging
from typing import Optional

from nicegui import ui
from sqlalchemy import select
from sqlalchemy.orm import Session

from open_aoi.enums import AccessorEnum
from open_aoi.models import engine, Accessor

logger = logging.getLogger("ui.utils")


def db_get_accessor(accessor_id: AccessorEnum) -> Optional[Accessor]:
    with Session(engine) as session:
        q = select(Accessor).where(Accessor.id == accessor_id.value)
        accessor = session.scalars(q).one_or_none()
        try:
            assert accessor is not None
        except AssertionError as e:
            logger.exception(e)
            ui.notify("Database inconsistency. Contact support.", type="warning")
            return
    return accessor
