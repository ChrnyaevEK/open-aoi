from datetime import datetime
from typing import Optional

from sqlalchemy import String, ForeignKey, Numeric, DateTime, func, Boolean
from sqlalchemy.orm import Mapped, mapped_column, DeclarativeBase, relationship

from core.enums import DefectTypeEnum, RoleEnum
from core.mixins.control_zone import Mixin as ControlZoneMixin
from core.mixins.user import Mixin as UserMixin


class Role(DeclarativeBase):
    """Define user rights"""

    id: Mapped[int] = mapped_column(
        primary_key=True, nullable=False, autoincrement=True
    )
    title: Mapped[str] = mapped_column(String(50), nullable=False)
    description: Mapped[str] = mapped_column(String(200), nullable=False)

    allow_system_view: Mapped[bool] = mapped_column(
        Boolean(), default=False, nullable=False
    )
    allow_inspection_log_view: Mapped[bool] = mapped_column(
        Boolean(), default=False, nullable=False
    )
    allow_inspection_details_view: Mapped[bool] = mapped_column(
        Boolean(), default=False, nullable=False
    )
    allow_inspection_flow_control: Mapped[bool] = mapped_column(
        Boolean(), default=False, nullable=False
    )
    allow_user_operations: Mapped[bool] = mapped_column(
        Boolean(), default=False, nullable=False
    )
    allow_device_operations: Mapped[bool] = mapped_column(
        Boolean(), default=False, nullable=False
    )
    allow_inspection_profile_operations: Mapped[bool] = mapped_column(
        Boolean(), default=False, nullable=False
    )
    allow_system_operations: Mapped[bool] = mapped_column(
        Boolean(), default=False, nullable=False
    )
    allow_statistics_view: Mapped[bool] = mapped_column(
        Boolean(), default=False, nullable=False
    )

    user_list: Mapped[list["User"]] = relationship(back_populates="role")
    registry = RoleEnum


class User(DeclarativeBase, UserMixin):
    id: Mapped[int] = mapped_column(
        primary_key=True, nullable=False, autoincrement=True
    )
    external_id: Mapped[Optional[str]] = mapped_column(
        String(100), nullable=True, default=None
    )

    name: Mapped[str] = mapped_column(String(100), nullable=False)
    surname: Mapped[Optional[str]] = mapped_column(
        String(100), nullable=True, default=None
    )
    # ! Important, nullable = False
    role_id: Mapped[int] = mapped_column(ForeignKey("Role.id"), nullable=False)
    role: Mapped["Role"] = relationship()

    username: Mapped[str] = mapped_column(String(100), nullable=False)
    password_bcrypt: Mapped[str] = mapped_column(String(60), nullable=False)

    created_at: Mapped[datetime] = mapped_column(DateTime, server_default=func.now())


class DefectType(DeclarativeBase):
    """Define system wide known error types"""

    __tablename__ = "DefectType"

    id: Mapped[int] = mapped_column(primary_key=True)
    title: Mapped[str] = mapped_column(String(50))
    description: Mapped[str] = mapped_column(String(200))

    registry = DefectTypeEnum


class ControlTarget(DeclarativeBase):
    """
    Helper object to map defect type to search for in test image against template to the control zone.
    Multiple control targets are allowed for single control zone (unique)
    """

    __tablename__ = "ControlTarget"

    id: Mapped[int] = mapped_column(primary_key=True)

    defect_type_id: Mapped[int] = mapped_column(ForeignKey("DefectType.id"))
    defect_type: Mapped["DefectType"] = relationship()

    control_zone_id: Mapped[int] = mapped_column(ForeignKey("ControlZone.id"))
    control_zone: Mapped["ControlZone"] = relationship(
        back_populates="control_target_list"
    )


class ControlZone(DeclarativeBase, ControlZoneMixin):
    """
    Small zone on template where related defect type detection is conducted
    """

    __tablename__ = "ControlZone"

    id: Mapped[int] = mapped_column(primary_key=True)

    top_left_x: Mapped[float] = mapped_column(Numeric(precision=10, scale=2))
    top_left_y: Mapped[float] = mapped_column(Numeric(precision=10, scale=2))

    bottom_right_x: Mapped[float] = mapped_column(Numeric(precision=10, scale=2))
    bottom_right_y: Mapped[float] = mapped_column(Numeric(precision=10, scale=2))

    control_target_list: Mapped[list["ControlTarget"]] = relationship(
        back_populates="control_zone", cascade="all, delete"
    )

    created_at: Mapped[datetime] = mapped_column(DateTime, server_default=func.now())

    created_by_user_id: Mapped[int] = mapped_column(ForeignKey("User.id"))
    created_by: Mapped["User"] = relationship()
