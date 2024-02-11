from datetime import datetime
from typing import Optional, List

from sqlalchemy import (
    String,
    ForeignKey,
    Numeric,
    DateTime,
    func,
    Boolean,
    Integer,
    MetaData,
)
from sqlalchemy import create_engine
from sqlalchemy.orm import Mapped, mapped_column, DeclarativeBase, relationship

from open_aoi.settings import (
    MYSQL_DATABASE,
    MYSQL_PASSWORD,
    MYSQL_USER,
    MYSQL_PORT
)
from open_aoi.enums import DefectTypeEnum, RoleEnum, AccessorEnum
from open_aoi.mixins.control_zone import Mixin as ControlZoneMixin
from open_aoi.mixins.accessor import Mixin as AccessorMixin
from open_aoi.mixins.inspection_record import Mixin as InspectionRecordMixin
from open_aoi.mixins.template import Mixin as TemplateMixin
from open_aoi.mixins.camera import Mixin as CameraMixin
from open_aoi.mixins.inspection_profile import Mixin as InspectionProfileMixin


class Base(DeclarativeBase):
    pass


metadata_obj = MetaData()
engine = create_engine(
    f"mysql+pymysql://{MYSQL_USER}:{MYSQL_PASSWORD}@localhost:{MYSQL_PORT}/{MYSQL_DATABASE}"
)


class Role(Base):
    """Define accessor rights"""

    __tablename__ = "Role"
    metadata = metadata_obj

    id: Mapped[int] = mapped_column(
        primary_key=True, nullable=False, autoincrement=True
    )

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
    allow_accessor_operations: Mapped[bool] = mapped_column(
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

    accessor_list: Mapped[List["Accessor"]] = relationship(back_populates="role")
    registry = RoleEnum


class Accessor(Base, AccessorMixin):
    __tablename__ = "Accessor"
    metadata = metadata_obj

    id: Mapped[int] = mapped_column(
        primary_key=True, nullable=False, autoincrement=True
    )

    title: Mapped[str] = mapped_column(String(50), nullable=False)
    description: Mapped[str] = mapped_column(String(200), nullable=False)

    role_id: Mapped[int] = mapped_column(ForeignKey("Role.id"), nullable=False)
    role: Mapped["Role"] = relationship()

    hash: Mapped[str] = mapped_column(String(60), nullable=False)
    registry = AccessorEnum


class DefectType(Base):
    """Define system wide known error types"""

    __tablename__ = "DefectType"
    metadata = metadata_obj

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)

    title: Mapped[str] = mapped_column(String(50), nullable=False)
    description: Mapped[str] = mapped_column(String(200), nullable=False)

    registry = DefectTypeEnum


class ControlTarget(Base):
    """
    Helper object to map defect type to search for in test image to the control zone.
    Multiple control targets are allowed for single control zone (unique)
    """

    __tablename__ = "ControlTarget"
    metadata = metadata_obj

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)

    defect_type_id: Mapped[int] = mapped_column(
        ForeignKey("DefectType.id"), nullable=False
    )
    defect_type: Mapped["DefectType"] = relationship()

    control_zone_id: Mapped[int] = mapped_column(
        ForeignKey("ControlZone.id"), nullable=False
    )
    control_zone: Mapped["ControlZone"] = relationship(
        back_populates="control_target_list"
    )


class ControlZone(Base, ControlZoneMixin):
    """
    Small zone on template where related defect type detection is conducted
    """

    __tablename__ = "ControlZone"
    metadata = metadata_obj

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)

    template_id: Mapped[int] = mapped_column(ForeignKey("Template.id"), nullable=False)
    template: Mapped["Template"] = relationship(back_populates="control_zone_list")

    top_left_x: Mapped[float] = mapped_column(Numeric(precision=10, scale=2))
    top_left_y: Mapped[float] = mapped_column(Numeric(precision=10, scale=2))

    bottom_right_x: Mapped[float] = mapped_column(Numeric(precision=10, scale=2))
    bottom_right_y: Mapped[float] = mapped_column(Numeric(precision=10, scale=2))

    control_target_list: Mapped[List["ControlTarget"]] = relationship(
        back_populates="control_zone", cascade="all, delete"
    )

    created_at: Mapped[datetime] = mapped_column(DateTime, server_default=func.now())
    created_by_accessor_id: Mapped[int] = mapped_column(
        ForeignKey("Accessor.id"), nullable=False
    )
    created_by: Mapped["Accessor"] = relationship()


class Defect(Base):
    """
    Helper to map defect type that was found to control zone. Multiple defects are allowed.
    """

    __tablename__ = "Defect"
    metadata = metadata_obj

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)

    defect_type_id: Mapped[int] = mapped_column(
        ForeignKey("DefectType.id"), nullable=False
    )
    defect_type: Mapped["DefectType"] = relationship()

    inspection_record_id: Mapped[int] = mapped_column(
        ForeignKey("InspectionRecord.id"), nullable=False
    )
    inspection_record: Mapped["InspectionRecord"] = relationship(
        back_populates="defect_list"
    )


class InspectionRecord(Base, InspectionRecordMixin):
    """
    Connect defect collection with template
    """

    __tablename__ = "InspectionRecord"
    metadata = metadata_obj

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)

    template_id: Mapped[int] = mapped_column(ForeignKey("Template.id"), nullable=False)
    template: Mapped["Template"] = relationship(back_populates="inspection_record_list")

    defect_list: Mapped[List["Defect"]] = relationship(
        back_populates="inspection_record", cascade="all, delete"
    )

    created_at: Mapped[datetime] = mapped_column(DateTime, server_default=func.now())


class Template(Base, TemplateMixin):
    """
    Main reference image. Aggregate control zones.
    """

    __tablename__ = "Template"
    metadata = metadata_obj

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    is_active: Mapped[bool] = mapped_column(Boolean(), default=False, nullable=False)

    image_uid: Mapped[str] = mapped_column(String(100), nullable=False)

    control_zone_list: Mapped[List["ControlZone"]] = relationship(
        back_populates="template", cascade="all, delete"
    )

    inspection_record_list: Mapped[List["InspectionRecord"]] = relationship(
        back_populates="template", cascade="all, delete"
    )

    inspection_profile_id: Mapped[int] = mapped_column(
        ForeignKey("InspectionProfile.id"), nullable=False
    )
    inspection_profile: Mapped["InspectionProfile"] = relationship(
        back_populates="template_list"
    )

    created_at: Mapped[datetime] = mapped_column(DateTime, server_default=func.now())
    created_by_accessor_id: Mapped[int] = mapped_column(
        ForeignKey("Accessor.id"), nullable=False
    )
    created_by: Mapped["Accessor"] = relationship()


class Camera(Base, CameraMixin):
    """
    Represent available cameras
    """

    __tablename__ = "Camera"
    metadata = metadata_obj

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)

    title: Mapped[str] = mapped_column(String(200), nullable=False)
    description: Mapped[str] = mapped_column(String(500), nullable=False)

    # TODO: insert validation ipv4
    ip_address: Mapped[str] = mapped_column(String(15), nullable=False)
    port: Mapped[str] = mapped_column(Integer, nullable=False)

    created_at: Mapped[datetime] = mapped_column(DateTime, server_default=func.now())
    created_by_accessor_id: Mapped[int] = mapped_column(
        ForeignKey("Accessor.id"), nullable=False
    )
    created_by: Mapped["Accessor"] = relationship()


class InspectionProfile(Base, InspectionProfileMixin):
    """
    Concrete instance of desired test configuration
    """

    __tablename__ = "InspectionProfile"
    metadata = metadata_obj

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)

    title: Mapped[str] = mapped_column(String(200), nullable=False)
    description: Mapped[str] = mapped_column(String(500), nullable=False)

    identification_code: Mapped[str] = mapped_column(String(100), nullable=False)

    # Point to all available templates
    template_list: Mapped[List["Template"]] = relationship(
        back_populates="inspection_profile", cascade="all, delete"
    )

    camera_id: Mapped[Optional[int]] = mapped_column(
        ForeignKey("Camera.id"), nullable=True
    )
    camera: Mapped[Optional["Camera"]] = relationship()

    created_at: Mapped[datetime] = mapped_column(DateTime, server_default=func.now())
    created_by_accessor_id: Mapped[int] = mapped_column(
        ForeignKey("Accessor.id"), nullable=False
    )
    created_by: Mapped["Accessor"] = relationship()