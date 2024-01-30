"""Scripts create system wide known records in DB"""

from sqlalchemy import create_engine
from sqlalchemy.orm import Session

from ros_docker_ws.core.models import *
from ros_docker_ws.core.enums import *
from ros_docker_ws.core.settings import (
    MYSQL_DATABASE,
    MYSQL_PASSWORD,
    MYSQL_USER,
    AOI_ADMINISTRATOR_INITIAL_PASSWORD,
    AOI_OPERATOR_INITIAL_PASSWORD,
)

if __name__ == "__main__":
    engine = create_engine(
        f"mysql+pymysql://{MYSQL_USER}:{MYSQL_PASSWORD}@localhost/{MYSQL_DATABASE}"
    )

    with Session(engine) as session:
        # Defect types
        dt_missing_component = DefectType(
            id=DefectTypeEnum.MISSING_COMPONENT.value,
            title="Missing component",
            description="Component is present on template, but is missing on tested image.",
        )

        dt_wrong_component_orientation = DefectType(
            id=DefectTypeEnum.WRONG_COMPONENT_ORIENTATION.value,
            title="Wrong component orientation",
            description="Component orientation is different against template.",
        )

        dt_typography = DefectType(
            id=DefectTypeEnum.TYPOGRAFY.value,
            title="Typografy",
            description="Typografy quality issues.",
        )

        # Roles
        r_operator = Role(
            id=RoleEnum.OPERATOR.value,
            allow_system_view=True,
            allow_inspection_log_view=True,
            allow_inspection_details_view=True,
            allow_inspection_flow_control=True,
            allow_accessor_operations=False,
            allow_device_operations=False,
            allow_inspection_profile_operations=False,
            allow_system_operations=False,
            allow_statistics_view=False,
        )

        r_administrator = Role(
            id=RoleEnum.ADMINISTRATOR.value,
            allow_system_view=True,
            allow_inspection_log_view=True,
            allow_inspection_details_view=True,
            allow_inspection_flow_control=True,
            allow_accessor_operations=True,
            allow_device_operations=True,
            allow_inspection_profile_operations=True,
            allow_system_operations=True,
            allow_statistics_view=True,
        )

        # Accessors
        a_operator = Accessor(
            id=AccessorEnum.OPERATOR.value,
            title="Operator",
            description="Operator is capable of basic sytem control including inspection requests.",
            role_id=RoleEnum.OPERATOR.value,
            hash=Accessor._hash_password(AOI_OPERATOR_INITIAL_PASSWORD),
        )
        a_administrator = Accessor(
            id=AccessorEnum.ADMINISTRATOR.value,
            title="Administrator",
            description="Administrator is granted full access to system including security section and inspection configuration.",
            role_id=RoleEnum.ADMINISTRATOR.value,
            hash=Accessor._hash_password(AOI_ADMINISTRATOR_INITIAL_PASSWORD),
        )

        session.add_all(
            [
                dt_missing_component,
                dt_wrong_component_orientation,
                dt_typography,
                r_operator,
                r_administrator,
                a_operator,
                a_administrator,
            ]
        )
        session.commit()
