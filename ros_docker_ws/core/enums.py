from enum import Enum


class DefectTypeEnum(Enum):
    """Supported defect types (reflected in DB)"""

    MISSING_COMPONENT = 0
    WRONG_COMPONENT_ORIENTATION = 1
    TYPOGRAFY = 2


class RoleEnum(Enum):
    """Supported role types (reflected in DB)"""

    OPERATOR = 0
    ADMINISTRATOR = 1
