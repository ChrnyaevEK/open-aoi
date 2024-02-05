from enum import Enum


class DefectTypeEnum(Enum):
    """Supported defect types (reflected in DB)"""

    MISSING_COMPONENT = 1
    WRONG_COMPONENT_ORIENTATION = 2
    TYPOGRAFY = 3


class RoleEnum(Enum):
    """Supported role types (reflected in DB)"""

    OPERATOR = 1
    ADMINISTRATOR = 2

class AccessorEnum(Enum):
    """Supported accessors (reflected in DB)"""

    OPERATOR = 1
    ADMINISTRATOR = 2
