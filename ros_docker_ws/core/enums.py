from enum import Enum


class DefectTypeEnum(Enum):
    """Supported defect types (reflected in DB)"""

    MISSING_COMPONENT = 1
    WRONG_COMPONENT_ORIENTATION = 2
    TYPOGRAFY = 3
