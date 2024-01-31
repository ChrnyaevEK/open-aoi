from abc import ABC


class General(Exception, ABC):
    pass


class SecurityException(General, ABC):
    """Base exception for security issues"""

    pass


class AuthException(SecurityException):
    """Raised on auth attempt fail"""

    pass
