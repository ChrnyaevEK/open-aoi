import bcrypt


class Mixin:
    @staticmethod
    def _hash_password(password: str) -> str:
        salt = bcrypt.gensalt()
        return bcrypt.hashpw(password.encode(), salt).decode()

    def set_password(self, password: str) -> None:
        raise NotImplemented()

    def assert_credentials(self, username: str, password: str) -> None:
        raise NotImplemented()
