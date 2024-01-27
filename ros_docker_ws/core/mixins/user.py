class Mixin:
    def set_username(self, username: str) -> None:
        raise NotImplemented()

    def set_password(self, password: str) -> None:
        raise NotImplemented()

    def assert_credentials(self, username: str, password: str) -> None:
        raise NotImplemented()
