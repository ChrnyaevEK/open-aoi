import bcrypt

from open_aoi.exceptions import AuthException


class Mixin:
    id: int
    hash: str

    @staticmethod
    def _hash_password(password: str) -> str:
        """Convert password to hash"""
        salt = bcrypt.gensalt()
        return bcrypt.hashpw(password.encode(), salt).decode()

    def set_password(self, password: str) -> None:
        """Override stored hash with new one"""
        self.hash = self._hash_password(password)

    def test_credentials(self, password: str) -> None:
        """Test password against stored hash"""

        try:
            assert bcrypt.checkpw(password.encode(), self.hash.encode())
        except AssertionError as e:
            raise AuthException("Credential test failed") from e

    def assert_access(self, storage: dict):
        """Test storage record for access granted in past"""
        try:
            assert storage["access_allowed"]
            assert storage["accessor"] == self.id
        except (AssertionError, KeyError) as e:
            raise AuthException("Access assertion failed") from e

    def grant_access(self, storage: dict):
        """Push flags and metadata to app store to reflect access has been granted to accessor in past"""
        storage["access_allowed"] = True
        storage["accessor"] = self.id

    @staticmethod
    def revoke_access(storage: dict):
        """Remove access flags and metadata from session"""
        del storage["access_allowed"]
        del storage["accessor"]
