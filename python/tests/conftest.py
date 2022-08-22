import pytest
import pathlib


@pytest.fixture
def resource_dir():
    """Get the test resource path."""
    return pathlib.Path(__file__).resolve().parent / "resources"
