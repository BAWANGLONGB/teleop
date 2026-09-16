import importlib
from pathlib import Path
import unittest

import xr_marvin_teleop


class TestPackageLayout(unittest.TestCase):
    def test_package_is_loaded_from_current_source_tree(self):
        source_root = Path(__file__).resolve().parents[1] / "src"
        package_file = Path(xr_marvin_teleop.__file__).resolve()

        self.assertTrue(package_file.is_relative_to(source_root.resolve()))

    def test_subpackages_are_regular_packages(self):
        for name in ("adapters", "collection", "control", "ros", "web"):
            module = importlib.import_module(f"xr_marvin_teleop.{name}")
            self.assertIsNotNone(module.__file__)


if __name__ == "__main__":
    unittest.main()
