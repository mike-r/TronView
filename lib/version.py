# Semantic versioning (major.minor.patch)
__version__ = "0.0.33"
__version_info__ = tuple(map(int, __version__.split('.')))

# Optional additional version info
__author__ = "TronView.org"
__license__ = "GPLv3"
__copyright__ = f"Copyright 2025 {__author__}"

# Build info
__build__ = "alpha"  # or "stable", "beta", etc.
__build_date__ = "2025-04-21"
__build_time__ = "14:26:27 PDT"

__build_version__ = f"{__version__} {__build__} {__build_date__} {__build_time__}"
