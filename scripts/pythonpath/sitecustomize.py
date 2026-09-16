"""Prefer PyQt on macOS.

python_qt_binding only tries PySide there, and conda-forge PySide6 imports
QtPdf which is not shipped with qt6-main. PyQt6 is already in the env.
"""
import sys

if sys.platform == "darwin":
    sys.SELECT_QT_BINDING_ORDER = ["pyqt", "pyside"]
