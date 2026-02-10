"""Widgets package for Motor Recording Studio."""

from .control_panel import ControlPanel
from .joint_monitor import JointMonitor
from .record_tab import RecordTab
from .playback_tab import PlaybackTab
from .browse_tab import BrowseTab
from .monitor_tab import MonitorTab
from .easy_mode import EasyModeWidget

__all__ = [
    'ControlPanel',
    'JointMonitor',
    'RecordTab',
    'PlaybackTab',
    'BrowseTab',
    'MonitorTab',
    'EasyModeWidget',
]
