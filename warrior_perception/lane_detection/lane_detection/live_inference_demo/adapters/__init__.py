"""Adapter package for live inference demo models."""

from .base_adapter import BaseAdapter
from .multitask_adapter import MultiTaskAdapter

__all__ = ["BaseAdapter", "MultiTaskAdapter"]
