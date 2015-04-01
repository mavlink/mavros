# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

__all__ = (
    'get_namespace',
    'set_namespace',
    'get_topic'
)

# global namespace storage
_mavros_ns = "/mavros"


def get_namespace():
    """
    Returns mavros node namespace
    """
    global _mavros_ns
    return _mavros_ns


def set_namespace(ns):
    """
    Sets namespace of mavros node
    """
    global _mavros_ns
    _mavros_ns = ns


def get_topic(*args):
    """
    Create topic name for mavros node
    """
    return '/'.join((get_namespace(), ) + args)
