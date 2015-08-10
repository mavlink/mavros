# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

__all__ = (
    'get_namespace',
    'set_namespace',
    'get_topic'
)

# global namespace storage
_mavros_ns = "/mavros"
_mavros_ns_update = []


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
    global _mavros_ns, _mavros_ns_update
    _mavros_ns = ns

    for cb in _mavros_ns_update:
        if callable(cb):
            cb()


def register_on_namespace_update(cb):
    """
    Call callback after namespace update
    """
    global _mavros_ns_update
    _mavros_ns_update.append(cb)


def get_topic(*args):
    """
    Create topic name for mavros node
    """
    return '/'.join((get_namespace(), ) + args)
