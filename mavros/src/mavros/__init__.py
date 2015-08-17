# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

__all__ = (
    'get_namespace',
    'set_namespace',
    'get_topic',
    'DEFAULT_NAMESPACE'
)

DEFAULT_NAMESPACE = '/mavros'

# global namespace storage
_mavros_ns = None
_mavros_ns_update = []


def get_namespace():
    """
    Returns mavros node namespace
    """
    global _mavros_ns
    if _mavros_ns is None:
        raise RuntimeError("mavros namespace is uninitialized! "
                           "Call mavros.set_namespace() first!")
    return _mavros_ns


def set_namespace(ns=DEFAULT_NAMESPACE):
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
