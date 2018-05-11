'''
Module with common cog generators used several times.
'''

import cog


def outl_using_console_bridge():
    for func in ('Debug', 'Inform', 'Warn', 'Error', 'Fatal', ):
        cog.outl('using log{func} = CONSOLE_BRIDGE_log{func};'.format(**locals()))
