'''
Module with common cog generators used several times.

Install:

    ln -s $PWD/mavros_cog.py $HOME/.local/lib/python3.8/site-packages/
'''

try:
    import cog
except:
    _cog_present = False
else:
    _cog_present = True

import pathlib
import re
import typing
import xml.etree.ElementTree as ET
from xml.dom import minidom

import attr
from comment_parser import comment_parser

CPP_MIME = 'text/x-c++'
REGISTER_PLUGIN_RE = re.compile(
    r'MAVROS_PLUGIN_REGISTER\((?P<klass>[a-zA-Z_\:]+)\)')
PLUGIN_NAME_RE = re.compile(r'@plugin\ (?P<name>[a-z_]+)')
PLUGIN_BRIEF_RE = re.compile(r'^@brief\ (?P<brief>.+)$')

plugins_without_macro = {
    "mavros/src/plugins/mission_protocol_base.cpp"
}

if _cog_present:

    def dbg(s):
        cog.msg(s)
else:

    def dbg(s):
        print(s)


class NoPluginRegister(Exception):
    """
    That Exception returned if plugin registration not found
    """
    pass


@attr.s(auto_attribs=True)
class PluginInfo:
    """
    Info parsed from plugin source file
    """
    path: pathlib.Path
    klass: str
    name: str
    description: str
    is_example: bool

    @property
    def factory_klass(self) -> str:
        return f"mavros::plugin::PluginFactoryTemplate<{self.klass}>"

    @property
    def as_xml(self) -> ET.Element:
        ret = ET.Element('class',
                         name=self.name,
                         type=self.factory_klass,
                         base_class_type='mavros::plugin::PluginFactory')
        desc = ET.SubElement(ret, 'description')
        desc.text = self.description

        if not self.is_example:
            return ret
        else:
            return ET.Comment(et_to_str(ret))

    @property
    def sort_key(self):
        return (not self.is_example, self.name)

    @classmethod
    def parse_file(cls, path: pathlib.Path) -> 'PluginInfo':
        with path.open('r') as fd:
            source = fd.read()

        m = REGISTER_PLUGIN_RE.search(source)
        if m is None:
            raise NoPluginRegister(
                f"plugin registration macro not found in file: {path}")

        klass = m.group('klass')
        comments = comment_parser.extract_comments_from_str(source, CPP_MIME)

        # assume that only one plugin is allowed per file
        comment = next(c.text() for c in comments if '@plugin' in c.text())

        # clean comment from *
        def remove_prefix(line: str) -> str:
            line = line.strip()
            if line.startswith('*'):
                line = line[1:]
            return line.strip()

        comment = '\n'.join(
            remove_prefix(line) for line in comment.splitlines()).strip()

        m = PLUGIN_NAME_RE.search(comment)
        name = m.group('name')

        is_example = '@example_plugin' in comment

        # TODO(vooon): reformat comment for description

        return cls(
            path=path,
            klass=klass,
            name=name,
            description=comment,
            is_example=is_example,
        )


def load_all_plugin_infos(dir: pathlib.Path) -> typing.Iterator[PluginInfo]:
    for fl in dir.glob('*.cpp'):
        try:
            if str(fl) not in plugins_without_macro:
                yield PluginInfo.parse_file(fl)
        except NoPluginRegister as ex:
            dbg(f"skipping file: {ex}")


def et_to_str(root: ET.Element) -> str:
    # XXX(vooon): pretty print
    xml_ = ET.tostring(root)
    xml_ = minidom.parseString(xml_).toprettyxml(indent='  ')
    return '\n'.join(xml_.splitlines()[1:])  # remove <? header ?>


def cwd() -> pathlib.Path:
    if _cog_present:
        return pathlib.Path(cog.inFile).parent
    else:
        return pathlib.Path('.')


def outl(s: str):
    if _cog_present:
        cog.outl(s)
    else:
        print(s)


def outl_plugins_xml(dir: str, lib_path: str):
    plugins = sorted(load_all_plugin_infos(cwd() / dir),
                     key=lambda p: p.sort_key)

    root = ET.Element("library", path=lib_path)
    for pl in plugins:
        root.append(pl.as_xml)

    xml_ = et_to_str(root)
    outl(xml_)


def outl_glob_files(dir: str, glob: str = "*.cpp"):
    for f in sorted((cwd() / dir).glob(glob)):
        outl(str(f.relative_to(cwd())))
