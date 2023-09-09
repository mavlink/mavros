"""
Module with common cog generators used several times.

Install:

    ln -s $PWD/mavros_cog.py $HOME/.local/lib/python3.8/site-packages/
"""

try:
    import cog
except Exception:
    _cog_present = False
else:
    _cog_present = True

import pathlib
import re
import typing
import xml.etree.ElementTree as ET  # nosemgrep
from collections import OrderedDict
from xml.dom import minidom  # nosemgrep

import attr
from comment_parser import comment_parser
from pymavlink.dialects.v20 import common

CPP_MIME = "text/x-c++"
REGISTER_PLUGIN_RE = re.compile(r"MAVROS_PLUGIN_REGISTER\((?P<klass>[a-zA-Z0-9_\:]+)\)")
PLUGIN_NAME_RE = re.compile(r"@plugin\ (?P<name>[a-z0-9_]+)")
PLUGIN_BRIEF_RE = re.compile(r"^@brief\ (?P<brief>.+)$")

plugins_without_macro = {"mavros/src/plugins/mission_protocol_base.cpp"}

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
        ret = ET.Element(
            "class",
            name=self.name,
            type=self.factory_klass,
            base_class_type="mavros::plugin::PluginFactory",
        )
        desc = ET.SubElement(ret, "description")
        desc.text = self.description

        if not self.is_example:
            return ret
        else:
            return ET.Comment(et_to_str(ret))

    @property
    def sort_key(self):
        return (not self.is_example, self.name)

    @classmethod
    def parse_file(cls, path: pathlib.Path) -> "PluginInfo":
        with path.open("r") as fd:
            source = fd.read()

        m = REGISTER_PLUGIN_RE.search(source)
        if m is None:
            raise NoPluginRegister(
                f"plugin registration macro not found in file: {path}"
            )

        klass = m.group("klass")
        comments = comment_parser.extract_comments_from_str(source, CPP_MIME)

        # assume that only one plugin is allowed per file
        comment = next(c.text() for c in comments if "@plugin" in c.text())

        # clean comment from *
        def remove_prefix(line: str) -> str:
            line = line.strip()
            if line.startswith("*"):
                line = line[1:]
            return line.strip()

        comment = "\n".join(
            remove_prefix(line) for line in comment.splitlines()
        ).strip()

        m = PLUGIN_NAME_RE.search(comment)
        name = m.group("name")

        is_example = "@example_plugin" in comment

        # TODO(vooon): reformat comment for description

        return cls(
            path=path,
            klass=klass,
            name=name,
            description=comment,
            is_example=is_example,
        )


def load_all_plugin_infos(dir: pathlib.Path) -> typing.Iterator[PluginInfo]:
    for fl in dir.glob("*.cpp"):
        try:
            if str(fl) not in plugins_without_macro:
                yield PluginInfo.parse_file(fl)
        except NoPluginRegister as ex:
            dbg(f"skipping file: {ex}")
        except Exception as ex:
            dbg(f"failed to load file {fl}: {ex}")


def et_to_str(root: ET.Element) -> str:
    # XXX(vooon): pretty print
    xml_ = ET.tostring(root)
    xml_ = minidom.parseString(xml_).toprettyxml(indent="  ")
    return "\n".join(xml_.splitlines()[1:])  # remove <? header ?>


def cwd() -> pathlib.Path:
    if _cog_present:
        return pathlib.Path(cog.inFile).parent
    else:
        return pathlib.Path(".")


def outl(s: str):
    if _cog_present:
        cog.outl(s)
    else:
        print(s)


def outl_plugins_xml(dir: str, lib_path: str):
    plugins = sorted(load_all_plugin_infos(cwd() / dir), key=lambda p: p.sort_key)

    root = ET.Element("library", path=lib_path)
    for pl in plugins:
        root.append(pl.as_xml)

    xml_ = et_to_str(root)
    outl(xml_)


def outl_glob_files(dir: str, glob: str = "*.cpp"):
    for f in sorted((cwd() / dir).glob(glob)):
        outl(str(f.relative_to(cwd())))


def clear_desc(s):
    return " ".join(v.strip() for v in s.splitlines() if v.strip())


def idl_decl_enum_mav_cmd(ename="MAV_CMD"):
    def wr_enum(enum, ename, pfx="", bsz=16):
        cog.outl("# " + ename + "_" + pfx)
        for k, e in enum:
            # exclude also deprecated commands
            if "MAV_CMD" + "_" + pfx in e.name and not re.search(
                "deprecated", e.description, re.IGNORECASE
            ):
                sn = e.name[len("MAV_CMD") + 1 :]
                l = "uint{bsz} {sn} = {k}".format(**locals())
                if e.description:
                    l += " " * (50 - len(l)) + " # " + clear_desc(e.description)
                cog.outl(l)
        cog.out("\n")

    enum = sorted(common.enums[ename].items())
    enum.pop()  # remove ENUM_END

    enumt = []
    # exception list of commands to not include
    exlist = ["SPATIAL", "USER", "WAYPOINT"]
    for k, e in enum:
        enumt.extend(e.name[len(ename) + 1 :].split("_")[0:1])

    enumt = sorted(set(enumt))
    enumt = [word for word in enumt if word not in exlist]

    for key in enumt:
        wr_enum(enum, ename, key)


def idl_decl_enum(ename, pfx="", bsz=8):
    enum = sorted(common.enums[ename].items())
    enum.pop()  # remove ENUM_END

    cog.outl("# " + ename)
    for k, e in enum:
        sn = e.name[len(ename) + 1 :]
        l = "uint{bsz} {pfx}{sn} = {k}".format(**locals())
        if e.description:
            l += " " * (40 - len(l)) + " # " + clear_desc(e.description)
        cog.outl(l)
