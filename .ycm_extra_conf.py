# -*- coding: utf-8 -*-

##########################################################################
# YouCompleteMe configuration for ROS                                    #
# Author: Gaël Ecorchard (2015)                                          #
#                                                                        #
# The file requires the definition of the $ROS_WORKSPACE variable in     #
# your shell.                                                            #
# Name this file .ycm_extra_conf.py and place it in $ROS_WORKSPACE to    #
# use it.                                                                #
#                                                                        #
# Tested with Ubuntu 14.04 and Indigo.                                   #
#                                                                        #
# License: CC0                                                           #
##########################################################################

import os
import ycm_core


def GetRosIncludePaths():
    """Return a list of potential include directories

    The directories are looked for in $ROS_WORKSPACE.
    """
    try:
        from rospkg import RosPack
    except ImportError:
        return []
    rospack = RosPack()
    includes = []
    includes.append(os.path.expandvars('$ROS_WORKSPACE') + '/devel/include')
    for p in rospack.list():
        if os.path.exists(rospack.get_path(p) + '/include'):
            includes.append(rospack.get_path(p) + '/include')

    for distribution in os.listdir('/opt/ros'):
        includes.append('/opt/ros/' + distribution + '/include')

    return includes


def GetRosIncludeFlags():
    includes = GetRosIncludePaths()
    flags = []
    for include in includes:
        flags.append('-isystem')
        flags.append(include)

    return flags

# These are the compilation flags that will be used in case there's no
# compilation database set (by default, one is not set).
# CHANGE THIS LIST OF FLAGS. YES, THIS IS THE DROID YOU HAVE BEEN LOOKING FOR.
# You can get CMake to generate the compilation_commands.json file for you by
# adding:
#   set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# to your CMakeLists.txt file or by once entering
#   catkin config --cmake-args '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
# in your shell.

default_flags = [
    '-Wall',
    '-Wextra',
    '-Werror',
    # '-Wc++98-compat',
    # '-Wno-long-long',
    # '-Wno-variadic-macros',
    '-fexceptions',
    '-DNDEBUG',
    # THIS IS IMPORTANT! Without a "-std=<something>" flag, clang won't know
    # which language to use when compiling headers. So it will guess. Badly. So
    # C++ headers will be compiled as C headers. You don't want that so ALWAYS
    # specify a "-std=<something>".
    # For a C project, you would set this to something like 'c99' instead of
    # 'c++11'.
    # '-std=c++03',
    '-std=c++11',
    # ...and the same thing goes for the magic -x option which specifies the
    # language that the files to be compiled are written in. This is mostly
    # relevant for c++ headers.
    # For a C project, you would set this to 'c' instead of 'c++'.
    '-x',
    'c++',
    '-I',
    '.',

    # include third party libraries
    # '-isystem',
    # '/some/path/include',
    '-isystem',
    '/usr/include/eigen3',

    # mavros flags
    #'-DMAVLINK_DIALECT=ardupilotmega',
]

flags = default_flags + GetRosIncludeFlags()


def GetCompilationDatabaseFolder(filename):
    """Return the directory potentially containing compilation_commands.json

    Return the absolute path to the folder (NOT the file!) containing the
    compile_commands.json file to use that instead of 'flags'. See here for
    more details: http://clang.llvm.org/docs/JSONCompilationDatabase.html.
    The compilation_commands.json for the given file is returned by getting
    the package the file belongs to.
    """
    try:
        import rospkg
    except ImportError:
        return ''
    pkg_name = rospkg.get_package_name(filename)
    if not pkg_name:
        return ''
    dir = (os.path.expandvars('$ROS_WORKSPACE') +
           os.path.sep +
           'build' +
           os.path.sep +
           pkg_name)

    return dir


def GetDatabase(compilation_database_folder):
    if os.path.exists(compilation_database_folder):
        return ycm_core.CompilationDatabase(compilation_database_folder)
    return None

SOURCE_EXTENSIONS = ['.cpp', '.cxx', '.cc', '.c', '.m', '.mm']


def DirectoryOfThisScript():
    return os.path.dirname(os.path.abspath(__file__))


def MakeRelativePathsInFlagsAbsolute(flags, working_directory):
    if not working_directory:
        return list(flags)
    new_flags = []
    make_next_absolute = False
    path_flags = ['-isystem', '-I', '-iquote', '--sysroot=']
    for flag in flags:
        new_flag = flag

        if make_next_absolute:
            make_next_absolute = False
            if not flag.startswith('/'):
                new_flag = os.path.join(working_directory, flag)

        for path_flag in path_flags:
            if flag == path_flag:
                make_next_absolute = True
                break

            if flag.startswith(path_flag):
                path = flag[len(path_flag):]
                new_flag = path_flag + os.path.join(working_directory, path)
                break

        if new_flag:
            new_flags.append(new_flag)

    return new_flags


def IsHeaderFile(filename):
    extension = os.path.splitext(filename)[1]
    return extension in ['.h', '.hxx', '.hpp', '.hh']


def GetCompilationInfoForHeaderSameDir(headerfile, database):
    """Return compile flags for src file with same base in the same directory
    """
    filename_no_ext = os.path.splitext(headerfile)[0]
    for extension in SOURCE_EXTENSIONS:
        replacement_file = filename_no_ext + extension
        if os.path.exists(replacement_file):
            compilation_info = database.GetCompilationInfoForFile(
                replacement_file)
            if compilation_info.compiler_flags_:
                return compilation_info
    return None


def GetCompilationInfoForHeaderRos(headerfile, database):
    """Return the compile flags for the corresponding src file in ROS

    Return the compile flags for the source file corresponding to the header
    file in the ROS where the header file is.
    """
    try:
        import rospkg
    except ImportError:
        return None
    pkg_name = rospkg.get_package_name(headerfile)
    if not pkg_name:
        return None
    try:
        pkg_path = rospkg.RosPack().get_path(pkg_name)
    except rospkg.ResourceNotFound:
        return None
    filename_no_ext = os.path.splitext(headerfile)[0]
    hdr_basename_no_ext = os.path.basename(filename_no_ext)
    for path, dirs, files in os.walk(pkg_path):
        for src_filename in files:
            src_basename_no_ext = os.path.splitext(src_filename)[0]
            if hdr_basename_no_ext != src_basename_no_ext:
                continue
            for extension in SOURCE_EXTENSIONS:
                if src_filename.endswith(extension):
                    compilation_info = database.GetCompilationInfoForFile(
                        path + os.path.sep + src_filename)
                    if compilation_info.compiler_flags_:
                        return compilation_info
    return None


def GetCompilationInfoForFile(filename, database):
    # The compilation_commands.json file generated by CMake does not have
    # entries for header files. So we do our best by asking the db for flags
    # for a corresponding source file, if any. If one exists, the flags for
    # that file should be good enough.
    # Corresponding source file are looked for in the same package.
    if IsHeaderFile(filename):
        # Look in the same directory.
        compilation_info = GetCompilationInfoForHeaderSameDir(
            filename, database)
        if compilation_info:
            return compilation_info
        # Look in the package.
        compilation_info = GetCompilationInfoForHeaderRos(filename, database)
        if compilation_info:
            return compilation_info
    return database.GetCompilationInfoForFile(filename)


def FlagsForFile(filename):
    database = GetDatabase(GetCompilationDatabaseFolder(filename))
    if database:
        # Bear in mind that compilation_info.compiler_flags_ does NOT return a
        # python list, but a "list-like" StringVec object
        compilation_info = GetCompilationInfoForFile(filename, database)
        if not compilation_info:
            # Return the default flags defined above.
            return {
                'flags': flags,
                'do_cache': True,
            }

        final_flags = MakeRelativePathsInFlagsAbsolute(
            compilation_info.compiler_flags_,
            compilation_info.compiler_working_dir_)
        final_flags += default_flags
    else:
        relative_to = DirectoryOfThisScript()
        final_flags = MakeRelativePathsInFlagsAbsolute(flags, relative_to)

    return {
        'flags': final_flags,
        'do_cache': True
    }
