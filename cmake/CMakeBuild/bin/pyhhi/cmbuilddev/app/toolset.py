from __future__ import print_function

import argparse
import sys
import shutil
import os

import pyhhi.build.common.system as system
import pyhhi.build.common.ver as ver
import pyhhi.build.common.util as util
import pyhhi.build.common.bldtools as bldtools


class ToolsetInfoApp(object):

    def __init__(self):
        self._sys_info = system.SystemInfo()

    def __call__(self):
        def lowercase_arg(string): return string.lower()

        _usage = """%(prog)s [options] [toolset]

This script lists the toolset properties in a single line separated by a ';'. The description
can be easily parsed by Boost.Build jam code to simplify toolset setup.

Examples: %(prog)s gcc
          %(prog)s msvc-12.0
          %(prog)s gcc-4.9
          %(prog)s --target-os=iphone clang
          %(prog)s --target-os=iphone --target-arch=armv7 clang
          %(prog)s --target-os=iphonesimulator clang
          %(prog)s arm-linux-androideabi-g++
          %(prog)s --api-level=19 arm-linux-androideabi-g++
          %(prog)s aarch64-linux-android-g++
          %(prog)s i686-linux-android-g++
          %(prog)s x86_64-linux-android-g++
"""

        parser = argparse.ArgumentParser(usage=_usage)
        parser.add_argument("toolset", action="store", nargs='?',
                            help="optional argument to override the default toolset.")
        util.app_args_add_log_level(parser)
        parser.add_argument("-v", action="store_true", dest="verbose", default=False,
                            help="enable verbose output suitable for humans. The default format is intended for scripting frameworks like Boost.Build/b2.")
        parser.add_argument("--internal-version", action="store_true", dest="internal_version", default=False,
                            help="list toolset's internal version, supported by msvc-x.y.")
        parser.add_argument("--api-level", action="store", dest="api_level", type=int,
                            help="override the default Android API level, the default is either the latest or fixed when the toolchain has been created.")
        parser.add_argument("--target-os", action="store", dest="target_os", type=lowercase_arg,
                            help="add a target OS filter, by default all toolset information is emitted.")
        parser.add_argument("--target-arch", action="store", dest="target_arch", type=lowercase_arg,
                            help="add a target ARCH filter, by default the complete toolset information is emitted.")
        parser.add_argument("--short-path-name", action="store_true", dest="short_path_name", default=False,
                            help="translate the compiler path into a short path on windows.")

        args = parser.parse_args()

        # configure the python logger
        util.app_configure_logging(args.log_level)

        if args.toolset:
            ts = bldtools.Toolset(self._sys_info, args.toolset)
        else:
            ts = bldtools.Toolset(self._sys_info)
        if args.api_level:
            # Override the default Android API level
            for platform_info in ts.get_platform_info():
                platform_info.set_api_level(args.api_level)
        if args.verbose:
            print(ts, "\n")
            #print("Summary: " + ts.get_toolset_info_short())
        elif args.internal_version:
            if ts.get_internal_version():
                print(ver.version_tuple_to_str(ts.get_internal_version()))
        else:
            if self._sys_info.is_windows() and args.short_path_name:
                compiler_command = self._sys_info.get_short_path(ts.get_compiler_command())
            else:
                compiler_command = ts.get_compiler_command()
            if ts.get_toolset() == 'msvc':
                tvl = list(ts.get_version())
                tvl.extend(ts.get_internal_version())
                toolset_version = tuple(tvl)
            else:
                toolset_version = ts.get_version()
            toolset_info_list = [ts.get_toolset(), ver.version_tuple_to_str(toolset_version), compiler_command]
            compiler_prefix = ts.get_compiler_prefix()
            if compiler_prefix is None:
                toolset_info_list.append('none')
            else:
                toolset_info_list.append(compiler_prefix)

            target_os_arch_list = []
            target_os_rtl_list = []
            for platform_info in ts.get_platform_info():
                target_os = platform_info.get_target_os()
                if args.target_os and (target_os != args.target_os):
                    # filter out platforms by target_os; e.g. return just the iphone toolset information.
                    continue
                isysroot = platform_info.get_isysroot()
                for target_arch in platform_info.get_target_arch():
                    if args.target_arch and (target_arch != args.target_arch):
                        # filter out target architectures if provided.
                        continue
                    if isysroot:
                        assert self._sys_info.is_macosx()
                        target_os_arch_list.append(target_os + '%' + target_arch + '%' + isysroot)
                    else:
                        if target_os == 'android':
                            target_os_arch_list.append('%s-%d%%%s' % (target_os, platform_info.get_api_level(), target_arch))
                        else:
                            target_os_arch_list.append(target_os + '%' + target_arch)
                    if (target_os == 'windows') and (ts.get_toolset() == 'gcc'):
                        # mingw: add runtime library information if available.
                        runtime_libs = platform_info.get_target_runtime_libs(target_arch)
                        if runtime_libs:
                            target_os_rtl_list.append(target_arch + '%' + '@'.join(runtime_libs))
            toolset_info_list.append('!'.join(target_os_arch_list))
            if target_os_rtl_list:
                toolset_info_list.append('!'.join(target_os_rtl_list))
            else:
                toolset_info_list.append('none')
            toolset_info_list.append(ts.get_compiler_tag())
            toolset_info_list.append(ts.get_boost_compiler_tag())
            print(";".join(toolset_info_list))


class MakeIosUniveralApp(object):

    def __call__(self):
        def lowercase_arg(string):
            return string.lower()

        parser = argparse.ArgumentParser()
        parser.add_argument("--rebuild", action="store_true", dest="rebuild_all", default=False,
                            help="force a rebuild of all universal libraries")
        parser.add_argument("--target-os", action="store", dest="target_os", type=lowercase_arg,
                            help="specifies the IOS target OS [default: iphone iphonesimulator]")
        parser.add_argument("lib_dir", action="store")

        args = parser.parse_args()
        lib_dir = args.lib_dir
        if not os.path.exists(lib_dir):
            print("make_ios_universal.py: error: the directory " + lib_dir + " does not exists.")
            sys.exit(1)
        fat_binary_tool = bldtools.FatBinaryTool()
        lib_dir = os.path.abspath(lib_dir)
        if args.target_os:
            platform_list = [args.target_os]
        else:
            platform_list = ['iphone', 'iphonesimulator']
        for platform in platform_list:
            dst_lib_dir = os.path.join(lib_dir, platform)
            if platform == 'iphone':
                iphone_targets = ['armv7', 'arm64']
            else:
                iphone_targets = ['x86_64', 'x86']
            src_lib_dirs = []
            for target in iphone_targets:
                src_lib_dir = os.path.join(lib_dir, platform + '-' + target)
                if os.path.exists(src_lib_dir):
                    src_lib_dirs.append(src_lib_dir)
            if len(src_lib_dirs) >= 2:
                if args.rebuild_all:
                    if os.path.exists(dst_lib_dir):
                        shutil.rmtree(dst_lib_dir)
                fat_binary_tool.createLibs(src_lib_dirs, dst_lib_dir)

