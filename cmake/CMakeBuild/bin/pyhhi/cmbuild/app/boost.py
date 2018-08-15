from __future__ import print_function

import argparse
import os
import re
import sys

import pyhhi.build.common.system as system
import pyhhi.build.common.util as util
import pyhhi.build.common.ver as ver
import pyhhi.cmbuild.boostbld as boostbld
from pyhhi.build.common.bldtools import MsvcToolsetSpecDict


class BoostInstallApp(object):

    def __init__(self):
        # creates the system info object and performs python version checks
        self._sys_info = system.SystemInfo(True)
        # Mapping between CMake VC toolsets and Boost.Build VC toolsets.
        self._msvc_dict = MsvcToolsetSpecDict()

    def __call__(self):
        self.main(sys.argv[1:])

    def main(self, argv):
        (boost_build_params, boost_dir, cxx_std) = self._parse_command_line(argv)
        # Change the current working directory to the top level directory in case the caller has moved
        # into CMakeBuild or bin to invoke boost_install.py.
        top_dir = util.get_top_dir()
        os.chdir(top_dir)
        msg_list_summary = []
        msg_list_summary.append('%-25s %s' % ('python version:', ver.version_tuple_to_str(self._sys_info.get_python_version())))
        msg_list_summary.append('%-25s %s' % ('python executable:', self._sys_info.get_python_executable()))
        msg_list_summary.append('%-25s %s' % ('platform:', self._sys_info.get_platform_long()))
        msg_list_todo = []
        self._sys_info.check_os_detection(msg_list_todo)

        if boost_dir is None:
            print("boost_install.py: no boost directory specified, use option --boost-dir.")
            sys.exit(1)
        else:
            boost_build_params.boost_dir = boost_dir

        if boost_build_params.boost_dir.startswith('/usr/include'):
            # system layout -> don't try to build the boost SDK and just extract the boost version
            msg_list_summary.append('%-25s %s' % ('boost version:', ver.version_tuple_to_str(boostbld.get_boost_version(boost_build_params.boost_dir))))
        else:
            # create the BoostBuilder
            boost_builder = boostbld.BoostBuilder(self._sys_info)

            if cxx_std:
                boost_build_params.cxx_std = cxx_std
            else:
                re_match = re.match(r'.*-(c\+\+[0-9]+)$', boost_dir)
                if re_match:
                    boost_build_params.cxx_std = re_match.group(1)
                else:
                    boost_build_params.cxx_std = 'c++11'

            # launch Boost building with the current set of build options.
            boost_builder.build_boost(boost_build_params)

            # compose the summary
            msg_list_summary.append('%-25s %s' % ('boost root:', boost_dir))
            msg_list_summary.append('%-25s %s' % ('boost version:', ver.version_tuple_to_str(boost_builder.get_boost_version())))
            if boost_build_params.targets:
                msg_list_summary.append('%-25s %s' % ('boost toolset:', boost_builder.get_toolset().get_toolset_info_short(boost_build_params.targets[0])))
            bjam_cmd_lines = boost_builder.get_bjam_cmd_lines()
            if bjam_cmd_lines:
                msg_list_summary.append('bjam command(s):')
                msg_list_summary.extend(boost_builder.get_bjam_cmd_lines())

        if msg_list_summary:
            print()
            print("                      SUMMARY                           ")
            print("                      =======                           ")
            print()
            for msg in msg_list_summary:
                print(msg)
            print()

        if msg_list_todo:
            print("\nTODO List:")
            for msg in msg_list_todo:
                print(msg)
            print()
        print("boost_install.py finished successfully.\n")

    def _parse_command_line(self, argv):
        _usage_header = "%(prog)s [options] [toolset=msvc-x.y|gcc|gcc-x[.y]|clang|intel[-msvc-x.y] [target-os=iphone|iphonesimulator]"
        _description = """

%(prog)s builds BOOST libraries from source, use option --boost-dir to specify the location of the BOOST SDK.
If the BOOST directory ends with -c++03, -c++11, -c++14, or -c++17, this suffix will be used as the c++ standard
unless option --std is specified. If neither condition is true, c++11 will be used. 

Notes on Intel Compiler Selection:
  - No support for side-by-side installation of multiple Intel compilers yet
  - No support for 32 bit builds on any platform
  - On Windows the latest MSVC runtime/IDE will be used unless overridden by a suffix -msvc-x.y". 
  
"""

        parser = argparse.ArgumentParser(usage=_usage_header, description=_description, formatter_class=argparse.RawDescriptionHelpFormatter)

        parser.add_argument("buildprops", action="store", nargs='*',
                            help="one or more build properties specified as key=value, e.g. toolset=msvc-12.0")

        parser.add_argument("--boost-dir", action="store", dest="boost_dir", required=True,
                            help="required option to specify the location of the BOOST SDK.")

        parser.add_argument("--std", action="store", dest="cxx_std", choices=['c++03', 'c++11', 'c++14', 'c++17'],
                            help="overrides the c++ standard which is deduced from the boost directory if it ends with "
                                 "one of the supported c++ standards prefixed by a hyphen.")

        parser.add_argument("--target", action="append", dest="targets",
                            help="override the toolset's default target. Typically used to build 32 bit libraries on Windows; "
                                 "example: --target x86 toolset=msvc-14.0")

        parser.add_argument("--rebuild", action="store_true", dest="rebuild_all", default=False,
                           help="force a rebuild of all boost libraries")

        group = parser.add_argument_group("MacOSX Options")

        g = group.add_mutually_exclusive_group()
        g.add_argument("--macosx-version-min", action="store", dest="macosx_version_min",
                           help="add backward compatibility down to the given MacOSX release.")

        g.add_argument("--ios-version-min", action="store", dest="macosx_version_min", metavar="IOS_VERSION_MIN",
                           help="add backward compatibility down to the given IOS release.")

        group = parser.add_argument_group("Advanced Options")

        group.add_argument("--c++-runtime", action="store", dest="cxx_runtime", choices=['shared', 'static', 'gnustl', 'libc++', 'libstdc++'],
                            help="overrides the default c++ runtime, supported for mingw, msvc, clang and Android.")

        group.add_argument("--with-mpi", action="store_true", dest="build_mpi", default=False,
                           help="enable Boost MPI (message passing interface) [default:off]")

        group.add_argument("--with-python", action="store_true", dest="build_python", default=False,
                           help="enable Boost Python [default:off]. If enabled, Boost MPI may not build.")

        group.add_argument("--use-user-config", action="store_true", dest="use_user_config", default=False,
                           help="enable the use of $HOME/user-config.jam [default:off].")

        util.app_args_add_log_level(group)

        group.add_argument("--dry-run", action="store_true", dest="dry_run", default=False,
                           help="check which libraries are missing, list the update actions but don't start the build.")

        group.add_argument("--debug-configuration", action="store_true", dest="debug_configuration", default=False,
                           help="invoke b2 with --debug-configuration, disables removal of temporary files.")

        group.add_argument("--jobs", action="store", type=int, dest="bjam_jobs",
                           help="specifies the number of parallel bjam jobs explicitly [default: number of processors]")

        args = parser.parse_args(argv)

        # configure the python logger
        util.app_configure_logging(args.log_level)

        boost_build_params = boostbld.BoostBuildParams(self._sys_info)
        re_toolset = re.compile(r'toolset=(\S+)')
        re_address_model = re.compile(r'address-model=(\S+)')
        re_target_os = re.compile(r'target-os=(\S+)')
        for arg in args.buildprops:
            arg_parsed = False
            re_match = re_toolset.match(arg)
            if re_match:
                boost_build_params.toolset = re_match.group(1)
                if boost_build_params.toolset.startswith('msvc-'):
                    boost_build_params.toolset = self._msvc_dict.transform_cmake_to_bbuild(boost_build_params.toolset)
                elif boost_build_params.toolset.startswith('intel-'):
                    re_match_intel = re.match(r'intel-(msvc-\d+\.\d+)$', boost_build_params.toolset)
                    if re_match_intel:
                        boost_build_params.toolset = 'intel'
                        boost_build_params.msvc_rt = re_match_intel.group(1)
                    else:
                        print("boost_install.py: argument", arg, " not understood, try --help to get more usage information.")
                        sys.exit(1)
                arg_parsed = True
            re_match = re_address_model.match(arg)
            if re_match:
                arg_parsed = True
                if re_match.group(1) == '32':
                    args.targets = ['x86']
                elif re_match.group(1) == '64':
                    args.targets = ['x86_64']
                else:
                    print("boost_install.py: argument", arg, " not understood, try --help to get more usage information.")
                    sys.exit(1)
            re_match = re_target_os.match(arg)
            if re_match:
                arg_parsed = True
                if re_match.group(1) == 'iphone':
                    boost_build_params.platform_index = 1
                elif re_match.group(1) == 'iphonesimulator':
                    boost_build_params.platform_index = 2
                else:
                    print("boost_install.py: argument", arg, " not understood, try --help to get more usage information.")
                    sys.exit(1)
            if not arg_parsed:
                print("boost_install.py: argument", arg, " not understood, try --help to get more usage information.")
                sys.exit(1)

        boost_dir = None
        if args.boost_dir:
            # boost_dir must be transformed before changing cwd.
            boost_dir = util.normalize_path(os.path.abspath(args.boost_dir))

        boost_build_params.targets = self._set_build_targets(boost_build_params.toolset, args.targets)

        if args.cxx_std:
            cxx_std = args.cxx_std
        else:
            cxx_std = None
        if args.cxx_runtime:
            boost_build_params.cxx_runtime = args.cxx_runtime

        boost_build_params.dry_run = args.dry_run
        boost_build_params.debug_configuration = args.debug_configuration
        boost_build_params.build_mpi = args.build_mpi
        boost_build_params.build_python = args.build_python
        boost_build_params.bjam_jobs = args.bjam_jobs
        boost_build_params.custom_user_config = args.use_user_config

        # options to select what to build
        boost_build_params.rebuild_all = args.rebuild_all

        if self._sys_info.is_macosx():
            # handle --macosx-version-min option, it will apply to MacOSX and iOS
            if args.macosx_version_min is not None:
                boost_build_params.macosx_version_min = ver.version_tuple_from_str(args.macosx_version_min)

        return boost_build_params, boost_dir, cxx_std

    def _set_build_targets(self, toolset, targets):
        if not self._sys_info.is_windows():
            return targets
        build_targets = []
        if (toolset is None) or toolset.startswith('msvc'):
            if targets:
                # user has specified one or more targets
                build_targets = targets
            else:
                # no targets specified, use x86_64
                build_targets = [self._sys_info.get_os_arch()]
        return build_targets
