from __future__ import print_function

import argparse
import sys
import os
import locale
import pyhhi.build.common.system
import pyhhi.build.common.ver as ver
import pyhhi.build.common.util as util


class SystemInfoApp(object):

    def __call__(self, argv=None):
        if argv is None:
            self.main(sys.argv[1:])
        else:
            self.main(argv)

    def main(self, argv):
        parser = argparse.ArgumentParser()
        parser.add_argument("-v", "--verbose", action="store_true", dest="verbose",
                            help="enable verbose output suitable for humans. The default format is intended for scripting frameworks like Boost.Build/bjam.")
        parser.add_argument("--print-env", action="store_true", dest="print_env", help="print environment variables" )

        util.app_args_add_log_level(parser)
        args = parser.parse_args(argv)
        # configure the python logger
        util.app_configure_logging(args.log_level)
        # creates the system info object and performs python version checks
        sys_info = pyhhi.build.common.system.SystemInfo(True)

        if args.verbose:
            print('%-30s %s' % ('python version:', ver.version_tuple_to_str(sys_info.get_python_version())))
            print('%-30s %s' % ('python executable:', sys_info.get_python_executable()))
            if sys_info.is_windows():
                print('%-30s %s' % ('python executable (short path):', sys_info.get_short_path(sys_info.get_python_executable())))
                print('%-30s %s' % ('python launcher:', sys_info.get_python_launcher()))
            print('%-30s %s' % ('python implementation:', sys_info.get_python_implementation()))
            print('%-30s %s' % ('python architecture:', sys_info.get_python_arch()))
            if sys_info.is_windows():
                print('%-30s %s' % ('python module win32api?:', str(sys_info.is_win32api_installed())))
            print('%-30s %s' % ('platform:', sys_info.get_platform_long()))
            print('%-30s %s' % ('os distro:', sys_info.get_os_distro()))
            if sys_info.is_linux():
                print('%-30s %s' % ('debian?:', str(sys_info.is_debian())))
                print('%-30s %s' % ('redhat?:', str(sys_info.is_redhat())))
                print('%-30s %s' % ('suse?:', str(sys_info.is_suse())))
                print('%-30s %s' % ('package format:', sys_info.get_pkg_fmt()))
                print('%-30s %s' % ('package architecture:', sys_info.get_pkg_arch()))
            elif sys_info.is_windows():
                print('%-30s %s' % ('windows8?:', str(sys_info.is_windows8())))
            print('%-30s %s' % ('os codename:', sys_info.get_os_codename()))
            print('%-30s %s' % ('os version:', ver.version_tuple_to_str(sys_info.get_os_version())))
            print('%-30s %s' % ('os architecture:', sys_info.get_os_arch()))
            print('%-30s %s' % ('locale.prefencoding:', locale.getpreferredencoding(False)))
            print('%-30s %s' % ('number of processors:', sys_info.get_number_processors()))
            print('%-30s %s' % ('home directory:', sys_info.get_home_dir()))
            desktop_dir = sys_info.get_desktop_dir()
            if desktop_dir is not None:
                print('%-30s %s' % ('desktop:', desktop_dir))
            # get_program_dir(self, target_arch):
            if sys_info.is_windows():
                program_dir = sys_info.get_program_dir(sys_info.get_os_arch())
                print('%-30s %s' % ('program dir:', program_dir))
                if sys_info.get_os_arch() == 'x86_64':
                    program_dir = sys_info.get_program_dir('x86')
                    print('%-30s %s' % ('program dir x86:', program_dir))
            label = 'path:'
            for d in sys_info.get_path():
                print('%-30s %s' % (label, d))
                label = ' '
        elif args.print_env:
            env_var_list = []
            for env_var in os.environ:
                env_var_list.append(env_var)
            env_var_list.sort()
            for env_var in env_var_list:
                print(env_var + '=' + os.getenv(env_var))
        else:
            print(sys_info.get_system_info_full_str())
