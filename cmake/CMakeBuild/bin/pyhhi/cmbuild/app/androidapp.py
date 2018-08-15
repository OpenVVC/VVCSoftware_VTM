from __future__ import print_function

import argparse
import os.path
import sys

import pyhhi.build.common.android
import pyhhi.build.common.util as util
import pyhhi.build.common.ver as ver


class NdkInfoApp(object):

    def __call__(self):
        parser = argparse.ArgumentParser()

        parser.add_argument("-v", action="store_true", dest="verbose", default=False,
                            help="enable verbose output suitable for humans. The default format is intended for scripting frameworks like Boost.Build.")
        parser.add_argument("--ignore-env-vars", action="store_false", dest="use_env", default=True,
                            help="disable environment variable ANDROID_NDK_ROOT in finding the desired NDK.")

        util.app_args_add_log_level(parser)

        args = parser.parse_args()

        # configure the python logger
        util.app_configure_logging(args.log_level)

        ndk_finder = pyhhi.build.common.android.NdkFinder(args.use_env)
        if args.verbose:
            print('%-42s %s' % ('NDK root:', ndk_finder.get_ndk_root()))
            print('%-42s %s' % ('NDK version:', ver.version_tuple_to_str(ndk_finder.get_ndk_version())))
            print('%-42s %s' % ('NDK-SA root:', ndk_finder.get_ndksa_root(None)))
            print('%-42s %s' % ('Avail. platforms:', ndk_finder.get_ndk_platforms()))
            toolchains = ndk_finder.get_ndksa_toolchains('gnustl')
            if toolchains:
                print('%-42s %s' % ('Avail. standalone toolchains using gnustl:', toolchains))
            toolchains = ndk_finder.get_ndksa_toolchains('libc++')
            if toolchains:
                print('%-42s %s' % ('Avail. standalone toolchains using libc++:', toolchains))


class CreateNdkToolchainApp(object):

    def __call__(self):
        self.main(sys.argv[1:])

    def main(self, argv):

        _description = """
This script creates a standalone NDK toolchain to be used with Boost.Build or another build system
outside of the Android SDK/NDK build environment.

        """

        _epilog = """
Examples:

    # create a default set of standalone toolchains
    %(prog)s
    
    # override the default API level
    %(prog)s --api-level=21      
        """
        parser = argparse.ArgumentParser(description=_description, epilog=_epilog, formatter_class=argparse.RawDescriptionHelpFormatter)

        util.app_args_add_log_level(parser)
        parser.add_argument("--unified-headers", action="store_true", dest="unified_headers", default=False,
                            help="enable unified headers [valid for NDK r14 and higher, enabled for NDK r15]." )
        parser.add_argument("--api-level", action="store", dest="api_level", type=int,
                            help="override the default Android API level [default: latest].")
        parser.add_argument("--stl", action="store", dest="stl", choices=['gnustl', 'libc++'], default='gnustl',
                            help="--stl=libc++ overrides the default c++ runtime, [default: gnustl to be compatible with Qt].")
        parser.add_argument("--inst-dir", action="store", dest="inst_dir",
                            help="specify the directory to save the toolchain, [default: $HOME/bin/ndksa].")
        parser.add_argument("toolchain", action="store", nargs='?',
                            help="specify the NDK toolchain to be converted into a standalone toolchain."
                            " Use default to create toolchains for all supported architectures.")

        args = parser.parse_args(argv)

        # configure the python logger
        util.app_configure_logging(args.log_level)
        # create the NDK finder
        ndk_finder = pyhhi.build.common.android.NdkFinder()
        ndk_version = ndk_finder.get_ndk_version()
        if ver.version_compare(ndk_version, (15, 0)) >= 0:
            args.unified_headers = True
        if (args.toolchain is None) or (args.toolchain == 'default'):
            for toolchain in ['arm-linux-androideabi-4.9', 'aarch64-linux-android-4.9', 'x86-4.9', 'x86_64-4.9']:
                ndk_finder.create_ndksa_toolchain(toolchain, api_level=args.api_level, ndk_stl=args.stl, inst_dir=args.inst_dir,
                                                  unified_headers=args.unified_headers)
        else:
            # create the NDK standalone toolchain
            ndk_finder.create_ndksa_toolchain(args.toolchain, api_level=args.api_level, ndk_stl=args.stl, inst_dir=args.inst_dir,
                                              unified_headers=args.unified_headers)

        # list available toolchains
        print('%-42s %s' % (' ', 'Summary'))
        if args.inst_dir is None:
            print('%-42s %s' % ('NDK-SA root:', ndk_finder.get_ndksa_root(None)))
        ndksa_toolchains = ndk_finder.get_ndksa_toolchains(args.stl)
        if ndksa_toolchains:
            print('%-42s %s' % ('Avail. standalone toolchains using ' + args.stl + ':', ' '.join(ndksa_toolchains)))
            if ver.version_compare(ndk_version, (11, 0)) >= 0:
                compiler_flavor = 'clang++'
            else:
                compiler_flavor = 'g++'
            if args.inst_dir is None:
                print('\nExample: boost_install.py toolset=' + ndksa_toolchains[0] + '-' + compiler_flavor)
            else:
                print('\nExample: boost_install.py toolset=' + os.path.join(args.inst_dir, 'bin', ndksa_toolchains[0] + '-' + compiler_flavor))
            print('\n')
