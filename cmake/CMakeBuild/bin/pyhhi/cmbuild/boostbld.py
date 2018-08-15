from __future__ import print_function

import glob
import logging
import os
import re
import shutil
import filecmp

import pyhhi.cmbuild.bjambld as bjambld
import pyhhi.build.common.bldtools as bldtools
import pyhhi.build.common.ver as ver
import pyhhi.build.common.util as util


def get_boost_version(boost_dir):
    """Returns the boost version as a version tuple."""
    if (not os.path.exists(boost_dir)) or (not os.path.isdir(boost_dir)):
        raise Exception("The boost directory " + boost_dir + " does not exist.")

    boost_dir = os.path.abspath(boost_dir)

    version_file = os.path.join(boost_dir, 'boost', 'version.hpp')
    if not os.path.exists(version_file):
        raise Exception("The directory " + boost_dir + " does not appear to be a boost directory.")

    # #define BOOST_VERSION 105300
    re_boost_version = re.compile(r'#define\s+BOOST_VERSION\s+([0-9]+)')
    with open(version_file) as f:
        for line in f:
            re_match = re_boost_version.match(line)
            if re_match:
                version_number = int(re_match.group(1), 10)
                boost_version = []
                boost_version.append(version_number // 100000)
                boost_version.append((version_number // 100) % 1000)
                boost_version.append(version_number % 100)
                return tuple(boost_version)

    raise Exception(version_file + ": does not contain a supported boost version string.")




class BoostBuildParams(object):

    def __init__(self, sys_info):
        self.boost_dir = None
        self.toolset = None

        # On Windows the Intel compiler may be used with different MSVC IDEs which provide
        # the C++ runtime, platform SDK, predefined macros.
        self.msvc_rt = None

        # Some toolsets support multiple platforms and the platform index selects the active platform;
        # example: clang supports macosx, iphone and iphonesimulator
        self.platform_index = 0

        self.cxx_std = "c++03"        # "c++11" "c++14", "c++17"
        self.cxx_runtime = "default"  # 'static', 'shared', 'gnustl', 'libc++', 'libstdc++'
        # Some toolsets support multiple targets; e.g. msvc, clang on macosx, or android arm cross compilers.
        self.targets = []

        self.dry_run = False
        self.bjam_jobs = None
        self.debug_configuration = False
        self.custom_user_config = False
        # macosx stuff
        self.macosx_version_min = None  # a version tuple to add -mmacosx-version-min=x.y
        self.max_install_names = True

        self.build_mpi = False
        self.build_python = False
        self.rebuild_all = False

        # a list of boost libraries to be built instead of the default set.
        # e.g. [python]
        self.boost_with_list = []

    def __str__(self):
        s = "boost_dir: %s\n" % self.boost_dir
        s += "toolset: %s\n" % self.toolset
        s += "platform index: %d\n" % self.platform_index
        if self.targets:
            s += "targets: %s\n" % self.targets
        s += "dry run: %s\n" % self.dry_run
        return s


class BoostBuilder(object):

    def __init__(self, sys_info):
        self._logger = logging.getLogger(__name__)
        self._sys_info = sys_info
        self._min_boost_version = (1, 40, 0)
        self._boost_version = None
        self._toolset = None
        self._bjam_cmd_lines = []
        self._boost_libs_android = ['system', 'serialization', 'filesystem', 'date_time', 'program_options', 'math']
        self._boost_libs_ios = ['system', 'serialization', 'filesystem', 'date_time', 'program_options', 'math']
        self._intel_msvc_suffix_dict = {'msvc-14.1': '-vs15', 'msvc-14.0': '-vs14', 'msvc-12.0': '-vs12'}
        self._intel_msvc_suffix = None

    def get_boost_version(self):
        return self._boost_version

    def get_toolset(self):
        assert self._toolset is not None
        return self._toolset

    def get_bjam_cmd_lines(self):
        return self._bjam_cmd_lines

    def get_boost_bin_dir(self, boost_dir, toolset, platform_index, target):
        platform_info = toolset.get_platform_info(platform_index)
        assert platform_info.get_target_os() == 'windows'
        if (toolset.get_toolset() == 'intel') and (platform_info.get_target_os() == 'windows'):
            assert self._intel_msvc_suffix is not None
            bin_dir = os.path.join(boost_dir, 'bin', toolset.get_compiler_tag() + self._intel_msvc_suffix, target)
        else:
            bin_dir = os.path.join(boost_dir, 'bin', toolset.get_compiler_tag(), target)
        return bin_dir

    def get_boost_lib_dir(self, boost_dir, toolset, platform_index, target):
        platform_info = toolset.get_platform_info(platform_index)
        if platform_info.get_target_os() == 'android':
            lib_dir = os.path.join(boost_dir, 'lib', toolset.get_compiler_tag(),
                                   'android-%d-%s' % (platform_info.get_api_level(), target))
        elif (platform_info.get_target_os() == 'iphone') or (platform_info.get_target_os() == 'iphonesimulator'):
            if target == 'combined':
                lib_dir = os.path.join(boost_dir, 'lib', toolset.get_compiler_tag(), platform_info.get_target_os())
            else:
                lib_dir = os.path.join(boost_dir, 'lib', toolset.get_compiler_tag(),
                                       '%s-%s' % (platform_info.get_target_os(), target))
        elif (toolset.get_toolset() == 'intel') and (platform_info.get_target_os() == 'windows'):
            assert self._intel_msvc_suffix is not None
            lib_dir = os.path.join(boost_dir, 'lib', toolset.get_compiler_tag() + self._intel_msvc_suffix, target)
        else:
            lib_dir = os.path.join(boost_dir, 'lib', toolset.get_compiler_tag(), target)
        return lib_dir

    def build_boost(self, build_params):
        #print("BoostBuilder.build_boost() starting ...")

        # extract the boost version
        boost_version = get_boost_version(build_params.boost_dir)
        self._boost_version = boost_version

        # print("BoostBuilder: boost version:", boost_version)

        # check the boost version against the minimal version we support
        if ver.version_compare(boost_version, self._min_boost_version) < 0:
            raise Exception("The boost version " + ver.version_tuple_to_str(boost_version) + " is not supported anymore, please contact technical support.")

        # construct a new toolset
        toolset = bldtools.Toolset(self._sys_info, build_params.toolset, build_params.cxx_runtime)

        self._toolset = toolset
        if toolset.get_toolset() == 'intel':
            if self._sys_info.is_windows():
                # Update MSVC environment to be used with the Intel compiler
                if build_params.msvc_rt is None:
                    # Use latest MSVC by default
                    msvc_registry = bldtools.MsvcRegistry()
                    msvc_version = msvc_registry.get_latest_version()
                    build_params.msvc_rt = "msvc-%d.%d" % (msvc_version[0], msvc_version[1])
                #
                self._intel_msvc_suffix = self._intel_msvc_suffix_dict[build_params.msvc_rt]
            elif self._sys_info.is_macosx():
                if ver.version_compare(boost_version, (1, 66, 0)) == 0:
                    # Is intel-darwin.jam patched?
                    intel_darwin_boost = os.path.join(build_params.boost_dir, 'tools', 'build', 'src', 'tools', 'intel-darwin.jam')
                    intel_darwin_patched = os.path.join(util.get_top_dir(), 'CMakeBuild', 'patches', 'Boost', ver.version_tuple_to_str(boost_version), 'tools', 'build', 'src', 'tools', 'intel-darwin.jam')
                    assert os.path.exists(intel_darwin_boost)
                    assert os.path.exists(intel_darwin_patched)
                    if not filecmp.cmp(intel_darwin_boost, intel_darwin_patched, shallow=False):
                        raise Exception("""intel-darwin.jam requires a patch. Consider a manual update:
   
cp %s %s

or contact technical support. 
""" % (intel_darwin_patched, intel_darwin_boost))

        platform_info = toolset.get_platform_info(build_params.platform_index)

        if (platform_info.get_target_os() == 'macosx') and (build_params.macosx_version_min is None):
            # At times Xcode ships with OSX SDK version > macosx version, this is not permitted by default.
            if ver.version_compare(platform_info.get_sdk_version(), self._sys_info.get_os_version()) > 0:
                build_params.macosx_version_min = self._sys_info.get_os_version()[:2]

        # check the optional target
        if build_params.targets:
            for target in build_params.targets:
                if target not in platform_info.get_target_arch():
                    raise Exception("The target " + target + " is not supported. Please check target and toolset.")
        else:
            # no target(s) specified, use the defaults
            if toolset.get_toolset() == 'msvc':
                build_params.targets = list(platform_info.get_target_arch())
            elif platform_info.get_target_os() in ['iphone', 'iphonesimulator']:
                if build_params.macosx_version_min:
                    target_os_version = build_params.macosx_version_min
                else:
                    target_os_version = platform_info.get_target_os_version()
                if ver.version_compare(target_os_version, (11, 0)) >=0:
                    # No support for 32 bit IOS targets anymore.
                    if platform_info.get_target_os() == 'iphone':
                        build_params.targets = ['arm64']
                    elif platform_info.get_target_os() == 'iphonesimulator':
                        build_params.targets = ['x86_64']
                    else:
                        assert False
                else:
                    if platform_info.get_target_os() == 'iphone':
                        build_params.targets = ['arm64', 'armv7']
                    elif platform_info.get_target_os() == 'iphonesimulator':
                        build_params.targets = ['x86_64', 'x86']
                    else:
                        assert False
            else:
                build_params.targets = [platform_info.get_target_arch(0)]

        # update the build parameters given the attributes of the toolset.
        self._update_build_params(toolset, build_params)

        #print(build_params)
        #return

        boost_lib_files_missing = False
        lib_dir_list = []
        if build_params.rebuild_all:
            boost_lib_files_missing = True
            if not build_params.dry_run:
                for target in build_params.targets:
                    if platform_info.get_target_os() == 'windows':
                        bin_dir = self.get_boost_bin_dir(build_params.boost_dir, toolset, build_params.platform_index, target)
                        if os.path.exists(bin_dir):
                            shutil.rmtree(bin_dir)
                    lib_dir = self.get_boost_lib_dir(build_params.boost_dir, toolset, build_params.platform_index, target)
                    lib_dir_list.append(lib_dir)
                    if os.path.exists(lib_dir):
                        shutil.rmtree(lib_dir)
        else:
            for target in build_params.targets:
                lib_dir = self.get_boost_lib_dir(build_params.boost_dir, toolset, build_params.platform_index, target)
                lib_dir_list.append(lib_dir)
                if not os.path.exists(lib_dir):
                    boost_lib_files_missing = True

        if boost_lib_files_missing:
            self._build_libs(boost_version, toolset, build_params)
            if not build_params.dry_run:
                if self._sys_info.is_macosx():
                    if len(build_params.targets) > 1:
                        # create a fat binary/universal library file
                        fat_binary_tool = bldtools.FatBinaryTool()
                        platform_info = toolset.get_platform_info(build_params.platform_index)
                        assert ((platform_info.get_target_os() == 'iphone') or (platform_info.get_target_os() == 'iphonesimulator'))
                        src_lib_dirs = []
                        for target in build_params.targets:
                            lib_dir = self.get_boost_lib_dir(build_params.boost_dir, toolset, build_params.platform_index, target)
                            src_lib_dirs.append(lib_dir)
                        dst_lib_dir = self.get_boost_lib_dir(build_params.boost_dir, toolset, build_params.platform_index, 'combined')
                        if os.path.exists(dst_lib_dir):
                            shutil.rmtree(dst_lib_dir)
                        fat_binary_tool.createLibs(src_lib_dirs, dst_lib_dir)
                    #
                    if ver.version_compare((1, 59, 0), self._boost_version) > 0:
                        platform_info = toolset.get_platform_info(build_params.platform_index)
                        if platform_info.get_target_os() == 'macosx':
                            lib_dir = self.get_boost_lib_dir(build_params.boost_dir, toolset, build_params.platform_index, build_params.targets[0])
                            print("Checking install names and dependencies in " + lib_dir)
                            dylib_list = glob.glob(lib_dir + '/*.dylib')
                            inst_name_inspector = bldtools.DyLibInstallNameInfoInspector()
                            for dylib in dylib_list:
                                inst_name_info = inst_name_inspector.create_install_name_info(dylib)
                                if not inst_name_info.inst_name.startswith('@rpath/'):
                                    inst_name_inspector.modify_install_name(dylib, os.path.join('@rpath', inst_name_info.basename))
                                    # Walk the dependency list and change all boost libraries from libboost_NNN.dylib to @rpath/libboost_NNN.dylib.
                                    depends_dict = {}
                                    for libname in inst_name_info.depends_list:
                                        if libname.startswith('libboost_'):
                                            depends_dict[libname] = os.path.join('@rpath', libname)
                                    if depends_dict:
                                        print("Changing dependencies of " + os.path.basename(dylib))
                                        inst_name_inspector.modify_depends(dylib, depends_dict)
                print("\n")
                print("-------------------- Post build phase --------------------")
                for lib_dir in lib_dir_list:
                    self._list_boost_components(lib_dir)
                print("----------------------------------------------------------")
        else:
            print("No boost library directories are missing, skipping library build step.")
            print("Use --rebuild if you intend to rebuild all libraries.")
            # testing
            #for lib_dir in lib_dir_list:
            #    self._list_boost_components(lib_dir)

    def _list_boost_components(self, lib_dir):
        print("Enumerating Boost components in %s:" % lib_dir)
        lib_file_list = glob.glob(os.path.join(lib_dir, '*.a'))
        if not lib_file_list:
            lib_file_list = glob.glob(os.path.join(lib_dir, '*.lib'))
        boost_comp_set = set()
        boost_comp_list = []
        re_boost_comp = re.compile(r'libboost_([^-]+).*$')
        for lib_file in lib_file_list:
            fname = os.path.basename(lib_file)
            re_match = re_boost_comp.match(fname)
            if re_match:
                boost_comp = re_match.group(1)
                if boost_comp not in boost_comp_set:
                    boost_comp_set.add(boost_comp)
                    boost_comp_list.append(boost_comp)
        if boost_comp_list:
            boost_comp_list.sort()
            for boost_comp in boost_comp_list:
                print("    %s" % boost_comp)
            # print(" ".join(boost_comp_list))

    def _update_build_params(self, toolset, build_params):
        platform_info = toolset.get_platform_info(build_params.platform_index)
        # modify the build parameters based on the toolset properties
        if platform_info.get_target_os() == 'android':
            if (toolset.get_toolset() == 'gcc') and (ver.version_compare(toolset.get_version(), (4, 8)) < 0):
                raise Exception("This gcc version is not supported on android, use gcc 4.8 or higher to get decent c++11 support.")
            build_params.cxx_std = 'c++11'
            if not build_params.boost_with_list:
                # use the standard boost library set for android
                build_params.boost_with_list = self._boost_libs_android
                # TODO: evaluate boost version and add some more libraries
        elif (platform_info.get_target_os() == 'iphone') or (platform_info.get_target_os() == 'iphonesimulator'):
            build_params.cxx_std = 'c++11'
            if not build_params.boost_with_list:
                # use the standard boost library set for ios
                build_params.boost_with_list = self._boost_libs_ios
                # TODO: evaluate boost version and add some more libraries
    
    def _is_boost_python_enabled(self, toolset, platform_index, target):
        build_python = False
        platform_info = toolset.get_platform_info(platform_index)
        if toolset.is_mingw() or (platform_info.get_target_os() == 'android'):
            return build_python
        python_version = self._sys_info.get_python_version()
        if platform_info.get_target_os() == 'linux':
            # This is neither mingw or android
            if target in ['x86_64', 'x86']:
                python_header_file = os.path.join('/usr/include', 'python' + ver.version_tuple_to_str(python_version[:2]), 'Python.h')
                if os.path.exists(python_header_file):
                    build_python = True
        elif platform_info.get_target_os() == 'macosx':
            #python_header_file = os.path.join('/usr/include', 'python' + ver.version_tuple_to_str(python_version[:2]), 'Python.h')
            #if os.path.exists(python_header_file):
            build_python = True
        elif toolset.get_toolset() == 'msvc':
            # This implies msvc and target either x86 or x86_64.
            if self._sys_info.get_python_arch() == target:
                build_python = True
        return build_python

    def _build_libs(self, boost_version, toolset, build_params):

        bjam_builder = None
        tmp_dirs = []
        remove_user_config = True

        user_config = os.path.join(self._sys_info.get_home_dir(), 'user-config.jam')
        # check for $HOME/user-config.jam
        if build_params.custom_user_config:
            remove_user_config = False
        else:
            if os.path.exists(user_config):
                raise Exception("$HOME/user-config.jam exists and may conflict with the current configured toolset to be used to compile the boost libraries.")

        platform_info = toolset.get_platform_info(build_params.platform_index)
        try:
            #print("Listing toolset attributes ...")
            #print(toolset)

            # If we don't have a bjam program at this point, it's clear we must build it from the boost source.
            # On windows the boost version matters because vc11 can only be used to build bjam for 1.50.0 or higher.
            bjam_builder = bjambld.BjamBuilder(self._sys_info, top_dir=build_params.boost_dir, bb_version=self._boost_version)
            if build_params.dry_run:
                print("dry run: a temporary bjam has to be built from source.")
                # just for the message summary and command lines
                bjam_prog = "b2"
            else:
                bjam_target_arch = 'x86_64'
                if self._sys_info.is_windows() and (ver.version_compare(boost_version, (1, 64, 0)) >= 0):
                    # Boost 1.64.0 and higher comes with build.bat which does not support building a 64 bit b2.
                    bjam_target_arch = 'x86'
                bjam_prog = bjam_builder.build(bjam_target_arch)
                # print("created temporary bjam:", bjam_prog)

                tmp_dirs.append(os.path.join(build_params.boost_dir, 'bin.v2' ))
                # The next temporary directory exists in 1.58.0 but not in earlier versions.
                tmp_dirs.append(os.path.join(build_params.boost_dir, 'libs', 'config', 'checks', 'architecture', 'bin' ))
                tmp_dirs.append(os.path.join(build_params.boost_dir, 'tmp' ))

            # Remove temporary build trees, they might exist if a previous build was terminated by ctrl-c or the user tried a manual build.
            for dir in tmp_dirs[:2]:
                if os.path.exists(dir):
                    print("Removing '" + dir + "' ...")
                    shutil.rmtree(dir)             

            bjam_launcher = bjambld.BjamLauncher(self._sys_info)

            if build_params.bjam_jobs is None:
                build_params.bjam_jobs = bjam_launcher.get_optimal_number_bjam_jobs()

            # save the bjam command lines as a single string for logging
            self._bjam_cmd_lines = []
            os.chdir(build_params.boost_dir)

            for target in build_params.targets:
                (bjam_arg_list, user_config_content, setup_cmd_file, setup_cmd_content) = self._compose_bjam_arg_list(bjam_prog, toolset, build_params, target)
                # Remember the complete command line to be included in the final summary message.
                self._bjam_cmd_lines.append(' '.join(bjam_arg_list))
                if user_config_content:
                    if not build_params.dry_run:
                        self._write_tmp_user_config(user_config, user_config_content, setup_cmd_file, setup_cmd_content)
                    self._bjam_cmd_lines.append('<user-config.jam>')
                    self._bjam_cmd_lines.extend(user_config_content)
                    self._bjam_cmd_lines.append('</user-config.jam>')
                    if setup_cmd_file:
                        self._bjam_cmd_lines.append('<' + setup_cmd_file + '>')
                        self._bjam_cmd_lines.extend(setup_cmd_content)
                        self._bjam_cmd_lines.append('</' + setup_cmd_file + '>')
                if not build_params.dry_run:
                    bjam_launcher.launch(bjam_arg_list)
                    if platform_info.get_target_os() == 'windows':
                        bin_dir = self.get_boost_bin_dir(build_params.boost_dir, toolset, build_params.platform_index, target)
                        lib_dir = self.get_boost_lib_dir(build_params.boost_dir, toolset, build_params.platform_index, target)
                        self._move_boost_dlls(lib_dir, bin_dir, tmp_dirs[0])
                    # Get rid of the binaries in case multiple targets are selected which may use the same build tree and config check tree.
                    for d in tmp_dirs[:2]:
                        if os.path.exists(d):
                            print("Removing '" + d + "' ...")
                            shutil.rmtree(d)
                if os.path.exists(user_config) and remove_user_config:
                    os.remove(user_config)
        finally:
            if remove_user_config:
                if os.path.exists(user_config):
                    os.remove(user_config)
                if setup_cmd_file and os.path.exists(setup_cmd_file):
                    os.remove(setup_cmd_file)
            if not build_params.debug_configuration:
                for dir in tmp_dirs:
                    if os.path.exists(dir):
                        print("Removing '" + dir + "' ...")
                        shutil.rmtree(dir)
            if bjam_builder is not None:
                # get rid of the temporary bjam executable
                bjam_builder.remove_tmp_files()

    def _compose_bjam_arg_list(self, bjam_prog, toolset, build_params, target_arch):
        platform_info = toolset.get_platform_info(build_params.platform_index)
        if platform_info.get_target_os() == 'android':
            is_android = True
        else:
            is_android = False
        cflags_list = []
        cxxflags_list = []
        linkflags_list = []

        bjam_common_args = [bjam_prog]

        if build_params.bjam_jobs > 1:
            bjam_common_args.append('-j' + str(build_params.bjam_jobs))
        if build_params.debug_configuration:
            bjam_common_args.append('--debug-configuration')
        bjam_common_args.append('--layout=versioned')
        bjam_common_args.append('--prefix=tmp/dist')
        #if toolset.get_toolset() == 'clang':
        #    if ver.version_compare(toolset.get_version(), (3, 2) ) >= 0:
        #        cxxflags_list.append('-ftemplate-depth=256')
        if platform_info.get_target_os() in ['macosx', 'iphone', 'iphonesimulator']:
            if build_params.max_install_names:
                # macosx specific flag to help the install_name_tool
                linkflags_list.append('-headerpad_max_install_names')
            if build_params.macosx_version_min:
                version_str = ver.version_tuple_to_str(build_params.macosx_version_min)
            else:
                version_str = None
                target_os_ver = platform_info.get_target_os_version()
                if platform_info.get_target_os() == 'macosx':
                    # deployment target cannot be larger than the macosx SDK version.
                    if ver.version_compare(target_os_ver, self._sys_info.get_os_version()) < 0:
                        version_str = ver.version_tuple_to_str(target_os_ver)
                else:
                    # a version hint is required for all portable platforms.
                    version_str = ver.version_tuple_to_str(target_os_ver)
            if version_str:
                if platform_info.get_target_os() == 'macosx':
                    cflags_list.append('-mmacosx-version-min=' + version_str)
                    linkflags_list.append('-mmacosx-version-min=' + version_str)
                elif platform_info.get_target_os() == 'iphone':
                    cflags_list.append('-miphoneos-version-min=' + version_str)
                    linkflags_list.append('-miphoneos-version-min=' + version_str)
                elif platform_info.get_target_os() == 'iphonesimulator':
                    cflags_list.append('-mios-simulator-version-min=' + version_str)
                    linkflags_list.append('-mios-simulator-version-min=' + version_str)
                else:
                    assert False

        if toolset.get_toolset() == 'clang':
            # For backward compatibility the user may have overridden the default c++ runtime on macosx which is only
            # sensible in c++03 mode.
            # On linux libc++ overrides the default c++ runtime.
            if self._sys_info.is_linux() and build_params.cxx_runtime in ['libc++']:
                cxxflags_list.append('-stdlib=' + build_params.cxx_runtime)
                linkflags_list.append('-stdlib=' + build_params.cxx_runtime)
            elif self._sys_info.is_macosx() and build_params.cxx_runtime in ['libstdc++']:
                cxxflags_list.append('-stdlib=' + build_params.cxx_runtime)
                linkflags_list.append('-stdlib=' + build_params.cxx_runtime)

        if toolset.is_mingw():
            if build_params.cxx_runtime == 'shared':
                linkflags_list.append('-shared-libstdc++')
                linkflags_list.append('-shared-libgcc')
            elif build_params.cxx_runtime == 'static':
                linkflags_list.append('-static-libstdc++')
                linkflags_list.append('-static-libgcc')

        bjam_common_args.append('variant=release,debug')
        if is_android:
            bjam_common_args.append('link=static')
        elif platform_info.get_target_os() in ['iphone', 'iphonesimulator']:
            bjam_common_args.append('link=static')
        else:
            if (toolset.get_toolset() == 'msvc') and (build_params.cxx_runtime == 'static'):
                # Experimental feature to build the static boost libraries with dependency on the static MSVC runtime.
                bjam_common_args.append('link=static')
                bjam_common_args.append('runtime-link=static')
            else:
                bjam_common_args.append('link=static,shared')
        bjam_common_args.append('install')

        if build_params.boost_with_list:
            boost_with_list = list(build_params.boost_with_list)
        else:
            boost_with_list = []
        bjam_arg_list = []
        bjam_arg_list.extend(bjam_common_args)

        if toolset.get_toolset() == 'msvc':
            # On windows msvc-x.y selects the VS compiler.
            bjam_arg_list.append('toolset=' + toolset.get_toolset_versioned())
        else:
            bjam_arg_list.append('toolset=' + toolset.get_toolset())

        boost_lib_dir = self.get_boost_lib_dir(build_params.boost_dir, toolset, build_params.platform_index, target_arch)
        boost_lib_dir = os.path.relpath(boost_lib_dir, build_params.boost_dir)
        bjam_arg_list.append('--libdir=' + boost_lib_dir)
        if is_android:
            # The NDK cross compilers generate position independent code by default, some do -fpic and others -fPIC.
            bjam_arg_list.append('target-os=android')
        elif self._sys_info.is_linux():
            if toolset.get_toolset() == 'gcc':
                if toolset.is_mingw():
                    if ver.version_compare(self.get_boost_version(), (1, 66, 0)) >= 0:
                        # Boost 1.66.0 and higher
                        if target_arch == 'x86_64':
                            bjam_arg_list.append('address-model=64')
                        elif target_arch == 'x86':
                            bjam_arg_list.append('address-model=32')
                    bjam_arg_list.append('target-os=windows')
                    bjam_arg_list.append('threadapi=win32')
                elif platform_info.get_target_arch(0) in ['arm', 'aarch64']:
                    # is this really supported by every arm compiler?
                    cflags_list.append('-fPIC')
                else:
                    cflags_list.append('-fPIC')
                    assert self._sys_info.get_os_arch() == target_arch
            elif toolset.get_toolset() == 'clang':
                cflags_list.append('-fPIC')
            elif toolset.get_toolset() == 'intel':
                cflags_list.append('-fPIC')
            else:
                assert False
        elif self._sys_info.is_windows():
            if ver.version_compare(self.get_boost_version(), (1, 66, 0)) >= 0:
                # Boost 1.66.0 and higher
                if target_arch == 'x86_64':
                    bjam_arg_list.append('address-model=64')
                elif target_arch == 'x86':
                    bjam_arg_list.append('address-model=32')
            else:
                if toolset.is_mingw():
                    if (target_arch == 'x86') and (target_arch != platform_info.get_target_arch(0)):
                        bjam_arg_list.append('address-model=32')
                elif toolset.get_toolset() == 'msvc':
                    # Assume this compiler is a msvc-x.y variant.
                    if target_arch == 'x86_64':
                        bjam_arg_list.append('address-model=64')
        elif self._sys_info.is_macosx():
            # target-os=iphonesimulator is undefined in Boost.Build inside the BOOST SDK tree.
            # target-os=iphone does not define dylib as the shared library suffix inside the BOOST SDK tree.
            #if platform_info.get_target_os() == 'iphone':
            #    bjam_arg_list.append('target-os=' + platform_info.get_target_os())
            pass
        else:
            assert False

        if build_params.cxx_std is not None:
            re_std = re.compile(r'c\+\+(\d+)$')
            re_match = re_std.match(build_params.cxx_std)
            if not re_match:
                raise Exception("C++ standard specification " + build_params.cxx_std + " is not supported.")
            if toolset.get_toolset() in ['gcc', 'clang', 'intel']:
                if ver.version_compare(self._boost_version, (1, 66, 0)) >= 0:
                    bjam_arg_list.append('cxxstd=' + re_match.group(1))
                else:
                    cxxflags_list.append('-std=' + build_params.cxx_std)
        build_boost_python = False
        if build_params.build_python:
            build_boost_python = self._is_boost_python_enabled(toolset, build_params.platform_index, target_arch)
        if boost_with_list:
            for item in boost_with_list:
                bjam_arg_list.append('--with-' + item)
        else:
            # add --without-xxx arguments which cannot be combined with --with-xxx
            if toolset.is_mingw():
                bjam_arg_list.append('--without-locale')
            if not build_boost_python:
                bjam_arg_list.append('--without-python')
            if not build_params.build_mpi:
                bjam_arg_list.append('--without-mpi')
        if build_params.custom_user_config:
            user_config_content = []
            setup_cmd_file = None
            setup_cmd_content = []
        else:
            (user_config_content, setup_cmd_file, setup_cmd_content) = self._create_user_config_content(toolset, build_params.platform_index, target_arch, build_params.build_mpi, build_boost_python,
                                                                       cppflags=cxxflags_list, cflags=cflags_list, lflags=linkflags_list, msvc_rt=build_params.msvc_rt)
        return bjam_arg_list, user_config_content, setup_cmd_file, setup_cmd_content

    def _create_user_config_content(self, toolset, platform_index, target, build_boost_mpi, build_boost_python, cppflags=None, cflags=None, lflags=None, msvc_rt=None):
        """Check the toolset attributes and determine whether a temporary user-config.jam is needed or not."""
        lines = []
        platform_info = toolset.get_platform_info(platform_index)
        toolset_str = toolset.get_toolset()
        toolset_version_str = ver.version_tuple_to_str(toolset.get_version()[:2])
        cxxflags = []
        compileflags = []
        linkflags = []
        rc = []
        using_line = None
        setup_cmd_content = []
        setup_cmd = None

        if cppflags:
            cxxflags.extend(cppflags)
        if cflags:
            compileflags.extend(cflags)
        if lflags:
            linkflags.extend(lflags)

        extra_cflags = platform_info.get_target_cflags(target)
        if extra_cflags:
            compileflags.extend(extra_cflags)

        if toolset_str == 'gcc':
            using_line = "using %s : %s : %s" % (toolset_str, toolset_version_str, toolset.get_compiler_command())
            if toolset.is_mingw():
                if self._sys_info.is_linux():
                    # mingw on linux is always a prefixed compiler and it does not figure out the path to windres by default.
                    compiler_dir = os.path.dirname(toolset.get_compiler_command())
                    windres = os.path.join(compiler_dir, toolset.get_compiler_prefix() + '-windres')
                    rc.append(windres)
        elif toolset_str == 'clang':
            if platform_info.get_target_os() in ['android', 'linux']:
                using_line = "using %s : %s : %s" % (toolset_str, toolset_version_str, toolset.get_compiler_command())
            elif platform_info.get_target_os() in ['macosx', 'iphone', 'iphonesimulator']:
                using_line = "using %s : : %s" % (toolset_str, toolset.get_compiler_command())

                isysroot = platform_info.get_isysroot()
                assert isysroot is not None
                compileflags.extend(['-isysroot', isysroot])
                if platform_info.get_target_os() == 'macosx':
                    # Add 3rd party native frameworks folder
                    compileflags.append('-F/Library/Frameworks')
                    linkflags.append('-F/Library/Frameworks')
                else:
                    arch_flags = None
                    if platform_info.get_target_os() == 'iphone':
                        if target in ['armv7', 'arm64']:
                            arch_flags = ['-arch', target]
                        elif target == 'combined':
                            arch_flags = ['-arch', 'armv7', '-arch', 'arm64']
                        else:
                            assert False
                    elif platform_info.get_target_os() == 'iphonesimulator':
                        if target == 'x86_64':
                            # Assume -arch x86_64 is the default
                            pass
                        elif target == 'x86':
                            arch_flags = ['-arch', 'i386']
                        elif target == 'combined':
                            arch_flags = ['-arch', 'x86_64', '-arch', 'i386']
                        else:
                            assert False
                    else:
                        assert False
                    if arch_flags:
                        compileflags.extend(arch_flags)
                        linkflags.extend(arch_flags)
            else:
                assert False
        elif toolset_str == 'intel':
            if self._sys_info.is_linux() or self._sys_info.is_macosx():
                using_line = "using %s : %s : %s" % (toolset_str, toolset_version_str, toolset.get_compiler_command())
            elif self._sys_info.is_windows():
                compiler_cmd = os.path.normpath(toolset.get_compiler_command())
                compiler_cmd_escaped = compiler_cmd.replace("\\", "\\\\")
                assert msvc_rt is not None
                re_match = re.match(r'msvc-(\d+\.\d+)$', msvc_rt)
                if re_match:
                    msvc_version_str = re_match.group(1)
                else:
                    assert False
                using_line = 'using %s : %s : "%s" :' % (toolset_str, toolset_version_str, compiler_cmd_escaped)
                #using_line += ' <rewrite-setup-scripts>off'
                using_line += ' <compatibility>vc' + msvc_version_str
            else:
                assert False
        elif toolset_str == 'msvc':
            if toolset_version_str in ['14.0']:
                msvc_registry = bldtools.MsvcRegistry()
                if msvc_registry.is_vs2017_toolset(ver.version_tuple_from_str(toolset_version_str)):
                    # target ...
                    setup_cmd = os.path.normpath(os.path.join(self._sys_info.get_home_dir(), 'setup_vs2017_vc140.bat'))
                    # replace single backslashes with double backslashes
                    setup_cmd_escaped = setup_cmd.replace("\\", "\\\\")
                    using_line = 'using %s : %s : : <setup>"%s"' % (toolset_str, toolset_version_str, setup_cmd_escaped)
                    using_line += ' <rewrite-setup-scripts>off'

                    # call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Professional\VC\Auxiliary\Build\vcvarsall.bat" amd64 -vcvars_ver=14.0
                    if target == 'x86_64':
                        setup_cmd_target = 'amd64'
                    elif target == 'x86':
                        setup_cmd_target = 'amd64_x86'
                    else:
                        assert False
                    setup_cmd_content.append('call "%s" %s -vcvars_ver=14.0' % (toolset.get_compiler_command(), setup_cmd_target))

        if cxxflags or compileflags or linkflags or rc:
            assert using_line is not None
            using_line += " :"
            if cxxflags:
                using_line += ' <cxxflags>"%s"' % ' '.join(cxxflags)
            if compileflags:
                using_line += ' <compileflags>"%s"' % ' '.join(compileflags)
            if linkflags:
                using_line += ' <linkflags>"%s"' % ' '.join(linkflags)
            if rc:
                using_line += ' <rc>"%s"' % ' '.join(rc)

        if using_line:
            lines.append(using_line + " ;")

        if build_boost_python:
            python_config_file = os.path.join(self._sys_info.get_home_dir(), 'user-config-python.jam')
            if os.path.exists(python_config_file):
                with open(python_config_file) as f:
                    for line in f:
                        lines.append(line.rstrip())
            else:
                python_version = self._sys_info.get_python_version()
                if platform_info.get_target_os() == 'windows':
                    python_executable_short_path = self._sys_info.get_short_path_win(self._sys_info.get_python_executable())
                    python_executable_cstring = python_executable_short_path.replace("\\", "\\\\")
                    lines.append('using python : %d.%d : "%s" ;' % (python_version[0], python_version[1], python_executable_cstring))
                elif platform_info.get_target_os() == 'macosx':
                    lines.append('using python : %d.%d : %s ;' % (python_version[0], python_version[1], self._sys_info.get_python_executable()))
                elif platform_info.get_target_os() == 'linux':
                    lines.append('using python : %d.%d : %s ;' % (python_version[0], python_version[1], self._sys_info.get_python_executable()))

        if build_boost_mpi:
            mpi_config_file = os.path.join(self._sys_info.get_home_dir(), 'user-config-mpi.jam')
            if os.path.exists(mpi_config_file):
                with open(mpi_config_file) as f:
                    for line in f:
                        lines.append(line.rstrip())
            else:
                lines.append("using mpi ;")

        return lines, setup_cmd, setup_cmd_content

    def _write_tmp_user_config(self, user_config_file, user_config_content, setup_cmd_file, setup_cmd_content):
        if user_config_content:
            if os.path.exists(user_config_file):
                raise Exception(
                    "The file " + user_config_file + " already exists and may conflict with the current configured toolset. Please contact technical support.")
            # and write the line(s)
            with open(user_config_file, "w") as user_configf:
                for line in user_config_content:
                    user_configf.write(line + "\n")
        if setup_cmd_content:
            with open(setup_cmd_file, "w") as setup_cmdf:
                for line in setup_cmd_content:
                    setup_cmdf.write(line + "\n")

    def _find_boost_pdb_files(self, boost_dir):
        pdb_info_dict = {}
        re_pdb = re.compile(r'.*\.pdb$', re.IGNORECASE)
        for dirpath, dirs, files in os.walk(boost_dir):
            for f in files:
                if re_pdb.match(f):
                    (fname, ext) = os.path.splitext(f)
                    pdb_info_dict[fname] = os.path.join(dirpath, f)
        return pdb_info_dict

    def _move_boost_dlls(self, lib_dir, bin_dir, boost_build_dir):

        if not os.path.exists(lib_dir):
            print("*** warning: the library directory does not exist, the build failed completely for some reason. ***")
            return
        if not os.path.exists(bin_dir):
            os.makedirs(bin_dir)
        # find pdb files which are not installed in lib_dir by BOOST SDK installer.
        pdb_info_dict = self._find_boost_pdb_files(os.path.join(boost_build_dir, 'libs'))
        re_dll = re.compile(r'.*\.dll$', re.IGNORECASE)
        file_list = os.listdir(lib_dir)
        for f in file_list:
            if re_dll.match(f):
                shutil.copy(os.path.join(lib_dir, f), bin_dir)
                os.remove(os.path.join(lib_dir, f))
                # Do we have a PDB file?
                (fname, ext) = os.path.splitext(f)
                if fname in pdb_info_dict:
                    shutil.copy(pdb_info_dict[fname], bin_dir)

