from __future__ import print_function

import os
import shutil
import subprocess
import logging

import pyhhi.build.common.ver as ver
import pyhhi.build.common.bldtools as bldtools
from pyhhi.build.common.system import SystemInfo


class BjamBuilder(object):
    """The BjamBuilder class supports building a new bjam executable."""

    def __init__(self, sys_info, top_dir, bb_version):
        self._logger = logging.getLogger(__name__)
        bjam_src_tree_list = []

        self._sys_info = sys_info
        self._bjam_src_dir = None
        self._top_dir = top_dir
        self._bb_version = bb_version
        self._toolset = None
        self._tmp_dirs = []

        if self._sys_info.is_windows():
            self._bjam_names = ('b2.exe', 'bjam.exe')
        else:
            self._bjam_names = ('b2', 'bjam')

        if sys_info.is_windows():
            build_script = 'build.bat'
        else:
            build_script = 'build.sh'

        # the bjam source is supposed to come from the boost source tree.
        assert bb_version is not None
        boost_tools_dir = os.path.join(self._top_dir, 'tools')
        bjam_src_tree_list.append(os.path.join(boost_tools_dir, 'build', 'src', 'engine'))
        bjam_src_tree_list.append(os.path.join(boost_tools_dir, 'build', 'v2', 'engine'))
        bjam_src_tree_list.append(os.path.join(boost_tools_dir, 'build', 'v2', 'engine', 'src'))
        bjam_src_tree_list.append(os.path.join(boost_tools_dir, 'jam', 'src'))

        for d in bjam_src_tree_list:
            # check for the build script to figure out which source location holds the bjam source files.
            if os.path.exists(os.path.join(d, build_script)):
                self._bjam_src_dir = d
                break

        if self._bjam_src_dir is not None:
            # create a new bldtools suitable to build bjam on this platform.
            self._toolset = bldtools.BjamToolset(sys_info, bb_version)

    def build(self, target_arch='x86_64'):
        """Builds the b2 executable from source and returns the full path to the executable."""
        assert self._bjam_src_dir is not None
        if self._sys_info.is_windows() and (ver.version_compare(self._bb_version, (1, 66, 0)) >= 0):
            target_arch = 'x86'
        # create a new list of temporary directories to be removed after the bjam executable has been installed.
        self._tmp_dirs = []

        bjam_bin_dir = os.path.join(self._bjam_src_dir, self._get_bjam_bin_dir_folder(target_arch))
        self._tmp_dirs.append(bjam_bin_dir)

        b2_prog_path = os.path.join(bjam_bin_dir, self._bjam_names[0])
        bjam_prog_path = os.path.join(bjam_bin_dir, self._bjam_names[1])

        bootstrap_dir = os.path.join(self._bjam_src_dir, 'bootstrap')
        self._tmp_dirs.append(bootstrap_dir)

        if os.path.exists(bootstrap_dir):
            # in case a previous build failed to remove the temporary files, remove bootstrap completely.
            shutil.rmtree(bootstrap_dir)

        cur_dir = os.getcwd()
        os.chdir(self._bjam_src_dir)

        print("========================================================")
        print("Start building bjam in", self._bjam_src_dir, "...")
        print("========================================================")

        build_script_args = []
        if self._sys_info.is_windows():
            build_script = os.path.join(self._bjam_src_dir, 'build.bat')
            build_script_args.append(build_script)
            bjam_toolset_arg = self._toolset.get_bjam_toolset(build_script_format=True)
            build_script_args.append(bjam_toolset_arg)

            if target_arch == 'x86_64':
                # build.bat builds a 32 bit b2 executable by default but we prefer a native b2.
                if bjam_toolset_arg in ['vc141', 'vc14']:
                    build_script_args.append('amd64')
                else:
                    build_script_args.append('x86_amd64')
        else:
            build_script = os.path.join(self._bjam_src_dir, 'build.sh')
            build_script_args.append(build_script)

        retv = subprocess.call(build_script_args)
        if retv != 0:
            raise Exception("Building bjam failed. Please contact technical support.")

        # restore the previous current working directory
        os.chdir(cur_dir)

        if os.path.exists(b2_prog_path):
            return b2_prog_path
        elif os.path.exists(bjam_prog_path):
            return bjam_prog_path
        else:
            assert False
            return None

    def remove_tmp_files(self):
        """Removes all temporary files created by the bjam build script."""
        for d in self._tmp_dirs:
            if os.path.exists(d):
                try:
                    shutil.rmtree(d)
                except WindowsError as exc:
                    print("WARNING: ignoring spurious windows error [" + str(exc.winerror) + "]: " + exc.strerror + " raised by shutil.rmtree().")
                    if os.path.exists(d):
                        file_list = os.listdir(d)
                        if file_list:
                            print("The directory '" + d + "' is not empty for unknown reason: ", file_list)
        self._tmp_dirs = []

    def _get_bjam_bin_dir_folder(self, target_arch='x86_64'):
        if self._sys_info.is_windows():
            bin_dir = 'bin.nt' + target_arch
        elif self._sys_info.is_linux():
            bin_dir = 'bin.linux' + target_arch
        elif self._sys_info.is_macosx():
            bin_dir = 'bin.macosx' + target_arch
        else:
            assert False
        return bin_dir


class BjamLauncher(object):

    def __init__(self, sys_info=None, verbosity=1):
        self._logger = logging.getLogger(__name__)
        if sys_info is None:
            sys_info = SystemInfo()
        self._sys_info = sys_info
        self._verbosity_level = verbosity

    def get_optimal_number_bjam_jobs(self):
        """Returns the optimal number of bjam jobs."""
        bjam_jobs = self._sys_info.get_number_processors()
        if 'BJAM_MAX_JOBS' in os.environ:
            bjam_max_jobs = int(os.environ['BJAM_MAX_JOBS'], 10)
            if bjam_jobs > bjam_max_jobs:
                bjam_jobs = bjam_max_jobs
        assert bjam_jobs >= 1
        return bjam_jobs

    def launch(self, argv):
        """Launch a bjam build and block until it terminates."""
        if self._verbosity_level > 0:
            # assemble the bjam command line for logging purposes
            joiner = ' '
            cmd_line = joiner.join(argv)
            print("Launching: " + cmd_line)
        retv = subprocess.call(argv)
        if retv < 0:
            self._logger.debug("child was terminated by signal: %d", -retv)
        else:
            self._logger.debug("child returned: %d", retv)
        return retv
