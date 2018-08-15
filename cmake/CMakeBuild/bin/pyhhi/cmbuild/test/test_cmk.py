import unittest

import pyhhi.build.common.ver as ver
import pyhhi.cmbuild.app.cmk as cmk
import pyhhi.cmbuild.cmksupp as cmksupp


class CMakeTestCase(unittest.TestCase):

    def setUp(self):
        self._cmk_version_test_set = [("cmake version 3.7.2", "3.7.2"),
                                      ("cmake version 3.8.0-rc2", "3.8.0"),
                                      ("cmake3 version 3.6.3", "3.6.3")]

    def test_cmake_command_line_parser(self):
        cmk_app = cmk.CMakeLauncherApp()

        for lnk_variant in ['static', 'shared']:
            (params, cmake_argv) = cmk_app._parse_command_line(['link=' + lnk_variant])
            self.assertEqual(params.link_variants, [lnk_variant])
        (params, cmake_argv) = cmk_app._parse_command_line(['link=static,shared'])
        self.assertEqual(params.link_variants, ['static', 'shared'])

        for bld_variant in ['debug', 'release', 'relwithdebinfo', 'minsizerel']:
            (params, cmake_argv) = cmk_app._parse_command_line(['variant=' + bld_variant])
            self.assertEqual(params.build_configs, [bld_variant])
        (params, cmake_argv) = cmk_app._parse_command_line(['variant=release,debug'])
        self.assertEqual(params.build_configs, ['release', 'debug'])

        (params, cmake_argv) = cmk_app._parse_command_line(['address-model=32'])
        self.assertEqual(params.target_arch, 'x86')
        (params, cmake_argv) = cmk_app._parse_command_line(['address-model=64'])
        self.assertEqual(params.target_arch, 'x86_64')

        (params, cmake_argv) = cmk_app._parse_command_line(['toolset=gcc-4.9'])
        self.assertEqual(params.toolset_str, 'gcc-4.9')

        (params, cmake_argv) = cmk_app._parse_command_line(['-DCMAKE_VAR=1', '-DCMAKE_VAR2:BOOL=ON', 'toolset=gcc-4.9'])
        self.assertEqual(params.toolset_str, 'gcc-4.9')
        self.assertEqual(params.cmk_cache_entries[0], '-DCMAKE_VAR=1')
        self.assertEqual(params.cmk_cache_entries[1], '-DCMAKE_VAR2:BOOL=ON')
        self.assertEqual(len(cmake_argv), 0)

    def test_cmake_version_parser(self):
        cmk_finder = cmksupp.CMakeFinder()
        for ver_tuple in self._cmk_version_test_set:
            cmk_version = cmk_finder._parse_cmake_version_retv(ver_tuple[0])
            self.assertEqual(ver_tuple[1], ver.version_tuple_to_str(cmk_version))
