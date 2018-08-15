
import unittest
import os.path
import pyhhi.build.common.bldtools as bldtools
import pyhhi.build.common.system as system


class BldToolsTestCase(unittest.TestCase):

    def setUp(self):
        self._sys_info = system.SystemInfo()

    def test_normalize_toolset_spec(self):
        # create a default toolset to be able to test _normalize_toolset_spec().
        ts = bldtools.Toolset(self._sys_info)
        self.assertEqual(ts._normalize_toolset_spec('clang++'), 'clang')
        self.assertEqual(ts._normalize_toolset_spec('clang++-3.5'), 'clang-3.5')
        self.assertEqual(ts._normalize_toolset_spec('g++'), 'gcc')
        self.assertEqual(ts._normalize_toolset_spec('g++-4.9'), 'gcc-4.9')

    def test_gcc_thread_model(self):
        if self._sys_info.is_linux():
            ts = bldtools.Toolset(self._sys_info)
            mingw_cmd = os.path.join('/usr', 'bin', 'x86_64-w64-mingw32-g++-win32')
            if os.path.exists(mingw_cmd):
                self.assertEqual(ts._get_gcc_thread_model(mingw_cmd), 'win32')
                mingw_cmd = os.path.join('/usr', 'bin', 'x86_64-w64-mingw32-g++-posix')
                if os.path.exists(mingw_cmd):
                    self.assertEqual(ts._get_gcc_thread_model(mingw_cmd), 'posix')
            else:
                mingw_cmd = os.path.join('/usr', 'bin', 'x86_64-w64-mingw32-g++')
                if os.path.exists(mingw_cmd):
                    self.assertEqual(ts._get_gcc_thread_model(mingw_cmd), 'posix')
