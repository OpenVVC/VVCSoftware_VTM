
import unittest
import sys
import os.path
import pyhhi.build.common.util as util

class UtilTestCase(unittest.TestCase):

    @unittest.skipUnless(sys.platform.startswith('win'), "requires windows")
    def test_find_tool_on_path_win(self):
        self.assertIsNotNone(util.find_tool_on_path('cmd'))
        self.assertIsNotNone(util.find_tool_on_path('cmd.exe'))
        self.assertIsNotNone(util.find_tool_on_path(os.path.join('C:\\Windows\\System32\\cmd')))
        self.assertIsNotNone(util.find_tool_on_path(os.path.join('C:\\Windows\\System32\\cmd.exe')))
        self.assertIsNone(util.find_tool_on_path('cmd-not-fnd'))
        self.assertIsNone(util.find_tool_on_path(os.path.join('C:\\Windows\\System32\\cmd-not-fnd.exe')))

    @unittest.skipIf(sys.platform.startswith('win'), "requires linux or macosx")
    def test_find_tool_on_path(self):
        ls_abs = os.path.join('/usr', 'bin', 'gcc')
        self.assertEqual(ls_abs, util.find_tool_on_path('gcc'))
        self.assertEqual(ls_abs, util.find_tool_on_path(ls_abs))
        self.assertIsNone(util.find_tool_on_path('gcc-not-fnd'))
        self.assertRaises(Exception, util.find_tool_on_path, 'gcc-not-fnd', True)
        self.assertIsNone(util.find_tool_on_path(ls_abs + '-not-fnd'))
        self.assertRaises(Exception, util.find_tool_on_path, ls_abs + '-not-fnd', True)

    def test_normalize_path(self):
        if sys.platform.startswith('win'):
            self.assertEqual(os.path.join(r'C:\qt\4.5.0'), util.normalize_path("c:/qt/4.5.0/"))
        else:
            self.assertEqual('/bin', util.normalize_path("/usr/../bin/"))

