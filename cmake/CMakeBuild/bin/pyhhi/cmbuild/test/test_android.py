from __future__ import print_function

import os.path
import re
import string
import sys
import unittest

import pyhhi.build.common.android as android


@unittest.skipUnless(sys.platform.startswith('linux'), "requires linux")
class AndroidTestCase(unittest.TestCase):

    def setUp(self):
        self._re_ndk_root = re.compile(r'android-ndk-r(\d+)([a-z]?)$')
        self._ndk_root_dir_list = ['android-ndk-r10c', 'android-ndk-r10', 'android-ndk-r10e']

    def test_re_ndk_root(self):
        re_ndk_root = self._re_ndk_root
        ndk_root_dir_list = self._ndk_root_dir_list
        for d in ndk_root_dir_list:
            self.assertTrue(re_ndk_root.match(d))

    def test_ndk_finder(self):
        ndk_finder = android.NdkFinder()
        self.assertTrue(os.path.exists(ndk_finder.get_ndk_root()))
        api_level = ndk_finder.get_api_level_from_platform('android-21')
        # print("api_level:", api_level)
        self.assertEqual(21, ndk_finder.get_api_level_from_platform('android-21'))

    def _get_ndk_patch_level_from_string(self, patch_level_str):
        l = string.ascii_lowercase.index(patch_level_str)
        return l
