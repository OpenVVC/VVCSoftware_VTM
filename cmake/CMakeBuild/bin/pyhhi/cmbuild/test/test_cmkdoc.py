from __future__ import print_function

import filecmp
import os.path
import shutil
import tempfile
import unittest

import pyhhi.cmbuild.app.cmkdocapp as cmkdocapp
import pyhhi.cmbuild.test.helper as helper


class CMakeDocUtilTestCase(unittest.TestCase):

    def setUp(self):
        (filenm, ext) = os.path.splitext(os.path.basename(__file__))
        self._test_data_dir = os.path.join(helper.get_test_data_top_dir(), "py_unittests", filenm)
        assert os.path.exists(self._test_data_dir)
        self._remove_temp_dir = True

    def test_0_cmake_docutil(self):
        pass

    def test_1_cmake_docutil(self):
        #print("\n")

        temp_dir = tempfile.mkdtemp()
        #print("Using temp_dir=" + temp_dir)
        try:
            app = cmkdocapp.CMakeDocUtilApp()
            output_file = os.path.join(temp_dir, 'test.rst')
            input_file = os.path.join(self._test_data_dir, 'cmake-modules.7.rst')
            argv = ['-f', input_file, '-m', 'BBuildEnvLldb', '-o', output_file]
            app.main(argv)
            self.assertTrue(filecmp.cmp(output_file, os.path.join(self._test_data_dir, 'test1.rst'), False))

            app = cmkdocapp.CMakeDocUtilApp()
            argv = ['-f', output_file, '-m', 'FindCodeMeter', '-m', 'BBuildEnvLldb']
            app.main(argv)
            self.assertTrue(filecmp.cmp(output_file, os.path.join(self._test_data_dir, 'test2.rst'), False))

            app = cmkdocapp.CMakeDocUtilApp()
            argv = ['-f', output_file, '-m', 'FindCodeMeterx', '-m', 'BBuildEnvLldbx', '-s', 'My Extensions']
            app.main(argv)
            self.assertTrue(filecmp.cmp(output_file, os.path.join(self._test_data_dir, 'test3.rst'), False))

            # revert to original -> removing two sections
            app = cmkdocapp.CMakeDocUtilApp()
            argv = ['-f', output_file, '-R']
            app.main(argv)
            self.assertTrue(filecmp.cmp(output_file, os.path.join(self._test_data_dir, 'cmake-modules.7.rst'), False))

            app = cmkdocapp.CMakeDocUtilApp()
            argv = ['-f', output_file, '-m', 'FindCodeMeter', '-m', 'BBuildEnvLldb']
            app.main(argv)
            self.assertTrue(filecmp.cmp(output_file, os.path.join(self._test_data_dir, 'test2.rst'), False))

            # remove a single section
            app = cmkdocapp.CMakeDocUtilApp()
            argv = ['-f', output_file, '-r']
            app.main(argv)
            self.assertTrue(filecmp.cmp(output_file, os.path.join(self._test_data_dir, 'cmake-modules.7.rst'), False))
        finally:
            #pass
            if self._remove_temp_dir and os.path.exists(temp_dir):
                shutil.rmtree(temp_dir)

