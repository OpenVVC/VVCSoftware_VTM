from __future__ import print_function

import platform
import argparse
import unittest
import os
import sys
import glob
import pyhhi.build.common.util as util


class UnitTestApp(object):

    def __init__(self, ):
        pass

    def __call__(self):

        parser = argparse.ArgumentParser()
        util.app_args_add_log_level(parser)
        parser.add_argument("-v", action="store", type=int, dest="verbosity_level", default=2,
                            help="change the default verbosity level of the python unit test framework.")
        parser.add_argument("--with-android", action="store_true", dest="with_android", default=False,
                            help="include android tests.")
        parser.add_argument("--with-cuda", action="store_true", dest="with_cuda", default=False,
                            help="include android tests.")
        parser.add_argument("--with-vlc", action="store_true", dest="with_vlc", default=False,
                            help="include VLC tests.")
        parser.add_argument("--with-all", action="store_true", dest="with_all", default=False,
                            help="include all optional tests.")
        parser.add_argument("unit_tests", action="store", nargs='*',
                            help="specify one or more unit tests to run. By default all tests will be run.")
        args = parser.parse_args()

        # configure the python logger asap
        util.app_configure_logging(args.log_level)

        # print("verbosity_level:", args.verbosity_level)

        # get the workspace
        top_dir = util.get_top_dir()
        # change to top_dir
        os.chdir(top_dir)

        # get the boost build script directory
        script_dir = util.get_script_dir()

        test_loader = unittest.TestLoader()

        # All unit tests live in the package bb.test
        test_package_prefix = 'pyhhi.build.test'

        py_module_nm_list = []
        if args.unit_tests:
            # One or more test modules are specified.
            for arg in args.unit_tests:
                if arg.startswith('test_'):
                    py_module_nm_list.append(arg)
                else:
                    py_module_nm_list.append('test_' + arg)
        else:
            # Construct the default test suite containing all test modules.
            # test_suite = test_loader.discover(test_package_prefix, 'test_*.py')
            for file in glob.glob(os.path.join(script_dir, 'pyhhi', 'build', 'test', 'test_*.py')):
                (py_module_nm, ext) = os.path.splitext(os.path.basename(file))
                py_module_nm_list.append(py_module_nm)
            for file in glob.glob(os.path.join(script_dir, 'pyhhi', 'cmbuild', 'test', 'test_*.py')):
                (py_module_nm, ext) = os.path.splitext(os.path.basename(file))
                py_module_nm_list.append(py_module_nm)
            py_module_nm_list = self._filter_module_list(args, py_module_nm_list)

        test_module_names = []
        for prefix in ['pyhhi.build.test', 'pyhhi.cmbuild.test']:
            #prefix_path = os.path.join(script_dir, prefix.split('.'))
            prefix_path = os.path.join(script_dir, prefix.replace('.', os.sep))
            for py_module_nm in py_module_nm_list:
                if os.path.exists(os.path.join(prefix_path, py_module_nm + '.py')):
                    test_module_names.append(prefix + '.' + py_module_nm)
        #test_module_names = [test_package_prefix + '.' + x for x in py_module_nm_list]
        print("test module list: ", test_module_names)

        test_suite = test_loader.loadTestsFromNames(test_module_names)
        # and run tests ...
        test_runner = unittest.TextTestRunner(verbosity=args.verbosity_level)
        test_runner.run(test_suite)

    def _filter_module_list(self, args, module_list):
        module_list_1 = []
        for py_module in module_list:
            if 'android' in py_module:
                if args.with_android or args.with_all:
                    module_list_1.append(py_module)
                else:
                    continue
            elif 'cuda' in py_module:
                if args.with_cuda or args.with_all:
                    module_list_1.append(py_module)
                else:
                    continue
            elif 'vlc' in py_module:
                if args.with_vlc or args.with_all:
                    module_list_1.append(py_module)
                else:
                    continue
            else:
                module_list_1.append(py_module)
        # pass 2: filter out test_jam_* and put them at the end of the module list
        module_list_2 = []
        jam_module_list = []
        for py_module in module_list_1:
            if py_module.startswith('test_jam'):
                if py_module == 'test_jam_if':
                    # somewhat a prerequisite for all other test_jam modules.
                    jam_module_list.insert(0, py_module)
                else:
                    jam_module_list.append(py_module)
            else:
                module_list_2.append(py_module)
        module_list_2.extend(jam_module_list)
        return module_list_2
