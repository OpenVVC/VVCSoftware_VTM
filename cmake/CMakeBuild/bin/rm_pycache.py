#!/usr/bin/env python
#
# rm_pycache.py
#
from __future__ import print_function

import argparse
import os
import sys
import re
import shutil


class CleanPyCacheApp(object):

    def __init__(self):
        self._verbose = True

    def main(self):
        python_dirs = self._parse_command_line(sys.argv[1:])
        #print("python_dirs", python_dirs)
        for py_dir in python_dirs:
            self._remove_pycache(py_dir)

    def _parse_command_line(self, argv):
        python_dirs = []
        parser = argparse.ArgumentParser()
        parser.add_argument("-q", action="store_true", dest="quiet", default=False,
                            help="suppress messages which cache file will be removed.")
        parser.add_argument("python_dirs", action="store", nargs='+',
                            help="one or more top-level python directories to search for cache files recursively.")
        args = parser.parse_args(argv)
        if args.quiet:
            self._verbose = False
        for dname in args.python_dirs:
            if not os.path.exists(dname):
                print("Path %s does not exist." % dname)
                sys.exit(1)
            if not os.path.isdir(dname):
                print("Path %s is not a directory." % dname)
                sys.exit(1)
            if not os.path.isabs(dname):
                python_dirs.append(os.path.abspath(dname))
            else:
                python_dirs.append(dname)
        return python_dirs

    def _remove_pycache(self, py_dir_root):
        os.chdir(py_dir_root)
        re_pyc_file = re.compile(r'.+\.pyc$', re.IGNORECASE)
        for root, dirs, files in os.walk(py_dir_root):
            if '__pycache__' in dirs:
                if self._verbose:
                    print("rm -rf %s" % os.path.join(root,'__pycache__'))
                shutil.rmtree(os.path.join(root,'__pycache__'))
                dirs.remove('__pycache__')
            for fname in files:
                if re_pyc_file.match(fname):
                    if self._verbose:
                        print("rm %s" % os.path.join(root, fname))
                    os.remove(os.path.join(root, fname))


if __name__ == '__main__':
    app = CleanPyCacheApp()
    app.main()
