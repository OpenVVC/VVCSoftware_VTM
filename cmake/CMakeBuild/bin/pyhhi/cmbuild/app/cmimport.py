from __future__ import print_function

import sys
import os
import re
import argparse
import shutil
import subprocess

import pyhhi.build.common.system as system
import pyhhi.build.common.ver as ver


class CmPackageFileInfo(object):
    def __init__(self, cm_file, dst_dir):
        self.cm_file = cm_file
        self.dst_dir = dst_dir


class CmSvnImporter(object):
    def __init__(self, cm_download_dir, svn_import_dir, dry_run=False, cm_version=None):
        self._cm_download_dir = cm_download_dir
        self._svn_import_dir = svn_import_dir
        self._dry_run = dry_run
        self._cm_version = None
        self._re_axprotector = re.compile(r'axprotector', re.IGNORECASE)
        self._re_fname_ignore = re.compile(r'(.*\.tar\.gz)|(CmSamples.*\.dmg)|(CodeMeterSDK32\.exe)', re.IGNORECASE)
        self._re_fname_doc = re.compile(r'.*\.pdf$', re.IGNORECASE)
        self._re_fname_axdev = re.compile(r'(axprotector-dev_)|(axprotector-devel-)', re.IGNORECASE)
        self._re_fname_dev = re.compile(r'(codemeter64-dev)|(codemeter-dev)|(codemetersdk)|(cmdevkit_)', re.IGNORECASE)

        # and perform an inventory to prepare the import later on.
        self._pkg_file_list = self._inventory(cm_download_dir)
        if cm_version:
            self._cm_version = cm_version

    def cm_import(self, svn_url):
        assert self._cm_version is not None
        dst_dir_base = os.path.join(self._svn_import_dir, ver.version_tuple_to_str(self._cm_version))

        # remove the destination directory to get rid of any existing files
        if os.path.exists(dst_dir_base):
            shutil.rmtree(dst_dir_base)
        os.makedirs(dst_dir_base)

        for file_info in self._pkg_file_list:
            # print("Copying " + file_info.cm_file + " -> " + file_info.dst_dir)
            dst_dir = os.path.join(dst_dir_base, file_info.dst_dir)
            if not os.path.exists(dst_dir):
                os.makedirs(dst_dir)
            print("Copying " + os.path.basename(file_info.cm_file) + " -> " + dst_dir)
            shutil.copy(file_info.cm_file, dst_dir)

        svn_import_args = ['svn', 'import', '--no-ignore', '-m']
        # add a standard import comment
        svn_import_args.append("cmsvn_import.py: importing CodeMeter software " + ver.version_tuple_to_str(self._cm_version))
        svn_import_args.append(self._svn_import_dir)
        svn_import_args.append(svn_url)

        # print("svn import command:", svn_import_args)
        # print("cmsvn_import.py: starting the import ... ")
        if self._dry_run:
            print("\n*** DRY RUN ***\n" + ' '.join(svn_import_args) + "\n")
        else:
            retv = subprocess.call(svn_import_args)
            if retv != 0:
                raise Exception("svn import failed, please contact technical support.")

            if os.path.exists(self._svn_import_dir):
                shutil.rmtree(self._svn_import_dir)

    def _inventory(self, cm_download_dir):
        if not os.path.exists(cm_download_dir):
            raise Exception("The CodeMeter download directory '" + cm_download_dir + " does not exist.")

        pkg_file_list = []
        category_dict = {'rt': 'CodeMeterRuntime', 'dev': 'CodeMeterDev', 'axrt': 'AxProtectorRuntime', 'axdev': 'AxProtectorDev'}
        re_version_fname = re.compile(r'codemeter6?4?_([0-9.]+)_amd64.deb')
        re_admin_guide = re.compile(r'CodeMeter_AdminManual')
        for fname in os.listdir(cm_download_dir):
            if os.path.isdir(os.path.join(cm_download_dir, fname)):
                if self._re_axprotector.match(fname):
                    ax_download_dir = os.path.join(cm_download_dir, fname)
                    for fname2 in os.listdir(ax_download_dir):
                        # print("processing axprotector file " + fname2)
                        category = self._get_file_category(fname2)
                        if category == 'ignore':
                            continue
                        if os.path.isfile(os.path.join(ax_download_dir, fname2)):
                            if category == 'doc':
                                file_info = CmPackageFileInfo(os.path.join(ax_download_dir, fname2), os.path.join(category_dict['axdev'], 'doc'))
                                pkg_file_list.append(file_info)
                            else:
                                # print("processing axprotector file " + fname2)
                                # analyze the axprotector filename
                                (platform, arch) = self._get_target_platform(fname2)
                                if platform:
                                    file_info = CmPackageFileInfo(os.path.join(ax_download_dir, fname2), os.path.join(category_dict[category], platform, arch))
                                    pkg_file_list.append(file_info)
                                else:
                                    print("CmSvnImporter: warning: ignoring file " + os.path.join(ax_download_dir, fname2))
            else:
                category = self._get_file_category(fname)
                if category == 'ignore':
                    continue
                if os.path.isfile(os.path.join(cm_download_dir, fname)):
                    if category == 'doc':
                        if re_admin_guide.match(fname):
                            file_info = CmPackageFileInfo(os.path.join(cm_download_dir, fname), os.path.join(category_dict['rt'], 'doc'))
                        else:
                            file_info = CmPackageFileInfo(os.path.join(cm_download_dir, fname), os.path.join(category_dict['dev'], 'doc'))
                        pkg_file_list.append(file_info)
                    else:
                        # analyze the codemeter filename
                        (platform, arch) = self._get_target_platform(fname)
                        if platform:
                            re_match = re_version_fname.match(fname)
                            if re_match:
                                self._cm_version = ver.version_tuple_from_str(re_match.group(1))
                            # print("CmSvnImporter: inventory: " + fname + " -> " + platform + "/" + arch)
                            # analyze the file type
                            if platform == 'all':
                                pass
                                assert False
                            else:
                                # binary file
                                if arch == 'any':
                                    file_info = CmPackageFileInfo(os.path.join(cm_download_dir, fname), os.path.join(category_dict[category], platform))
                                else:
                                    file_info = CmPackageFileInfo(os.path.join(cm_download_dir, fname), os.path.join(category_dict[category], platform, arch))
                                    # print("file_info: " + file_info.cm_file + " -> " + file_info.dst_dir)
                            pkg_file_list.append(file_info)
                        else:
                            print("CmSvnImporter: warning: ignoring file " + os.path.join(cm_download_dir, fname))
        return pkg_file_list

    def _get_target_platform(self, fname):
        platform = None
        arch = None
        (fname_root, fname_ext) = os.path.splitext(fname)
        fname_ext = fname_ext.lower()

        if fname_ext in ['.pdf', '.gz']:
            platform = 'all'
            arch = 'any'
        elif fname_ext == '.exe':
            platform = 'windows'
            re_win32 = re.compile(r'.*32\.exe$', re.IGNORECASE)
            re_win64 = re.compile(r'.*64\.exe$', re.IGNORECASE)
            if re_win32.match(fname):
                arch = 'x86'
            elif re_win64.match(fname):
                arch = 'x86_64'
            else:
                arch = 'any'
        elif (fname_ext == '.dmg') or (fname == 'AxProtectorMacX'):
            platform = 'macosx'
            arch = 'x86_64'
        elif (fname_ext == '.rpm') or (fname_ext == '.deb'):
            platform = 'linux'
            re_linux_x86 = re.compile(r'.*i386\....$')
            re_linux_x86_64 = re.compile(r'.*(amd64|x86_64)\....$')
            # print("fname: " + fname)
            if re_linux_x86.match(fname):
                arch = 'x86'
            elif re_linux_x86_64.match(fname):
                arch = 'x86_64'
            else:
                assert False

        return (platform, arch)

    def _get_file_category(self, fname):
        if self._re_fname_ignore.match(fname):
            category = 'ignore'
        elif self._re_fname_doc.match(fname):
            category = 'doc'
        elif self._re_axprotector.match(fname):
            # All axprotector packages start with 'axprotector'.
            if self._re_fname_axdev.match(fname):
                category = 'axdev'
            else:
                category = 'axrt'
        else:
            if self._re_fname_dev.match(fname):
                category = 'dev'
            else:
                category = 'rt'
        return category


class CmSvnImportApp(object):

    def __init__(self):
        self._sys_info = system.SystemInfo()
        assert not self._sys_info.is_windows()

    def __call__(self):
        dflt_download_dir = os.path.join(self._sys_info.get_home_dir(), 'Downloads', 'CodeMeter')

        parser = argparse.ArgumentParser()

        parser.add_argument("--download-dir", action="store", dest="download_dir", metavar="dir", default=dflt_download_dir,
                            help="the download directory containing CodeMeter packages [default: $HOME/Downloads/CodeMeter]")
        parser.add_argument("--dry-run", action="store_true", dest="dry_run", default=False,
                            help="copy the downloaded CodeMeter files into import area but don't issue the final svn import [default: off]")
        parser.add_argument("--cm-version", action="store", dest="cm_version",
                            help="overrides the CodeMeter version which is picked up from the ubuntu package file by default.")
        parser.add_argument("codemeter_version_folder", action="store",
                            help="codemeter version folder to import into subversion.")

        args = parser.parse_args()
        if args.cm_version:
            # convert into a version tuple
            args.cm_version = ver.version_tuple_from_str(args.cm_version)
        cm_download_dir = os.path.abspath(os.path.join(args.download_dir, args.codemeter_version_folder))
        if not os.path.exists(cm_download_dir):
            print("cmsvn_import.py: error: the download directory '" + cm_download_dir + "' does not exist.")
            sys.exit(1)

        print("download dir:", cm_download_dir)
        svn_import_dir = os.path.join(os.getcwd(), 'svn_import')
        svn_importer = CmSvnImporter(cm_download_dir, svn_import_dir, args.dry_run, args.cm_version)
        svn_importer.cm_import("https://bslinux3.hhi.fraunhofer.de/svn/svn_CodeMeterSoftware/branches/import")
