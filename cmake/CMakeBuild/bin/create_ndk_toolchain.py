#!/usr/bin/env python
#
# create_ndk_toolchain.py
#
import pyhhi.build.common.util
import pyhhi.cmbuild.app.androidapp

app = pyhhi.cmbuild.app.androidapp.CreateNdkToolchainApp()
pyhhi.build.common.util.exec_main_default_try(app)
