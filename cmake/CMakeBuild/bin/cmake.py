#!/usr/bin/env python
#
# cmake.py
#

import pyhhi.build.common.util
import pyhhi.cmbuild.app.cmk


app = pyhhi.cmbuild.app.cmk.CMakeLauncherApp()
pyhhi.build.common.util.exec_main_default_try(app)
