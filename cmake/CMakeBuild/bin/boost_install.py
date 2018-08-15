#!/usr/bin/env python
#
# boost_install.py
#
import pyhhi.build.common.util
import pyhhi.cmbuild.app.boost

app = pyhhi.cmbuild.app.boost.BoostInstallApp()
pyhhi.build.common.util.exec_main_default_try(app)
