import os
import sys

import eol_scons

# Any environment setting will be the default, but it can be overridden by
# setting the configuration variable.

variables = eol_scons.GlobalVariables()

tools = Split("""
boost_thread
logx
ReadyFlow71620
doxygen
""")

env = Environment(tools = ['default'] + tools)

variables.Update(env)

libsources = Split("""
p716x.cpp
p716x_sd3c.cpp
p716xDn.cpp
p716xDn_sd3c.cpp
p716xUp.cpp
FilterSpec.cpp
BuiltinFilters.cpp
BuiltinGaussian.cpp
BuiltinKaiser.cpp
SingleMutex.cpp
""")

headers = Split("""
p716x.h
p716x_sd3c.h
p716xDn.h
p716xDn_sd3c.h
p716xUp.h
BuiltinFilters.h
BuiltinGaussian.h
BuiltinKaiser.h
FilterSpec.h
DDCregisters.h
SingleMutex.h
""")

libpentek = env.Library('Pentek716x', libsources)
Default(libpentek)

env['DOXYFILE_DICT'].update({'PROJECT_NAME':'Pentek716x'})
html = env.Apidocs(libsources + headers)
Default(html)

thisdir = env.Dir('.').srcnode().abspath
def Pentek716x(env):
    env.AppendUnique(CPPPATH = [thisdir])
    env.AppendLibrary('Pentek716x')
    env.AppendDoxref('Pentek716x')
    env.Require(tools)

Export('Pentek716x')

# Make sure the test programs are built before we leave...
SConscript('test/SConscript')
