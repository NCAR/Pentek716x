import os
import sys

import eol_scons

# Any environment setting will be the default, but it can be overridden by
# setting the configuration variable.
PENTEK_ROOT = "/opt/Pentek/P7140driver-2.3/Linux"
try:
    PENTEK_ROOT = os.environ['PENTEK_ROOT']
except KeyError:
    pass

variables = eol_scons.GlobalVariables()
variables.AddVariables(PathVariable('PENTEK_ROOT', 'PENTEK_ROOT directory.', 
                                    PENTEK_ROOT))

PENTEK_INCLUDE = os.path.join('$PENTEK_ROOT', 'include')

tools = Split("""
doxygen
""")

env = Environment(tools = ['default'] + tools)

variables.Update(env)
env.AppendUnique(CPPPATH   =[PENTEK_INCLUDE,])
env.AppendUnique(CPPDEFINES=['PENTEK_LINUX',])

libsources = Split("""
p71xx.cpp
p7142.cpp
p7142sd3c.cpp
FilterSpec.cpp
BuiltinFilters.cpp
BuiltinGaussian.cpp
BuiltinKaiser.cpp
SingleMutex.cpp
""")

headers = Split("""
p7142.h
p7142sd3c.h
p71xx.h
BuiltinFilters.h
BuiltinGaussian.h
BuiltinKaiser.h
FilterSpec.h
DDCregisters.h
SingleMutex.h
""")

libpentek = env.Library('pentek', libsources)

html = env.Apidocs(libsources + headers, DOXYFILE_FILE = "Doxyfile")

Default(libpentek, html)

thisdir = env.Dir('.').srcnode().abspath
def pentek(env):
    env.AppendUnique(CPPPATH   =[thisdir, PENTEK_INCLUDE,])
    env.AppendUnique(CPPDEFINES=['PENTEK_LINUX',])
    env.AppendLibrary('pentek')
    env.AppendDoxref('pentek')
    env.Require(tools)

Export('pentek')

