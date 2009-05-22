import os
import sys

try:
    PENTEK_ROOT = os.environ['PENTEK_ROOT']
except KeyError:
    print "tool_pentek: environment variable PENTEK_ROOT must be set!"
    sys.exit(1)
    
PENTEK_INCLUDE = os.path.join(PENTEK_ROOT, 'include')

tools = Split("""
doxygen
""")

env = Environment(tools = ['default'] + tools)

env.AppendUnique(CPPPATH   =[PENTEK_INCLUDE,])
env.AppendUnique(CPPDEFINES=['PENTEK_LINUX',])

libsources = Split("""
p71xx.cpp
p7140.cpp
p7142.cpp
p7142hcr.cpp
FilterSpec.cpp
BuiltinFilters.cpp
BuiltinGaussian.cpp
BuiltinKaiser.cpp
""")

headers = Split("""
p7140.h
p7142.h
p7142hcr.h
p71xx.h
BuiltinFilters.h
BuiltinGaussian.h
BuiltinKaiser.h
FilterSpec.h
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

