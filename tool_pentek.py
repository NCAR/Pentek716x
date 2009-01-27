import os


PENTEK_ROOT = os.environ['PENTEK_ROOT']
PENTEK_INCLUDE = PENTEK_ROOT+'/include'

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
""")

headers = Split("""
p7140.h
p7142.h
p7142hcr.h
p71xx.h
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

