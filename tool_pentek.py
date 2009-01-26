import os


PENTEK_ROOT = os.environ['PENTEK_ROOT']
PENTEK_INCLUDE = PENTEK_ROOT+'/include'

tools = Split("""
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

libpentek = env.Library('pentek', libsources)

Default(libpentek)

thisdir = env.Dir('.').srcnode().abspath
def pentek(env):
    env.AppendUnique(CPPPATH   =[thisdir, PENTEK_INCLUDE,])
    env.AppendUnique(CPPDEFINES=['PENTEK_LINUX',])
    env.AppendLibrary('pentek')
    env.Require(tools)

Export('pentek')

