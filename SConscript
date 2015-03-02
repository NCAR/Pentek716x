# Execute this SConscript to build the binaries in the 'test' directory.
# Building of the library is handled via tool_pentek.py, which is loaded when
# the SCons tool 'pentek' is required by a build environment.
env = Environment(tools = ['default'], GLOBAL_TOOLS = ['prefixoptions'])

SConscript(dirs = 'test')
