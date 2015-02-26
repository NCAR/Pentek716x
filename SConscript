# If this pentek directory is the top level checkout, you can create a 
# symbolic link SConstruct -> SConscript to allow for building this 
# tree by itself. The next few lines are boilerplate needed to also use of 
# this SConscript as a top-level SConstruct.

import eol_scons
import SCons
 
options = eol_scons.GlobalOptions()

env = Environment(tools = ['default'], GLOBAL_TOOLS = ['prefixoptions'])
 
# End of SConstruct boilerplate

SConscript(dirs = 'test')
