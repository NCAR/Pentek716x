This directory contains a modified version of virtexload7140. Pentek 
provided this when we discovered that the official version would not program our P7142revD board. I assume that Pentek will eventually 
roll this into an updated release of their driver package.

This version of virtexload7140.c should be substituted into the Pentek
driver source code, and that code rebuilt so that an updated binary is
created.

Note that they did not do anything to address the issue that the virtexload7142
program never has worked at all.
