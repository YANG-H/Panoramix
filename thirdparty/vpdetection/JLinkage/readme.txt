JLinkageLib
Author: Roberto Toldo (roberto.toldo@univr.it)

------------- License --------------------
Feel free to use the code for any academic project.
If you publish any work based on this code, please cite (any of) the original paper(s):
- R. Toldo and A. Fusiello. Robust multiple structures estimation with j-linkage. In Andrew Zisserman David Forsyth, Philip Torr, editor, Proceedings of European Conference on Computer Vision (ECCV 2008), volume 5302, pages 537–547, 2008.
- R. Toldo and A. Fusiello. Real-time Incremental J-Linkage for Robust Multiple Structures Estimation. 3DPVT 2010 Conference, to appear.

If your planning to use the code for a commercial product, please ask me first: roberto.toldo@univr.it .

------------- Brief Package Description -------------------
This archive contains the precompiled library/mex files for win32/64 as well as the Visual Studio 2008 project files.
The code is splitted in 3 sub-projects:
JLinkageLib - Use this one if you want to embed JLinkage inside your c/c++ project.
The code is structured in two main classed: Random Sampler (RandomSampler.h/cpp) class and a JLinkage class(JLinkage.h/cpp).
The code should be self-understandable.
RandomSamplerMex/ClusterizeMex - interfaces for mex/matlab. If you need to use jlinkage inside matlab you need to call these mex files from matlab. 
For a jump start have a look at the /bin/runme.m file.


------------- Other libraries included -------------------
The code makes use of two other opensource library. Please refer to their website for the usage license:
Bitmagic: http://sourceforge.net/projects/bmagic/
Kdtree++: http://libkdtree.alioth.debian.org/
