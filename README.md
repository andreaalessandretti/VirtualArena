
<div align="center"><img src=".\logo.jpg" width="200 align="middle"> </div>

VirtualArena is Object-Oriented Matlab Toolkit for Control Design and System Simulation.

## Getting Started
To install the tollkit, open Matlab on the folder where you want to create the VirtualArena folder and run the following code

```matlab
urlwrite('https://github.com/andreaalessandretti/VirtualArena/archive/master.zip','master.zip');
unzip('master.zip');
movefile('VirtualArena-master','VirtualArena');
cd VirtualArena/;
addPathsOfVirtualarena;
```

The folder `VirtualArena/exampels` presents a list of examples that are described in the paper 

>[Alessandretti, A., Aguiar, A. P., & Jones, C. N. (2017). VirtualArena : An Object-Oriented MATLAB Toolkit for Control System Design and Simulation. In Proc. of the 2017 International Conference on Unmanned Aircraft Systems (ICUAS). Miami, USA.](./docs/ICUAS17_VirtualArena.pdf)

and in the slides [here](./docs/ICUAS17_slides.pdf).

The following BibTeX entry can be used to cite the toolkit:

```
@inproceedings{Alessandretti2017,
address = {Miami, USA},
author = {Alessandretti, A. and Aguiar, A. P. and Jones, C. N.},
booktitle = {Proc. of the 2017 International Conference on Unmanned Aircraft Systems (ICUAS)},
title = {{VirtualArena : An Object-Oriented MATLAB Toolkit for Control System Design and Simulation}},
year = {2017}
}

```
