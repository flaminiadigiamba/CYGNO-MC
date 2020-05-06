#PBS -j oe  

#!/bin/bash
#this is a very simple example of submission script
#####for ROOT
source /storage/local/exp_soft/cygnorm3/root-v6-12-06-install/bin/thisroot.sh
#### for geant4
source /storage/local/exp_soft/cygnorm3/geant4-v10.5.1-install/bin/geant4.sh
#### CADMesh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/storage/local/exp_soft/cygnorm3/CADMesh-install/lib/

cd /storage/local/home/cygnorm3/$USER/CYGNO/CYGNO-MC-build/
./CYGNO ../CYGNO-MC/macro/CYGNOtest_surface_gamma.mac > log
