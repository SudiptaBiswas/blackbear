##############################

#!/bin/bash

#PBS -N A1-001a

#PBS -l walltime=12:00:00

#PBS -l select=1:ncpus=4:mpiprocs=4

#PBS -P neams

cd $PBS_O_WORKDIR

module purge
module load pbs

## modules for falcon
# module load use.moose PETSc/3.10.5-GCC

# modules for sawtooth
module load use.moose
module load git/2.25.0-gcc-9.2.0-v4mv
module load cmake
module load binutils
module load gcc/5.4.0-gcc-9.2.0-m6ks
module load mvapich2/2.3.3-gcc-5.4.0
module load python/3.7.4-gcc-5.4.0-seda

#module purge
#module load pbs
#module load mvapich2 cmake
#module load use.moose PETSc/3.11.4-GCC

#module load PETSc/3.10.3-foss-2018b
#module load MVAPICH2/2.2-GCC-5.4.0-2.28
#module load PETSc
#export OMP_NUM_THREADS=1

#which mpiexec

mpiexec $HOME/projects/sawtooth/concrete/blackbear/blackbear-opt -i A1-001a.i

#############################
date     #! print date and time
