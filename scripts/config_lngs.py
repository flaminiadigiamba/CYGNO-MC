#######################################################################
## THIS PART OF CODE MAY NEED SOME CHANGES TO MATCH THE USER'S NEEDS ##
#######################################################################

import socket, getpass

#Job properties
MacrosList = ['CYGNOtest.mac'] #This is the list of configuration macros to be submitted. They must have extension .mac and must be located inside the CODEDIR+'macro/' directory
NEvts = ['20000'] #This list has to be the same lenght of MacrosList and specifies the number of events to be simulated for each macro
NEvtsPerJob = '10000' #The maximum number of events per job. If NEvtsPerJob<NEvts, the simulation will be splitted in  NEvts/NEvtsPerJob jobs
MaxNJobs=500 #This depends on the machine cluster used. Some of these system have a maximum limit on the number of jobs that can be submitted
TAG='test1' #this is the reference name of the simulation that you have submitted. The output of the simulation will be contained in a folder with this name

hostname = socket.gethostname()
username = getpass.getuser()

#Software/machine config
GEANT4VERSION='10.5.1'
GEANT4CONFIG=''
ROOTCONFIG=''
QUEUE='normal64'

#Directory paths
CODEDIR='/users/'+username[0]+'/'+username+'/CYGNO/CYGNO-MC/' # this folder contains the code of the simulation and the background information (subdir backgrounds/)
OUTDIR='/nfs/cygno/CYGNO-MC-data/' # this is the folder where results of the simulations are stored: the folder will contain BSUBOUTDIR and pbs_logs subdirectories containing the outputs of simulation and the logs, respectively. 
PBSOUTDIR='pbs_outputs/' #For hist_maker.py this is the folder where the simulation root files are loaded from. The output of hist_maker.py is saved in subfolders of this directory: output/ (root files with histograms), pbs/ (script submitted and logs)
TMPDIR='/nfs/scratch/'+username #this is a temporary directory where the simulation is compiled per each job. It is then cleaned up at the end of each job
ANADIR='' # this folder contains the code to make histograms
BUILDDIR='/users/'+username[0]+'/'+username+'/CYGNO/CYGNO-MC-build/' 


ISOTOPES={'K40':['K40'],'Co60':['Co60'],'Cs137':['Cs137'],'U238':['Pb214','Bi214','Pb210','Bi210','U238','Th234','Pa234','Pa234m','U234','Th230','Ra226','Rn222','Po218','Po218','Po214','Tl210','Po210','Tl206'],'Th232':['Ac228','Pb212','Bi212','Tl208','Th232','Ra228','Th228','Ra224','Rn220','Po216','Po212'],'U235':['Pb211','Tl207'],'C14':['C14'],'Be7':['Be7'],'Ar39':['Ar39'],'Kr85':['Kr85'],'H3':['H3'],'Sn113':['Sn113']} # List of isotopes that are checked for by the CYGNOAnalysis code. They are grouped under the name of the key

MATERIALS={'Pb':'G4_Pb', 'PE':'G4_POLYETHYLENE', 'Air':'G4_AIR', 'Rock':'LNGSRock', 'Cu':'G4_Cu', 'Steel':'G4_STAINLESS-STEEL', 'Teflon':'G4_TEFLON','Water':'Water','Acrylic':'Acrylic','CYGNO_gas':'CYGNO_gas'} #List of known materials. Used to give name to the outuput file and (obsolete) to identify shielding layers
