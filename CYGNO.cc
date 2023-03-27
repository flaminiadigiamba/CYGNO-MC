#include "CYGNODetectorConstruction.hh"
#include "CYGNOPhysicsList.hh"
#include "CYGNOPrimaryGeneratorAction.hh"
#include "CYGNOActionInitialization.hh"
#include "G4EmProcessOptions.hh"
#include "G4PhysListFactory.hh"
#include "G4StepLimiterPhysics.hh"
//#ifdef G4MULTITHREADED
//#include "G4MTRunManager.hh"
//#else
#include "G4RunManager.hh"
//#endif

#include "G4Timer.hh"
#include "Randomize.hh"

#include "G4UImanager.hh"
#include "G4UIterminal.hh"
#include "G4UItcsh.hh"
//#include "Shielding.hh"

#ifdef G4VIS_USE
#include "G4VisExecutive.hh"
#endif

#ifdef G4UI_USE
#include "G4UIExecutive.hh"
#endif

int main(int argc,char** argv)
{
  //Instantiate the G4Timer object, to monitor the CPU time spent for 
  //the entire execution
  G4Timer* theTimer = new G4Timer();
  //Start the benchmark
  theTimer->Start();

   //
  // Choose the Random engine
  //
  G4Random::setTheEngine(new CLHEP::RanecuEngine);

  // User Verbose output class
  //
  //  G4VSteppingVerbose* verbosity = new G4VSteppingVerbose;
  //  G4VSteppingVerbose::SetInstance(verbosity);
  
  // Run manager
  
  //#ifdef G4MULTITHREADED
  //G4MTRunManager* runManager = new G4MTRunManager;
  //runManager->SetNumberOfThreads(G4Threading::G4GetNumberOfCores());
  //#else
  G4RunManager * runManager = new G4RunManager;
  //#endif

  // User Initialization classes (mandatory)
  //
  CYGNODetectorConstruction* detector = new CYGNODetectorConstruction;
  runManager->SetUserInitialization(detector);

#ifdef G4VIS_USE
  // visualization manager
  //
  G4VisManager* visManager = new G4VisExecutive;
  visManager->Initialize();
#endif

  // PHYSICS
  // CYGNO physics list is the Shielding physics list with the addition of emlists option4 and of fluo,PIXE,Auger. In a second moment we can add the optical photons and simulate the scintillation
  // For the study of systematics, change Shielding to QGSP_BERT_HP and QGSP_BIC_HP, and the option4 to livermore and penelope.
  CYGNOPhysicsList* physics = new CYGNOPhysicsList;
  physics->RegisterPhysics(new G4StepLimiterPhysics());
  runManager->SetUserInitialization(physics);
   
  // User Action classes
  runManager->SetUserInitialization(new CYGNOActionInitialization(detector));
  
  //CYGNOPrimaryGeneratorAction* myaction = new CYGNOPrimaryGeneratorAction(detector);
  //runManager->SetUserAction(myaction);

  // Initialize G4 kernel
  //
  runManager->Initialize();
      
  // Get the pointer to the User Interface manager
  //
  G4UImanager * UImanager = G4UImanager::GetUIpointer();  

  if (argc!=1)   // batch mode  
    {
     G4String command = "/control/execute ";
     G4String fileName = argv[1];
     UImanager->ApplyCommand(command+fileName);
    }
    
  else           // interactive mode : define visualization and UI terminal
    { 
#ifdef G4UI_USE
      G4UIExecutive * ui = new G4UIExecutive(argc,argv);
#ifdef G4VIS_USE
      UImanager->ApplyCommand("/control/execute vis.mac");     
#endif    
      ui->SessionStart();
      delete ui;
#endif
     
      G4UIsession * session = 0;
#ifdef G4UI_USE_TCSH
      session = new G4UIterminal(new G4UItcsh);      
#else
      session = new G4UIterminal();
#endif
      //

      //      UI->ApplyCommand("/control/execute vis.mac");     
      session->SessionStart();
      delete session;
     
#ifdef G4VIS_USE
      delete visManager;
#endif     
    }

//Stop the benchmark here
theTimer->Stop();
G4cout << "The simulation took: " << theTimer->GetRealElapsed() << " s to run (real time)" << G4endl;

  // Free the store: user actions, physics_list and detector_description are
  //                 owned and deleted by the run manager, so they should not
  //                 be deleted in the main() program !
  //delete analysis;
  //delete runManager;
  //  delete verbosity;

  return 0;
}

