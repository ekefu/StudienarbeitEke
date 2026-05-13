Simulation für Teleoperationsscenario

Im Rahmen der Studienarbeit von Furkan Eke
________________________________________________

Die Softwarepaket beinhaltet;

- Human Hand- und Roboter Handmodelle,
- Deren Abmessungen und Vorwärts- Kinematiklösungen,
- Die Roboterhand Rückwärtslösung Skript und eine 
  resultierende Rückwärtslösung .mat Datei,

und

- Den Skript "initials_for_robotics.m" welche den 
  Startpunkt für die Simulation darstellt. Wähle das 
  Simulationsmodus (binär; es darf nur eine von folgenden
  modi "1" sein)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First, choose here the type of simulation you want to experience:

	hhand_fwd_demo_mode  = 0;  % Shows a linear (linear joint angle) closing 
               		           % demonstration of the human hand model.
                         
	bhand_fwd_demo_mode  = 0;  % Shows a linear (linear joint angle) closing 
               		           % demonstration of the barrett hand model.
                         
	bhand_rwl_demo_mode  = 0;  % Representative backward kinematics demonstration; 
               		           % does not show the generalization ability of the network.
                         
	%bhand_rwl_train_mode = 0;  % Trains a bhand backward solution network 

	teleopera_demo_mode  = 1;  % Shows the teleoperation system simulation. 

	mapper_optimise_mode = 0;  % Shows the teleoperation system simulation 
               		           % with mapper parameter optimisation algorithm together. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Die "bhand_rwl_train_mode" ist auskommentiert, da, es für schnelle anschauung 
nicht geeignet ist. Wenn sie die training doch ausführen möchten, rechnen sie 
mit paar Stunden von Rechnenzeit. Wenn das kein problem ist, nehmen Sie die
Kommentar symbol % weg, wählen Sie bhand_rwl_train_mode = 1 und alle andere 0,
dann rufen Sie den Skript "initials_for robotics" erneut auf.