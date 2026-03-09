package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

    public class Intake extends SubsystemBase {
        private final Raiser raiser;
        private final Spinner spinner;
       ;
    
        public Intake() {
            raiser = new Raiser();
            spinner = new Spinner();
           
        }
    
        public Raiser getRaiser() {
            return raiser;
        }
    
        public Spinner getSpinner() {
            return spinner;
        }
    
        
    }
    


