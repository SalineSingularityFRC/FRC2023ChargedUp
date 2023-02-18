package frc.robot.auton;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class RunAuton extends AutonControlScheme {
    
    // SendableChooser<Boolean> doDrive = new SendableChooser<>();
    // SendableChooser<Boolean> doShoot = new SendableChooser<>();
    // SendableChooser<Boolean> doDriveReverse = new SendableChooser<>();
    // SendableChooser<Boolean> doSearch = new SendableChooser<>();
    // SendableChooser<Boolean> doShoot2 = new SendableChooser<>();
    // SendableChooser<Boolean> testD = new SendableChooser<>();
    // SendableChooser<Boolean> doFixedAuton = new SendableChooser<>();
    // SendableChooser<Boolean> doMainModularAuton = new SendableChooser<>();


    public RunAuton(ClawPneumatics clawPneumatics, SwerveSubsystem drive, String color) {
        super(clawPneumatics, drive, color);
    }

    public void TestAutonCommands() {
        super.driveDistance(400, 0);
        //super.turnAngle(0);
    }
}