package frc.robot.auton;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
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

    SendableChooser<Boolean> isCenter = new SendableChooser<>();


    public RunAuton(ArmSubsystem arm, ClawPneumatics clawPneumatics, SwerveSubsystem drive, Pigeon2 gyro, String color) {
        super(arm, clawPneumatics, drive, gyro, color);

        SmartDashboard.putData(isCenter);

    }

    public void testAutonCommands() {
        //super.driveDistance(400, 0);
        super.turnAngle(Math.PI);

        // if (isCenter.getSelected()) {
        //     centerCommands();
        // }
        // else {
        //     nonCenterCommands();
        // }
    }

    public void centerCommands() { // if we're in the center and going on the powerstation
        super.setHighClawPreset();
        super.driveDistance(100, Math.PI);
        super.getOnChargeStation();
    }

    public void nonCenterCommands() {
        super.setHighClawPreset();
        super.driveDistance(100, Math.PI); // get out of community
        super.turnAngle(Math.PI);
    }
}