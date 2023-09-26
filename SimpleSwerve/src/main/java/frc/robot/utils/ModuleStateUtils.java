package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ModuleStateUtils {

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing
   * in [0, 2pi]
   *
   * @param desiredState The desired state in radians
   * @param currentAngle The current module angle in radians
   */
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = nearestEquivalentAngle(currentAngle.getRadians(), desiredState.angle.getRadians());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getRadians();

    // We know the targetAngle is within 180 deg of the current angle. If the difference is more than
    // 90 deg, there is a closer angle in the opposite direction we can use if we reverse the 
    // desired speed. For example, suppose the wheel is current at 0 deg, and we want to go forward
    // 1m/s with an angle of 135 deg. Rather than turning the wheel 135 deg to get to 135 deg, we
    // could turn the wheel to -45 (45 deg in the other direction) and drive backward at 1m/s.
    if (Math.abs(delta) > Math.PI/2){
        targetSpeed = -targetSpeed;
        targetAngle = delta > Math.PI/2 ? (targetAngle - Math.PI) : (targetAngle + Math.PI);
    }        
    return new SwerveModuleState(targetSpeed, Rotation2d.fromRadians(targetAngle));
  }

  /**
   * Adjust the target angle to an equivalent angle that is nearest the reference angle.
   * Given the current angle (scopeReference), and the desired target angle (newAngle), compute
   * an equivalent angle that is within pi (180 deg) of the current angle. 
   * @param scopeReference double, current angle (radians)
   * @param newAngle double, target angle (radians)
   * @return double, closest angle within scope (radians)
   */
  private static double nearestEquivalentAngle(double scopeReference, double newAngle) {
    double lowerOffset = scopeReference % 2.0*Math.PI;

    // Determine the angle range band [lowerBound, upperBound] that scopeReference is in 
    // as a multiple of the [0,2pi] band. 

    // First, a note about modulus math on negative numbers.
    // -90 % 360 = +270, not -90! To find the modulus x % y, you find the largest integer z less than
    // or equal to x / y. Then we determine the modulus as the difference x - (z * y).
    // Here we have -90 / 360 = -0.25, so the largest integer less than or equal to -0.25 is -1. 
    // So the modulus is -90 - 360 * (-1) => 360 -  90 => 270. That makes sense because
    // -90 and 270 are 360 deg apart. 

    // lowerBound is scopeReference/2pi: how many positive wraps around the circle
    // IOW, the 0 degrees in the multiple of revolutions we're in
    // e.g., if we were doing math in degrees and the scope reference was 365 deg:
    //   scopeReference = 365           scopeReference = -450
    //   lowerOffset = 5                lowerOffset = 270
    //   lowerBound = 360               lowerBound = -450 - 270 = -720
    //   upperBound = 720               upperBound = -450 + (360 - 270) = -450 + 90 = -360
    //   logica band: [360,720]         logical band: [-720, -360]
    double lowerBound = scopeReference - lowerOffset;
    double upperBound = scopeReference + (2.0*Math.PI - lowerOffset);

    // Adjust newAngle so that it's in the range [lowerBound, upperBound]
    // IOW, get newAngle into the same [0,360] band as scopeReference
    // We take the mod of 2pi to get it in the range [0, 2pi] and then add in the lowerBound.
    newAngle = (newAngle % 2.0*Math.PI) + lowerBound;

    // Now newAngle is in the same 360 degree band as the reference. However, if it's more than 180 degrees apart, 
    // there's a closer angle in the band above or below the one scopReference is in. For example, if the
    // reference angle is 5 degrees, and new angle is 359. -1 degrees is actually closer.
    if (newAngle - scopeReference > Math.PI) {
        // If newAngle is more than pi (180 deg) larger than the reference, substract 2pi (360 deg),
        // as that angle will be less movement
        newAngle -= 2.0*Math.PI;
    } else if (newAngle - scopeReference < -Math.PI) {
        // If newAngle is less than the reference by more than pi (180 deg), add 2pi (360 deg),
        // as that angle will be less movement
        newAngle += 2.0*Math.PI;
    }

    // Return the adjusted angle, which should be within pi (180 deg) of the reference.
    return newAngle;
  }
}
