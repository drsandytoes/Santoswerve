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
        targetAngle = delta > Math.PI/2 ? (targetAngle -= Math.PI) : (targetAngle += Math.PI);
    }        
    return new SwerveModuleState(targetSpeed, Rotation2d.fromRadians(targetAngle));
  }

  /**
   * Adjust the target angle to an equivalent angle that is nearest the reference angle.
   * Given the current angle (scopeReference), and the desired target angle (newAngle), compute
   * an equivalent angle that is within pi (180 deg) of the current angle. 
   * @param scopeReference Current angle (radians)
   * @param newAngle Target angle (radians)
   * @return Closest angle within scope (radians)
   */
  private static double nearestEquivalentAngle(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 2.0*Math.PI;

    // Determine the angle range band [lowerBound, upperBound] that scopeReference is in 
    // as a multiple of the [0,2pi] band. 
    if (lowerOffset >= 0) {
        // lowerBound is scopeReference/2pi: how many positive wraps around the circle
        // IOW, the 0 degrees in the multiple of revolutions we're in
        // e.g., if we were doing math in degrees and the scope reference was 365 deg:
        //   lowerOffset = 5
        //   lowerBound = 360
        //   upperBound = 720
        //   we're logically in the 360-720 band
        lowerBound = scopeReference - lowerOffset;
        // upperBound is lowerBound + 2pi
        upperBound = scopeReference + (2.0*Math.PI - lowerOffset);
    } else {
        // We're in a negative band. And modulus with negative numbers may surprise you.
        // -90%360 = +270, not -90. To find the modulus, you find the largest integer less than
        // or equal to -90/360 = -0.25, which would be -1. Now, with -90%360, we have:
        // 360 * (-1) + <mod> = -90 => <mod> = 360 -  90 => <mod> = 270. That makes sense because
        // -90 and 270 are 360 deg apart. 
        // So here, if we were doing math in degrees and the scope reference was -5 deg:
        //   lowerOffset = 355
        //   upperBound = -5 - 355 = -360
        //   lowerBound = -5 - (360 + 355) = -720
        // MDS: This seems wrong. Wouldn't we want to be in the [-360, 0] band?
        // And if the scope reference was -450, we'd compute:
        //   lowerOffset = 270
        //   upperBound = -450 - 270 = -720
        //   lowerBound = -450 - (360 + 270) = -450 - 630 = -1080
        // upperBound = scopeReference - lowerOffset;
        // lowerBound = scopeReference - (2.0*Math.PI + lowerOffset);

        // MDS: Shouldn't this be the same as the positive case?
        // So here, if we were doing math in degrees and the scope reference was -5 deg:
        //   lowerOffset = 355
        //   lowerBound = -5 - 355 = -360
        //   upperBound = -5 + (360 - 355) = 0
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + (2.0*Math.PI - lowerOffset);
        // And if the scope reference was -450, we'd compute:
        //   lowerOffset = 270
        //   lowerBound = -450 - 270 = -720
        //   upperBound = -450 + (360 - 270) = -450 + 90 = --360
    }

    // Adjust newAngle so that it's in the range [lowerBound, upperBound]
    // IOW, get newAngle into the same [0,360] band as scopeReference
    while (newAngle < lowerBound) {
        newAngle += 2.0*Math.PI;
    }
    while (newAngle > upperBound) {
        newAngle -= 2.0*Math.PI;
    }

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
