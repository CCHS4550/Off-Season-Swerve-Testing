package frc.robot.Util;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

public class SparkUtil {
  // allows us to see if the sparkmax has had an error in the past
  public static boolean stickyFault = false;

  // only takes the sparks value if the spark isn't erroring
  public static void ifOk(
      SparkBase spark, DoubleSupplier doubleSupplier, DoubleConsumer doubleConsumer) {
    double num = doubleSupplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      doubleConsumer.accept(num);
    } else {
      stickyFault = true;
    }
  }

  // same thing but can take in values as a array
  public static void ifOK(
      SparkBase spark, DoubleSupplier[] doubleSuppliers, Consumer<double[]> doubleConsumers) {
    double[] nums = new double[doubleSuppliers.length];
    for (int i = 0; i < doubleSuppliers.length; i++) {
      nums[i] = doubleSuppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        stickyFault = true;
        return;
      }
    }
    doubleConsumers.accept(nums);
  }

  // returns if a spark max has had an error
  public static boolean isOK(SparkBase spark) {
    boolean isFine = true;
    if (spark.getLastError() != REVLibError.kOk) {
      isFine = false;
    }
    return isFine;
  }

  // returns if a generic spark object has had an error
  public static boolean isOK(SparkBase[] spark) {
    boolean isFine = true;
    for (int i = 0; i < spark.length; i++) {
      if (spark[i].getLastError() != REVLibError.kOk) {
        isFine = false;
        break;
      }
    }
    return isFine;
  }

  // repeatedly calls a command until there is no error in running it
  public static void makeItWork(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        stickyFault = true;
      }
    }
  }

  // returns an array of timestamps for simulation to use
  public static double[] getSimulationOdometryTimeStamps() {
    final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] =
          Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
    }

    return odometryTimeStamps;
  }
}
