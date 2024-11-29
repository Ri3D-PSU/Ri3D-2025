package frc.robot;

import static edu.wpi.first.wpilibj.RobotBase.isReal;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Arrays;

public final class Utility {
  private Utility() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Deletes old logs if the free space on the roboRIO is less than the specified amount. Also
   * creates the log directory if it does not exist.
   *
   * @param logDirectory The directory to delete logs from
   * @param minFreeSpace The minimum amount of free space in bytes
   */
  public static void deleteOldLogs(String logDirectory, long minFreeSpace) {
    if (isReal()) {
      var directory = new File(logDirectory);
      if (!directory.exists()) {
        directory.mkdir();
      }

      // ensure that there is enough space on the roboRIO to log data
      if (directory.getFreeSpace() < minFreeSpace) {
        var files = directory.listFiles();
        if (files != null) {
          // Sorting the files by name will ensure that the oldest files are deleted first
          files = Arrays.stream(files).sorted().toArray(File[]::new);

          long bytesToDelete = minFreeSpace - directory.getFreeSpace();

          for (File file : files) {
            if (file.getName().endsWith(".wpilog")) {
              try {
                bytesToDelete -= Files.size(file.toPath());
              } catch (IOException e) {
                System.out.println("Failed to get size of file " + file.getName());
                continue;
              }
              if (file.delete()) {
                System.out.println("Deleted " + file.getName() + " to free up space");
              } else {
                System.out.println("Failed to delete " + file.getName());
              }
              if (bytesToDelete <= 0) {
                break;
              }
            }
          }
        }
      }
    }
  }
}
