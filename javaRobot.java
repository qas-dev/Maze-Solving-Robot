import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Stack;

import lejos.hardware.Battery;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.NXTSoundSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import lejos.utility.Delay;

public class javaRobot {

  public static void breadth_first_search(char[][] matrix,
                                          int matrix_length_horizontal,
                                          int matrix_length_vertical,
                                          List<int[]> path_list, int start_x,
                                          int start_y, grid_cell begin) {

    Stack<grid_cell> new_stack = new Stack<grid_cell>();
    HashMap<grid_cell, grid_cell> map = new HashMap<grid_cell, grid_cell>();

    Queue<grid_cell> queue = new LinkedList<>();
    grid_cell cell_End = null;

    int[] x_vectors = {0, 0, 1, -1};
    int[] y_vectors = {1, -1, 0, 0};

    boolean break_flag = false;
    boolean[][] note_visited =
        new boolean[matrix_length_vertical][matrix_length_horizontal];

    queue.add(begin);

    while (queue.size() != 0) {

      int index = 0;
      grid_cell queue_start = queue.poll();

      while (index < 4) {

        int X = queue_start.R;
        int Y = queue_start.C;

        X += x_vectors[index];
        Y += y_vectors[index];
        index += 1;

        if ((X >= matrix_length_vertical || X < 0) == true)
          continue;
        if ((Y >= matrix_length_horizontal || Y < 0 || matrix[X][Y] == '0' ||
             note_visited[X][Y]) == true)
          continue;

        grid_cell cellNext = new grid_cell(X, Y);
        note_visited[X][Y] = true;

        queue.add(cellNext);
        map.put(cellNext, queue_start);

        if ((matrix[X][Y] == 'E') == true) {

          break_flag = true;
          cell_End = new grid_cell(X, Y);

          new_stack.push(cell_End);
          break;
        }
      }

      if (break_flag == true)
        break;
    }

    for (; true;) {

      grid_cell cell_temp = map.get(cell_End);
      new_stack.push(cell_temp);

      if ((cell_temp == begin) == true)
        break;

      cell_End = cell_temp;
    }

    for (; new_stack.isEmpty() == false;) {

      grid_cell popped_element = new_stack.pop();
      path_list.add(new int[] {popped_element.R, popped_element.C});
    }
  }

  public static List<int[]> shortest_Path_Finder(char[][] matrix,
                                                 int matrix_length_horizontal,
                                                 int matrix_length_vertical,
                                                 int start_x, int start_y) {

    List<int[]> new_list_path = new ArrayList<int[]>();

    breadth_first_search(matrix, matrix_length_horizontal,
                         matrix_length_vertical, new_list_path, start_x,
                         start_y, new grid_cell(start_x, start_y));

    return new_list_path;
  }

  public static void main(String[] args) {

    LCD.drawString("Java Robot Maze Solver", 2, 1);
    LCD.drawString("Contributors:", 2, 2);
    LCD.drawString(" - AF", 2, 3);
    LCD.drawString(" - QI", 2, 4);
    LCD.drawString(" - JB", 2, 5);
    LCD.drawString(" - IA", 2, 6);
    Delay.msDelay(5000);
    LCD.clear();

    float[] colours = new float[1];
    float[] distance = new float[1];
    float[] redColors = new float[1];

    EV3ColorSensor colourSensor = new EV3ColorSensor(SensorPort.S3);
    EV3ColorSensor redColourSensor = new EV3ColorSensor(SensorPort.S4);
    UnregulatedMotor mLeft = new UnregulatedMotor(MotorPort.A);

    UnregulatedMotor mRight = new UnregulatedMotor(MotorPort.B);
    EV3UltrasonicSensor us = new EV3UltrasonicSensor(SensorPort.S2);
    NXTSoundSensor soundSensor = new NXTSoundSensor(SensorPort.S1);

    SampleProvider sp = us.getDistanceMode();
    SampleProvider redColour = redColourSensor.getColorIDMode();
    SampleProvider colour = colourSensor.getRedMode();
    SampleProvider sound = soundSensor.getDBAMode();

    char[][] matrix = {

        {'1', '1', '1', '0', '0'},
        {'1', '0', '1', '0', '1'},
        {'1', '1', '1', '1', '1'},
        {'1', '0', '0', '0', '1'},
        {'1', '1', '0', 'E', '1'}};

    List<int[]> coor = shortest_Path_Finder(matrix, 5, 5, 0, 2);

    Behavior driveOnLineBehavior = new driveOnLineBehavior(
        mLeft, mRight, redColour, colour, colours, redColors);
    Behavior distanceBehaviour =
        new distanceBehaviour(sp, distance, mLeft, mRight);
    Behavior turnBehaviour = new turnBehaviour(redColour, redColors, mLeft,
                                               mRight, coor, colour, colours);

    Behavior batteryLevelBehaviour = new batteryLevelBehaviour();
    Behavior buttonPressBehaviour = new buttonPressBehaviour();
    Behavior clapBehaviour = new clapBehaviour(sound);

    Arbitrator ab = new Arbitrator(new Behavior[] {
        turnBehaviour, driveOnLineBehavior, distanceBehaviour,
        batteryLevelBehaviour, buttonPressBehaviour, clapBehaviour});
    ab.go();
  }
}

class turnBehaviour implements Behavior {

  private float[] redColors;
  private float[] colours;
  private List<int[]> path;

  public static boolean supressed = false;
  private int indx = 1;

  private UnregulatedMotor mLeft;
  private UnregulatedMotor mRight;

  private String currentDirection = "N";
  private SampleProvider redColour;
  private SampleProvider colour;

  turnBehaviour(SampleProvider redColour, float[] redColors,
                UnregulatedMotor mLeft, UnregulatedMotor mRight,
                List<int[]> path, SampleProvider colour, float[] colours) {

    this.redColors = redColors;
    this.redColour = redColour;
    this.mLeft = mLeft;
    this.mRight = mRight;
    this.path = path;
    this.colours = colours;
    this.colour = colour;
  }

  // Turn the robot depending on which coordinate it need to move to next.
  public void action() {
    if (supressed == false) {
      if (indx < path.size()) {
        int[] currentCoord = path.get(indx - 1);
        int[] futureCoord = path.get(indx);
        // Check which direction to move based on which e and y coordinate is
        // greater than the next position
        if (futureCoord[0] > currentCoord[0]) {
          currentDirection = MoveBot(currentDirection, "right");
        } else if (futureCoord[0] < currentCoord[0]) {
          currentDirection = MoveBot(currentDirection, "left");
        } else if (futureCoord[1] > currentCoord[1]) {
          currentDirection = MoveBot(currentDirection, "forward");
        } else if (futureCoord[1] < currentCoord[1]) {
          currentDirection = MoveBot(currentDirection, "down");
        }
        indx++;
      } else {
        // At the end of  the coordinate list stop the robot.
        supressed = true;
        driveOnLineBehavior.supressed = true;
        mLeft.stop();
        mRight.stop();
        System.exit(0);
      }
    }
  }
  public void suppress() {}

  public boolean takeControl() {
    // If the behaviour is not suppressed then run the action
    redColour.fetchSample(redColors, 0);
    return !(supressed);
  }

  public void forward() {
    // Set both motors to power level 50 and drive forward for .8 seconds
    mLeft.setPower(50);
    mRight.setPower(50);
    long t = System.currentTimeMillis();
    long end = t + 800;
    while (System.currentTimeMillis() < end) {
    }
    // Suppress the turn behaviour
    supressed = true;
    // Un suppress the follow line drive on line behaviour
    driveOnLineBehavior.supressed = false;
  }

  public void negativeNinetyDegrees() {
    LCD.drawString("RIGHT", 2, 2);
    // Drive forward for .5 seconds
    mLeft.setPower(50);
    mRight.setPower(50);
    long t = System.currentTimeMillis();
    long end = t + 500;
    while (System.currentTimeMillis() < end) {
    }
    // Start to turn the robot for 0.6 seconds to get the light sensor off the
    // current line.
    mLeft.setPower(0);
    mRight.setPower(50);
    long t1 = System.currentTimeMillis();
    long end1 = t1 + 600;
    while (System.currentTimeMillis() < end1) {
    }
    // Reduce the turn speed so that the light sensor has time to react to the
    // dark line and then white surface.
    mLeft.setPower(0);
    mRight.setPower(40);
    boolean status = false;
    while (true) {
      colour.fetchSample(colours, 0);
      // Update the status to true when black has been detected.
      if (colours[0] < 0.2)
        status = true;
      // If black has been detected check if the light sensor is now detecting
      // white.
      if (status == true) {
        // Break the while loop when white is detected.
        if (colours[0] > 0.6)
          break;
      }
    }
    // Set both motors to power level 0.
    mLeft.setPower(0);
    mRight.setPower(0);
    // Suppress the turn behaviour.
    supressed = true;
    // Un suppress the drive on line behaviour.
    driveOnLineBehavior.supressed = false;
  }

  public void ninetyDegrees() {
    LCD.drawString("LEFT", 2, 2);
    // Turn for .8 seconds to get the light sensor off the current line.
    mLeft.setPower(50);
    mRight.setPower(0);
    long t1 = System.currentTimeMillis();
    long end1 = t1 + 800;
    while (System.currentTimeMillis() < end1) {
    }
    // Set both motors to power level 20 so that the robot moves forward
    // straight.
    mLeft.setPower(20);
    mRight.setPower(20);
    boolean status = false;
    while (true) {
      colour.fetchSample(colours, 0);
      // Update the status to true when black has been detected.
      if (colours[0] < 0.2)
        status = true;
      // If black has been detected check if the light sensor is now detecting
      // white.
      if (status == true) {
        // Break the while loop when white is detected.
        if (colours[0] > 0.6)
          break;
      }
    }
    // Suppress the turn behaviour.
    supressed = true;
    // Un suppress the drive on line behaviour.
    driveOnLineBehavior.supressed = false;
    // Set both motors to power level 0.
    mLeft.setPower(0);
    mRight.setPower(0);
  }

  public String MoveBot(String directionFacing, String neededDirection) {
    if (directionFacing == "N") {
      if (neededDirection == "forward") {
        forward();
        return "N";
      } else if (neededDirection == "right") {
        negativeNinetyDegrees();
        return "E";
      } else if (neededDirection == "left") {
        ninetyDegrees();
        return "W";
      }

    } else if (directionFacing == "S") {
      if (neededDirection == "right") {
        ninetyDegrees();
        return "E";
      } else if (neededDirection == "left") {
        negativeNinetyDegrees();
        return "W";
      } else if (neededDirection == "down") {
        forward();
        return "S";
      }

    } else if (directionFacing == "E") {
      if (neededDirection == "forward") {
        ninetyDegrees();
        return "N";
      } else if (neededDirection == "right") {
        forward();
        return "E";
      } else if (neededDirection == "down") {
        negativeNinetyDegrees();
        return "S";
      }

    } else if (directionFacing == "W") {
      if (neededDirection == "forward") {
        negativeNinetyDegrees();
        return "N";
      } else if (neededDirection == "left") {
        forward();
        return "W";
      } else if (neededDirection == "down") {
        ninetyDegrees();
        return "S";
      }
    }
    return "S";
  }
}

class driveOnLineBehavior implements Behavior {
  private UnregulatedMotor mLeft;
  private UnregulatedMotor mRight;
  private SampleProvider redColour;
  private SampleProvider colour;
  private float[] colours;
  private float[] redColors;
  public static boolean supressed = false;
  private int speedMultiplier = 170;
  private double constant = 0.08;

  driveOnLineBehavior(UnregulatedMotor mLeft, UnregulatedMotor mRight,
                      SampleProvider redColour, SampleProvider colour,
                      float[] colours, float[] redColors) {
    this.mLeft = mLeft;
    this.mRight = mRight;
    this.redColour = redColour;
    this.colour = colour;
    this.colours = colours;
    this.redColors = redColors;
  }

  public void action() {
    if (supressed == false) {
      colour.fetchSample(colours, 0);
      redColour.fetchSample(redColors, 0);

      double leftMotorPower;
      double rightMotorPower;

      float multiplier = (float)(0.2 - colours[0]);
      if (multiplier <= 0) {
        leftMotorPower = constant - multiplier;
        rightMotorPower = constant;
      } else {
        leftMotorPower = constant;
        rightMotorPower = constant + multiplier;
      }

      if (redColors[0] == 0.00) {
        supressed = true;
        mLeft.setPower(0);
        mRight.setPower(0);
        turnBehaviour.supressed = false;
      }
      mLeft.setPower((int)(leftMotorPower * speedMultiplier));
      mRight.setPower((int)(rightMotorPower * speedMultiplier));
    }
  }

  public void suppress() {}

  public boolean takeControl() { return !(supressed); }
}

class batteryLevelBehaviour implements Behavior {
  batteryLevelBehaviour() {}
  public void action() {
    Sound.buzz();
    System.exit(0);
  }
  public void suppress() {}
  public boolean takeControl() {
    int power = Battery.getVoltageMilliVolt();
    return power <= 1500;
  }
}

class distanceBehaviour implements Behavior {
  private SampleProvider sp;
  private float[] distance;
  private UnregulatedMotor mLeft;
  private UnregulatedMotor mRight;

  distanceBehaviour(SampleProvider sp, float[] distance, UnregulatedMotor mLeft,
                    UnregulatedMotor mRight) {
    this.sp = sp;
    this.distance = distance;
    this.mLeft = mLeft;
    this.mRight = mRight;
  }

  public void action() {
    Sound.beep();
    mLeft.setPower(0);
    mRight.setPower(0);
  }
  public void suppress() {}
  public boolean takeControl() {
    sp.fetchSample(distance, 0);
    return distance[0] < 0.4;
  }
}

class buttonPressBehaviour implements Behavior {
  buttonPressBehaviour() {}

  public void action() {
    Sound.beepSequenceUp();
    System.exit(0);
  }
  public void suppress() {}
  public boolean takeControl() { return Button.ENTER.isDown(); }
}

class clapBehaviour implements Behavior {
  private SampleProvider sp;
  private float[] samples = new float[1];
  private boolean supressed = false;

  clapBehaviour(SampleProvider sp) { this.sp = sp; }

  public void action() {
    sp.fetchSample(samples, 0);
    if (samples[0] > 0.8) {
      supressed = true;
    } else {
      supressed = false;
    }
  }

  public void suppress() {}

  public boolean takeControl() {
    if (!(supressed)) {
      return true;
    } else {
      return false;
    }
  }
}
