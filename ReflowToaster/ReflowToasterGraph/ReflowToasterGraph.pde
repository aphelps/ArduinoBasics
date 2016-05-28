/*
 * Graph the results of the CapTest sketch
 */

import processing.serial.*;
import java.util.regex.Pattern;
import java.util.regex.Matcher;
import java.awt.Toolkit;

static final String DATA_PATH = "/tmp/taillog";

Serial myPort;

BufferedReader reader;

void setup() {
  size(1400, 400);
//  size(200, 400);
  
  // List all the available serial ports
  println(Serial.list());
  // I know that the first port in the serial list on my mac
  // is always my  Arduino, so I open Serial.list()[0].
  // Open whatever port is the one you're using.
//  myPort = new Serial(this, Serial.list()[5], 9600);
  // don't generate a serialEvent() unless you get a newline character:
//  myPort.bufferUntil('\n');
  // set inital background:

  reader = createReader(DATA_PATH); 

  background(0);

  textSize(12);
}
void draw () {
  readValues();
}

static final int MARGIN = 20;
static final int LEFT_MARGIN = 50;

static final int NUM_LINES = 5;

int X = LEFT_MARGIN;
int shiftX = 1;

float mintempC = 999999;
float maxtempC = 0;

static final int AVE_NUM = 10;
long average_sum = 0;
long average_values[] = new long[AVE_NUM];
int place = -1;
int wrap = 0;
long running_average(long new_value) 
{
  place++;
  if (place >= AVE_NUM) {
    place = 0;
    wrap++;
  }

  if (wrap != 0) {
    average_sum -= average_values[place];
  }
  
  average_sum += new_value;
  average_values[place] = new_value;
  return (average_sum / AVE_NUM);
}

/* Reset the min and max when spacebar is hit*/ 
boolean setLine = false;
void keyPressed() 
{
  switch (key) {
      case ' ':
        mintempC = 999999;
        maxtempC = 0;
        setLine = true;
        break;
  }
}

Pattern internalPat = Pattern.compile("Internal Temp:([0-9.]+).");
Pattern cPat = Pattern.compile("C:([0-9.]+).");
Pattern setPat = Pattern.compile("Set:([0-9.]+).");
Pattern pidPat = Pattern.compile("pid:([0-9.]+).");

void readValues () {

  float tempC = 0;
  float pidVal = 0;
  float setTemp = 0;
 
  String line;
  try {
    line = reader.readLine();
    //println(line);
   } catch (IOException e) {
     e.printStackTrace();
     line = null;
  }
  if (line == null) {
    return;
  }
  
  Matcher m = cPat.matcher(line);
    if (m.find()) {
    tempC = float(m.group(1));
  }
  
  m = setPat.matcher(line);
  if (m.find()) {
    setTemp = int(m.group(1));
    
    if (abs(setTemp - tempC) < 1) {
      Toolkit.getDefaultToolkit().beep();  
    }
  }
   
  m = pidPat.matcher(line);
  if (m.find()) {
    pidVal = int(m.group(1));
  }
  
  /* Get the value */
  
  long averageRunning = running_average((int)tempC);

  println("tempC: " + tempC + " pidVal: " + pidVal + " setTemp: " + setTemp);

  if (tempC != 0) {

    /* Map the values to the window */
//    if (tempC > maxtempC) maxtempC = tempC;
//    else if (tempC < mintempC) mintempC = tempC;

  maxtempC = 255;
  mintempC = 0;

    float tempCY = MARGIN + map(tempC,
                                mintempC, maxtempC,
                                0, height - MARGIN * 2);
    float pidValY = MARGIN + map(pidVal,
                                mintempC, maxtempC,
                                0, height - MARGIN * 2);
    float setTempY = MARGIN + map(setTemp,
                                mintempC, maxtempC,
                                0, height - MARGIN * 2);
    

    fill(0);
    rect(0, 0, LEFT_MARGIN, height);

    /* Draw level lines and labels */
    stroke(0, 0, 255);
    fill(0, 0, 255);
    for (int i = 0; i <= NUM_LINES; i++) {
      float linetempC = map(i, 0, NUM_LINES, mintempC, maxtempC);
      float lineY = MARGIN + map(linetempC,
                               mintempC, maxtempC,
                               0, height - MARGIN * 2);

      line(LEFT_MARGIN, lineY, width, lineY);
      text((int)(linetempC), 0, height - lineY);
    }

    if (setLine) {
      line(X, 0, X, height);
      setLine = false;
    }

    noStroke();

    /* Draw the data point */
    fill(255, 0, 0);
    ellipse(X, height - tempCY, 3, 3);
    
    fill(0, 255, 0);
    ellipse(X, height - setTempY, 3, 3);


    fill(255, 255, 0);
    ellipse(X, height - pidValY, 3, 3);
    
    X++;
    if (X >= width - 1) {
      /* When the data gets to the right edge shift everything to the left */
      copy(LEFT_MARGIN + shiftX, 0, width - (LEFT_MARGIN + shiftX), height,
           LEFT_MARGIN, 0, width - (LEFT_MARGIN + shiftX), height);
      fill(0);
      rect(width - shiftX, 0, width, height);
      
      X -= shiftX;
    }
  }
}
