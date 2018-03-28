import oscP5.*;
import netP5.*;
import controlP5.*;

NetAddress remoteDisplay;
NetAddress remoteControl;
NetAddress remoteTurntable;
OscP5 oscP5;
OscP5 oscP5tcpClient;
ControlP5 cp5;

int START_T = 0;

int desired_fps = 60;
double leftT = -1;
double rightT = -1;
int turnN = 50;
int scanN = 1;
String scanName = "";
Slider abc;
float scanProgressV = 0;
float sequenceProgressV = 0;

int sin_axis = 0;
int sin_inv = 0;
float sin_phase = 0;
float sin_freq = 10.0;

boolean dirtyScanN = false;
boolean dirtyTurnN = false;
boolean dirtyTotalN = false;
int needRefresh = 0;

float STEP_PER_DEG = 5.8845029123241;

void setup() {
  START_T = millis()+2000;
  size(600, 450);
  frameRate(desired_fps);

  //size(512, 200);
  PFont font = createFont("arial", 20);
  oscP5 = new OscP5(this, 4011);
  remoteControl   = new NetAddress("127.0.0.1", 4010);
  remoteDisplay   = new NetAddress("127.0.0.1", 4021);
  //remoteTurntable = new NetAddress("127.0.0.1", 4013);
  noStroke();

  int px, py;
  px = 10;
  py = 10;

  // GUI
  cp5 = new ControlP5(this);
  cp5.addButton("start_capture")
    .setValue(0)
      .setPosition(px, py)
        .setSize(150, 50)
          .setColorBackground(color(0, 128, 0, 255))
            .setColorActive(color(0, 0, 0))
              ;

  px = px + 150+10;
  cp5.addButton("cancel_capture")
    .setValue(0)
      .setPosition(px, py)
        .setSize(150, 50)
          .setColorBackground(color(128, 0, 0))
            .setColorActive(color(0, 0, 0))
              ;
  px = px + 150+10;
  cp5.addTextfield("scan_name")
    .setPosition(px, py)
      .setSize(width - px - 10, 40)
        .setFont(font)
          .setFocus(true)
            .setColor(color(255, 255, 255))
              .setColorBackground(color(64, 64, 64, 255))
                .setAutoClear(false)
                  ;

  px = 10;
  py = py + 50+10;
  cp5.addButton("turn_less")
    .setValue(0)
      .setPosition(px, py)
        .setSize(55, 50)
          .setColorBackground(color(96, 96, 96))
            .setColorActive(color(0, 0, 0))
              ;
  px = width - 55 - 10;
  cp5.addButton("turn_more")
    .setValue(0)
      .setPosition(px, py)
        .setSize(55, 50)
          .setColorBackground(color(96, 96, 96))
            .setColorActive(color(0, 0, 0))
              ;
  px = width - 40 -55*2;
  cp5.addSlider("turnAmount")
    .setPosition(10+55+10, 10+50+10)
      .setSize(width-40-55*2, 50)
        .setRange(0, 400)
          .setValue(15)
            .setColorBackground(color(32, 32, 32, 255));

  cp5.getController("turnAmount").getCaptionLabel().align(ControlP5.RIGHT, ControlP5.BOTTOM).setPaddingY(10);


  int scan_y = 10+50+10+50+10;

  cp5.addButton("less_scan")
    .setValue(0)
      .setPosition(10, scan_y)
        .setSize(55, 50)
          .setColorBackground(color(96, 96, 96))
            .setColorActive(color(0, 0, 0))
              ;

  cp5.addButton("more_scan")
    .setValue(0)
      .setPosition(width-55-10, scan_y)
        .setSize(55, 50)
          .setColorBackground(color(96, 96, 96))
            .setColorActive(color(0, 0, 0))
              ;

  cp5.addSlider("scanAmount")
    .setPosition(10+55+10, scan_y)
      .setSize(width-40-55*2, 50)
        .setRange(0, 120)
          .setValue(1)
            .setColorBackground(color(32, 32, 32, 255));

  cp5.getController("scanAmount").getCaptionLabel().align(ControlP5.RIGHT, ControlP5.BOTTOM).setPaddingY(10);

  int degree_y =  10+50+10+50+10 + 60;
  int degree_x = width - 50 - 25-50;
  degree_x = width/2+5;
  degree_x = width - 75;

  px = degree_x-50-10;
  py = degree_y+3;

  cp5.addNumberbox("total_degrees")
    .setPosition(degree_x, degree_y+20)
      .setSize(50, 14)
        .setDecimalPrecision(1)
          .setLock(true)
            ;

  cp5.addButton("set_360")
    .setPosition(px, py)
      .setSize(55, 14)
        .setColorBackground(color(96, 128, 128))
          .setColorActive(color(0, 0, 0))
            ;
  py = py + 20;
  cp5.addButton("set_180")
    .setPosition(px, py)
      .setSize(55, 14)
        .setColorBackground(color(96, 128, 128))
          .setColorActive(color(0, 0, 0))
            ;
  py = py + 20;
  cp5.addButton("set_90")
    .setPosition(px, py)
      .setSize(55, 14)
        .setColorBackground(color(96, 128, 128))
          .setColorActive(color(0, 0, 0))
            ;

  px = 10;
  py = degree_y;
  // scan progress bar
  cp5.addSlider("scanProgress")
    .setPosition(px, py)
      .setSize(width-40-55*2, 25)
        .setRange(0, 100)
          .setValue(25)
            .setColorBackground(color(64, 64, 64, 255))
              .setColorForeground(color(255, 255, 255, 128))
                .setColorActive(color(255, 255, 255, 200))
                  .setDecimalPrecision(0)
                    .setLock(true);
  cp5.getController("scanProgress").getCaptionLabel().align(ControlP5.RIGHT, ControlP5.BOTTOM).setPaddingY(10);

  py = py + 35;

  // sequence progress bar
  cp5.addSlider("sequenceProgress")
    .setPosition(px, py)
      .setSize(width-40-55*2, 25)
        .setRange(0, 100)
          .setValue(50)
            .setColorBackground(color(64, 64, 64, 255))
              .setColorForeground(color(0, 160, 0, 128))
                .setColorActive(color(0, 160, 0, 200))
                  .setDecimalPrecision(0)
                    .setLock(true);

  cp5.getController("sequenceProgress").getCaptionLabel().align(ControlP5.RIGHT, ControlP5.BOTTOM).setPaddingY(10);


  int button_width = 150;
  py = py + 35;
  px = width - button_width - 10;

  cp5.addButton("latency_check")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(128, 128, 132, 255))
            .setColorActive(color(0, 0, 0))
              ;

  px = 10;
  button_width = 75;

  cp5.addButton("BLACK")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(0, 0, 0, 255))
            .setColorForeground(color(32, 32, 32))
              ;

  px = px + 10 + button_width;

  cp5.addButton("GRAY_64")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(64, 64, 64, 255))
            .setColorForeground(color(32, 32, 32))
              ;

  px = px + 10 + button_width;

  cp5.addButton("GRAY_128")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(128, 128, 128, 255))
            .setColorForeground(color(32, 32, 32))
              ;

  px = px + 10 + button_width;

  cp5.addButton("GRAY_192")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(192, 192, 192, 255))
            .setColorForeground(color(32, 32, 32))
              ;

  px = px + 10 + button_width;


  cp5.addButton("WHITE")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(192, 192, 200, 255))
            .setColorForeground(color(230, 230, 230))
              ;
  //px = 10;
  //py = py + 10 + 35;
  py = py + 35+10;
  px = 10;//px + 10 + button_width;


  cp5.addButton("RED")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(96, 32, 32, 255))
            .setColorForeground(color(192, 64, 64))
              ;
  px = px + 10 + button_width;

  cp5.addButton("GREEN")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(32, 96, 32, 255))
            .setColorForeground(color(64, 192, 64))
              ;
  px = px + 10 + button_width;

  cp5.addButton("BLUE")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(32, 32, 96, 255))
            .setColorForeground(color(64, 64, 192))
              ;

  px = 10;
  py = py + 10 + 35;

  cp5.addButton("GCB_X0")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(32, 32, 32, 255))
            .setColorForeground(color(64, 64, 64))
              ;
  px = px + 10 + button_width;

  cp5.addButton("GCB_X1")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(32, 32, 32, 255))
            .setColorForeground(color(64, 64, 64))
              ;

  px = px + 10 + button_width;

  cp5.addButton("GCB_Y0")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(32, 32, 32, 255))
            .setColorForeground(color(64, 64, 64))
              ;
  px = px + 10 + button_width;

  cp5.addButton("GCB_Y1")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(32, 32, 32, 255))
            .setColorForeground(color(64, 64, 64))
              ;

  px = px + 10 + button_width;

  cp5.addButton("GCB_Y2")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(32, 32, 32, 255))
            .setColorForeground(color(64, 64, 64))
              ;

  px = px + 10 + button_width;

  cp5.addButton("GCB_Y3")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(32, 32, 32, 255))
            .setColorForeground(color(64, 64, 64))
              ;

  py = py + 10 + 35;
  px = 10;

  cp5.addButton("SIN_A")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(32, 32, 32, 255))
            .setColorForeground(color(64, 64, 64))
              ;
  px = px + 10 + button_width;

  cp5.addButton("SIN_B")
    .setValue(0)
      .setPosition(px, py)
        .setSize(button_width, 35)
          .setColorBackground(color(32, 32, 32, 255))
            .setColorForeground(color(64, 64, 64))
              ;
  px = px + 10 + button_width;

  cp5.addButton("SIN_INV_TOGGLE")
    .setValue(0)
      .setPosition(px, py)
        .setSize(35, 35)
          .setColorBackground(color(32, 32, 32, 255))
            .setColorForeground(color(64, 64, 64))
              ;
  px = px + 10 + button_width;

/*
  cp5.addSlider("SIN_FREQ")
    .setPosition(px, py)
      .setSize(300, 25)
        .setRange(0, 20)
          .setValue(10.0)
            .setColorBackground(color(64, 64, 64, 255))
              .setColorForeground(color(255, 255, 255, 50))
                .setColorActive(color(255, 255, 255, 100))
                  .setDecimalPrecision(2)
                    .setLock(false);
  py = py + 25 + 10;
  cp5.addSlider("SIN_PHASE")
    .setPosition(px, py)
      .setSize(300, 25)
        .setRange(0, 3.1415*2)
          .setValue(10.0)
            .setColorBackground(color(64, 64, 64, 255))
              .setColorForeground(color(255, 255, 255, 50))
                .setColorActive(color(255, 255, 255, 100))
                  .setDecimalPrecision(2)
                    .setLock(false);
  */

  //total_degrees(scanN*turnN);
  cp5.get(Numberbox.class, "total_degrees").setValue(scanN*turnN);

  cp5.getTooltip().setDelay(500);
  cp5.getTooltip().register("turn_less", "manually turn once clockwise");
  cp5.getTooltip().register("turn_more", "manually turn once counterclockwise");
  cp5.getTooltip().register("more_scan", "increase number of scans in sequence");
  cp5.getTooltip().register("less_scan", "decrease number of scans in sequence");
  cp5.getTooltip().register("scan_name", "name of output directory");
  cp5.getTooltip().register("scanProgress", "progress of current scan");
  cp5.getTooltip().register("seqProgress", "progress of overall sequence");
}

public void scan_name(String theText) {
  println("a textfield event for controller 'input' : "+theText);
}

void dirty() {
  needRefresh = needRefresh + 1;
}

void drawProgressBar(int x, int y, float a_prog, float b_prog)
{
}

void draw() {
  background(0);
  if (dirtyTurnN)
  {
    cp5.get(Slider.class, "turnAmount").setValue(turnN);
    dirtyTurnN = false;
  }
  if (dirtyScanN)
  {
    cp5.get(Slider.class, "scanAmount").setValue(scanN);
    dirtyScanN = false;
  }
  if (dirtyTotalN)
  {
    cp5.get(Numberbox.class, "total_degrees").setValue(turnN*scanN);
    dirtyTotalN = false;
  }


  if (needRefresh>0) {
    //    turnAmount(turnN);
    //    scanAmount(scanN);
    //    total_degrees(turnN*scanN);

    cp5.get(Slider.class, "scanProgress").setValue(scanProgressV*100);
    cp5.get(Slider.class, "sequenceProgress").setValue(sequenceProgressV*100);
    println("---------------------");
    println("num scans = " + scanN);
    println("num steps = " + turnN);
    println("total degrees = " + turnN*scanN);

    if (needRefresh > 20)
    {
      needRefresh = 0;
    } else
    {
      needRefresh = max(0, needRefresh-4);
    }
    println("needRefresh = " + needRefresh);
  }
}

public void latency_check(int theValue) {
  if (millis()>START_T+1000) {
    println("triggering latency check: "+theValue+" @"+millis());
    sendLatencyCheckCommand();
  }
}

void set_360(int theValue)
{
  turnN = ceil(360 / max(scanN, 1));
  if (turnN*scanN<360) {
    turnN = ceil(360 / max(scanN, 1))+1;
  }
  println("SET 360");
  dirtyTotalN = true;
  dirtyTurnN = true;
}
void set_180(int theValue)
{
  turnN = ceil(180 / scanN);
  println("SET 180");
  dirtyTotalN = true;
  dirtyTurnN = true;
}
void set_90(int theValue)
{
  turnN = ceil(90 / scanN);
  println("SET 90");
  dirtyTotalN = true;
  dirtyTurnN = true;
}

void total_degrees(int theValue)
{
  //  cp5.get(Numberbox.class, "total_degrees").setValue(theValue);
  turnN = ceil(theValue / scanN);
  println("total_degrees");
  //  dirtyTotalN = true;
  //  dirtyTurnN = true;
  //cp5.get(Slider.class, "turnAmount").setValue(turnN);
}

void turnAmount(int theValue)
{
  turnN = theValue;
  //  total_degrees(scanN*turnN);
  println("turnAmount");
  dirty();
}

void scanAmount(int theValue) {
  //  println("num scans = " + theValue);
  scanN = max(theValue, 1);
  //total_degrees(scanN*turnN);
  //  cp5.get(Slider.class, "scanAmount").setValue(scanN);
  //  total_degrees(scanN*turnN);
  println("scanAmount");
  dirty();

  //cp5.get(Numberbox.class,"input").getText()
}

void more_scan(int theValue) {
  //  scanAmount(scanN+1);
  scanN = max(scanN+1, 1);
  println("more_scan : " + scanN);
  dirtyScanN = true;
  dirtyTotalN = true;
  //  cp5.get(Slider.class, "scanAmount").setValue(scanN);
  //  total_degrees(scanN*turnN);
}
void less_scan(int theValue) {
  //  scanAmount(scanN-1);
  scanN = max(scanN-1, 1);
  dirtyTotalN = true;
  println("less_scan : " + scanN);
  dirtyScanN = true;

  //  cp5.get(Slider.class, "scanAmount").setValue(scanN);
  //  total_degrees(scanN*turnN);
}

public void start_capture(int theValue) {
  if (millis()>START_T+1000) {
    println("a button event from start_capture: "+theValue);    
    OscMessage startMessage = new OscMessage("/start_capture");
    startMessage.add(scanN);
    startMessage.add(int(16*turnN*STEP_PER_DEG));
    startMessage.add(cp5.get(Textfield.class, "scan_name").getText());
    oscP5.send(startMessage, remoteControl);
  }
}
public void cancel_capture(int theValue) {
  if (millis()>START_T+1000) {
    println("a button event from cancel_capture: "+theValue);
    OscMessage stopMessage  = new OscMessage("/cancel_capture");
    stopMessage.add(1);
    oscP5.send(stopMessage, remoteControl);
  }
}

public void turn_less(int theValue) {
  if (millis()>START_T+1000) {
    turnN = max(turnN - 1, 0);
    needRefresh = 1;
    cp5.get(Slider.class, "turnAmount").setValue(turnN);
    dirtyTotalN = true;
    //println("a button event from turn_less: "+theValue+" @"+millis());
    //sendTurnCommand(1, turnN);
  }
}
public void turn_more(int theValue) {
  if (millis()>START_T+1000) {
    turnN = turnN + 1;
    needRefresh = 1;
    cp5.get(Slider.class, "turnAmount").setValue(turnN);
    dirtyTotalN = true;

    //    println("a button event from turn_more: "+theValue+" @"+millis());
    //    sendTurnCommand(0, turnN);
  }
}

public void turn_left(int theValue) {
  if (millis()>START_T+1000) {
    println("a button event from turn_less: "+theValue+" @"+millis());
    sendTurnCommand(1, turnN);
  }
}
public void turn_right(int theValue) {
  if (millis()>START_T+1000) {
    println("a button event from turn_more: "+theValue+" @"+millis());
    sendTurnCommand(0, turnN);
  }
}

void sendTurnCommand(int dir, int num) {
  if (millis()>START_T+1000) {
    OscMessage turnMessage  = new OscMessage("/turn");
    turnMessage.add(new Integer(dir));
    turnMessage.add(new Integer(int(16*num*STEP_PER_DEG)));
    oscP5.send(turnMessage, remoteControl);
  }
}

void sendLatencyCheckCommand() {
  if (millis()>START_T+1000) {
    OscMessage lagMessage  = new OscMessage("/latency_check");
    lagMessage.add(new Integer(1));
    oscP5.send(lagMessage, remoteControl);
  }
}

void BLACK() {
  if (millis()>START_T+1000) {
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(2));
    msg.add(new Integer(0));
    msg.add(new Float(0));
    msg.add(new Float(0));
    msg.add(new Float(0));
    oscP5.send(msg, remoteDisplay);
  }
}
void GRAY_64() {
  if (millis()>START_T+1000) {
    float v = 64/255.f;
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(2));
    msg.add(new Integer(0));
    msg.add(new Float(v));
    msg.add(new Float(v));
    msg.add(new Float(v));
    oscP5.send(msg, remoteDisplay);
  }
}
void GRAY_128() {
  if (millis()>START_T+1000) {
    float v = 128/255.f;
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(2));
    msg.add(new Integer(0));
    msg.add(new Float(v));
    msg.add(new Float(v));
    msg.add(new Float(v));
    oscP5.send(msg, remoteDisplay);
  }
}
void GRAY_192() {
  if (millis()>START_T+1000) {
    float v = 192/255.f;
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(2));
    msg.add(new Integer(0));
    msg.add(new Float(v));
    msg.add(new Float(v));
    msg.add(new Float(v));
    oscP5.send(msg, remoteDisplay);
  }
}

void WHITE() {
  if (millis()>START_T+1000) {
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(2));
    msg.add(new Integer(0));
    msg.add(new Float(1));
    msg.add(new Float(1));
    msg.add(new Float(1));
    oscP5.send(msg, remoteDisplay);
  }
}
void RED() {
  if (millis()>START_T+1000) {
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(2));
    msg.add(new Integer(0));
    msg.add(new Float(1));
    msg.add(new Float(0));
    msg.add(new Float(0));
    oscP5.send(msg, remoteDisplay);
  }
}
void GREEN() {
  if (millis()>START_T+1000) {
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(2));
    msg.add(new Integer(0));
    msg.add(new Float(0));
    msg.add(new Float(1));
    msg.add(new Float(0));
    oscP5.send(msg, remoteDisplay);
  }
}
void BLUE() {
  if (millis()>START_T+1000) {
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(2));
    msg.add(new Integer(0));
    msg.add(new Float(0));
    msg.add(new Float(0));
    msg.add(new Float(1));
    oscP5.send(msg, remoteDisplay);
  }
}
void GCB_X0() {
  if (millis()>START_T+1000) {
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(0));
    msg.add(new Integer(1));
    msg.add(new Integer(0));
    msg.add(new Integer(5));
    msg.add(new Float(1));
    msg.add(new Float(1));
    msg.add(new Float(1));
    oscP5.send(msg, remoteDisplay);
  }
}
void GCB_X1() {
  if (millis()>START_T+1000) {
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(0));
    msg.add(new Integer(0));
    msg.add(new Integer(0));
    msg.add(new Integer(5));
    msg.add(new Float(1));
    msg.add(new Float(1));
    msg.add(new Float(1));
    oscP5.send(msg, remoteDisplay);
  }
}
void GCB_Y0() {
  if (millis()>START_T+1000) {
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(0));
    msg.add(new Integer(1));
    msg.add(new Integer(1));
    msg.add(new Integer(5));
    msg.add(new Float(1));
    msg.add(new Float(1));
    msg.add(new Float(1));
    oscP5.send(msg, remoteDisplay);
  }
}
void GCB_Y1() {
  if (millis()>START_T+1000) {
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(0));
    msg.add(new Integer(0));
    msg.add(new Integer(1));
    msg.add(new Integer(5));
    msg.add(new Float(1));
    msg.add(new Float(1));
    msg.add(new Float(1));
    oscP5.send(msg, remoteDisplay);
  }
}
void GCB_Y2() {
  if (millis()>START_T+1000) {
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(0));
    msg.add(new Integer(1));
    msg.add(new Integer(1));
    msg.add(new Integer(8));
    msg.add(new Float(1));
    msg.add(new Float(1));
    msg.add(new Float(1));
    oscP5.send(msg, remoteDisplay);
  }
}
void GCB_Y3() {
  if (millis()>START_T+1000) {
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(0));
    msg.add(new Integer(0));
    msg.add(new Integer(1));
    msg.add(new Integer(8));
    msg.add(new Float(1));
    msg.add(new Float(1));
    msg.add(new Float(1));
    oscP5.send(msg, remoteDisplay);
  }
}
void send_cur_sin_pattern()
{
  if (millis()>START_T+1000) {
    OscMessage msg = new OscMessage("/pattern_state");
    msg.add(new Integer(1));
    msg.add(new Integer(sin_inv));
    msg.add(new Integer(sin_axis));
    msg.add(new Float(sin_freq));
    msg.add(new Float(sin_phase));
    msg.add(new Float(1));
    msg.add(new Float(1));
    msg.add(new Float(1));
    oscP5.send(msg, remoteDisplay);
  }
}
void SIN_A(float v) {
  sin_inv = 0;
  send_cur_sin_pattern();
}
void SIN_B() {
  sin_inv = 1;
  send_cur_sin_pattern();
}
void SIN_FREQ() {
  sin_freq = cp5.get(Slider.class, "SIN_FREQ").getValue();
  send_cur_sin_pattern();
}
void SIN_PHASE() {
  sin_phase = cp5.get(Slider.class, "SIN_PHASE").getValue();
  send_cur_sin_pattern();
}
void SIN_INV_TOGGLE() {
  sin_inv = 1 - sin_inv;
  send_cur_sin_pattern();
}


void mousePressed() {
  /* in the following different ways of creating osc messages are shown by example */
  //OscMessage startMessage = new OscMessage("/start_capture");
  //OscMessage stopMessage  = new OscMessage("/cancel_capture");

  /* send the message */
  //oscP5.send(myMessage, myRemoteLocation);
}

void keyPressed()
{
  if (key == CODED) {
    if (keyCode == LEFT) {
      turn_left(0);
    } else if (keyCode == RIGHT) {
      turn_right(0);
    } else if (keyCode == UP) {
      turn_more(0);
    } else if (keyCode == DOWN) {
      turn_less(0);
    }
  } else {
    if ( key == '\n' ) {
      start_capture(1);
    }
  }
}

/* incoming osc message are forwarded to the oscEvent method. */
void oscEvent(OscMessage theOscMessage) {
  /* print the address pattern and the typetag of the received OscMessage */
  print("### received an osc message.");
  print(" addrpattern: "+theOscMessage.addrPattern());
  println(" typetag: "+theOscMessage.typetag());
  if ( theOscMessage.checkAddrPattern("/progress") == true )
  {
    if ( theOscMessage.checkTypetag("ff") )
    {
      float scanV = theOscMessage.get(0).floatValue();
      float seqV  = theOscMessage.get(1).floatValue();
      print("scanProgressV = " + scanV + ", sequenceProgressV = " + seqV + "\n");
      scanProgressV     = scanV;
      sequenceProgressV = seqV;
      dirty();
    }
  }
}

