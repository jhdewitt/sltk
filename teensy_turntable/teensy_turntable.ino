#include <Bounce.h>

#define RAWHID_TX_SIZE          64       // transmit packet size
#define RAWHID_TX_INTERVAL      32       // max # of ms between transmit packets
#define RAWHID_RX_SIZE          64       // receive packet size
#define RAWHID_RX_INTERVAL      32       // max # of ms between receive packets

// pins for teensy 3.0
const int ledPin = 13;
const int butPin0 = 11;
const int butPin1 = 14;

const int dirPin = 15;
const int stePin = 16;
const int slePin = 17;

Bounce button0 = Bounce(butPin0, 80);
Bounce button1 = Bounce(butPin1, 80);

#define STEP_PER_MM 100.52356020942
int mm2step(float mm)
{
  return (int)floor(STEP_PER_MM*mm);
}


int step = 0;
int stepDir = 0;
int queuedSteps = 0;
#define MM_RANGE 20 // 142 max
#define STEP_MOD 1
#define DBUG 0
unsigned stepnum  = floor((STEP_PER_MM*MM_RANGE) / STEP_MOD) * STEP_MOD;

bool press_0 = false;
bool press_1 = false;
bool stepTrigger = false;
bool driverPower = false;
unsigned long lastcheck = 0;
unsigned long checkdelay = 250;
unsigned long lastStep = 0;
unsigned long lastreport = 0;
unsigned long timeOut = 30000;
double curspd = 0;

byte buffer[64];
elapsedMillis msUntilNextSend;
unsigned int packetCount = 0;

/*
void process_seq()
{
  int seq_max, seq_min;
  for (int i = 0; i < seq_n; i++)
  {
    int v = seq[i];
    if (i == 0)
    {
      seq_max = v;
      seq_min = v;
    }
    else
    {
      if (v > seq_max) seq_max = v;
      if (v < seq_min) seq_min = v;
    }
  }

  for (int i = 0; i < seq_n; i++)
  {
    float v = seq[i];
    // normalize range and invert
    v = 1.0 - (v - seq_min) / (float)(seq_max - seq_min);
    seq[i] = min(max(v,0.0),1.0);
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(v);

  }
}
*/

void setup()
{
  // Serial.begin(38400);
  pinMode(butPin0, INPUT);
  pinMode(butPin1, INPUT);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(dirPin, OUTPUT);
  pinMode(stePin, OUTPUT);
  pinMode(slePin, OUTPUT);

  digitalWrite(dirPin, LOW);
  digitalWrite(stePin, LOW);
  digitalWrite(slePin, LOW);
  digitalWrite(slePin, LOW);

  delayMicroseconds(1000);
  //process_seq();
}


/*
double CubicInterpolate(
   double y0,double y1,
   double y2,double y3,
   double mu)
{
   double a0,a1,a2,a3,mu2;

   mu2 = mu*mu;
   a0 = y3 - y2 - y0 + y1;
   a1 = y0 - y1 - a0;
   a2 = y2 - y0;
   a3 = y1;

//   a0 = -0.5*y0 + 1.5*y1 - 1.5*y2 + 0.5*y3;
//   a1 = y0 - 2.5*y1 + 2*y2 - 0.5*y3;
//   a2 = -0.5*y0 + 0.5*y2;
//   a3 = y1;

   return(a0*mu*mu2+a1*mu2+a2*mu+a3);
}
*/

void report_ready()
{
  // first 2 bytes are signature
  buffer[0] = 0xAB;
  buffer[1] = 0xCD;

  //buffer[0] = 0; // direction
  //buffer[1] = 0; // MSB steps
  buffer[2] = 0; // LSB steps
  buffer[3] = 1; // ready

  for (int i = 4; i < 62; i++)
  {
    buffer[i] = 0;
  }

  buffer[62] = highByte(packetCount);
  buffer[63] = lowByte(packetCount);
  int n = RawHID.send(buffer, 100);

  if (n > 0)
  {
    if(DBUG)
    {
        Serial.print(F("Transmit packet "));
        Serial.println(packetCount);
    }
    packetCount = packetCount + 1;
  }
  else
  {
    if(DBUG)
        Serial.println(F("Unable to transmit packet"));

  }
}





/////////////////////////////////////////////////////////////////
bool paused = false;
void loop()
{
  button0.update();
  button1.update();
  
  //int steps = 8426;
  int steps = 2098*16; //280 // 2206
  steps = 100*16;
  
  if(button0.fallingEdge())
  {
    stepDir = 0;
    stepTrigger = true;
    queuedSteps = steps;
  }
  
  if(button1.fallingEdge())
  {
    stepDir = 1;
    stepTrigger = true;
    queuedSteps = steps;
  }
  
  // usb hid handling
  int n = RawHID.recv(buffer, 0);
  //delayMicroseconds(1000);
  if (n == 64)
  {
    stepDir = (buffer[0]==0)?0:1;
    byte high = buffer[1];
    byte low  = buffer[2];
    unsigned int tmpsteps = abs( (high << 8) | low );
    queuedSteps = tmpsteps;

    if(DBUG)
    {
        // message from computer
        Serial.print("received packet: ");
        Serial.println((int)buffer[0]);
        Serial.print("   ");
        for (int i = 0; i < 8; i++)
        {
          int b = buffer[0] & (1 << i);
          Serial.print(b);
          Serial.print(" ");
        }
    }
    stepTrigger = true;
  }
  
  // step if turn has been triggered
  if (stepTrigger && queuedSteps > 0)
  {
    elapsedMicros waiting;
    int turntime = 8000; // µs
    
    //unsigned long start_t = millis();
    int stepNumToBreak = 16*10;

    // turn on LED and wake up driver
    digitalWrite(ledPin, HIGH);
    digitalWrite(slePin, HIGH);
    driverPower = true;
    
    if (stepDir)
        digitalWrite(dirPin, LOW);
    else
        digitalWrite(dirPin, HIGH);
    
    double maxspd = 5000.0;
    double minspd = 150.0;
    double spd = minspd;
    double accelScale = 6;
    //unsigned long stept = micros();
    
    int rampSteps = -1;
    int totalSteps = queuedSteps;
    while(queuedSteps > 0 /*&& stepNumToBreak > 0*/ && waiting < turntime )
    {
        
        // speed up
        if(spd<maxspd && queuedSteps>totalSteps/2)
        {
            spd = min(spd + accelScale,maxspd);
            rampSteps = totalSteps-queuedSteps;
        }
        
        // slow down
        //if(spd>minspd && queuedSteps<totalSteps/2)
        if(spd>minspd && queuedSteps<totalSteps/2 && queuedSteps < rampSteps)
        {
            spd = max(spd - accelScale,minspd);
        }

        //double v = (queuedSteps)/(double)totalSteps;
        //double s = sin(v*3.14159);
        //spd = minspd + pow(s,1.0)*(maxspd-minspd);

        // step once
        for(int i=0;i<16;i++)
        {
            if( queuedSteps>0 )//&& stepNumToBreak>0 )
            {
                digitalWrite(stePin, HIGH);
                delayMicroseconds(1);
                digitalWrite(stePin, LOW);
                delayMicroseconds(10);
                lastStep = millis();
                queuedSteps--;
                //stepNumToBreak--;
            } else {
              break;
            }
        }
        delayMicroseconds(1e6/spd);
        
    }
    
    if( queuedSteps <= 0 )
    {
        stepTrigger = false;
        queuedSteps = 0;
    }
  }
  else
  {
    digitalWrite(ledPin, LOW);
  }
  
  if(driverPower && lastStep < millis() - 1e3*2.0)
  {
    // turn off LED and driver
    digitalWrite(ledPin, LOW);
    digitalWrite(slePin, LOW);
  }

  if(lastreport < millis()-2000 && queuedSteps == 0)
  {
    lastreport = millis();
    report_ready();
  }
}

