#include <NeoPixelBus.h>

const uint16_t PIXEL_COUNT = 60;
const int PIXEL_PIN = 6;
const uint8_t STATUS_PIN = 4;
const uint8_t DATA_REQUEST_PIN = 3; 
const int TOO_CLOSE_DISTANCE = 60; //cm
const int OPTIMAL_DISTANCE = 80; //cm
//const int CAPTURE_MAX_DISTANCE = 110; //cm
const int CLOSE_OBJECT = 100; //cm
const float ARRIVAL_MAX_DISTANCE = 300.0; //cm
const float ARRIVAL_MIN_DISTANCE = 90.0; //cm
const float MAX_DISTANCE = 9999;
const uint16_t MAX_DELAY = 1500; //uS

//External state
byte PREV_EXT_SYS_STATE = 0;
byte EXT_SYS_STATE = 0;

//This controllers state
byte SYS_STATE = 0;

//State flags
bool objectPresent = false;
bool motionDetected = false;
bool motionTimedout = true;
bool shutdownComplete = false;
float currentDistance = -1;
float meanDistance = 0.0;
byte md[] = { 0,0,0,0 };
long nextUpdate = 0;

NeoPixelBus<NeoGrbwFeature, Neo800KbpsMethod> ring(60, PIXEL_PIN);
/*
 * Transition from shutdown to ready.
 */
void awaitArrival() {}
void awaitDeparture() {}
void arriving() {}
void capture() {}

void setup() {
  Serial.begin(115200);
  Serial.println("Spot-60 Light Controller.");

  //setup the pins.
  pinMode(DATA_REQUEST_PIN, OUTPUT);
  digitalWrite(DATA_REQUEST_PIN, LOW);
  
  pinMode(STATUS_PIN, INPUT);

  //Initialize the light ring.
  ring.Begin();
  ring.Show();

  nextUpdate = micros();

  //Ask the controller for the range.
  Serial.println("Start-up complete.");
}

void loop() {
  readRemoteUntil(100000);
  processStates();
}

void clearSerial()
{
  while (Serial.available() > 0) { Serial.read(); }
}

void signalReady()
{
  RgbwColor color = ring.GetPixelColor(0);
  
  int lowerBound = color.W;
  int upperBound = 115;
  int brightnessStep = 1;
  float stepDelay = 5;

  for (int i = lowerBound; i <= upperBound; i += brightnessStep)
  {
    color.W = i;
    
    for (int n = 0; n < PIXEL_COUNT; n++)
      ring.SetPixelColor(n, color);

    ring.Show();

    delay(stepDelay);
  }

  delay(50);
  
  for (int i = upperBound; i >= lowerBound; i -= brightnessStep)
  {
    color.W = i;
    
    for (int n = 0; n < PIXEL_COUNT; n++)
      ring.SetPixelColor(n, color);
      
    ring.Show();
    delay(stepDelay);
  }
}

int signalAdvance()
{
  //if we have advanced outside of the max range, transfer control
  //if (currentDistance > ARRIVAL_MAX_DISTANCE)
  //  return -1;

  //if we have advanced beyond the min range, transfer control
  //if (currentDistance < ARRIVAL_MIN_DISTANCE)
  //  return 1;
  RgbwColor currentTrail(0, 0, 0, 0);

  //Zone colors
  RgbwColor startZone(0, 64, 0, 10);
  RgbwColor currentBase(0, 64, 0, 10);
  RgbwColor endZone(200, 0, 0, 10);
  
  float rangeScale = ARRIVAL_MAX_DISTANCE - ARRIVAL_MIN_DISTANCE;
  float currentScale = 0.0;
  float pctOfScale = 0.0;
  
  int tailLength = 7;

  float spinDelay = 0;
  long delayTime = 0;
  
  for (int i = 0; i < PIXEL_COUNT + tailLength; i++)
  {    
    if (motionTimedout)
      return 0;
    
    currentScale = (currentDistance - ARRIVAL_MIN_DISTANCE);
    pctOfScale = currentScale / rangeScale;
    
    if (pctOfScale > 1)
      pctOfScale = 1;
    else if (pctOfScale < 0)
      pctOfScale = 0;

    currentBase = RgbwColor::LinearBlend(endZone, startZone, pctOfScale);
    
    if (i == PIXEL_COUNT + tailLength - 1)
    {
      i = tailLength - 1; 
    }
    
    setRingColor(currentBase);

    for (int n = 0; n < tailLength; n++)
    {
        currentTrail = currentBase;
        currentTrail.W = 200 * ((tailLength - n) * .1);
        ring.SetPixelColor(i - n, currentTrail);

        if (i >= PIXEL_COUNT)
        {
          currentTrail = currentBase;
          currentTrail.W = 200 * ((tailLength - n) * .1);
          ring.SetPixelColor((i - PIXEL_COUNT) - n, currentTrail);
        }
    }
    
    ring.Show();

    spinDelay = computeAdvanceSpinDelay(pctOfScale);

    readRemoteUntil(spinDelay * 1000);

    if (meanDistance < ARRIVAL_MIN_DISTANCE)
      return 1;
  }
}

float computeAdvanceSpinDelay(float pctOfScale)
{ 
  int minDelay = 2;
  int maxDelay = 85;
  float rateScale = 83;
  int spinDelay = 85; //ms
  
  spinDelay = (pctOfScale * rateScale);

  if (spinDelay <= 0)
    spinDelay = minDelay;
  else
    spinDelay += minDelay;

  return spinDelay;
}

void setRingColor(RgbwColor color)
{
  //init the ring with the base color.
  for (int i = 0; i < PIXEL_COUNT; i++)
  {
    ring.SetPixelColor(i, color);
  }
}

void sysShutdown()
{
  signalShutdown();
  Serial.println("Shutting down");
  shutdownComplete = true;
}

void signalShutdown()
{
  RgbwColor color = ring.GetPixelColor(0);
  RgbwColor black = RgbwColor(0,0,0,0);
  RgbwColor workingColor;
  
  int brightnessStep = 1;
  float stepDelay = 8;
  
  for (float i = 0.0; i <= 1; i += .005)
  {
    workingColor = RgbwColor::LinearBlend(color, black, i);
    
    for (int n = 0; n < PIXEL_COUNT; n++)
      ring.SetPixelColor(n, workingColor);
      
    ring.Show();
    delay(stepDelay);
  }
}

void signalStartup()
{ 
  RgbwColor color = RgbwColor(20, 100, 255, 0);
  RgbwColor black = RgbwColor(0,0,0,0);
  RgbwColor workingColor;
  
  int brightnessStep = 1;
  float stepDelay = 8;
  
  for (float i = 0.0; i <= 1; i += .005)
  {
    workingColor = RgbwColor::LinearBlend(black, color, i);
    
    for (int n = 0; n < PIXEL_COUNT; n++)
      ring.SetPixelColor(n, workingColor);
      
    ring.Show();
    delay(stepDelay);
  }
}

/* Declare the system state array after the functions are defined. */
/*
 * 0 - shutdown
 * 1 - ready
 * 2 - awaiting arrival
 * 3 - arriving
 * 4 - capture
 * 5 - complete
 * 6 - awaiting departure
 */
 
void processStates()
{
    Serial.println("Processing State");
    Serial.print("State ");
    Serial.print(SYS_STATE);
    Serial.print(" Distance ");
    Serial.println(currentDistance);
    
  //if we are in shutdown and the motion timeout is lifted, startup
  if (!motionTimedout && SYS_STATE == 0)
    SYS_STATE = 1;
  else if (motionTimedout)
    //shutdown immediately after motion timeout.
    SYS_STATE = 0;

  switch (SYS_STATE)
  {
    case 0:
      readRemoteUntil(100000);
      if (!shutdownComplete)
        sysShutdown();
      break;
    case 1:
      sysReady();
      if (currentDistance < CLOSE_OBJECT)
      //if an object is detected await for departure and subsequent shutdown
        SYS_STATE = 6;
      if (currentDistance > CLOSE_OBJECT)
      // if no object, switch to await arrival state
        SYS_STATE = 2;
      break;
    case 2:
      SYS_STATE += processArrival();
      break;
    case 3:
      SYS_STATE += processCapture();
      break;
    case 4:
      SYS_STATE += processCaptureComplete();
      break;
    case 5:
      SYS_STATE += processTooClose();
      break;
    case 6:
      signalReady();
      //updateExternalState();
      //awaitDeparture();
      break;
  }
}

void updateExternalState()
{ 
  long timeout = 0;

  //if low, then it is OK to ask
  if (digitalRead(STATUS_PIN) == LOW) {
    
    clearSerial();

    //set the interrupt pin high causing the server to write the data to the SP
    digitalWrite(DATA_REQUEST_PIN, HIGH);
    //wait for a delay
    //set the request line low.
    digitalWrite(DATA_REQUEST_PIN, LOW);

    timeout = micros() + MAX_DELAY;

    while (Serial.available() < 5 && (timeout >= micros())) { }
    
    if (Serial.available() >= 5)
    {
      EXT_SYS_STATE = Serial.read();
      mapState(EXT_SYS_STATE);
      
      md[0] = Serial.read();
      md[1] = Serial.read();
      md[2] = Serial.read();
      md[3] = Serial.read();

      memcpy(&meanDistance, &md, sizeof(float));
      
      if (objectPresent)
        currentDistance = meanDistance;
      else
        currentDistance = MAX_DISTANCE;
    }
  }
}

void mapState(byte state)
{
  objectPresent = 0x04 & state;
  motionDetected = 0x02 & state;
  motionTimedout = 0x01 & state;

  PREV_EXT_SYS_STATE = EXT_SYS_STATE;
}

int sysReady() 
{
  shutdownComplete = false;
  signalStartup();
  signalReady();
  return 0;
}

int processArrival()
{
  //this must be done in this function due to state issues.
  return signalAdvance();
}

int processCapture()
{
  long samples = 0;
  float last = meanDistance;
  float averageDistanceVariance;
  
  while (meanDistance < ARRIVAL_MIN_DISTANCE && meanDistance > TOO_CLOSE_DISTANCE) {
    signalCapture();
    readRemoteUntil(100000);
    samples++;
    averageDistanceVariance  += last - meanDistance;
    averageDistanceVariance = averageDistanceVariance / 2;
    last = meanDistance;
    
    if (averageDistanceVariance < 5.0 && samples > 10)
    {
      return 1;
    }  
  }

  if (meanDistance >= ARRIVAL_MIN_DISTANCE)
    return -1;

  if (meanDistance <= TOO_CLOSE_DISTANCE)
    return 2; 
}

int signalCapture()
{
  RgbwColor hiColor = RgbwColor(200, 0, 0, 0);
  RgbwColor lowColor = RgbwColor(20,0,0,0);
  RgbwColor workingColor;

  setRingColor(hiColor);
  ring.Show();
  
  int brightnessStep = 1;
  float stepDelay = 1;
  
  for (float i = 0.0; i <= 1; i += .015)
  {
    workingColor = RgbwColor::LinearBlend(lowColor, hiColor, i);
    
    for (int n = 0; n < PIXEL_COUNT; n++)
      ring.SetPixelColor(n, workingColor);
      
    ring.Show();
    delayMicroseconds(250);
  }

  for (float i = 0.0; i <= 1; i += .015)
  {
    workingColor = RgbwColor::LinearBlend(hiColor, lowColor, i);
    
    for (int n = 0; n < PIXEL_COUNT; n++)
      ring.SetPixelColor(n, workingColor);
      
    ring.Show();
    delayMicroseconds(250);
  }
  
  return 0;
}

int processCaptureComplete()
{
  fadeToColor(RgbwColor(0,0,20,0));
  
  while (meanDistance < ARRIVAL_MIN_DISTANCE && meanDistance > TOO_CLOSE_DISTANCE && !motionTimedout) {
    signalCaptureComplete();
    readRemoteUntil(100000);
  }

  if (meanDistance < ARRIVAL_MIN_DISTANCE)
    return -1;

  return 0;
}

int signalCaptureComplete()
{
  RgbwColor color = RgbwColor(10, 25, 50, 0);
  RgbwColor black = RgbwColor(0,0,20,0);
  RgbwColor workingColor;
  
  int brightnessStep = 1;
  float stepDelay = 8;
  
  for (float i = 0.0; i <= 1; i += .005)
  {
    workingColor = RgbwColor::LinearBlend(black, color, i);
    
    for (int n = 0; n < PIXEL_COUNT; n++)
      ring.SetPixelColor(n, workingColor);
      
    ring.Show();
    delay(stepDelay);
  }

  for (float i = 0.0; i <= 1; i += .005)
  {
    workingColor = RgbwColor::LinearBlend(color, black, i);
    
    for (int n = 0; n < PIXEL_COUNT; n++)
      ring.SetPixelColor(n, workingColor);
      
    ring.Show();
    delay(stepDelay);
  }

  return 0;
}

int fadeToColor(RgbwColor color)
{
  RgbwColor start = ring.GetPixelColor(0);
  RgbwColor black = color;
  RgbwColor workingColor;
  
  int brightnessStep = 1;
  float stepDelay = 8;
  
  for (float i = 0.0; i <= 1; i += .005)
  {
    workingColor = RgbwColor::LinearBlend(start, black, i);
    
    for (int n = 0; n < PIXEL_COUNT; n++)
      ring.SetPixelColor(n, workingColor);
      
    ring.Show();
    delay(stepDelay);
  }
}


void signalTooClose()
{
  RgbwColor workingColor = RgbwColor(200,0,0,0);

  for (int n = 0; n < PIXEL_COUNT; n++)
    ring.SetPixelColor(n, workingColor);

  ring.Show();

  delay(240);

  workingColor = RgbwColor(0,0,0,0);

  for (int n = 0; n < PIXEL_COUNT; n++)
    ring.SetPixelColor(n, workingColor);

  ring.Show();

  delayMicroseconds(120000);

  workingColor = RgbwColor(0,0,0,64);

  for (int n = 0; n < PIXEL_COUNT; n++)
    ring.SetPixelColor(n, workingColor);

  ring.Show();

  delay(100);
}

void readRemoteUntil(long delayValue)
{
  long delayUntil = micros() + delayValue;
  
  while (micros() <= delayUntil)
  {
    updateExternalState();
  }
}

int processTooClose()
{
    while (meanDistance < TOO_CLOSE_DISTANCE) {
      signalTooClose();
      readRemoteUntil(100000);
    }

    return -2;
}




