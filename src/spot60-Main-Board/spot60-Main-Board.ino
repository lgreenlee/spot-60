//pin defintions
const byte SAMPLE_COUNT = 6;
int PIR_PIN = 2;
int TRIGGER_PIN = 4;
int ECHO_PIN = 6;
int STATUS_PIN = 8;
int DATA_REQUEST_PIN = 3;
float MAXIMUM_RANGE = 200;
int MOTION_TIMEOUT = 30; //s
int RANGING_DELAY = 60; //ms
float samples[] = { 400, 400, 400, 400, 400, 400 };
byte sampleIndex = 0;
float accumulator = 0.0;

int flightTime = 0;
int lastDistance = 0;
float meanDistance = 0;
byte md[4] = { 0,0,0,0 };

bool motionTimedout;
bool motion = false;
long lastMotion = 0;

bool objectPresent = false;
long lastObjectDetection = 0;

bool readReady = false;

/*
 * 000001 No object present, no motion, motion timer active
 * 000000 No object present, no motion, motion timer inactive
 * 000011 No object present, motion, motion timer active
 * 000010 No object present, motion, motion timer inactive
 * 000101 Object present, no motion, motion timer active
 * 000100 Object present, no motion, motion timer inactive
 * 000111 Object present, motion, motion timer active
 * 000110 Object present, motion, motion timer inactive
 */
byte state = 1;

void setup() 
{  
  Serial.begin(115200);
  //Setup the PIR input pin
  pinMode(PIR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), motionChange, CHANGE);
  
  //Setup the ranging trigger pin
  pinMode(TRIGGER_PIN, OUTPUT);
  //The start state is low
  digitalWrite(TRIGGER_PIN, LOW);
  
  //Listening for the ranger response here
  pinMode(ECHO_PIN, INPUT);

  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, HIGH);

  pinMode(DATA_REQUEST_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(DATA_REQUEST_PIN), writeData, FALLING);
}

void rangingStateChange()
{
  readReady = false;
  //indicates that the system is busy ranging.
  digitalWrite(STATUS_PIN, HIGH); //talk to the hand
}

void listeningStateChange()
{
  readReady = true;
  //indicates that the system has updated the data and is ready to read.
  digitalWrite(STATUS_PIN, LOW); //now you can talk to me
}

int getFlightTime()
{
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  int flightTime = pulseIn(ECHO_PIN, HIGH, 28000); //returns the microseconds of the pulse from the ranger.

  if (flightTime <= 0)
    flightTime = -1;

  return flightTime;
}

int calculateRange(int flightTime)
{
  if (flightTime == -1)
    return flightTime;
  else
    return (flightTime / 29 / 2);
}

void writeData()
{ 
  //if the latest data has not been read, write it to the serial port.
  if (readReady) 
  {
    readReady = false;
    digitalWrite(STATUS_PIN, HIGH);
    
    Serial.write(state);
    Serial.write(md[0]);
    Serial.write(md[1]);
    Serial.write(md[2]);
    Serial.write(md[3]);
    Serial.flush();
  }
}

void motionChange()
{
  motion = digitalRead(PIR_PIN);
  lastMotion = micros();
}

bool isObjectPresent()
{
  if (lastDistance  == -1)
    return false;
  else {
    return true;
  }
}

bool isMotionTimeout()
{
  return (micros() > (lastMotion + MOTION_TIMEOUT * 1000000));
}

void processState()
{
  state = 0x00 | objectPresent;
  state = state << 1;
  state = state | motion;
  state = state << 1;
  state = state | motionTimedout;
}

void updateAverage(float sample)
{
  //if the sample is a timeout, don't include it in the measurement.
  if (sample == -1)
    return;
  
  sampleIndex++;

  //if the end is reached, roll over
  if (sampleIndex == SAMPLE_COUNT)
    sampleIndex = 0;

  samples[sampleIndex] = sample; 
}

float calculateMeanDistance()
{
  accumulator = 0.0;

  for (int i = 0; i < SAMPLE_COUNT; i++)
    accumulator += samples[i];
  return accumulator / SAMPLE_COUNT;
}

void loop() {
  rangingStateChange();
  lastDistance = calculateRange(getFlightTime());
  updateAverage(lastDistance);
  meanDistance = calculateMeanDistance();
  memcpy(&md, &meanDistance, sizeof(float));
  objectPresent = isObjectPresent();
  motionTimedout = isMotionTimeout();
  processState();
  listeningStateChange();
  delay(RANGING_DELAY); 
}
