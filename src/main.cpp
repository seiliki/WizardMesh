#include <Arduino.h>
#include <painlessMesh.h>
#include <FastLED.h>
#include <ArduinoJson.h>

#define   LED             D4 
#define   BLINK_PERIOD    3000 // milliseconds until cycle repeat
#define   BLINK_DURATION  100  // milliseconds LED is on for
#define   MESH_SSID       "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

#define   NUM_LEDS 5
#define   DATA_PIN 3

CRGB leds[NUM_LEDS];
CRGB NodeColor = CRGB::MediumPurple; // set your color here

/*
 * COLOR CODE DRAFT:
 * RED: COME HELP ME
 * YELLOW: MY LIFE IS FORFEIT STAY AWAY
 * GREEN: THIS IS GOOD
 * BLUE: I AM WIZARD
 * WHITE: WE LIT
 * PURPLE: I NEED DRINK
 * PINK: HUNGRY
 */

vector<CRGB> colorlist = {
  CRGB::Red,
  CRGB::Yellow,
  CRGB::Green,
  CRGB::Blue,
  CRGB::WhiteSmoke,
  CRGB::BlueViolet,
  CRGB::PaleVioletRed
};


// Prototypes
void sendMessage(); 
void receivedCallback(uint32_t from, String & msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback(); 
void nodeTimeAdjustedCallback(int32_t offset); 
void delayReceivedCallback(uint32_t from, int32_t delay);
void setLightsforStaff(uint32_t nodeid, uint8_t r, uint8_t g, uint8_t b);
CRGB potToColor(int pot);
CRGB makeRandomColor();
CRGB nextColor();

//
Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;
bool calc_delay = false;
SimpleList<uint32_t> nodes;
void sendMessage() ; // Prototype
Task taskSendMessage( TASK_SECOND * 1, TASK_FOREVER, &sendMessage ); // start with a one second interval
// Task to blink the number of nodes

Task blinkNoNodes;
bool onFlag = false;

int colorIndex = 0;

int l_sw;
int sw;

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  // mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  // mesh.setDebugMsgTypes(ERROR | DEBUG | CONNECTION | COMMUNICATION);  // set before init() so that you can see startup messages
  // mesh.setDebugMsgTypes(ERROR | DEBUG | CONNECTION);  // set before init() so that you can see startup messages
  mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);
  userScheduler.addTask( taskSendMessage );
  taskSendMessage.enable();
  blinkNoNodes.set(BLINK_PERIOD, (mesh.getNodeList().size() + 1) * 2, []() {
      // If on, switch off, else switch on
      if (onFlag)
        onFlag = false;
      else
        onFlag = true;
      blinkNoNodes.delay(BLINK_DURATION);

      if (blinkNoNodes.isLastIteration()) {
        // Finished blinking. Reset task for next run 
        // blink number of nodes (including this node) times
        blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
        // Calculate delay based on current mesh time and BLINK_PERIOD
        // This results in blinks between nodes being synced
        blinkNoNodes.enableDelayed(BLINK_PERIOD - 
            (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
      }
  });
  userScheduler.addTask(blinkNoNodes);
  blinkNoNodes.enable();
  randomSeed(analogRead(A0));
  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  leds[2] = CRGB::Black;
  leds[3] = CRGB::Black;
  leds[4] = CRGB::Black;
}

void loop() {
  userScheduler.execute(); // it will run mesh scheduler as well
  mesh.update();
  digitalWrite(LED, !onFlag);
  
  leds[0] = NodeColor;
  FastLED.show();

  l_sw = sw;
  sw = digitalRead(D1);
  if(l_sw == 0 && sw == 1){
    NodeColor = nextColor();
  }
  // sw = analogRead(D2);
  // NodeColor = potToColor(sw);
}

CRGB makeRandomColor(){
  return(CRGB(random(0,255),random(0,255),random(0,255)));
}

CRGB potToColor(int pot){
  int idx = (pot-1)/(1024.0/(7.0));
  Serial.printf("%i\n", pot);
  return colorlist[idx];
}

CRGB nextColor(){
  colorIndex++;
  if (colorIndex == colorlist.size()){
    colorIndex = 0 ;
  }
  return colorlist[colorIndex];
}

void sendMessage() {
  DynamicJsonBuffer jsonBuffer;
  JsonObject& wmsg = jsonBuffer.createObject();
  wmsg["nodeid"] = mesh.getNodeId();
  wmsg["r"] = NodeColor[0];
  wmsg["g"] = NodeColor[1];
  wmsg["b"] = NodeColor[2];
  String msg;
  wmsg.printTo(msg);
  mesh.sendBroadcast(msg);

  if (calc_delay) {
    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      mesh.startDelayMeas(*node);
      node++;
    }
    calc_delay = false;
  }

  Serial.printf("Sending message: %s\n", msg.c_str());
  
  taskSendMessage.setInterval( TASK_SECOND * 1);
}


void receivedCallback(uint32_t from, String & msg) {
  DynamicJsonBuffer jsonBuffer;
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
  if(msg[0] == '{'){
    JsonObject& root = jsonBuffer.parseObject(msg);
    uint32_t nid = root["nodeid"];
    uint8_t r , g , b;
    r = root["r"];
    g = root["g"];
    b = root["b"];
    setLightsforStaff(nid, r , g , b);
  }
}

void newConnectionCallback(uint32_t nodeId) {
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
 
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections %s\n", mesh.subConnectionJson().c_str());
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
 
  nodes = mesh.getNodeList();

  Serial.printf("Num nodes: %d\n", nodes.size());
  Serial.printf("Connection list:");

  SimpleList<uint32_t>::iterator node = nodes.begin();
  while (node != nodes.end()) {
    Serial.printf(" %u", *node);
    node++;
  }
  Serial.println();
  calc_delay = true;
  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  leds[2] = CRGB::Black;
  leds[3] = CRGB::Black;
  leds[4] = CRGB::Black;
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void delayReceivedCallback(uint32_t from, int32_t delay) {
  Serial.printf("Delay to node %u is %d us\n", from, delay);
}

void setLightsforStaff(uint32_t nodeid, uint8_t r, uint8_t g, uint8_t b) { 
  int idx = 0;
  nodes = mesh.getNodeList();
  SimpleList<uint32_t>::iterator node = nodes.begin();
  while (node != nodes.end()) {
    node++;
    idx++;
  }

  if (idx <= NUM_LEDS){
    leds[idx] = CRGB(r, g, b);
  }
}