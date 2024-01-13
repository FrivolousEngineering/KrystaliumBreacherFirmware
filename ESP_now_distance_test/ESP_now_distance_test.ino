// NOTE; In order for the namedmesh to work, you need to simlink the files into the arduino folder. What i did; I downloaded the painless mesh and replaced the src folder with the
//painless mesh version that i've added

#include "namedMesh.h" // Use a variant of the painless mesh that allows for names instead of just unique ID's
#include "kalman_simple.h"

#include <Adafruit_NeoPixel.h>

#define   MESH_PREFIX     "KrystaliumBreacher"
#define   MESH_PASSWORD   "somekindofpassword"
#define   MESH_PORT       5555

Scheduler userScheduler; // to control your personal task
namedMesh  mesh;

Kalman myFilter(0.15, 20, 20, -50.);

Adafruit_NeoPixel strip = Adafruit_NeoPixel(12, D1, NEO_GRB + NEO_KHZ800);

// Prototype functions
void sendMessage() ; 
void printSubConnection();
void printWifiRSSI();
void calculateDistanceFromRSSI();

String nodeName;

Task taskSendMessage( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );
Task taskPrintSubConnection(TASK_SECOND * 1, TASK_FOREVER, &printSubConnection);
Task printWifiRSSITask(TASK_SECOND * 1, TASK_FOREVER, &printWifiRSSI);
Task calculateDistanceFromRSSITask(TASK_SECOND * 0.5, TASK_FOREVER, &calculateDistanceFromRSSI);


float reference_power  = -54; // rssi reference (Power at 1 meter)
float distance_factor = 3.5; 

double rssi_average = -50.0;
double last_wifi_rssi = -50;

float getDistance(const float rssi)
{ 
  return pow(10, (reference_power - rssi) / (10 * distance_factor)); 
}

void printWifiRSSI()
{

    //Serial.println("WiFi signal: " + String(WiFi.RSSI()) + " db");
}

void calculateDistanceFromRSSI()
{
  last_wifi_rssi = (double)WiFi.RSSI();
  rssi_average = (double)myFilter.getFilteredValue(last_wifi_rssi);
  // Hacked in there. Yaaay
  if(mesh.isRoot())
  {
    // DO NOTHING!
    return;
  }
  int r_factor = 0;
  int g_factor = 0;

  String parent_name = findName(painlessmesh::tcp::encodeNodeId(WiFi.BSSID()));

  if(parent_name == "generator")
  {
    r_factor = 0;
    g_factor = 1;
  } else
  {
    r_factor = 1;
    g_factor = 0;
  }

  if(WiFi.status() != WL_CONNECTED)
  {
    r_factor = 0;
    g_factor = 0;
  }

  
  for(int i = 0; i < 12; i++)
  {
    if(i < getDistance(rssi_average))
    {
      strip.setPixelColor(i, r_factor * 80, g_factor * 80, 0); 
    } else
    {
      strip.setPixelColor(i,0,0,0);
    } 
  }
  strip.show();
}

void sendMessage() {
  if(mesh.isRoot())
  {
    return;
  }
  String msg = "Estimated distance from node ";
  msg += mesh.getNodeId();
  msg += " " + String(getDistance(rssi_average)) + " meter and RSSI_avg " + rssi_average + " rssi_cur " + last_wifi_rssi;
  mesh.sendBroadcast( msg );

  //Serial.println("Sending message: " + msg);  
  taskSendMessage.setInterval(TASK_SECOND * 5);
}
void printSubConnection()
{
  Serial.println(mesh.subConnectionJson());
  taskPrintSubConnection.setInterval(TASK_SECOND * 10);
}

String findName(uint32_t node_id)
{
  // Yeaaah i know. It's a bit ugly hardcoded, but it's for a quick test.
  if(node_id == 4225178198)
  {
    return "generator";
  } else if(node_id == 4225208703) 
  {
    return "device_a";
  } else if(node_id == 4225251478)
  {
    return "device_b";
  }
  return "unset";
}


void setup() {
  Serial.begin(115200);

  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | CONNECTION | SYNC);  // set before init() so that you can see startup messages
  
  uint8_t MAC[] = {0, 0, 0, 0, 0, 0};
  WiFi.softAPmacAddress(MAC);
  uint32_t nodeId = tcp::encodeNodeId(MAC);
  Serial.println(nodeId);
  if(nodeId == 4225178198)
  {
    // This means that it is the generator and we want it to start in AP mode. 
    // The reason for this is that we want devices to connect to the generator. We don't want the generator to connect to the devices.
    // If a generator connects to a device, the device can't broadcast the RSSI back, since it will always report "31 db" since it's not connected, something
    // is connecting to it. 
    Serial.println("Starting in WIFI_AP mode");
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP);
  } else {
    Serial.println("Starting in WIFI_STA mode");
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_STA);
  }


  //mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  nodeName = findName(mesh.getNodeId());
  mesh.setName(nodeName);

  mesh.onReceive([](String &from, String &msg) {
    //Serial.printf("Received message by name from: %s, \"%s\"\n", from.c_str(), msg.c_str());
  });

  mesh.onChangedConnections([]() {
    Serial.printf("Changed connection\n");
  });

  
  if(mesh.getName() == "generator")
  {
    mesh.setRoot(true);
  }

  Serial.printf("This node is known as: %s\n", mesh.getName().c_str());
  
  // Tell the network that there is a root. 
  mesh.setContainsRoot(true);

  userScheduler.addTask( taskSendMessage );
  taskSendMessage.enable();

  userScheduler.addTask(printWifiRSSITask);
  printWifiRSSITask.enable();

  userScheduler.addTask(calculateDistanceFromRSSITask);
  calculateDistanceFromRSSITask.enable();


  userScheduler.addTask( taskPrintSubConnection );
  taskPrintSubConnection.enable();
  
  if(mesh.isRoot())
  {
    Serial.println("This device is root, starting printNetworkTask");
    
  } else {
    Serial.println("This device is not setup to be root");
  }

  strip.begin();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}
