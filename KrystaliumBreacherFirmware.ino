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


// Number of pixels in the ledring
#define NUMPIXELS                       12 
// If set to 1, it will consider all pixels in the ring as the same. If set to the same number, it will animate each of them individually
#define NUM_LED_GROUPS                  12 

// Any unconnected pin, to try to generate a random seed
#define UNCONNECTED_PIN                 2
#define LEDRINGPIN                      D2 
 
// The LED can be in only one of these states at any given time
#define BRIGHT                          0
#define UP                              1
#define DOWN                            2
#define DIM                             3
#define BRIGHT_HOLD                     4
#define DIM_HOLD                        5


// Percent chance the LED will suddenly fall to minimum brightness
#define FLICKER_BOTTOM_PERCENT          10
// Absolute minimum of the flickering
#define FLICKER_ABSOLUTE_MIN_INTENSITY  32
// Minimum intensity during "normal" flickering (not a dramatic change)
#define FLICKER_MIN_INTENSITY           128
// Maximum intensity of the flickering
#define FLICKER_MAX_INTENSITY           255


// Decreasing brightness will take place over a number of milliseconds in this range
#define DOWN_MIN_MSECS                  20
#define DOWN_MAX_MSECS                  250
// Increasing brightness will take place over a number of milliseconds in this range
#define UP_MIN_MSECS                    20
#define UP_MAX_MSECS                    250
// Percent chance the color will hold unchanged after brightening
#define BRIGHT_HOLD_PERCENT             20
// When holding after brightening, hold for a number of milliseconds in this range
#define BRIGHT_HOLD_MIN_MSECS           0
#define BRIGHT_HOLD_MAX_MSECS           100
// Percent chance the color will hold unchanged after dimming
#define DIM_HOLD_PERCENT                5
// When holding after dimming, hold for a number of milliseconds in this range
#define DIM_HOLD_MIN_MSECS              0
#define DIM_HOLD_MAX_MSECS              50

// Mixes in a certain amount of red in every group.
#define MIN_IMPURITY                    32
#define MAX_IMPURITY                    96
 
#define MINVAL(A,B)             (((A) < (B)) ? (A) : (B))
#define MAXVAL(A,B)             (((A) > (B)) ? (A) : (B))


#define MAX_DISTANCE_METER               12

byte state[NUM_LED_GROUPS];
unsigned long flicker_msecs[NUM_LED_GROUPS];
unsigned long flicker_start[NUM_LED_GROUPS];
byte index_start[NUM_LED_GROUPS];
byte index_end[NUM_LED_GROUPS];
byte impurity[NUM_LED_GROUPS];

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, LEDRINGPIN, NEO_GRB + NEO_KHZ800);


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

double intensity_factor = 0; // Goes from 0 to 1, defined by how far the device is away from the generator

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
  intensity_factor = MAXVAL(MINVAL(getDistance(rssi_average) / MAX_DISTANCE_METER, 1), 0);
  Serial.print("Intensity factor is :");
  Serial.println(intensity_factor);
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

void setFlickerIntensity(byte intensity, int group_index)
{
  int led_index = group_index * (NUMPIXELS / NUM_LED_GROUPS);
  int max_led_index = led_index + (NUMPIXELS / NUM_LED_GROUPS);
  int secondary_intensity;
  int impurity_base = impurity[group_index] / 2;
  int impurity_intensity = impurity_base + (impurity_base * ((float)intensity / (float)FLICKER_MAX_INTENSITY));
  
  // Clamp intensity between max and absolute min.
  intensity = MAXVAL(MINVAL(intensity, FLICKER_MAX_INTENSITY), FLICKER_ABSOLUTE_MIN_INTENSITY);

  if (intensity >= FLICKER_MIN_INTENSITY)
  {
    secondary_intensity = intensity * 3 / 8; 
  } else {
    secondary_intensity = intensity * 3.25 / 8;
  }

  // Scale all the intensities on the factor.
  intensity *= intensity_factor;
  secondary_intensity *= intensity_factor;
  impurity_intensity *= intensity_factor;
  
  for(; led_index < max_led_index; led_index++)
  {
    strip.setPixelColor(led_index, impurity_intensity, secondary_intensity, intensity); 
  }
    
  strip.show();
  return;
}

void setupNeoPixel()
{
  // There is no good source of entropy to seed the random number generator,
  // so we'll just read the analog value of an unconnected pin.  This won't be
  // very random either, but there's really nothing else we can do.
  //
  // True randomness isn't strictlyfor(int group_index necessary, we just don't want a whole
  // string of these things to do exactly the same thing at the same time if
  // they're all powered on simultaneously.
  randomSeed(analogRead(UNCONNECTED_PIN));
  Serial.println("SETTING UP NEOPIXEL");
  for(int group_index = 0; group_index < NUM_LED_GROUPS; group_index++)
  {
    setFlickerIntensity(255, group_index);
    index_start[group_index] = 255;
    index_end[group_index] = 255;
    setFlickerState(BRIGHT, group_index);
    impurity[group_index] = int (random(MIN_IMPURITY, MAX_IMPURITY) + 0.5);
  }
  strip.begin();
}

String findName(uint32_t node_id)
{
  // Yeaaah i know. It's a bit ugly hardcoded, but it's for a quick test.
  if(node_id == 4225251478)
  {
    return "generator";
  } else if(node_id == 4225208703) 
  {
    return "device_a";
  } else if(node_id == 4225178198)
  {
    return "device_b";
  }
  return "unset";
}


void setup() 
{
  Serial.begin(115200);

  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | CONNECTION | SYNC);  // set before init() so that you can see startup messages
  
  uint8_t MAC[] = {0, 0, 0, 0, 0, 0};
  WiFi.softAPmacAddress(MAC);
  uint32_t nodeId = tcp::encodeNodeId(MAC);
  Serial.println(nodeId);
  if(nodeId == 4225251478)
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

  setupNeoPixel();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
  neopixelLoop();
}


void neopixelLoop()
{
  unsigned long current_time; 
  current_time = millis();
  //Serial.println("loop!");
  for(int group_index = 0; group_index < NUM_LED_GROUPS; group_index++)
  {
    switch (state[group_index])
    {
      case BRIGHT:
      {   
        //Serial.println("Bright"); 
        flicker_msecs[group_index] = random(DOWN_MAX_MSECS - DOWN_MIN_MSECS) + DOWN_MIN_MSECS;
        flicker_start[group_index] = current_time;
        index_start[group_index] = index_end[group_index];
        if (index_start[group_index] > FLICKER_ABSOLUTE_MIN_INTENSITY && random(100) < FLICKER_BOTTOM_PERCENT)
        {
          index_end[group_index] = random(index_start[group_index] - FLICKER_ABSOLUTE_MIN_INTENSITY) + FLICKER_ABSOLUTE_MIN_INTENSITY;
        } else {
          index_end[group_index] = random(index_start[group_index]- FLICKER_MIN_INTENSITY) + FLICKER_MIN_INTENSITY;
        }
   
        setFlickerState(DOWN, group_index);
        break;  
      }  
      case DIM:
      {
        //Serial.println("Dim");
        flicker_msecs[group_index] = random(UP_MAX_MSECS - UP_MIN_MSECS) + UP_MIN_MSECS;
        flicker_start[group_index] = current_time;
        index_start[group_index] = index_end[group_index];
        index_end[group_index] = random(FLICKER_MAX_INTENSITY - index_start[group_index]) + FLICKER_MIN_INTENSITY;
        setFlickerState(UP, group_index);
        break;
      }
      case BRIGHT_HOLD:  
      case DIM_HOLD:
      {
        //Serial.println("DIM Hold");
        if (current_time >= (flicker_start[group_index] + flicker_msecs[group_index]))
        {
          setFlickerState(state[group_index] == BRIGHT_HOLD ? BRIGHT : DIM, group_index); 
        }
        break;
      }
      case UP:
      case DOWN:
      {
        //  Serial.println("Down");
        if (current_time < (flicker_start[group_index] + flicker_msecs[group_index])) {
          setFlickerIntensity(index_start[group_index] + ((index_end [group_index]- index_start[group_index]) * (((current_time - flicker_start[group_index]) * 1.0) / flicker_msecs[group_index])), group_index);
        } else {
          setFlickerIntensity(index_end[group_index], group_index);
   
          if (state[group_index] == DOWN)
          {
            if (random(100) < DIM_HOLD_PERCENT)
            {
              flicker_start[group_index] = current_time;
              flicker_msecs[group_index] = random(DIM_HOLD_MAX_MSECS - DIM_HOLD_MIN_MSECS) + DIM_HOLD_MIN_MSECS;
              setFlickerState(DIM_HOLD, group_index);
            } else {
              setFlickerState(DIM, group_index);
            } 
          } else {
            if (random(100) < BRIGHT_HOLD_PERCENT)
            {
              flicker_start[group_index] = current_time;
              flicker_msecs[group_index] = random(BRIGHT_HOLD_MAX_MSECS - BRIGHT_HOLD_MIN_MSECS) + BRIGHT_HOLD_MIN_MSECS;
              setFlickerState(BRIGHT_HOLD, group_index);
            } else {
              setFlickerState(BRIGHT,group_index);
            }
          }
        }
        break;
      }
    }
  }
}

void setFlickerState(byte new_state, int group_index)
{
  state[group_index] = new_state;
}
