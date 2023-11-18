#include "namedMesh.h" // Use a variant of the painless mesh that allows for names instead of just unique ID's

#define   MESH_PREFIX     "KrystaliumBreacher"
#define   MESH_PASSWORD   "somekindofpassword"
#define   MESH_PORT       5555

Scheduler userScheduler; // to control your personal task
namedMesh  mesh;

// Prototype functions
void sendMessage() ; 
void printSubConnection();

String nodeName;

Task taskSendMessage( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );
Task taskPrintSubConnection(TASK_SECOND * 1, TASK_FOREVER, &printSubConnection);

void sendMessage() {
  String msg = "Hi from node ";
  msg += mesh.getNodeId();
  mesh.sendBroadcast( msg );
  taskSendMessage.setInterval(TASK_SECOND * 5);
}

void printSubConnection()
{
  Serial.print(mesh.subConnectionJson());
  taskPrintSubConnection.setInterval(TASK_SECOND * 10);
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


void setup() {
  Serial.begin(115200);

  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  //mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  nodeName = findName(mesh.getNodeId());
  mesh.setName(nodeName);
  
  mesh.onReceive([](uint32_t from, String &msg) {
    Serial.printf("Received message by id from: %u, \"%s\"\n", from, msg.c_str());
  });

  mesh.onReceive([](String &from, String &msg) {
    Serial.printf("Received message by name from: %s, \"%s\"\n", from.c_str(), msg.c_str());
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
  
  if(mesh.isRoot())
  {
    Serial.println("This device is root, starting printNetworkTask");
    userScheduler.addTask( taskPrintSubConnection );
    taskPrintSubConnection.enable();
  } else {
    Serial.println("This device is not setup to be root");
  }
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}
