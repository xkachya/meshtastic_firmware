/**
 * @file DzhagaModule.cpp
 * @brief Implementation of the DzhagaModule class.
 *
 * This file contains the implementation of the DzhagaModule class, which is responsible for handling the Dzhaga module functionality.
 * The Dzhaga module can be configured to operate in two modes: remote mode and target mode. In remote mode, it listens for detection events
 * from external sensors and sends messages to the mesh network when an event is detected. In target mode, it receives messages from the mesh
 * network and performs actions based on the received messages.
 *
 * The DzhagaModule class provides methods for initializing the module, sending detection messages, sending current state messages,
 * checking for detection events, and allocating reply packets.
 */
#include "DzhagaModule.h"
#include "Default.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "configuration.h"
#include "main.h"
DzhagaModule *dzhagaModule;

#include <assert.h>

#define GPIO_POLLING_INTERVAL 100
#define DELAYED_INTERVAL 1000

// Assuming these constants are defined appropriately elsewhere in your codebase
const int MESSAGE_BUFFER_SIZE = 40;
const char BELL_CHARACTER = '\7';

int32_t DzhagaModule::runOnce()
{
    /*
        Uncomment the preferences below if you want to use the module
        without having to configure it from the PythonAPI or WebUI.
    */
    moduleConfig.dzhaga.enabled = false;
    // moduleConfig.dzhaga.mode = meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_REMOTE;
    // moduleConfig.dzhaga.mode = meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_TARGET;
    // moduleConfig.dzhaga.ready_btn_pin = 2; // 
    // moduleConfig.dzhaga.frbtn_pin_1 = 13;  // 
    // moduleConfig.dzhaga.frbtn_pin_2 = 15;
    // moduleConfig.dzhaga.frbtn_pin_3 = 14;
    // moduleConfig.dzhaga.frbtn_triggered_high = false;
    // moduleConfig.dzhaga.frbtn_use_pullup = true;
    // strcpy(moduleConfig.dzhaga.remote_node, "Motion");
    // moduleConfig.dzhaga.remote_node_num = 2;

    if (!moduleConfig.dzhaga.enabled || 
         moduleConfig.dzhaga.mode == meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_NONE)
        return disable();

    remoteNodeNumber = moduleConfig.dzhaga.remote_node_num;

    if (moduleConfig.dzhaga.mode == meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_REMOTE) {
        if (firstTime) {
            firstTime = false;
            if (moduleConfig.dzhaga.ready_btn_pin > 0) {
                pinMode(moduleConfig.dzhaga.ready_btn_pin, moduleConfig.dzhaga.frbtn_use_pullup ? INPUT_PULLUP : INPUT);
            } 
            if (moduleConfig.dzhaga.frbtn_pin_1 > 0) {
                pinMode(moduleConfig.dzhaga.frbtn_pin_1, moduleConfig.dzhaga.frbtn_use_pullup ? INPUT_PULLUP : INPUT);
            } 
            if (moduleConfig.dzhaga.frbtn_pin_2 > 0) {
                pinMode(moduleConfig.dzhaga.frbtn_pin_2, moduleConfig.dzhaga.frbtn_use_pullup ? INPUT_PULLUP : INPUT);
            }
            if (moduleConfig.dzhaga.frbtn_pin_3 > 0) {
                pinMode(moduleConfig.dzhaga.frbtn_pin_3, moduleConfig.dzhaga.frbtn_use_pullup ? INPUT_PULLUP : INPUT);
            } 
            if (moduleConfig.dzhaga.ready_btn_pin <= 0 || moduleConfig.dzhaga.frbtn_pin_1 <= 0 || moduleConfig.dzhaga.frbtn_pin_2 <= 0 || moduleConfig.dzhaga.frbtn_pin_3 <= 0) {
                LOG_WARN("Dzhaga Module: Set to enabled but not one button pins are set. Disabling module...\n");
                return disable();
            }
            LOG_INFO("Dzhaga Module: Initializing\n");

            return DELAYED_INTERVAL;
        }

        if ((millis() - lastSentToMesh) >= Default::getConfiguredOrDefaultMs(minimum_broadcast_secs) &&
            hasDetectionEvent()) {
            sendDetectionMessage();
            return DELAYED_INTERVAL;
        }
        // Even if we haven't detected an event, broadcast our current state to the mesh on the scheduled interval as a sort
        // of heartbeat. We only do this if the minimum broadcast interval is greater than zero, otherwise we'll only broadcast state
        // change detections.
        else if (state_broadcast_secs > 0 &&
                 (millis() - lastSentToMesh) >= Default::getConfiguredOrDefaultMs(state_broadcast_secs)) {
            sendCurrentStateMessage();
            return DELAYED_INTERVAL;
        }
    } else if (moduleConfig.dzhaga.mode == meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_TARGET) {
        // TODO
    }


    return GPIO_POLLING_INTERVAL;
}


void DzhagaModule::sendDetectionMessage()
{
    LOG_DEBUG("Detected event observed. Sending message\n");
    char *message = new char[MESSAGE_BUFFER_SIZE];
    int offset = 0;

    offset += snprintf(message + offset, MESSAGE_BUFFER_SIZE - offset, "DZHAGA");
    for (int i = 0; i < 4; ++i) {
        offset += snprintf(message + offset, MESSAGE_BUFFER_SIZE - offset, ":%s", detectedState[i] ? "TRUE" : "FALSE");
        if (offset >= MESSAGE_BUFFER_SIZE - 1) break;                   // Ensure room for null terminator
    }

    //sprintf(message, "%s detected", moduleConfig.detection_sensor.name);
    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p) {
        LOG_ERROR("Failed to allocate meshtastic_MeshPacket");
        delete[] message;
        return;                                                         // Early return on allocation failure
    }

    p->want_ack = true;
    p->to = remoteNodeNumber;
    p->decoded.payload.size = strlen(message);
    memcpy(p->decoded.payload.bytes, message, p->decoded.payload.size);

    if (moduleConfig.detection_sensor.send_bell && p->decoded.payload.size + 1 < meshtastic_Constants_DATA_PAYLOAD_LEN) {
        p->decoded.payload.bytes[p->decoded.payload.size] = BELL_CHARACTER;
        p->decoded.payload.size++;
        p->decoded.payload.bytes[p->decoded.payload.size] = '\0';       // Ensure null terminator after bell character
    }

    LOG_INFO("Sending message id=%d, dest=%x, msg=%.*s\n", p->id, p->to, p->decoded.payload.size, p->decoded.payload.bytes);
    lastSentToMesh = millis();
    service.sendToMesh(p);
    delete[] message;    
}


void DzhagaModule::sendCurrentStateMessage()
{
    LOG_DEBUG("Events status update. Sending message\n");
    char *message = new char[MESSAGE_BUFFER_SIZE];
    int offset = 0;

    offset += snprintf(message + offset, MESSAGE_BUFFER_SIZE - offset, "DZHAGA");
    for (int i = 0; i < 4; ++i) {
        offset += snprintf(message + offset, MESSAGE_BUFFER_SIZE - offset, ":%s", detectedState[i] ? "TRUE" : "FALSE");
        if (offset >= MESSAGE_BUFFER_SIZE - 1) break;                   // Ensure room for null terminator
    }

    //sprintf(message, "%s detected", moduleConfig.detection_sensor.name);
    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p) {
        LOG_ERROR("Failed to allocate meshtastic_MeshPacket");
        delete[] message;
        return;                                                         // Early return on allocation failure
    }

    p->want_ack = true;
    p->to = remoteNodeNumber;
    p->decoded.payload.size = strlen(message);
    memcpy(p->decoded.payload.bytes, message, p->decoded.payload.size);

    LOG_INFO("Sending message id=%d, dest=%x, msg=%.*s\n", p->id, p->to, p->decoded.payload.size, p->decoded.payload.bytes);
    lastSentToMesh = millis();
    service.sendToMesh(p);
    delete[] message;
}


bool DzhagaModule::hasDetectionEvent()
{
    bool detectionState = false;
    detectedState = {false, false, false, false};
    bool triggeredHigh = moduleConfig.dzhaga.frbtn_triggered_high; // assuming frbtn_triggered_high is the same for all sensors

    bool currentStateA = digitalRead(moduleConfig.dzhaga.ready_btn_pin);
    bool currentStateB = digitalRead(moduleConfig.dzhaga.frbtn_pin_1);
    bool currentStateC = digitalRead(moduleConfig.dzhaga.frbtn_pin_2);
    bool currentStateD = digitalRead(moduleConfig.dzhaga.frbtn_pin_3);
    LOG_DEBUG("Dzhaga Module: Current pins state: A=%i, B=%i, C=%i, D=%i\n", currentStateA, currentStateB, currentStateC, currentStateD);
  
    detectedState[0] = triggeredHigh ? currentStateA : !currentStateA;
    detectedState[1] = triggeredHigh ? currentStateB : !currentStateB;
    detectedState[2] = triggeredHigh ? currentStateC : !currentStateC;
    detectedState[3] = triggeredHigh ? currentStateD : !currentStateD;
    LOG_DEBUG("Dzhaga Module: Detected pins state: %i, %i, %i, %i\n", detectedState[0], detectedState[1], detectedState[2], detectedState[3]);
  
    if (triggeredHigh) {
        detectionState = currentStateA || currentStateB || currentStateC || currentStateD;
    } else {
        detectionState = !currentStateA || !currentStateB || !currentStateC || !currentStateD;
    }

    LOG_DEBUG("Dzhaga Module: Detected state: %i\n", detectionState);
    return detectionState;
}

/// Find a node in our DB, return null for missing
/// NOTE: This function might be called from an ISR
/*
meshtastic_NodeInfoLite *DzhagaModule::getMeshNode(NodeNum n)
{
    pb_size_t numMeshNodes = nodeDB->numMeshNodes;
    meshtastic_NodeInfoLite *ourNode = nodeDB->getMeshNode(nodeDB->getNodeNum());
    
    for (int i = 0; i < numMeshNodes; i++)
    //    if (meshNodes->at(i).num == n)
    //        return &meshNodes->at(i);

    return NULL;
}
*/

meshtastic_MeshPacket *DzhagaModule::allocReply()
{
    assert(currentRequest); // should always be !NULL
#ifdef DEBUG_PORT
    auto req = *currentRequest;
    auto &p = req.decoded;
    // The incoming message is in p.payload
    LOG_INFO("Received message from=0x%0x, id=%d, msg=%.*s\n", req.from, req.id, p.payload.size, p.payload.bytes);
#endif

    screen->print("Sending reply\n");

    const char *replyStr = "Message Received";
    auto reply = allocDataPacket();                 // Allocate a packet for sending
    reply->decoded.payload.size = strlen(replyStr); // You must specify how many bytes are in the reply
    memcpy(reply->decoded.payload.bytes, replyStr, reply->decoded.payload.size);

    return reply;
}
