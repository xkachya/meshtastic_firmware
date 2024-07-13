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
#include "sleep.h"

DzhagaModule *dzhagaModule;

bool targetCurrentState[4] = {};
uint32_t targetTurnedOn[4] = {};

// Power cycle variables
bool isReadyOne = false;
bool pairPresent = false;
uint32_t readyOneLastCheckTime = 0;
uint32_t readyOneLastReplyTime = 0;
//uint32_t readyOneLastReplyTime = MAX_UINT32;
uint32_t activePhaseStartTime = 0;


//#include <assert.h>
#include <sstream>


#define GPIO_POLLING_INTERVAL 100
#define DELAYED_INTERVAL 1000
#define ASCII_BELL 0x07

// Assuming these constants are defined appropriately elsewhere in your codebase
const int MESSAGE_BUFFER_SIZE = 40;
const char BELL_CHARACTER = '\7';

int32_t DzhagaModule::runOnce()
{
    // In summary, this code snippet is responsible for managing the ReadyOne status of the Dzhaga module.
    if (moduleConfig.dzhaga.mode == meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_REMOTE) {

        if (readyOneLastCheckTime == 0) {
            checkReadyOneStatus();
            LOG_DEBUG("[DZHAGA]: ReadyOne status initial check: isReadyOne = %s\n", isReadyOne ? "TRUE" : "FALSE");
        } else if ((millis() - readyOneLastCheckTime) > Default::getConfiguredOrDefaultMs(moduleConfig.dzhaga.ready_one_secs)) {
            // If the ready_one_secs is reached, chexck for the isReadyOne status
            checkReadyOneStatus();
            LOG_DEBUG("[DZHAGA]: ReadyOne status check due to {ready_one_secs} paassed: isReadyOne = %s\n", isReadyOne ? "TRUE" : "FALSE");
        }

    }

    // In summary, this code snippet is responsible for managing the ReadyOne status of the Dzhaga module.
    if (moduleConfig.dzhaga.mode == meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_TARGET) {

        if (readyOneLastCheckTime == 0) {
            checkReadyOneStatus();
            LOG_DEBUG("[DZHAGA]: ReadyOne status initial check: isReadyOne = %s\n", isReadyOne ? "TRUE" : "FALSE");
        } else if ((millis() - readyOneLastReplyTime) > Default::getConfiguredOrDefaultMs(moduleConfig.dzhaga.ready_one_timeout)) {
            // If the ready_one_timeout is reached, go to sleep
            if (isReadyOne) {
                isReadyOne = false;
                LOG_DEBUG("[DZHAGA]: due to {ready_one_timeout} paassed: isReadyOne = FALSE\n");
            }
        } else if ((millis() - readyOneLastCheckTime) > Default::getConfiguredOrDefaultMs(moduleConfig.dzhaga.ready_one_secs)) {
            // If the ready_one_secs is reached, chexck for the isReadyOne status
            checkReadyOneStatus();
            LOG_DEBUG("[DZHAGA]: ReadyOne status check due to {ready_one_secs} paassed: isReadyOne = %s\n", isReadyOne ? "TRUE" : "FALSE");
        }

    }

    // In summary, this code snippet is responsible for managing the active phase of the Dzhaga module's 
    // operation based on a set of conditions. It uses configuration parameters and state variables to 
    // determine when to start and end the active phase, transitioning the device between active and sleep 
    // modes as part of its power management strategy.
    if (!isReadyOne && 
        moduleConfig.dzhaga.power_saving_enabled &&
        moduleConfig.dzhaga.power_sleep_secs > 0 &&
        moduleConfig.dzhaga.power_active_secs > 0 &&
        moduleConfig.dzhaga.mode == meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_TARGET &&
        (millis() - activePhaseStartTime) > Default::getConfiguredOrDefaultMs(moduleConfig.dzhaga.power_active_secs)) {
        
        if (activePhaseStartTime == 0) {
            activePhaseStartTime = millis();
            LOG_DEBUG("[DZHAGA]: Starting active phase for %d seconds \n", moduleConfig.dzhaga.power_active_secs);
        } else {
            LOG_DEBUG("[DZHAGA]: Ending active phase\n");
            sleepOnNextExecution = true;
            activePhaseStartTime = 0;
        }

    }
    
    
    // In summary, this code snippet is designed to manage a device's power consumption 
    // by putting it into a deep sleep state for a configured duration. It ensures that 
    // this action is taken only once per instruction by resetting the sleep flag and provides 
    // flexibility in configuring the sleep duration. 
    // Additionally, it includes logging for debugging and operational verification.
    if (sleepOnNextExecution == true) {
        sleepOnNextExecution = false;
        uint32_t nightyNightMs = Default::getConfiguredOrDefaultMs(moduleConfig.dzhaga.power_sleep_secs);
        LOG_DEBUG("[DZHAGA]: Sleeping for %ims, then awaking to send metrics again.\n", nightyNightMs);
        doDeepSleep(nightyNightMs, true);
    } else {
        //LOG_DEBUG("[DZHAGA]: sleepOnNextExecution = %d\n", sleepOnNextExecution);
    }

    /*
        Uncomment the preferences below if you want to use the module
        without having to configure it from the PythonAPI or WebUI.
    */
    // moduleConfig.dzhaga.enabled = false;
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
    // moduleConfig.dzhaga.ready_one_pin = 12;
    // moduleConfig.dzhaga.ready_one_led_pin = 25;
    // moduleConfig.dzhaga.ready_one_secs = 3;
    // moduleConfig.dzhaga.ready_one_timeout = 120;

    //LOG_DEBUG("runOnce(): Start ...\n");
    //LOG_DEBUG("Module enabled: %d\n", moduleConfig.dzhaga.enabled);
    //LOG_DEBUG("Module mode: %d\n", moduleConfig.dzhaga.mode);

    if (!moduleConfig.dzhaga.enabled || 
         moduleConfig.dzhaga.mode == meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_NONE)
        return disable();

    //remoteNodeNumber = moduleConfig.dzhaga.remote_node_num;
    //remoteNodeNumber = nodeDB->getNodeNum();
    //remoteNodeNumber = getNodeNumber(moduleConfig.dzhaga.remote_node);

    // Convert the remote node name to a node number
    std::string remoteNodeStr(moduleConfig.dzhaga.remote_node);
    if (remoteNodeStr.length() > 0) {
        remoteNodeNumber = static_cast<uint32_t>(std::stoul(moduleConfig.dzhaga.remote_node, nullptr, 16));
    } else {
        // Handle the case when remote_node is empty
        remoteNodeNumber = nodeDB->getNodeNum();
    }
    //remoteNodeNumber = static_cast<uint32_t>(std::stoul(moduleConfig.dzhaga.remote_node, nullptr, 16));
    //LOG_DEBUG("Remote node number: %d || %x\n", remoteNodeNumber, remoteNodeNumber);
    
    if (moduleConfig.dzhaga.mode == meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_REMOTE) {
        //LOG_DEBUG("Configuring pins for remote mode\n");
        //LOG_DEBUG("The firstTimer: %x\n", firstTime);
        //LOG_DEBUG("frbtn_use_pullup value: %d\n", moduleConfig.dzhaga.frbtn_use_pullup);
        
        if (firstTime) {
            firstTime = false;
            LOG_INFO("[DZHAGA]: Initializing REMOTE\n");

            // Set the LED pin to output mode and turn it OFF
            if (moduleConfig.dzhaga.led_pin > 0) {
                pinMode(moduleConfig.dzhaga.led_pin, OUTPUT);
                //digitalWrite(moduleConfig.dzhaga.led_pin, 1 ^ LED_INVERTED); // turn on for now
                digitalWrite(moduleConfig.dzhaga.led_pin, 0 ^ LED_INVERTED); // turn off the LED
            }

            // Set the ReadyONE LED pin to output mode and turn it OFF
            if (moduleConfig.dzhaga.ready_one_led_pin > 0) {
                pinMode(moduleConfig.dzhaga.ready_one_led_pin, OUTPUT);
                //digitalWrite(moduleConfig.dzhaga.ready_one_led_pin, 1 ^ LED_INVERTED); // turn on for now
                digitalWrite(moduleConfig.dzhaga.ready_one_led_pin, 0 ^ LED_INVERTED); // turn off the LED
            }

            uint8_t pullup_down_mode = moduleConfig.dzhaga.frbtn_triggered_high ? INPUT_PULLDOWN : INPUT_PULLUP;
            uint8_t pin_mode = moduleConfig.dzhaga.frbtn_use_pullup ? pullup_down_mode : INPUT;
            LOG_DEBUG("[DZHAGA]: PULL UP/DOWN mode: %d\n", pullup_down_mode);
            LOG_DEBUG("[DZHAGA]: Pin mode: %d\n", pin_mode);            

            if (moduleConfig.dzhaga.frbtn_pin_0 > 0) {
                pinMode(moduleConfig.dzhaga.frbtn_pin_0, pin_mode);
                LOG_DEBUG("[DZHAGA]: Pin frbtn_pin_0 set to %d\n", pin_mode);
            } 
            if (moduleConfig.dzhaga.frbtn_pin_1 > 0) {
                pinMode(moduleConfig.dzhaga.frbtn_pin_1, pin_mode);
            } 
            if (moduleConfig.dzhaga.frbtn_pin_2 > 0) {
                pinMode(moduleConfig.dzhaga.frbtn_pin_2, pin_mode);
            }
            if (moduleConfig.dzhaga.frbtn_pin_3 > 0) {
                pinMode(moduleConfig.dzhaga.frbtn_pin_3, pin_mode);
            } 
            if (moduleConfig.dzhaga.ready_one_pin > 0) {
                pinMode(moduleConfig.dzhaga.ready_one_pin, pin_mode);
            } 
            if (moduleConfig.dzhaga.frbtn_pin_0 <= 0 || 
                moduleConfig.dzhaga.frbtn_pin_1 <= 0 || 
                moduleConfig.dzhaga.frbtn_pin_2 <= 0 || 
                moduleConfig.dzhaga.frbtn_pin_3 <= 0 || 
                moduleConfig.dzhaga.ready_one_pin <= 0) {
                LOG_WARN("[DZHAGA]: Set to enabled but not one button pins are set. Disabling module...\n");
                return disable();
            }

            LOG_DEBUG("[DZHAGA]: Local node number: %d || %x\n", nodeDB->getNodeNum(), nodeDB->getNodeNum());
            LOG_DEBUG("[DZHAGA]: Remote node number: %d || %x\n", remoteNodeNumber, remoteNodeNumber);
            //LOG_DEBUG("[DZHAGA]: Current {Ready One} pin stat: A=%i\n", digitalRead(moduleConfig.dzhaga.ready_one_pin));
            //LOG_DEBUG("[DZHAGA]: Current pins state: A=%i, B=%i, C=%i, D=%i\n", digitalRead(moduleConfig.dzhaga.ready_btn_pin), digitalRead(moduleConfig.dzhaga.frbtn_pin_1), digitalRead(moduleConfig.dzhaga.frbtn_pin_2), digitalRead(moduleConfig.dzhaga.frbtn_pin_3));

            return DELAYED_INTERVAL;
        }

        // ready_one_led_pin rutine
        if (moduleConfig.dzhaga.ready_one_led_pin > 0) {
            //digitalWrite(moduleConfig.dzhaga.ready_one_led_pin, 0 ^ LED_INVERTED);          // turn OFF the LED
            if (isReadyOne) {
                //digitalWrite(moduleConfig.dzhaga.ready_one_led_pin, 1 ^ LED_INVERTED);        // turn ON the LED
                if (pairPresent) {
                    digitalWrite(moduleConfig.dzhaga.ready_one_led_pin, 1 ^ LED_INVERTED);    // turn ON the LED
                } else {
                    digitalWrite(moduleConfig.dzhaga.ready_one_led_pin, 0 ^ LED_INVERTED);    // turn OFF the LED
                }
            } else {
                digitalWrite(moduleConfig.dzhaga.ready_one_led_pin, 0 ^ LED_INVERTED);        // turn OFF the LED
            }

        }
        
        // ready_one_led_pin rutine
        if (moduleConfig.dzhaga.led_pin > 0) {
            digitalWrite(moduleConfig.dzhaga.led_pin, 0 ^ LED_INVERTED); // turn OFF the LED
        }

        //LOG_DEBUG("Remote mode: part 2\n");
        //LOG_DEBUG("Current pins state: A=%i, B=%i, C=%i, D=%i\n", digitalRead(moduleConfig.dzhaga.ready_btn_pin), digitalRead(moduleConfig.dzhaga.frbtn_pin_1), digitalRead(moduleConfig.dzhaga.frbtn_pin_2), digitalRead(moduleConfig.dzhaga.frbtn_pin_3));

        if ((millis() - lastSentToMesh) >= Default::getConfiguredOrDefaultMs(moduleConfig.dzhaga.minimum_broadcast_secs) &&
            hasDetectionEvent()) {
            if (pairPresent) sendDetectionMessage();
            return DELAYED_INTERVAL;
        }
        // Even if we haven't detected an event, broadcast our current state to the mesh on the scheduled interval as a sort
        // of heartbeat. We only do this if the minimum broadcast interval is greater than zero, otherwise we'll only broadcast state
        // change detections.
        else if (moduleConfig.dzhaga.state_broadcast_secs > 0 &&
                 (millis() - lastSentToMesh) >= Default::getConfiguredOrDefaultMs(moduleConfig.dzhaga.state_broadcast_secs)) {
            // визначити правило щоб відправляти повідомлення
            if (pairPresent) sendCurrentStateMessage();
            return DELAYED_INTERVAL;
        }
    } else if (moduleConfig.dzhaga.mode == meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_TARGET) {
        //LOG_DEBUG("Configuring pins for target mode\n");
        //LOG_DEBUG("The firstTimer: %x\n", firstTime);
        
        if (firstTime) {
            firstTime = false;
            LOG_INFO("[DZHAGA]: Initializing TARGET\n");

            // Set the LED pin to output mode and turn it on
            if (moduleConfig.dzhaga.ready_one_led_pin > 0) {
                pinMode(moduleConfig.dzhaga.ready_one_led_pin, OUTPUT);
                //digitalWrite(moduleConfig.dzhaga.ready_one_led_pin, 1 ^ LED_INVERTED); // turn on for now
                digitalWrite(moduleConfig.dzhaga.ready_one_led_pin, 0 ^ LED_INVERTED); // turn off the LED
            }

            if (moduleConfig.dzhaga.frbtn_pin_0 > 0) {
                pinMode(moduleConfig.dzhaga.frbtn_pin_0, OUTPUT);
            } 
            if (moduleConfig.dzhaga.frbtn_pin_1 > 0) {
                pinMode(moduleConfig.dzhaga.frbtn_pin_1, OUTPUT);
            } 
            if (moduleConfig.dzhaga.frbtn_pin_2 > 0) {
                pinMode(moduleConfig.dzhaga.frbtn_pin_2, OUTPUT);
            }
            if (moduleConfig.dzhaga.frbtn_pin_3 > 0) {
                pinMode(moduleConfig.dzhaga.frbtn_pin_3, OUTPUT);
            } 
            if (moduleConfig.dzhaga.frbtn_pin_0 <= 0 || moduleConfig.dzhaga.frbtn_pin_1 <= 0 || moduleConfig.dzhaga.frbtn_pin_2 <= 0 || moduleConfig.dzhaga.frbtn_pin_3 <= 0) {
                LOG_WARN("[DZHAGA]: Set to enabled but not one button pins are set. Disabling module...\n");
                return disable();
            }

            LOG_DEBUG("[DZHAGA]: Local node number: %d || %x\n", nodeDB->getNodeNum(), nodeDB->getNodeNum());
            LOG_DEBUG("[DZHAGA]: Remote node number: %d || %x\n", remoteNodeNumber, remoteNodeNumber);
            LOG_DEBUG("[DZHAGA]: Current pins state: A=%i, B=%i, C=%i, D=%i\n", digitalRead(moduleConfig.dzhaga.frbtn_pin_0), digitalRead(moduleConfig.dzhaga.frbtn_pin_1), digitalRead(moduleConfig.dzhaga.frbtn_pin_2), digitalRead(moduleConfig.dzhaga.frbtn_pin_3));

            return DELAYED_INTERVAL; 

        }

        // ready_one_led_pin rutine
        if (moduleConfig.dzhaga.ready_one_led_pin > 0) {
            if (isReadyOne) {
                digitalWrite(moduleConfig.dzhaga.ready_one_led_pin, 1 ^ LED_INVERTED);        // turn ON the LED
            } else {
                digitalWrite(moduleConfig.dzhaga.ready_one_led_pin, 0 ^ LED_INVERTED);        // turn OFF the LED
            }
        }

        for (int i = 0; i < 4; i++) {
            if (nagCycleCutoff[i] < millis()) {
                nagCycleCutoff[i] = UINT32_MAX;
                LOG_INFO("[DZHAGA]: Turning OFF Target channel %d\n", i);
                setExternalOff(i);
                targetTurnedOn[i] = 0;
                isNagging[i] = false;
            
            } else if (isNagging[i]) {
                // If the output is turned on, turn it back off after the given period of time.
                if (millis() > targetTurnedOn[i] + Default::getConfiguredOrDefaultMs(moduleConfig.dzhaga.frbtn_sig_secs)) {
                    LOG_INFO("[DZHAGA]: Turning %s Target channel %d\n", getTarget(i) ? "OFF" : "ON", i);
                    getTarget(i) ? setExternalOff(i) : setExternalOn(i);
                }
            }
        }


    }


    return GPIO_POLLING_INTERVAL;
}

/**
 * REMOTE/TARGET mode
 * Gets the node number from the remote node name.
 * @param remote_node The remote node name.
 * @return The node number.
 */
NodeNum DzhagaModule::getNodeNumber(char remote_node[15]){
    //LOG_DEBUG("getNodeNumber(): Start ...\n");
    //LOG_DEBUG("Remote node: %s\n", remote_node);
    //LOG_DEBUG("NodeDB: %d\n", nodeDB->numMeshNodes);
    NodeNum tempNodeNumber = static_cast<uint32_t>(std::stoul(remote_node, nullptr, 16));
    for (int i = 0; i < nodeDB->numMeshNodes; i++) {
        //LOG_DEBUG("NodeDB: %d\n", i);
        //LOG_DEBUG("NodeDB: %s\n", nodeDB->getMeshNodeByIndex(i)->name);
        if (strcmp(std::to_string(nodeDB->getMeshNodeByIndex(i)->num).c_str(), remote_node) == 0) {
            //LOG_DEBUG("getNodeNumber(): Found node number: %d\n", nodeDB->getMeshNodeByIndex(i)->num);
            return nodeDB->getMeshNodeByIndex(i)->num;
        }
    }
    //LOG_DEBUG("getNodeNumber(): Node number not found\n");
    return 0;
}

void DzhagaModule::checkReadyOneStatus()
{
    //LOG_DEBUG("checkReadyOneStatus(): Start ...\n");
    // ready_one_pin
    // ready_one_secs 
    // ready_one_timeout
    
    if (moduleConfig.dzhaga.mode == meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_TARGET) {

        if (readyOneLastReplyTime == 0 && readyOneLastCheckTime == 0) {
            //LOG_DEBUG("[DZHAGA]: checkReadyOneStatus() - First time\n");
            sendReadyOneCheckMessage();
            readyOneLastReplyTime = millis();
            readyOneLastCheckTime = millis();
   
        } else if ((millis() - readyOneLastCheckTime) > 
                    Default::getConfiguredOrDefaultMs(moduleConfig.dzhaga.ready_one_secs)){
            // відправити запит на ReadyOneStatus
            sendReadyOneCheckMessage();
            readyOneLastCheckTime = millis();
            
        }
        return;
    }


    if (moduleConfig.dzhaga.mode == meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_REMOTE){
        
        if (moduleConfig.dzhaga.ready_one_pin > 0){
            //LOG_DEBUG("[DZHAGA]: checkReadyOneStatus() - ReadyOne pin is set\n");
            // assuming frbtn_triggered_high is the same for all sensors
            bool triggeredHigh = moduleConfig.dzhaga.frbtn_triggered_high;
            //LOG_DEBUG("[DZHAGA]: checkReadyOneStatus() - triggeredHigh = %s\n", triggeredHigh ? "HIGH" : "LOW");
            bool currentState = digitalRead(moduleConfig.dzhaga.ready_one_pin);
            //LOG_DEBUG("[DZHAGA]: checkReadyOneStatus() - Current state of ReadyOne pin: %d\n", currentState);
            if ((triggeredHigh ? currentState : !currentState) == HIGH) {
                //LOG_DEBUG("[DZHAGA]: checkReadyOneStatus() - button for isReadyOne is ON\n");
                if(!isReadyOne) {
                    isReadyOne = true;
                } else if ((millis() - readyOneLastReplyTime) > 
                            Default::getConfiguredOrDefaultMs(moduleConfig.dzhaga.ready_one_timeout)) {
                    //LOG_DEBUG("[DZHAGA]: checkReadyOneStatus() - ReadyOne timeout\n");
                    pairPresent = false;
                    readyOneLastReplyTime = 0;
                }
            } else {
                isReadyOne = false;
                if (pairPresent) {
                    //LOG_DEBUG("[DZHAGA]: checkReadyOneStatus() - pairPresent = TRUE >> FALSE\n");
                    pairPresent = false;
                    readyOneLastReplyTime = 0;
                }
            }

        } else {
            //LOG_DEBUG("[DZHAGA]: checkReadyOneStatus() - ReadyOne pin is not set\n");
            isReadyOne = false;
            pairPresent = false;
        }

        readyOneLastCheckTime = millis();
        //LOG_DEBUG("[DZHAGA]: ReadyOne status check: isReadyOne = %s\n", isReadyOne ? "TRUE" : "FALSE");
        //LOG_DEBUG("[DZHAGA]: ReadyOne status check: pairPresent = %s\n", pairPresent ? "TRUE" : "FALSE");
        return;
    }

    return;
}


/**
 * REMOTE mode
 * Sends a DETECTED message to the mesh network when a detection event is observed.
 */
void DzhagaModule::sendDetectionMessage()
{
    LOG_DEBUG("[DZHAGA]: Action event observed. Sending message\n");
    char *message = new char[MESSAGE_BUFFER_SIZE];
    int offset = 0;

    offset += snprintf(message + offset, MESSAGE_BUFFER_SIZE - offset, "DETECTED");
    for (int i = 0; i < 4; ++i) {
        offset += snprintf(message + offset, MESSAGE_BUFFER_SIZE - offset, ":%s", detectedState[i] ? "TRUE" : "FALSE");
        if (offset >= MESSAGE_BUFFER_SIZE - 1) break;                   // Ensure room for null terminator
    }

    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p) {
        LOG_ERROR("[DZHAGA]: Failed to allocate meshtastic_MeshPacket");
        delete[] message;
        return;                                                         // Early return on allocation failure
    }

    p->want_ack = false;
    p->to = remoteNodeNumber;
    p->priority = meshtastic_MeshPacket_Priority_ACK;
    p->decoded.payload.size = strlen(message);
    memcpy(p->decoded.payload.bytes, message, p->decoded.payload.size);

    if (p->decoded.payload.size + 1 < meshtastic_Constants_DATA_PAYLOAD_LEN) {
        p->decoded.payload.bytes[p->decoded.payload.size] = ASCII_BELL; // Bell character
        p->decoded.payload.bytes[p->decoded.payload.size + 1] = '\0';   // Ensure null terminator after bell character
        p->decoded.payload.size++;
        //LOG_DEBUG("[DZHAGA]: Bell character added to message\n");
    }

    LOG_INFO("[DZHAGA]: Sending message id=%d, from=%x, dest=%x, msg=%.*s\n", p->id, p->from, p->to, p->decoded.payload.size, p->decoded.payload.bytes);
    //LOG_INFO("Nodes sage dest[d]=%d, dest[x]=%x\n", p->to, p->to);
    //LOG_DEBUG("[DZHAGA]: Message size: %d\n", p->decoded.payload.size);
    lastSentToMesh = millis();
    service.sendToMesh(p);

    // send to mesh, cc to phone. Even if there's no phone connected, this stores the message to match ACKs
    // if p->want_ack = true;
    // service.sendToMesh(p, RX_SRC_LOCAL,true);

    delete[] message;

    // опрацювання дій після відправки повідомлення
    if (moduleConfig.dzhaga.led_pin > 0) {
        digitalWrite(moduleConfig.dzhaga.led_pin, 1 ^ LED_INVERTED); // turn ON the LED
    }
    
    return; 
}

/**
 * REMOTE mode
 * Sends a OBSERVED message to the mesh network with the current state of the pins.
 */
void DzhagaModule::sendCurrentStateMessage()
{
    LOG_DEBUG("[DZHAGA]: Status update. Sending message\n");
    char *message = new char[MESSAGE_BUFFER_SIZE];
    int offset = 0;

    offset += snprintf(message + offset, MESSAGE_BUFFER_SIZE - offset, "OBSERVED");
    for (int i = 0; i < 4; ++i) {
        offset += snprintf(message + offset, MESSAGE_BUFFER_SIZE - offset, ":%s", detectedState[i] ? "TRUE" : "FALSE");
        if (offset >= MESSAGE_BUFFER_SIZE - 1) break;                   // Ensure room for null terminator
    }

    //sprintf(message, "%s detected", moduleConfig.detection_sensor.name);
    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p) {
        LOG_ERROR("[DZHAGA]: Failed to allocate meshtastic_MeshPacket");
        delete[] message;
        return;                                                         // Early return on allocation failure
    }

    p->want_ack = false;
    p->to = remoteNodeNumber;
    p->decoded.payload.size = strlen(message);
    memcpy(p->decoded.payload.bytes, message, p->decoded.payload.size);

    LOG_INFO("[DZHAGA]: Sending message id=%d, dest=%x, msg=%.*s\n", p->id, p->to, p->decoded.payload.size, p->decoded.payload.bytes);
    lastSentToMesh = millis();
    //service.sendToMesh(p);
    delete[] message;
}

// REMOTE mode
// Sends a READYONE message to the mesh network with the current state of the pins.
void DzhagaModule::sendReadyOneStatusMessage()
{
    LOG_DEBUG("[DZHAGA]: ReadyOne status check requested. Sending status message\n");
    char *message = new char[MESSAGE_BUFFER_SIZE];
    int offset = 0;

    offset += snprintf(message + offset, MESSAGE_BUFFER_SIZE - offset, "READYONE");
    offset += snprintf(message + offset, MESSAGE_BUFFER_SIZE - offset, ":%s", isReadyOne ? "TRUE" : "FALSE");

    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p) {
        LOG_ERROR("[DZHAGA]: Failed to allocate meshtastic_MeshPacket");
        delete[] message;
        return;                                                         // Early return on allocation failure
    }

    p->want_ack = false;
    p->to = remoteNodeNumber;
    p->priority = meshtastic_MeshPacket_Priority_ACK;
    p->decoded.payload.size = strlen(message);
    memcpy(p->decoded.payload.bytes, message, p->decoded.payload.size);

    if (p->decoded.payload.size + 1 < meshtastic_Constants_DATA_PAYLOAD_LEN) {
        p->decoded.payload.bytes[p->decoded.payload.size] = ASCII_BELL; // Bell character
        p->decoded.payload.bytes[p->decoded.payload.size + 1] = '\0';   // Ensure null terminator after bell character
        p->decoded.payload.size++;
        LOG_DEBUG("[DZHAGA]: Bell character added to message\n");
    }

    LOG_INFO("[DZHAGA]: Sending message id=%d, dest=%x, msg=%.*s\n", p->id, p->to, p->decoded.payload.size, p->decoded.payload.bytes);
    lastSentToMesh = millis();
    service.sendToMesh(p);
    delete[] message;
}


// REMOTE mode
// Check if any of the pins have changed state
bool DzhagaModule::hasDetectionEvent()
{
    //LOG_DEBUG("hasDetectionEvent(): Start ...\n");
    bool detectionState = false;
    detectedState = {false, false, false, false};
    bool triggeredHigh = moduleConfig.dzhaga.frbtn_triggered_high; // assuming frbtn_triggered_high is the same for all sensors

    bool currentStateA = digitalRead(moduleConfig.dzhaga.frbtn_pin_0);
    bool currentStateB = digitalRead(moduleConfig.dzhaga.frbtn_pin_1);
    bool currentStateC = digitalRead(moduleConfig.dzhaga.frbtn_pin_2);
    bool currentStateD = digitalRead(moduleConfig.dzhaga.frbtn_pin_3);
    //LOG_DEBUG("Pins Current States: A=%i, B=%i, C=%i, D=%i\n", currentStateA, currentStateB, currentStateC, currentStateD);
  
    detectedState[0] = triggeredHigh ? currentStateA : !currentStateA;
    detectedState[1] = triggeredHigh ? currentStateB : !currentStateB;
    detectedState[2] = triggeredHigh ? currentStateC : !currentStateC;
    detectedState[3] = triggeredHigh ? currentStateD : !currentStateD;
    //LOG_DEBUG("Pins detected Statuses: %i, %i, %i, %i\n", detectedState[0], detectedState[1], detectedState[2], detectedState[3]);
  
    if (triggeredHigh) {
        detectionState = currentStateA || currentStateB || currentStateC || currentStateD;
    } else {
        detectionState = !currentStateA || !currentStateB || !currentStateC || !currentStateD;
    }

    //LOG_DEBUG("hasDetectionEvent(): result: %i\n", detectionState);
    return detectionState;
}

/**
 * TARGET mode
 * Sends a RESPOND message to the mesh network with the current state of the pins.
 */
void DzhagaModule::sendRespondMessage()
{
    LOG_DEBUG("[DZHAGA]: Request processed. Sending reply ...\n");
    char *message = new char[MESSAGE_BUFFER_SIZE];
    int offset = 0;

    offset += snprintf(message + offset, MESSAGE_BUFFER_SIZE - offset, "DONE ");
    for (int i = 0; i < 4; ++i) {
        if (detectedState[i] == true) {
            offset += snprintf(message + offset, MESSAGE_BUFFER_SIZE - offset, "[ %d ]", i+1);
            if (offset >= MESSAGE_BUFFER_SIZE - 1) break;
        }
    }

    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p) {
        LOG_ERROR("[DZHAGA]: Failed to allocate meshtastic_MeshPacket");
        delete[] message;
        return;                                                         // Early return on allocation failure
    }

    p->want_ack = false;
    p->to = remoteNodeNumber;
    p->priority = meshtastic_MeshPacket_Priority_ACK;
    p->decoded.payload.size = strlen(message);
    memcpy(p->decoded.payload.bytes, message, p->decoded.payload.size);

    lastSentToMesh = millis();
    service.sendToMesh(p);
    delete[] message;

}

void DzhagaModule::sendReadyOneCheckMessage()
{
    LOG_DEBUG("[DZHAGA]: ReadyOne status check. Sending message\n");
    char *message = new char[MESSAGE_BUFFER_SIZE];
    int offset = 0;

    offset += snprintf(message + offset, MESSAGE_BUFFER_SIZE - offset, "READYONE:CHECK");

    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p) {
        LOG_ERROR("[DZHAGA]: Failed to allocate meshtastic_MeshPacket");
        delete[] message;
        return;                                                         // Early return on allocation failure
    }

    p->want_ack = false;
    p->to = remoteNodeNumber;
    p->priority = meshtastic_MeshPacket_Priority_ACK;
    p->decoded.payload.size = strlen(message);
    memcpy(p->decoded.payload.bytes, message, p->decoded.payload.size);

    if (p->decoded.payload.size + 1 < meshtastic_Constants_DATA_PAYLOAD_LEN) {
        p->decoded.payload.bytes[p->decoded.payload.size] = ASCII_BELL; // Bell character
        p->decoded.payload.bytes[p->decoded.payload.size + 1] = '\0';   // Ensure null terminator after bell character
        p->decoded.payload.size++;
        LOG_DEBUG("[DZHAGA]: Bell character added to message\n");
    }

    LOG_INFO("[DZHAGA]: Sending message id=%d, dest=%x, msg=%.*s\n", p->id, p->to, p->decoded.payload.size, p->decoded.payload.bytes);
    lastSentToMesh = millis();
    service.sendToMesh(p);
    delete[] message;
}


/**
 * TARGET mode
 * Sets the target pin notification on for the specified index.
 * @param index The index of the target notification to turn on.
 */
void DzhagaModule::setExternalOn(uint8_t index)
{
    targetCurrentState[index] = true;
    targetTurnedOn[index] = millis();

    switch (index) {
    case 0:
        if (moduleConfig.dzhaga.frbtn_pin_0 > 0)
            digitalWrite(moduleConfig.dzhaga.frbtn_pin_0, true);
        break;
    case 1:
        if (moduleConfig.dzhaga.frbtn_pin_1 > 0)
            digitalWrite(moduleConfig.dzhaga.frbtn_pin_1, true);
        break;
    case 2:
        if (moduleConfig.dzhaga.frbtn_pin_2 > 0)
            digitalWrite(moduleConfig.dzhaga.frbtn_pin_2, true);
        break;
    case 3:
        if (moduleConfig.dzhaga.frbtn_pin_3 > 0)
            digitalWrite(moduleConfig.dzhaga.frbtn_pin_3, true);
        break;
    }

    LOG_DEBUG("[DZHAGA]: Target channel %d turned ON\n", index);
}

/**
 * TARGET mode
 * Sets the target pin notification OFF for the specified index.
 * @param index The index of the target notification to turn OFF.
 */
void DzhagaModule::setExternalOff(uint8_t index)
{
    targetCurrentState[index] = 0;
    targetTurnedOn[index] = millis();

    switch (index) {
    case 0:
        if (moduleConfig.dzhaga.frbtn_pin_0 > 0)
            digitalWrite(moduleConfig.dzhaga.frbtn_pin_0, false);
        break;
    case 1:
        if (moduleConfig.dzhaga.frbtn_pin_1 > 0)
            digitalWrite(moduleConfig.dzhaga.frbtn_pin_1, false);
        break;
    case 2:
        if (moduleConfig.dzhaga.frbtn_pin_2 > 0)
            digitalWrite(moduleConfig.dzhaga.frbtn_pin_2, false);
        break;
    case 3:
        if (moduleConfig.dzhaga.frbtn_pin_3 > 0)
            digitalWrite(moduleConfig.dzhaga.frbtn_pin_3, false);
        break;
    }

    LOG_DEBUG("[DZHAGA]: Target channel %d turned OFF\n", index);

}

/**
 * TARGET mode
 * Gets the current state of the target pin notification for the specified index.
 * @param index The index of the target notification to get the state of.
 * @return The state of the target pin notification.
 */
bool DzhagaModule::getTarget(uint8_t index)
{
    return targetCurrentState[index];
}

/**
 * TARGET mode
 * Handles a received message in target mode.
 * @param mp The received message.
 * @return ProcessMessage::STOP if the message has been handled and no other handlers should be considered for it.
 */
ProcessMessage DzhagaModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    LOG_DEBUG("[DZHAGA]: handleReceived() - Start ...\n");
    if (moduleConfig.dzhaga.mode == meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_TARGET) {
        if (moduleConfig.dzhaga.enabled){
            LOG_DEBUG("[DZHAGA]: Handling message by TARGET\n");
            //LOG_DEBUG("Received message from=0x%0x, id=%d, msg=%.*s\n", mp.from, mp.id, mp.decoded.payload.size, mp.decoded.payload.bytes);
    
            if (getFrom(&mp) != nodeDB->getNodeNum() &&
                getFrom(&mp) == remoteNodeNumber) {
                // Check if the message contains a BELL character. Don't do this loop for every pin, just once.
                auto &p = mp.decoded;
                bool containsBell = false;   // change to false, if you want to check for bell character
                for (int i = 0; i < p.payload.size; i++) {
                    if (p.payload.bytes[i] == ASCII_BELL) {
                        containsBell = true;
                    }
                }
                //LOG_DEBUG("[DZHAGA]: Bell character detected: %s\n", containsBell ? "true" : "false");
            
                std::string messageStr(p.payload.bytes, p.payload.bytes + p.payload.size);

                // Check if the message contains the word "DETECTED"
                bool containsDetected = false;
                if (messageStr.find("DETECTED") != std::string::npos) {
                    // Perform actions when "DETECTED" word is found
                    containsDetected = true;                    
                }
                //LOG_DEBUG("[DZHAGA]: DETECTED word observed: %s\n", containsDetected ? "true" : "false");

                // Check if the message contains the word "OBSERVED"
                bool containsObserved = false;
                if (messageStr.find("OBSERVED") != std::string::npos) {
                    // Perform actions when "OBSERVED" word is found
                    containsObserved = true;                    
                }
                //LOG_DEBUG("[DZHAGA]: OBSERVED word observed: %s\n", containsObserved ? "true" : "false");

                // Check if the message contains the word "READYONE"
                bool containsReadyOne = false;
                if (messageStr.find("READYONE") != std::string::npos) {
                    // Perform actions when "READYONE" word is found
                    containsReadyOne = true;                    
                }
                //LOG_DEBUG("[DZHAGA]: READYONE word observed: %s\n", containsReadyOne ? "true" : "false");

                // Find and remove all instances of the ASCII bell character
                size_t pos;
                while ((pos = messageStr.find(ASCII_BELL)) != std::string::npos) {
                    messageStr.erase(pos, 1);
                }


                if (containsDetected || containsObserved) {
                    LOG_DEBUG("[DZHAGA]: Message contains DETECTED or OBSERVED\n");
                    std::vector<std::string> messageParts;
                    std::istringstream iss(messageStr);
                    std::string part;
                    while (std::getline(iss, part, ':')) {
                        messageParts.push_back(part);
                    }

                    if (messageParts.size() == 5) {
                        std::string prefix = messageParts[0];
                        std::string state1 = messageParts[1];
                        std::string state2 = messageParts[2];
                        std::string state3 = messageParts[3];
                        std::string state4 = messageParts[4];

                        // Use the parts as needed
                        // For example:
                        if (prefix == "DETECTED" || prefix == "OBSERVED") {
                            detectedState[0] = (state1 == "TRUE");
                            detectedState[1] = (state2 == "TRUE");
                            detectedState[2] = (state3 == "TRUE");
                            detectedState[3] = (state4 == "TRUE");

                            // Perform actions based on the states
                            // ...
                        }
                        LOG_DEBUG("[DZHAGA]: Detected States: %s, %s, %s, %s\n", state1.c_str(), state2.c_str(), state3.c_str(), state4.c_str());
                    }
                } 

                if (containsReadyOne) {
                    //LOG_DEBUG("[DZHAGA]: Message contains READYONE\n");
                    std::vector<std::string> messageParts;
                    std::istringstream iss(messageStr);
                    std::string part;

                    while (std::getline(iss, part, ':')) {
                        messageParts.push_back(part);
                    }

                    if (messageParts.size() == 2) {
                        std::string prefix = messageParts[0];
                        std::string state1 = messageParts[1];
                        if (prefix == "READYONE") {
                            isReadyOne = (state1 == "TRUE");
                        }
                        //LOG_DEBUG("[DZHAGA]: READYONE state: %s\n", state1.c_str());
                        readyOneLastReplyTime = millis();
                    }

                }

                if (moduleConfig.dzhaga.frbtn_pin_0 > 0 && detectedState[0] && containsDetected) {
                    if (containsBell) {
                        LOG_INFO("[DZHAGA]: frbtn_pin_0 - Notification BELL detected\n");
                        isNagging[0] = true;
                        setExternalOn(0);
                        nagCycleCutoff[0] = millis() + Default::getConfiguredOrDefaultMs(moduleConfig.dzhaga.frbtn_sig_secs);

                    }
                }

                if (moduleConfig.dzhaga.frbtn_pin_1 > 0 && detectedState[1] && containsDetected) {
                    if (containsBell) {
                        LOG_INFO("[DZHAGA]: frbtn_pin_1 - Notification BELL detected\n");
                        isNagging[1] = true;
                        setExternalOn(1);
                        nagCycleCutoff[1] = millis() + Default::getConfiguredOrDefaultMs(moduleConfig.dzhaga.frbtn_sig_secs);

                    }
                }

                if (moduleConfig.dzhaga.frbtn_pin_2 > 0 && detectedState[2] && containsDetected) {
                    if (containsBell) {
                        LOG_INFO("[DZHAGA]: frbtn_pin_2 - Notification BELL detected\n");
                        isNagging[2] = true;
                        setExternalOn(2);
                        nagCycleCutoff[2] = millis() + Default::getConfiguredOrDefaultMs(moduleConfig.dzhaga.frbtn_sig_secs);

                    }
                }

                if (moduleConfig.dzhaga.frbtn_pin_3 > 0 && detectedState[3] && containsDetected) {
                    if (containsBell) {
                        LOG_INFO("[DZHAGA]: frbtn_pin_3 - Notification BELL detected\n");
                        isNagging[3] = true;
                        setExternalOn(3);
                        nagCycleCutoff[3] = millis() + Default::getConfiguredOrDefaultMs(moduleConfig.dzhaga.frbtn_sig_secs);

                    }
                }
            
                // send the respond to the sender
                if (containsDetected) {
                    sendRespondMessage();
                } else if (containsObserved) {
                    //LOG_DEBUG("[DZHAGA]: Message contains OBSERVED\n");
                    //sendCurrentStateMessage();
                }
                
            
            } else {
                LOG_WARN("[DZHAGA]: Message received from wrong node\n");
                return ProcessMessage::CONTINUE;
            }

            return ProcessMessage::STOP;
        
        } else {
            LOG_WARN("[DZHAGA]: Message received but module is disabled\n");
        }
    }
    // Handling message by REMOTE
    if (moduleConfig.dzhaga.mode == meshtastic_ModuleConfig_DzhagaConfig_Dzhaga_Mode_REMOTE) {
        if (moduleConfig.dzhaga.enabled){

            LOG_DEBUG("[DZHAGA]: Handling message by REMOTE\n");
            //LOG_DEBUG("Received message from=0x%0x, id=%d, msg=%.*s\n", mp.from, mp.id, mp.decoded.payload.size, mp.decoded.payload.bytes);
    
            if (getFrom(&mp) != nodeDB->getNodeNum() &&
                getFrom(&mp) == remoteNodeNumber) {
                // Check if the message contains a BELL character. Don't do this loop for every pin, just once.
                auto &p = mp.decoded;
                bool containsBell = false;   // change to false, if you want to check for bell character
                for (int i = 0; i < p.payload.size; i++) {
                    if (p.payload.bytes[i] == ASCII_BELL) {
                        containsBell = true;
                    }
                }
                LOG_DEBUG("[DZHAGA]: Bell character detected: %s\n", containsBell ? "true" : "false");
            
                std::string messageStr(p.payload.bytes, p.payload.bytes + p.payload.size);

                // Check if the message contains the word "READYONE"
                bool containsReadyOne = false;
                if (messageStr.find("READYONE") != std::string::npos) {
                    // Perform actions when "READYONE" word is found
                    containsReadyOne = true;                    
                }
                LOG_DEBUG("[DZHAGA]: READYONE word observed: %s\n", containsReadyOne ? "true" : "false");

                // Find and remove all instances of the ASCII bell character
                size_t pos;
                while ((pos = messageStr.find(ASCII_BELL)) != std::string::npos) {
                    messageStr.erase(pos, 1);
                }

                if (containsReadyOne && containsBell) {
                    LOG_DEBUG("[DZHAGA]: Message contains READYONE and BELL\n");
                    std::vector<std::string> messageParts;
                    std::istringstream iss(messageStr);
                    std::string part;

                    while (std::getline(iss, part, ':')) {
                        messageParts.push_back(part);
                    }

                    if (messageParts.size() == 2) {
                        std::string prefix = messageParts[0];
                        std::string text1 = messageParts[1];
                        if (prefix == "READYONE" && text1 == "CHECK") {
                            sendReadyOneStatusMessage();
                            pairPresent = true;
                            readyOneLastReplyTime = millis();
                        }
                        LOG_DEBUG("[DZHAGA]: READYONE state replied");
                        
                    }

                }
            } else {
                LOG_WARN("[DZHAGA]: Message received from wrong node\n");
                return ProcessMessage::CONTINUE;
            }
            
            return ProcessMessage::STOP;

        }
    }
    LOG_DEBUG("[DZHAGA]: handleReceived() - Ended (not processed)\n");
    return ProcessMessage::CONTINUE;
}

/**
 * TARGET mode
 * Checks if the module should handle the received message.
 */
bool DzhagaModule::wantPacket(const meshtastic_MeshPacket *p)
{
    return MeshService::isTextPayload(p);
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
/*
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
*/



DzhagaModule::DzhagaModule()
    //: SinglePortModule("DzhagaModule", meshtastic_PortNum_DZHAGA_APP),
    : SinglePortModule("DzhagaModule", meshtastic_PortNum_TEXT_MESSAGE_APP),
      concurrency::OSThread("DzhagaModule")
{

    if (moduleConfig.dzhaga.enabled) {
        LOG_INFO("[DZHAGA]: DzhagaModule() - Module Enabled\n");
        
    } else {
        LOG_INFO("[DZHAGA]: DzhagaModule() - Module Disabled\n");
        disable();
    }
}