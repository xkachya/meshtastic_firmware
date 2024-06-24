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

int32_t DzhagaModule::runOnce()
{
    /*
        Uncomment the preferences below if you want to use the module
        without having to configure it from the PythonAPI or WebUI.
    */
    // moduleConfig.detection_sensor.enabled = true;
    // moduleConfig.detection_sensor.monitor_pin = 10; // WisBlock PIR IO6
    // moduleConfig.detection_sensor.monitor_pin = 21; // WisBlock RAK12013 Radar IO6
    // moduleConfig.detection_sensor.minimum_broadcast_secs = 30;
    // moduleConfig.detection_sensor.state_broadcast_secs = 120;
    // moduleConfig.detection_sensor.detection_triggered_high = true;
    // strcpy(moduleConfig.detection_sensor.name, "Motion");

    return GPIO_POLLING_INTERVAL;
}


bool DzhagaModule::hasDetectionEvent()
{
    bool detectionState = false;

    bool currentStateA = digitalRead(moduleConfig.dzhaga.ready_btn_pin);
    bool currentStateB = digitalRead(moduleConfig.dzhaga.frbtn_pin_1);
    bool currentStateC = digitalRead(moduleConfig.dzhaga.frbtn_pin_2);
    bool currentStateD = digitalRead(moduleConfig.dzhaga.frbtn_pin_3);

    bool triggeredHigh = moduleConfig.dzhaga.frbtn_triggered_high; // assuming frbtn_triggered_high is the same for all sensors

    if (triggeredHigh) {
        detectionState = currentStateA || currentStateB || currentStateC || currentStateD;
    } else {
        detectionState = !currentStateA || !currentStateB || !currentStateC || !currentStateD;
    }

    return detectionState;
}

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
