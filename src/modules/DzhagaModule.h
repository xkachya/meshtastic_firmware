#pragma once
#include "SinglePortModule.h"

/**
 * A simple example module that just replies with "Message received" to any message it receives.
 */
class DzhagaModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    /** Constructor
     * name is for debugging output
     */
    //DzhagaModule() : SinglePortModule("dzhaga", meshtastic_PortNum_DZHAGA_APP), OSThread("DzhagaModule") {}
    //DzhagaModule() : SinglePortModule("DzhagaModule", meshtastic_PortNum_DZHAGA_APP), OSThread("DzhagaModule") {}
    //DzhagaModule() : SinglePortModule("DzhagaModule", meshtastic_PortNum_DZHAGA_APP), concurrency::OSThread("DzhagaModule") {}

    DzhagaModule();

    std::array<uint32_t, 4> nagCycleCutoff{{UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX}};

    void setExternalOn(uint8_t index = 0);
    void setExternalOff(uint8_t index = 0);
    bool getTarget(uint8_t index = 0);

  protected:
    /** Called to handle a particular incoming message
    @return ProcessMessage::STOP if you've guaranteed you've handled 
    this message and no other handlers should be considered for it
    */
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

    virtual int32_t runOnce() override;

    virtual bool wantPacket(const meshtastic_MeshPacket *p) override;

  protected:
    bool firstTime = true;
    NodeNum remoteNodeNumber;
    uint32_t lastSentToMesh = 0;
    uint32_t lastSentToNode = 0;
    std::array<bool, 4> detectedState{{false, false, false, false}};
    std::array<bool, 4> isNagging{{false, false, false, false}};

    NodeNum getNodeNumber(char remote_node[15]);
    
    void checkReadyOneStatus();

    // REMOTE mode
    bool hasDetectionEvent();
    void sendDetectionMessage();
    void sendCurrentStateMessage();
    void sendReadyOneStatusMessage();

    // TARGET mode
    void sendRespondMessage();
    void sendReadyOneCheckMessage();

    //meshtastic_NodeInfoLite *DzhagaModule::getMeshNode(NodeNum n);

    /** For reply module we do all of our processing in the (normally optional)
     * want_replies handling
     */
    //virtual meshtastic_MeshPacket *allocReply() override;
};

extern DzhagaModule *dzhagaModule;