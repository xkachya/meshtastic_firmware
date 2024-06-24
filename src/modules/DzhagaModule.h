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
    DzhagaModule() : SinglePortModule("dzhaga", meshtastic_PortNum_DZHAGA_APP), OSThread("DzhagaModule") {}

  protected:
    virtual int32_t runOnce() override;

  protected:
    bool firstTime = true;
    uint32_t lastSentToMesh = 0;
    uint32_t lastSentToNode = 0;

    bool hasDetectionEvent();

    /** For reply module we do all of our processing in the (normally optional)
     * want_replies handling
     */
    virtual meshtastic_MeshPacket *allocReply() override;
};

extern DzhagaModule *dzhagaModule;