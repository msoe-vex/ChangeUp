#include "NodeManager.h"

NodeManager::NodeManager() {

}

void NodeManager::addCallback(void (*callbackPtr)(void), int triggerMillis) {
    Callback fnCallback = {callbackPtr, triggerMillis, 0};
    _callbacks.push_back(fnCallback);
}

void NodeManager::execute(int elapsedMillis) {
    for (auto callback : _callbacks) {
        if (elapsedMillis - callback.lastExecutedMillis >= callback.triggerMillis) {
            callback.callbackPtr();
            callback.lastExecutedMillis = elapsedMillis;
        }
    }
}

NodeManager::~NodeManager() {
    _callbacks.clear();
}