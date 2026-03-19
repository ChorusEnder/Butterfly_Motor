#include "daemon.h"

static Daemon_Instance daemon_instances[5];
static uint8_t idx;

Daemon_Instance *DaemonInit(Daemon_Init_Config_s *config)
{
    Daemon_Instance *instance = &daemon_instances[idx++];
    instance->reload_count = config->reload_count;
    instance->temp_count = config->init_count;
    instance->callback = config->callback;
    instance->owner_id = config->owner_id;

    return instance;
}

void DaemonTask(void)
{
    Daemon_Instance *instance;
    for (uint8_t i = 0; i < idx; i++) {
        instance = &daemon_instances[i];

        if (instance->temp_count > 0) {
            instance->temp_count--;
        } else {
            instance->callback(instance->owner_id);
        }
    }
}


void DaemonReload(Daemon_Instance *instance)
{
    instance->temp_count = instance->reload_count;
}