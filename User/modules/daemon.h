#ifndef DAEMON_H
#define DAEMON_H

#include "main.h"

typedef void (*deamon_callback)(void*);

typedef struct {
    uint16_t reload_count;
    uint16_t temp_count;
    deamon_callback callback;

    void *owner_id;//所守护实例的id
} Daemon_Instance;

typedef struct {
    uint16_t reload_count;
    uint16_t init_count;
    deamon_callback callback;

    void *owner_id;
} Daemon_Init_Config_s;

void DaemonTask(void);
Daemon_Instance *DaemonInit(Daemon_Init_Config_s *config);
void DaemonReload(Daemon_Instance *instance);


#endif