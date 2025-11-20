
#pragma once

#include <cstdint>
#include <cstddef>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "platform_config.h"

namespace fc {
namespace tasking {

struct ITask {
    virtual void run() = 0;
    virtual ~ITask() = default;
};

class TaskRunner {
public:
    TaskRunner(ITask& task,
               const char* name,
               UBaseType_t priority,
               uint32_t stackSizeWords,
               BaseType_t coreId,
               uint32_t periodMs)
        : _task(task)
        , _name(name)
        , _priority(priority)
        , _stackSizeWords(stackSizeWords)
        , _coreId(coreId)
        , _periodMs(periodMs)
        , _handle(nullptr)
    {}

    void start() {
        xTaskCreatePinnedToCore(&TaskRunner::taskFunc,
                                _name,
                                _stackSizeWords,
                                this,
                                _priority,
                                &_handle,
                                _coreId);
    }

private:
    static void taskFunc(void* arg) {
        TaskRunner* self = static_cast<TaskRunner*>(arg);
        if (!self) vTaskDelete(nullptr);

        TickType_t lastWake = xTaskGetTickCount();
        const TickType_t periodTicks = pdMS_TO_TICKS(self->_periodMs);

        for (;;) {
            self->_task.run();
            vTaskDelayUntil(&lastWake, periodTicks);
        }
    }

    ITask&      _task;
    const char* _name;
    UBaseType_t _priority;
    uint32_t    _stackSizeWords;
    BaseType_t  _coreId;
    uint32_t    _periodMs;
    TaskHandle_t _handle;
};

} // namespace tasking
} // namespace fc
