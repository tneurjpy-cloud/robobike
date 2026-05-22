////////////////////////////////////////////////////////////////////////
// taskwatch.c  show task status
////////////////////////////////////////////////////////////////////////
/*
--- Task Stats        712 ms -------------------------------------------
Task Name        | Status     | MinFreeStack | Priority  
------------------------------------------------------------------------
main             | Running    | 1396         | 1         
IDLE             | Ready      | 1320         | 0         
tiT              | Blocked    | 1772         | 18        
wifi             | Blocked    | 4524         | 23        
httpd            | Blocked    | 5524         | 5         
sys_evt          | Blocked    | 2240         | 20        
dns_server       | Blocked    | 1784         | 5         
esp_timer        | Suspended  | 2248         | 22        
ControlTask      | Suspended  | 1284         | 24        
Tmr Svc          | Blocked    | 1776         | 1         
--- Heap Stats ---------------------------------------------------------
Total Free Heap  :   87012 bytes
Internal RAM Free:   87012 bytes
Minimum Ever Free:   86624 bytes
Largest Free Blk :   65536 bytes
------------------------------------------------------------------------
*/

#include "userdefine.h"
#include "esp_heap_caps.h"

void showTasks()
{
    UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();
    TaskStatus_t *pxTaskStatusArray;
    uint32_t ulTotalRunTime;

    pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

    if (pxTaskStatusArray != NULL) {
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

        printf("\n--- Task Stats %10lu ms -------------------------------------------\n", millis());
        printf("%-16s | %-10s | %-10s | %-10s\n", 
               "Task Name", "Status", "MinFreeStack", "Priority");
        printf("------------------------------------------------------------------------\n");

        for (int i = 0; i < uxArraySize; i++) {
            const char *status;
            switch (pxTaskStatusArray[i].eCurrentState) {
                case eRunning:   status = "Running";   break;
                case eReady:     status = "Ready";     break;
                case eBlocked:   status = "Blocked";   break;
                case eSuspended: status = "Suspended"; break;
                case eDeleted:   status = "Deleted";   break;
                default:         status = "Unknown";   break;
            }

            printf("%-16s | %-10s | %-12u | %-10u\n",
                   pxTaskStatusArray[i].pcTaskName,
                   status,
                   (unsigned int)pxTaskStatusArray[i].usStackHighWaterMark,
                   (unsigned int)pxTaskStatusArray[i].uxCurrentPriority);
        }
        vPortFree(pxTaskStatusArray);

        size_t total_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        size_t min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
        size_t max_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

        printf("--- Heap Stats ---------------------------------------------------------\n");
        printf("Total Free Heap  : %7u bytes\n", total_free);
        printf("Internal RAM Free: %7u bytes\n", internal_free);
        printf("Minimum Ever Free: %7u bytes\n", min_free);
        printf("Largest Free Blk : %7u bytes\n", max_block);
        printf("------------------------------------------------------------------------\n\n");
    }
}
