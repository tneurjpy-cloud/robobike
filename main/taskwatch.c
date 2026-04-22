///////////////////////////////////////////////////////
// taskwatch.c  show task status
////////////////////////////////////////////////////////////////////////
/* After control,setup done ROBOBIKE, 2 Clients connected
--- Task Stats ---------------------------------------------------------
Task Name        | Status     | MinFreeStack | Priority  
------------------------------------------------------------------------
main             | Running    | 1388         | 1         
IDLE             | Ready      | 1252         | 0         
ControlTask      | Blocked    | 1184         | 24        
tiT              | Blocked    | 1572         | 18        
dns_server       | Blocked    | 1708         | 5         
wifi             | Blocked    | 4292         | 23        
esp_timer        | Suspended  | 3636         | 22        
Tmr Svc          | Blocked    | 1708         | 1         
sys_evt          | Blocked    | 972          | 20        
httpd            | Blocked    | 2680         | 5         
--- Heap Stats ---------------------------------------------------------
Total Free Heap  :  171268 bytes (全体)
Internal RAM Free:  171268 bytes (内蔵RAMのみ)
Minimum Ever Free:  158184 bytes (史上最小の空き)
Largest Free Blk :  114688 bytes (最大連続空き領域)
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

        printf("\n--- Task Stats ---------------------------------------------------------\n");
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

            // エラーの原因となった xCoreID を削除しました
            printf("%-16s | %-10s | %-12u | %-10u\n",
                   pxTaskStatusArray[i].pcTaskName,
                   status,
                   (unsigned int)pxTaskStatusArray[i].usStackHighWaterMark,
                   (unsigned int)pxTaskStatusArray[i].uxCurrentPriority);
        }
        vPortFree(pxTaskStatusArray);

        // --- ヒープ情報の表示 (MALLOC_CAP_INTERNAL は内蔵RAMを指します) ---
        size_t total_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        size_t min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
        size_t max_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

        printf("--- Heap Stats ---------------------------------------------------------\n");
        printf("Total Free Heap  : %7u bytes (全体)\n", total_free);
        printf("Internal RAM Free: %7u bytes (内蔵RAMのみ)\n", internal_free);
        printf("Minimum Ever Free: %7u bytes (史上最小の空き)\n", min_free);
        printf("Largest Free Blk : %7u bytes (最大連続空き領域)\n", max_block);
        printf("------------------------------------------------------------------------\n\n");
    }
}
