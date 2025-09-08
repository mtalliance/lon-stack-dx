#include "abstraction/IzotConfig.h"

ptrU32_Callback_V os_ticks_get = 0;
inline void setPtr_os_ticks_get( ptrU32_Callback_V fPtr)
{
   os_ticks_get = fPtr;
}

ptrV_Callback_U32 os_thread_sleep = 0;
void setPtr_os_thread_sleep(ptrV_Callback_U32 fPtr)
{
   os_thread_sleep = fPtr;
}


ptrU32_Callback_U32 os_msec_to_ticks = 0;
void setPtr_os_msec_to_ticks(ptrU32_Callback_U32 fPtr)
{
   os_msec_to_ticks = fPtr;
}


ptrVp_Callback_U32 os_mem_alloc = 0;
void setPtr_os_mem_alloc(ptrVp_Callback_U32 fPtr)
{
   os_mem_alloc = fPtr;
}


ptrV_Callback_Vp os_mem_free = 0;
void setPtr_os_mem_free(ptrV_Callback_Vp fPtr)
{
   os_mem_free = fPtr;
}
