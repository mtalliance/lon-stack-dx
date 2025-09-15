#include "abstraction/IzotConfig.h"
#include "abstraction/vldv.h"


ptrFct_Callback_vldv_open vldv_open = 0;
void setPtr_vldv_open( ptrFct_Callback_vldv_open fPtr)
{
   vldv_open = fPtr;
}

ptrFct_Callback_vldv_close vldv_close = 0;
void setPtr_vldv_close( ptrFct_Callback_vldv_close fPtr)
{
   vldv_close = fPtr;
}

ptrFct_Callback_vldv_read vldv_read = 0;
void setPtr_vldv_read( ptrFct_Callback_vldv_read fPtr)
{
   vldv_read = fPtr;
}

ptrFct_Callback_vldv_write vldv_write = 0;
void setPtr_vldv_write( ptrFct_Callback_vldv_write fPtr)
{
   vldv_write = fPtr;
}
