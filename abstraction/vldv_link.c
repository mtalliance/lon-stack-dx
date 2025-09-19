#include "abstraction/IzotConfig.h"
#include "abstraction/vldv.h"


ptrFct_Callback_OpenLonLink OpenLonLink = 0;
void setPtr_OpenLonLink( ptrFct_Callback_OpenLonLink fPtr)
{
   OpenLonLink = fPtr;
}

ptrFct_Callback_CloseLonLink CloseLonLink = 0;
void setPtr_CloseLonLink( ptrFct_Callback_CloseLonLink fPtr)
{
   CloseLonLink = fPtr;
}

ptrFct_Callback_ReadLonLink ReadLonLink = 0;
void setPtr_ReadLonLink( ptrFct_Callback_ReadLonLink fPtr)
{
   ReadLonLink = fPtr;
}

ptrFct_Callback_WriteLonLink WriteLonLink = 0;
void setPtr_WriteLonLink( ptrFct_Callback_WriteLonLink fPtr)
{
   WriteLonLink = fPtr;
}
