#pragma once

#include "AP_Swivel.h"

#if AP_SWIVEL_ENABLED

class AP_Swivel_Backend
{
public:

    AP_Swivel_Backend(AP_Swivel &_ap_swivel, AP_Swivel::Swivel_State &_state);

    virtual ~AP_Swivel_Backend(void) {}

    virtual void update() = 0;

protected:

    AP_Swivel &ap_swivel;
    AP_Swivel::Swivel_State &state;
};

#endif   // AP_SWIVEL_ENABLED
