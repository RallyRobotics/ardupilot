#include "AP_Swivel_Backend.h"

#if AP_SWIVEL_ENABLED

#include "AP_Swivel.h"

AP_Swivel_Backend::AP_Swivel_Backend(AP_Swivel &_ap_swivel, AP_Swivel::Swivel_State &_state) :
        ap_swivel(_ap_swivel),
        state(_state)
{}

#endif  // AP_SWIVEL_ENABLED
