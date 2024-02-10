#include "mmu2_fsensor.h"
#include "Filament_sensor.h"

namespace MMU2 {

FilamentState WhereIsFilament(){
#ifdef FILAMENT_SENSOR
    return fsensor.getFilamentPresent() ? FilamentState::AT_FSENSOR : FilamentState::NOT_PRESENT;
#else
    return FilamentState::AT_FSENSOR;
#endif
}

} // namespace MMU2
