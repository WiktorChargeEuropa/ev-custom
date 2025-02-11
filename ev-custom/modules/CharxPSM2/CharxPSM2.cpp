/*
                                 Apache License
                           Version 2.0, January 2004
                        http://www.apache.org/licenses/
*/
#include "CharxPSM2.hpp"

namespace module {

void CharxPSM2::init() {
    invoke_init(*p_main);
}

void CharxPSM2::ready() {
    invoke_ready(*p_main);
}

} // namespace module
