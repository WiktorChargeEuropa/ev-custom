/* https://opensource.org/licenses/Apache-2.0 */
#include "PowermeterSim.hpp"

namespace module {

void PowermeterSim::init() {
    invoke_init(*p_main);
}

void PowermeterSim::ready() {
    invoke_ready(*p_main);
}

} // namespace module
