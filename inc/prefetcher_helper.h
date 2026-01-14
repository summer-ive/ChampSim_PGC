#include <cstdint>

#include "champsim.h"

std::pair<champsim::page_number, bool> va_to_pa_ideal(uint32_t cpu_num, champsim::page_number vpage);