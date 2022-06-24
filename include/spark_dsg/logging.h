#pragma once

#include <iostream>

#define ENABLE_LOGGING() 0

#if ENABLE_LOGGING()
#define SG_LOG(level) std::cout << "[Scene Graph] " #level ": "
#else
#define SG_LOG(level) std::ostream(nullptr)
#endif
