// utils.h

#pragma once
#include <vcclr.h>
#include <string>

using namespace System;

namespace utils {
    std::string toStdString(System::String ^ s);
}

