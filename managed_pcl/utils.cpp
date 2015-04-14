#include "utils.h"

namespace utils {
std::string toStdString(System::String ^ s) {
    if(System::String::IsNullOrEmpty(s))
        return std::string();
    using namespace System;
    using namespace System::Runtime::InteropServices;
    const char* chars = 
        (const char*)(Marshal::StringToHGlobalAnsi(s)).ToPointer();
    std::string res = chars;
    Marshal::FreeHGlobal(IntPtr((void*)chars));
    return res;
}

}
