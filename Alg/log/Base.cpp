#include "Base.h"

bool isRepeat(unsigned int code)
{
    static std::mutex mutex;
    static std::map<unsigned int, unsigned int> last;
    time_t now = time(NULL);
    std::unique_lock<std::mutex> autoLock(mutex);
    if (last.count(code) > 0 && now < (last[code] + 3))
    {
        return true;
    }
    last[code] = now;
    return false;
}