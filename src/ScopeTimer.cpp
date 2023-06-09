#include "../include/pcheaders.h"
#include "../include/ScopeTimer.h"

ScopeTimer::ScopeTimer(const std::string& name, const bool verbose){
    this->m_name = name;
    this->m_verbose = verbose;
    this->m_start = std::chrono::steady_clock::now();
}

ScopeTimer::~ScopeTimer(){
    if(this->m_verbose){
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        auto duration = end - this->m_start;
        long long int milis = (long long int) std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
        printf("ScopeTimer (%s): %lld.%lld (s)\n", this->m_name.c_str(), milis / 1000, milis % 1000);
    }
}