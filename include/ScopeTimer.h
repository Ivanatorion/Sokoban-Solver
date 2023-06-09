#ifndef SCOPE_TIMER_H
#define SCOPE_TIMER_H

class ScopeTimer {
    public:
        ScopeTimer(const std::string& scopeName, const bool verbose);
        ~ScopeTimer();

    private:
        std::string m_name;
        bool m_verbose;
        std::chrono::steady_clock::time_point m_start;
};

#endif