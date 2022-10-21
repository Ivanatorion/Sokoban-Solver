#ifndef GAME_SOKOBAN_H
#define GAME_SOKOBAN_H

class GameSokoban{
    public:
        GameSokoban();

        bool getLevel(const int levelN, const char filePath[]);
        void sendSolution(const int levelN, const std::vector<ACTION>& solution);

    private:
    
};

#endif