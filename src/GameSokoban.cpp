#include "../include/pcheaders.h"
#include "../include/BoxPositionSet.h"
#include "../include/Sokoban.h"
#include "../include/GameSokoban.h"

#ifdef _WIN32
#include <urlmon.h>

void GameSokoban::sendSolution(const int levelN, const std::vector<ACTION>& solution){
    INPUT ip;
    ip.type = INPUT_KEYBOARD;
    ip.ki.wScan = 0;
    ip.ki.time = 0;
    ip.ki.dwExtraInfo = 0;

    char buffer[256];
    sprintf(buffer, "http://www.game-sokoban.com/index.php?mode=level&lid=%d", levelN);

    ShellExecute(NULL, NULL, buffer, NULL, NULL, SW_SHOW);
    Sleep(3000);

    for(int i = 0; i < (int) solution.size(); i++){
      Sleep(400);
      switch(solution[i]){
        case UP:
            ip.ki.wVk = VK_UP;
            break;
        case DOWN:
            ip.ki.wVk = VK_DOWN;
            break;
        case LEFT:
            ip.ki.wVk = VK_LEFT;
            break;
        case RIGHT:
            ip.ki.wVk = VK_RIGHT;
            break;
      }
      
      ip.ki.dwFlags = 0; // 0 for key pressw
      SendInput(1, &ip, sizeof(INPUT));

      ip.ki.dwFlags = KEYEVENTF_KEYUP; // KEYEVENTF_KEYUP for key release
      SendInput(1, &ip, sizeof(INPUT));
    }
}

#endif

#ifdef __linux__
#include <curl/curl.h>

size_t write_data(void *ptr, size_t size, size_t nmemb, FILE *stream) {
    size_t written = fwrite(ptr, size, nmemb, stream);
    return written;
}

void URLDownloadToFile(void* nullP, const char *bufferURL, const char *filePath, const int zero, void* nullP2){
    CURL *curl = curl_easy_init();
    if (curl) {
        FILE *fp = fopen(filePath,"wb");
        curl_easy_setopt(curl, CURLOPT_URL, bufferURL);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
        curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        fclose(fp);
    }
}

void GameSokoban::sendSolution(const int levelN, const std::vector<ACTION>& solution){
    printf("ERROR: sendSolution not implemented on linux!\n");
}

#endif

GameSokoban::GameSokoban(){
    
}

bool GameSokoban::getLevel(const int levelN, const char filePath[]){
    char bufferURL[1024];
    char fileBuffer[30000];
    sprintf(bufferURL, "http://www.game-sokoban.com/index.php?mode=level_info&ulid=%d&view=general", levelN);
    URLDownloadToFile(NULL, bufferURL, filePath, 0, NULL);

    FILE *fp = fopen(filePath, "r");
    int bi = 0;
    char c = fgetc(fp);
    while(!feof(fp)){  
        fileBuffer[bi] = c;
        bi++;
        c = fgetc(fp);
    }
    fileBuffer[bi] = '\0';
    fclose(fp);

    char *start = strstr(fileBuffer, "Code");
    if(start == NULL)
        return false;
        
    start = strstr(start, "#");
    while(*start != '>')
        start--;
    start++;

    fp = fopen(filePath, "w");
    c = *start;
    while(c != '<'){
        fprintf(fp, "%c", c);
        start++;
        c = *start;
    }
    fclose(fp);

    return true;
}