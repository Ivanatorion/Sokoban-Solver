#include "../include/pcheaders.h"
#include "../include/BoxPositionSet.h"
#include "../include/Sokoban.h"
#include "../include/GameSokoban.h"

#include <iostream>

#ifdef _WIN32
#include <urlmon.h>
#include <thread>

BOOL CALLBACK _enumWindowCallback(HWND hWnd, LPARAM lparam) {
    char buffer[256];
    GetWindowText(hWnd, (LPSTR) buffer, 256);
    std::string windowTitle(buffer);

    // List visible windows
    if (IsWindowVisible(hWnd)) {
        if(windowTitle.find("level") != std::string::npos){
            *((HWND *) lparam) = hWnd;
            std::cout << hWnd << ":  " << windowTitle << std::endl;
        }
    }
    return TRUE;
}

HWND getWindow(){
    HWND wnd = NULL;
    EnumWindows(_enumWindowCallback, (LPARAM) &wnd);
    return wnd;
}

void GameSokoban::sendSolution(const int levelN, const std::vector<ACTION>& solution){
    char buffer[256];

    INPUT ip;
    ip.type = INPUT_KEYBOARD;
    ip.ki.wScan = 0;
    ip.ki.time = 0;
    ip.ki.dwExtraInfo = 0;

    INPUT ip2 = ip;

    ip.ki.wVk = VK_CONTROL;
    ip2.ki.wVk = 'T';

    ip.ki.dwFlags = 0; // 0 for key pressw
    SendInput(1, &ip, sizeof(INPUT));

    ip2.ki.dwFlags = 0; // 0 for key pressw
    SendInput(1, &ip2, sizeof(INPUT));

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ip.ki.dwFlags = 0; // 0 for key pressw
    SendInput(1, &ip, sizeof(INPUT));

    ip2.ki.dwFlags = KEYEVENTF_KEYUP; // KEYEVENTF_KEYUP for key release
    SendInput(1, &ip2, sizeof(INPUT));

    std::this_thread::sleep_for(std::chrono::milliseconds(2200));

    sprintf(buffer, "http://www.game-sokoban.com/index.php?mode=level&lid=%d", levelN); 
    
    if(OpenClipboard(NULL))
    {
        HGLOBAL clipbuffer;
        char * buffer2;
        EmptyClipboard();
        clipbuffer = GlobalAlloc(GMEM_DDESHARE, strlen(buffer) + 1);
        buffer2 = (char*)GlobalLock(clipbuffer);
        strcpy(buffer2, LPCSTR(buffer));
        GlobalUnlock(clipbuffer);
        SetClipboardData(CF_TEXT,clipbuffer);
        CloseClipboard();
    }

    ip.ki.wVk = VK_CONTROL;
    ip2.ki.wVk = 'V';

    ip.ki.dwFlags = 0; // 0 for key pressw

    SendInput(1, &ip, sizeof(INPUT));

    ip2.ki.dwFlags = 0; // 0 for key pressw
    SendInput(1, &ip2, sizeof(INPUT));

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ip.ki.dwFlags = KEYEVENTF_KEYUP; // KEYEVENTF_KEYUP for key release
    SendInput(1, &ip, sizeof(INPUT));

    ip2.ki.dwFlags = KEYEVENTF_KEYUP; // KEYEVENTF_KEYUP for key release
    SendInput(1, &ip2, sizeof(INPUT));

    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    ip.ki.wVk = VK_RETURN;
    ip.ki.dwFlags = 0; // 0 for key pressw
    SendInput(1, &ip, sizeof(INPUT));

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ip.ki.dwFlags = KEYEVENTF_KEYUP; // KEYEVENTF_KEYUP for key release
    SendInput(1, &ip, sizeof(INPUT));

    std::this_thread::sleep_for(std::chrono::milliseconds(3100));

    for(int i = 0; i < (int) solution.size(); i++){
      std::this_thread::sleep_for(std::chrono::milliseconds(330));
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

    std::this_thread::sleep_for(std::chrono::milliseconds(1800));

    ip.ki.wVk = VK_CONTROL;
    ip2.ki.wVk = 'W';

    ip.ki.dwFlags = 0; // 0 for key pressw
    SendInput(1, &ip, sizeof(INPUT));

    ip2.ki.dwFlags = 0; // 0 for key pressw
    SendInput(1, &ip2, sizeof(INPUT));

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ip.ki.dwFlags = KEYEVENTF_KEYUP; // KEYEVENTF_KEYUP for key release
    SendInput(1, &ip, sizeof(INPUT));

    ip2.ki.dwFlags = KEYEVENTF_KEYUP; // KEYEVENTF_KEYUP for key release
    SendInput(1, &ip2, sizeof(INPUT));

    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
}

GameSokoban::GameSokoban(){
    ShellExecute(NULL, NULL, "http://www.game-sokoban.com/", NULL, NULL, SW_SHOW);
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

GameSokoban::GameSokoban(){
    
}

#endif

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