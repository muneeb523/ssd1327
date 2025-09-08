#include <iostream>
#include <chrono>
#include <algorithm>
#include <thread>
#include <cstring>
#include <ctime>
#include <arpa/inet.h>
#include <unistd.h>
#include <sstream>
#include <cstdlib>
#include <sys/types.h>
#include <signal.h>
#include <sys/wait.h>
#include <cstdio>
#include <stdint.h>
#include <string.h>
#include <fstream>
#include <vector>
#include <cstdint>
#include <mutex>
#include <nlohmann/json.hpp> // JSON library: https://github.com/nlohmann/json
#include <atomic>
#include <time.h>


extern "C"
{
#include "../gfx.h"
#include "../fonts.h"
#include "../ssd1327_oled.h"
#include "../font_8x5.h"

}


int main() {
    // Init SPI, GPIO, reset OLED
    SSD1327_SpiInit();

    // Wait a bit after init
    usleep(200000); // 200 ms

    // 1) Fill screen white
    SSD1327_Clear(WHITE);
    SSD1327_Display();
    sleep(2);

    // 2) Fill screen black
    SSD1327_Clear(BLACK);
    SSD1327_Display();
    sleep(2);

    // 3) Checkerboard pattern
    for (int y = 0; y < SSD1327_LCDHEIGHT; y++) {
        for (int x = 0; x < SSD1327_LCDWIDTH; x++) {
            uint8_t color = ((x / 8 + y / 8) % 2) ? WHITE : BLACK;
            SSD1327_DrawPixel(x, y, color);
        }
    }
    SSD1327_Display();
    sleep(5);

    return 0;
}
