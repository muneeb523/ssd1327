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


int main(){

  SSD1327_SpiInit();
  SSD1327_Clear(BLACK);
  GFX_SetFont(font_8x5);
  GFX_SetFontSize(1);


  GFX_DrawString(6, 80, "  Hello  From here   ", WHITE, BLACK);
  SSD1327_Display();

}