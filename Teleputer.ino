/**
 * @file Teleputer.ino
 * @brief Bidirectional Real-Time Telegram Bot Client Firmware for M5Stack Cardputer ADV (ESP32-S3)
 *        - Zero-Lag Dual-Core Architecture: Core 0 handles HTTPS socket waits ONLY;
 *          Core 1 owns all SPI (Display + MicroSD), Keyboard, and Audio hardware (zero bus contention).
 *        - Low-RAM Footprint (>170 KB free heap for mbedTLS & Wi-Fi at all times).
 *        - Double-Buffered DMA Voice Recording & Gapless WAV Playback.
 *        - Full MicroSD Chat History & Audio Persistence (/tg_chats & /tg_audio).
 *        - Multi-Stage Screen Timeout (15s dim -> 5s off) & Adaptive Polling (1.5s active / 10s sleep).
 *
 * Key Controls:
 *  - [TAB]            : Toggle between View A (Chat List Hub) and View B (Conversation Thread)
 *  - [;] / [.]        : Navigate chat list in View A
 *  - [Fn + ;] / [Fn + .] : Move message selection Up / Down (and auto-scroll) in View B
 *  - [ENTER]          : Open chat (View A) / Send text OR play .wav / view Photo when input is empty (View B)
 *  - [Fn + R]         : Start / Stop & Send Voice Recording (.wav) in View B
 */

#include <Arduino.h>
#include <M5Cardputer.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <vector>
#include <deque>

// ============================================================================
// HARDWARE PIN DEFINITIONS & CONSTANTS
// ============================================================================
static constexpr int SD_SPI_SCK_PIN  = 40;
static constexpr int SD_SPI_MISO_PIN = 39;
static constexpr int SD_SPI_MOSI_PIN = 14;
static constexpr int SD_SPI_CS_PIN   = 12;

static constexpr int SCREEN_WIDTH    = 240;
static constexpr int SCREEN_HEIGHT   = 135;

// Layout Geometry - View A (Chat Selection Hub)
static constexpr int HEADER_HEIGHT       = 18;
static constexpr int VIEW_A_LEFT_WIDTH   = 96;   // 40% of 240px
static constexpr int VIEW_A_RIGHT_WIDTH  = 144;  // 60% of 240px
static constexpr int VIEW_A_BODY_Y       = HEADER_HEIGHT;
static constexpr int VIEW_A_BODY_HEIGHT  = SCREEN_HEIGHT - HEADER_HEIGHT; // 117px
static constexpr int VIEW_A_MAX_VISIBLE  = 5;
static constexpr int VIEW_A_ROW_HEIGHT   = 23;   // 5 * 23 = 115px

// Layout Geometry - View B (Conversation Thread)
static constexpr int VIEW_B_HEADER_H     = 18;
static constexpr int VIEW_B_INPUT_H      = 15;   // Dedicated 15px bottom input bar
static constexpr int VIEW_B_INPUT_Y      = SCREEN_HEIGHT - VIEW_B_INPUT_H; // y = 120
static constexpr int VIEW_B_BODY_Y       = VIEW_B_HEADER_H;
static constexpr int VIEW_B_BODY_H       = SCREEN_HEIGHT - VIEW_B_HEADER_H - VIEW_B_INPUT_H; // 102px

// Adaptive Polling & Power Timeout Constants
static constexpr unsigned long POLL_INTERVAL_ACTIVE_MS = 1500UL;  // 1.5s near-instant when awake/dimmed
static constexpr unsigned long POLL_INTERVAL_SLEEP_MS  = 10000UL; // 10.0s battery saver when screen off
static constexpr unsigned long SCREEN_DIM_TIMEOUT_MS   = 15000UL; // 15s inactivity -> dim screen
static constexpr unsigned long SCREEN_OFF_TIMEOUT_MS   = 20000UL; // +5s dimmed (20s total) -> screen off

static constexpr uint8_t BRIGHTNESS_AWAKE  = 180;
static constexpr uint8_t BRIGHTNESS_DIMMED = 22;
static constexpr uint8_t BRIGHTNESS_OFF    = 0;

// Voice Recording & Playback Constants (2 KB total ping-pong DMA RAM -> zero heap pressure!)
static constexpr uint32_t VOICE_SAMPLE_RATE      = 16000; // 16 kHz mono 16-bit PCM
static constexpr size_t   AUDIO_DMA_SAMPLES      = 512;   // 32ms per DMA slot (1024 bytes)
static constexpr uint32_t MAX_RECORD_SECONDS     = 12;    // Max 12s per voice note

// Memory & Storage Limits
static constexpr size_t MAX_CONTACTS            = 25;
static constexpr size_t MAX_MESSAGES_IN_RAM     = 60;
static constexpr size_t JSON_CONFIG_DOC_SIZE    = 1024;
static constexpr size_t JSON_POLL_DOC_SIZE      = 8192;
static constexpr size_t JSON_SEND_DOC_SIZE      = 1536;
static constexpr size_t JSON_LINE_DOC_SIZE      = 768;

static const char* CONFIG_FILE_PATH    = "/tg_config.json";
static const char* CONTACTS_INDEX_PATH = "/tg_contacts.json";
static const char* CHAT_HISTORY_DIR    = "/tg_chats";
static const char* AUDIO_STORAGE_DIR   = "/tg_audio";
static const char* PHOTO_STORAGE_DIR   = "/tg_photos";
static const char* TEMP_REC_RAW_PATH   = "/tg_audio/_rec_tmp.raw";

// ============================================================================
// DARK THEME COLOR PALETTE (RGB565)
// ============================================================================
static constexpr uint16_t COL_BG_DARK        = 0x0842; // #080810 Deep obsidian
static constexpr uint16_t COL_BG_PANEL_LEFT  = 0x1084; // #101020 Left list background
static constexpr uint16_t COL_BG_PANEL_RIGHT = 0x0863; // #080C18 Right preview background
static constexpr uint16_t COL_BG_ROW_ALT     = 0x14A5; // #141428 Alternating row background (View A list)
static constexpr uint16_t COL_BG_ROW_SEL     = 0x1D5E; // #182A70 Highlighted active chat in View A
static constexpr uint16_t COL_BG_MSG_SEL     = 0x2104; // #202020 Subtle dark-grey highlight for selected message in View B
static constexpr uint16_t COL_BG_HEADER      = 0x10A6; // #101430 Header bar dark navy
static constexpr uint16_t COL_BG_INPUT       = 0x18E7; // #181C38 Bottom input bar background
static constexpr uint16_t COL_BG_INPUT_BUSY  = 0x2945; // Dimmed input when freezing/sending
static constexpr uint16_t COL_BG_INPUT_REC   = 0x4802; // Deep crimson bar while recording voice
static constexpr uint16_t COL_BORDER         = 0x31A9; // #303448 Crisp panel divider/border
static constexpr uint16_t COL_MSG_SEPARATOR  = 0x2126; // Thin dark line separating messages in View B
static constexpr uint16_t COL_TEXT_PRIMARY   = 0xFFFF; // Crisp white
static constexpr uint16_t COL_TEXT_SECONDARY = 0x9CF3; // Muted silver-gray
static constexpr uint16_t COL_TEXT_INCOMING  = 0xCE79; // Grey for received messages
static constexpr uint16_t COL_TEXT_OUTGOING  = 0x55FF; // Blue for sent messages
static constexpr uint16_t COL_UNREAD_DOT     = 0xF986; // Vibrant crimson red indicator
static constexpr uint16_t COL_ACCENT_CYAN    = 0x05FF; // Telegram cyan accent
static constexpr uint16_t COL_STATUS_OK      = 0x2689; // Emerald green
static constexpr uint16_t COL_STATUS_ERR     = 0xF986; // Error red

// ============================================================================
// DATA STRUCTURES
// ============================================================================
enum class AppView : uint8_t {
    CHAT_LIST_HUB = 0,     // View A: Chat Selection List (Main Hub)
    CONVERSATION_THREAD    // View B: Conversation Chat Thread
};

enum class ScreenPowerState : uint8_t {
    AWAKE = 0,             // 0 - 15s since last key: Full brightness, 1.5s fast polling
    DIMMED,                // 15 - 20s since last key: Dimmed brightness (5s), 1.5s polling
    OFF                    // > 20s since last key: Screen off, 10s battery-saving polling
};

enum class MediaType : uint8_t {
    NONE = 0,
    VOICE,                 // Voice note (.wav playable; .oga blocked on Cardputer)
    AUDIO,                 // Audio file (.wav playable; non-wav blocked)
    VIDEO_AUDIO_ONLY,      // Video message (blocked unless .wav)
    PHOTO                  // Photo / Image (.jpg viewable fullscreen on Cardputer)
};

struct ChatMessage {
    bool isOutgoing;       // false = Incoming (grey, left), true = Outgoing (blue, right)
    String text;           // UTF-8 / ASCII sanitized message body
    uint32_t timestamp;    // Unix epoch or relative seconds
    MediaType mediaType    = MediaType::NONE;
    String mediaFileId     = "";
    String localAudioPath  = ""; // Permanent file path on MicroSD (/tg_audio/... or /tg_photos/...)
    uint16_t mediaDurationSec = 0;
};

struct ContactChat {
    int64_t chatId;
    String name;           // First name / username
    String username;
    bool hasUnread;
    uint32_t lastActivity;
    std::deque<ChatMessage> messages;
};

struct AppConfig {
    String wifiSsid;
    String wifiPass;
    String botToken;
    String transcodeUrl;
    bool valid = false;
};

struct WrappedMessageLine {
    int  msgIndex;
    bool isOutgoing;
    bool isFirstLineOfMsg;
    bool isLastLineOfMsg;
    String lineText;
};

// Parsed incoming message passed from Core 0 network poll to Core 1
struct IncomingUpdateEvent {
    int64_t updateId;
    int64_t chatId;
    String displayName;
    String username;
    String text;
    uint32_t timestamp;
    MediaType mediaType;
    String mediaFileId;
    uint16_t mediaDurationSec;
};

#pragma pack(push, 1)
struct WavHeader {
    char     riff[4]       = {'R', 'I', 'F', 'F'};
    uint32_t overallSize   = 36;
    char     wave[4]       = {'W', 'A', 'V', 'E'};
    char     fmtChunk[4]   = {'f', 'm', 't', ' '};
    uint32_t fmtLength     = 16;
    uint16_t formatType    = 1; // 1 = PCM
    uint16_t channels      = 1; // Mono
    uint32_t sampleRate    = VOICE_SAMPLE_RATE;
    uint32_t byteRate      = VOICE_SAMPLE_RATE * sizeof(int16_t);
    uint16_t blockAlign    = sizeof(int16_t);
    uint16_t bitsPerSample = 16;
    char     dataChunk[4]  = {'d', 'a', 't', 'a'};
    uint32_t dataSize      = 0;
};
#pragma pack(pop)

// ============================================================================
// GLOBAL APPLICATION STATE
// ============================================================================
static M5Canvas canvas(&M5Cardputer.Display);
static AppConfig g_config;
static AppView g_currentView = AppView::CHAT_LIST_HUB;
static volatile ScreenPowerState g_screenState = ScreenPowerState::AWAKE;

// Lightweight mutex ONLY for queuing parsed incoming messages from Core 0 to Core 1
static SemaphoreHandle_t g_queueMutex = nullptr;
static TaskHandle_t g_pollTaskHandle  = nullptr;
static std::vector<IncomingUpdateEvent> g_incomingQueue;
static volatile bool g_pauseBackgroundPoll = false;
static volatile int64_t g_lastUpdateId     = 0;
static String g_netStatusBadge             = "LIVE";

// Owned exclusively by Core 1 (UI / SD / Keyboard / Audio) -> zero mutex stalls!
static std::vector<ContactChat> g_chats;
static int g_selectedChatIndex  = 0;
static int g_listScrollOffset   = 0;
static int g_threadScrollOffset = 0;
static int g_selectedMsgIndex   = -1;

static String g_inputBuffer      = "";
static bool g_isSendingMessage   = false;
static bool g_isPlayingAudio     = false;
static bool g_isViewingPhoto     = false;
static String g_activePhotoPath  = "";
static String g_activePhotoTitle = "";
static String g_audioPlayBanner  = "";
static bool g_uiDirty            = true;

// 3-slot Ring DMA Buffer (only 3 KB total!) used for Recording & Playback
static int16_t g_audioDmaBuf[3][AUDIO_DMA_SAMPLES];
static bool g_isRecordingVoice        = false;
static File g_recordFile;
static String g_activeRecordPath      = "";
static unsigned long g_recordStartMs  = 0;
static uint32_t g_recordedDataBytes   = 0;
static uint32_t g_recWriteStep        = 0;
static uint32_t g_recQueueStep        = 0;
static int64_t  g_recSampleSum        = 0;
static uint32_t g_recSampleCount      = 0;
static int32_t  g_recMaxSample        = -32768;
static int32_t  g_recMinSample        =  32767;
static uint32_t g_audioFileCounter    = 0;

// Timers & Debounce State
static unsigned long g_lastUserActivityMs = 0;
static unsigned long g_lastKeyPressMs     = 0;

// ============================================================================
// SPEAKER / MICROPHONE SWITCHING (PRESERVES M5CARDPUTER ADV ES8311 CODEC)
// ============================================================================
void boostCardputerAdvMicGain() {
    if (M5.getBoard() == m5::board_t::board_M5CardputerADV) {
        // ES8311 derives MCLK from BCLK (0x01=0xBA) and ignores 0x0D/0x0E power-up writes made before
        // I2S BCLK is running. Re-arm the full ES8311 ADC capture path once I2S RX clock is active:
        M5.In_I2C.writeRegister8(0x18, 0x00, 0x80, 400000); // CSM POWER ON
        M5.In_I2C.writeRegister8(0x18, 0x01, 0xBA, 400000); // MCLK = BCLK, enable ADC/DAC clocks
        M5.In_I2C.writeRegister8(0x18, 0x02, 0x18, 400000); // MULT_PRE = 3
        M5.In_I2C.writeRegister8(0x18, 0x0D, 0x01, 400000); // Power up analog circuitry (arms ADC while BCLK runs)
        M5.In_I2C.writeRegister8(0x18, 0x0E, 0x02, 400000); // Enable analog PGA & ADC modulator
        M5.In_I2C.writeRegister8(0x18, 0x14, 0x1A, 400000); // Mic1p-Mic1n +30dB analog PGA gain
        M5.In_I2C.writeRegister8(0x18, 0x17, 0xDF, 400000); // +16dB digital ADC volume
        M5.In_I2C.writeRegister8(0x18, 0x1C, 0x6A, 400000); // ADC EQ bypass + DC offset cancel
    }
}

void configureCardputerMicrophone() {
    // Keep M5Unified's native channel & oversampling configuration intact so Mic_Class::_rec_task
    // accumulates samples properly, and boost software magnification:
    auto micCfg = M5Cardputer.Mic.config();
    micCfg.sample_rate        = VOICE_SAMPLE_RATE;
    micCfg.magnification      = 32;
    micCfg.noise_filter_level = 0;
    M5Cardputer.Mic.config(micCfg);
}

void ensureSpeakerActive() {
    if (M5Cardputer.Mic.isEnabled()) {
        while (M5Cardputer.Mic.isRecording()) {
            delay(1);
        }
        M5Cardputer.Mic.end();
    }
    if (!M5Cardputer.Speaker.isEnabled()) {
        M5Cardputer.Speaker.begin();
        M5Cardputer.Speaker.setVolume(255);
    }
}

void ensureMicActive() {
    if (M5Cardputer.Speaker.isEnabled()) {
        M5Cardputer.Speaker.stop();
        M5Cardputer.Speaker.end();
    }
    if (!M5Cardputer.Mic.isEnabled()) {
        configureCardputerMicrophone();
        M5Cardputer.Mic.begin();
    }
}

void playNotificationChime() {
    if (g_isRecordingVoice || g_isPlayingAudio) return;
    ensureSpeakerActive();
    M5Cardputer.Speaker.setVolume(190);
    M5Cardputer.Speaker.tone(1046, 70); // C6
    delay(80);
    M5Cardputer.Speaker.tone(1318, 95); // E6
}

void playSendChime() {
    if (g_isRecordingVoice || g_isPlayingAudio) return;
    ensureSpeakerActive();
    M5Cardputer.Speaker.setVolume(150);
    M5Cardputer.Speaker.tone(880, 45);
}

void playErrorChime() {
    if (g_isRecordingVoice || g_isPlayingAudio) return;
    ensureSpeakerActive();
    M5Cardputer.Speaker.setVolume(190);
    M5Cardputer.Speaker.tone(330, 120);
    delay(130);
    M5Cardputer.Speaker.tone(220, 160);
}

// ============================================================================
// SCREEN POWER & BATTERY MANAGEMENT
// ============================================================================
void registerUserActivity() {
    g_lastUserActivityMs = millis();
    if (g_screenState != ScreenPowerState::AWAKE) {
        if (g_screenState == ScreenPowerState::OFF) {
            M5Cardputer.Display.wakeup();
            esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        }
        M5Cardputer.Display.setBrightness(BRIGHTNESS_AWAKE);
        g_screenState = ScreenPowerState::AWAKE;
        g_uiDirty = true;
    }
}

void updateScreenPowerState() {
    if (g_isRecordingVoice || g_isPlayingAudio || g_isSendingMessage) {
        g_lastUserActivityMs = millis();
        return;
    }

    const unsigned long idleMs = millis() - g_lastUserActivityMs;

    if (idleMs >= SCREEN_OFF_TIMEOUT_MS) {
        if (g_screenState != ScreenPowerState::OFF) {
            g_screenState = ScreenPowerState::OFF;
            M5Cardputer.Display.setBrightness(BRIGHTNESS_OFF);
            M5Cardputer.Display.sleep();
            esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
        }
    } else if (idleMs >= SCREEN_DIM_TIMEOUT_MS) {
        if (g_screenState != ScreenPowerState::DIMMED) {
            if (g_screenState == ScreenPowerState::OFF) {
                M5Cardputer.Display.wakeup();
            }
            g_screenState = ScreenPowerState::DIMMED;
            M5Cardputer.Display.setBrightness(BRIGHTNESS_DIMMED);
        }
    }
}

unsigned long getCurrentPollIntervalMs() {
    return (g_screenState == ScreenPowerState::OFF)
           ? POLL_INTERVAL_SLEEP_MS
           : POLL_INTERVAL_ACTIVE_MS;
}

// ============================================================================
// STRING & WORD-WRAPPING UTILITIES
// ============================================================================
String sanitizeText(const String& raw) {
    String out;
    out.reserve(raw.length());
    for (size_t i = 0; i < raw.length(); ++i) {
        char c = raw[i];
        if (c == '\r') continue;
        if (c == '\n' || c == '\t') {
            out += ' ';
        } else if (static_cast<uint8_t>(c) >= 32 && static_cast<uint8_t>(c) <= 126) {
            out += c;
        } else if (static_cast<uint8_t>(c) >= 0xC0) {
            out += '?';
            while (i + 1 < raw.length() && (static_cast<uint8_t>(raw[i + 1]) & 0xC0) == 0x80) {
                ++i;
            }
        }
    }
    return out;
}

void wrapTextToLines(M5Canvas& gfx, const String& text, int maxWidthPx, std::vector<String>& outLines) {
    if (text.isEmpty()) {
        outLines.push_back("");
        return;
    }

    String currentLine = "";
    int idx = 0;
    const int len = text.length();

    while (idx < len) {
        int nextSpace = text.indexOf(' ', idx);
        if (nextSpace == -1) nextSpace = len;
        String word = text.substring(idx, nextSpace);

        if (gfx.textWidth(word) > maxWidthPx) {
            if (!currentLine.isEmpty()) {
                outLines.push_back(currentLine);
                currentLine = "";
            }
            String chunk = "";
            for (size_t cIdx = 0; cIdx < word.length(); ++cIdx) {
                String candidate = chunk + word[cIdx];
                if (gfx.textWidth(candidate) > maxWidthPx && !chunk.isEmpty()) {
                    outLines.push_back(chunk);
                    chunk = String(word[cIdx]);
                } else {
                    chunk = candidate;
                }
            }
            currentLine = chunk;
        } else {
            String candidate = currentLine.isEmpty() ? word : (currentLine + " " + word);
            if (gfx.textWidth(candidate) <= maxWidthPx) {
                currentLine = candidate;
            } else {
                if (!currentLine.isEmpty()) {
                    outLines.push_back(currentLine);
                }
                currentLine = word;
            }
        }

        idx = nextSpace + 1;
    }

    if (!currentLine.isEmpty()) {
        outLines.push_back(currentLine);
    }
}

String ellipsizeText(M5Canvas& gfx, const String& text, int maxWidthPx) {
    if (gfx.textWidth(text) <= maxWidthPx) return text;
    String suffix = "..";
    int suffixW = gfx.textWidth(suffix);
    String clipped = "";
    for (size_t i = 0; i < text.length(); ++i) {
        if (gfx.textWidth(clipped + text[i]) + suffixW > maxWidthPx) {
            break;
        }
        clipped += text[i];
    }
    return clipped + suffix;
}

// ============================================================================
// SD CARD CONFIGURATION, FULL CHAT HISTORY & AUDIO FILE PERSISTENCE (CORE 1)
// ============================================================================
String getChatHistoryFilePath(int64_t chatId) {
    return String(CHAT_HISTORY_DIR) + "/chat_" + String(chatId) + ".jsonl";
}

String generateUniqueAudioFilePath(int64_t chatId, bool isOutgoing, const char* ext) {
    ++g_audioFileCounter;
    String prefix = isOutgoing ? "/out_" : "/in_";
    return String(AUDIO_STORAGE_DIR) + prefix + String(chatId) + "_" +
           String(millis()) + "_" + String(g_audioFileCounter) + ext;
}

String generateUniquePhotoFilePath(int64_t chatId) {
    ++g_audioFileCounter;
    return String(PHOTO_STORAGE_DIR) + "/img_" + String(chatId) + "_" +
           String(millis()) + "_" + String(g_audioFileCounter) + ".jpg";
}

void ensureStorageDirectories() {
    if (!SD.exists(CHAT_HISTORY_DIR)) {
        SD.mkdir(CHAT_HISTORY_DIR);
    }
    if (!SD.exists(AUDIO_STORAGE_DIR)) {
        SD.mkdir(AUDIO_STORAGE_DIR);
    }
    if (!SD.exists(PHOTO_STORAGE_DIR)) {
        SD.mkdir(PHOTO_STORAGE_DIR);
    }
}

void renderFatalErrorScreen(const String& title, const String& subtitle, const String& hint1, const String& hint2) {
    canvas.fillSprite(COL_BG_DARK);

    canvas.fillRect(0, 0, SCREEN_WIDTH, 22, COL_STATUS_ERR);
    canvas.setTextColor(COL_TEXT_PRIMARY, COL_STATUS_ERR);
    canvas.setTextDatum(top_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(1);
    canvas.drawString("! SD CARD SETUP FAILED", 8, 7);

    canvas.drawRect(4, 26, SCREEN_WIDTH - 8, SCREEN_HEIGHT - 30, COL_BORDER);
    canvas.fillRect(5, 27, SCREEN_WIDTH - 10, SCREEN_HEIGHT - 32, COL_BG_PANEL_LEFT);

    canvas.setTextColor(COL_ACCENT_CYAN);
    canvas.drawString(title, 12, 34);

    canvas.setTextColor(COL_TEXT_PRIMARY);
    canvas.drawString(subtitle, 12, 48);

    canvas.drawFastHLine(12, 62, SCREEN_WIDTH - 24, COL_BORDER);

    canvas.setTextColor(COL_TEXT_SECONDARY);
    canvas.drawString(hint1, 12, 68);
    canvas.drawString(hint2, 12, 80);

    canvas.setTextColor(COL_TEXT_OUTGOING);
    canvas.drawString("{\"wifi_ssid\":\"...\",", 12, 95);
    canvas.drawString(" \"wifi_pass\":\"...\",", 12, 106);
    canvas.drawString(" \"token\":\"...\"}", 12, 117);

    canvas.pushSprite(0, 0);
}

bool initSDAndLoadConfig(AppConfig& outCfg) {
    SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);

    if (!SD.begin(SD_SPI_CS_PIN, SPI, 20000000)) {
        playErrorChime();
        renderFatalErrorScreen(
            "MicroSD Mount Failed (CS: GPIO 12)",
            "Insert FAT32 formatted MicroSD card.",
            "Required file: /tg_config.json",
            "Press RST after inserting card."
        );
        return false;
    }

    if (!SD.exists(CONFIG_FILE_PATH)) {
        playErrorChime();
        renderFatalErrorScreen(
            "Missing /tg_config.json",
            "File not found in MicroSD root.",
            "Create /tg_config.json with keys:",
            "wifi_ssid, wifi_pass, token"
        );
        return false;
    }

    File cfgFile = SD.open(CONFIG_FILE_PATH, FILE_READ);
    if (!cfgFile || cfgFile.isDirectory()) {
        playErrorChime();
        renderFatalErrorScreen(
            "Cannot Open /tg_config.json",
            "File read permission or I/O error.",
            "Check FAT32 filesystem integrity.",
            "Expected valid JSON payload."
        );
        return false;
    }

    DynamicJsonDocument doc(JSON_CONFIG_DOC_SIZE);
    DeserializationError err = deserializeJson(doc, cfgFile);
    cfgFile.close();

    if (err) {
        doc.clear();
        playErrorChime();
        renderFatalErrorScreen(
            "Corrupt /tg_config.json",
            String("JSON Error: ") + err.c_str(),
            "Verify quotes and commas in file.",
            "Required keys below:"
        );
        return false;
    }

    outCfg.wifiSsid     = String(doc["wifi_ssid"] | "");
    outCfg.wifiPass     = String(doc["wifi_pass"] | "");
    outCfg.botToken     = String(doc["token"]     | "");
    outCfg.transcodeUrl = String(doc["transcode_url"] | "");
    doc.clear();

    if (outCfg.wifiSsid.isEmpty() || outCfg.botToken.isEmpty()) {
        playErrorChime();
        renderFatalErrorScreen(
            "Incomplete /tg_config.json",
            "wifi_ssid or token is empty!",
            "Populate all 3 fields in JSON:",
            "wifi_ssid, wifi_pass, token"
        );
        return false;
    }

    ensureStorageDirectories();
    outCfg.valid = true;
    return true;
}

void appendMessageToSDHistory(int64_t chatId, const ChatMessage& msg) {
    ensureStorageDirectories();
    String path = getChatHistoryFilePath(chatId);
    File f = SD.open(path.c_str(), FILE_APPEND);
    if (!f) {
        f = SD.open(path.c_str(), FILE_WRITE);
    }
    if (!f) return;

    DynamicJsonDocument doc(JSON_LINE_DOC_SIZE);
    doc["out"]   = msg.isOutgoing;
    doc["text"]  = msg.text;
    doc["ts"]    = msg.timestamp;
    doc["mtype"] = static_cast<uint8_t>(msg.mediaType);
    if (!msg.mediaFileId.isEmpty())    doc["fid"]  = msg.mediaFileId;
    if (!msg.localAudioPath.isEmpty()) doc["path"] = msg.localAudioPath;
    if (msg.mediaDurationSec > 0)      doc["dur"]  = msg.mediaDurationSec;

    serializeJson(doc, f);
    f.print('\n');
    f.close();
    doc.clear();
}

void rewriteFullChatHistoryToSD(const ContactChat& chat) {
    ensureStorageDirectories();
    String path = getChatHistoryFilePath(chat.chatId);
    if (SD.exists(path.c_str())) {
        SD.remove(path.c_str());
    }
    File f = SD.open(path.c_str(), FILE_WRITE);
    if (!f) return;

    DynamicJsonDocument doc(JSON_LINE_DOC_SIZE);
    for (const auto& msg : chat.messages) {
        doc.clear();
        doc["out"]   = msg.isOutgoing;
        doc["text"]  = msg.text;
        doc["ts"]    = msg.timestamp;
        doc["mtype"] = static_cast<uint8_t>(msg.mediaType);
        if (!msg.mediaFileId.isEmpty())    doc["fid"]  = msg.mediaFileId;
        if (!msg.localAudioPath.isEmpty()) doc["path"] = msg.localAudioPath;
        if (msg.mediaDurationSec > 0)      doc["dur"]  = msg.mediaDurationSec;
        serializeJson(doc, f);
        f.print('\n');
    }
    f.close();
    doc.clear();
}

void loadChatHistoryFromSD(ContactChat& chat) {
    String path = getChatHistoryFilePath(chat.chatId);
    if (!SD.exists(path.c_str())) return;

    File f = SD.open(path.c_str(), FILE_READ);
    if (!f) return;

    chat.messages.clear();
    DynamicJsonDocument doc(JSON_LINE_DOC_SIZE);

    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.isEmpty()) continue;

        doc.clear();
        if (deserializeJson(doc, line) == DeserializationError::Ok) {
            ChatMessage m;
            m.isOutgoing       = doc["out"] | false;
            m.text             = doc["text"] | "";
            m.timestamp        = doc["ts"] | 0;
            m.mediaType        = static_cast<MediaType>(doc["mtype"] | 0);
            m.mediaFileId      = doc["fid"] | "";
            m.localAudioPath   = doc["path"] | "";
            m.mediaDurationSec = doc["dur"] | 0;

            if (chat.messages.size() >= MAX_MESSAGES_IN_RAM) {
                chat.messages.pop_front();
            }
            chat.messages.push_back(m);
        }
    }
    f.close();
    doc.clear();
}

void saveContactsIndexToSD() {
    File f = SD.open(CONTACTS_INDEX_PATH, FILE_WRITE);
    if (!f) return;

    DynamicJsonDocument doc(4096);
    doc["last_update_id"] = g_lastUpdateId;
    JsonArray arr = doc.createNestedArray("contacts");
    for (const auto& c : g_chats) {
        JsonObject o = arr.createNestedObject();
        o["id"]     = c.chatId;
        o["name"]   = c.name;
        o["user"]   = c.username;
        o["unread"] = c.hasUnread;
    }
    serializeJson(doc, f);
    f.close();
    doc.clear();
}

void loadContactsAndHistoriesFromSD() {
    if (!SD.exists(CONTACTS_INDEX_PATH)) return;
    File f = SD.open(CONTACTS_INDEX_PATH, FILE_READ);
    if (!f) return;

    DynamicJsonDocument doc(4096);
    if (deserializeJson(doc, f) == DeserializationError::Ok) {
        JsonArray arr;
        if (doc.is<JsonObject>()) {
            g_lastUpdateId = doc["last_update_id"] | 0;
            arr = doc["contacts"].as<JsonArray>();
        } else if (doc.is<JsonArray>()) {
            arr = doc.as<JsonArray>();
        }

        for (JsonObject o : arr) {
            if (g_chats.size() >= MAX_CONTACTS) break;
            ContactChat c;
            c.chatId       = o["id"].as<int64_t>();
            c.name         = o["name"] | "Unknown";
            c.username     = o["user"] | "";
            c.hasUnread    = o["unread"] | false;
            c.lastActivity = 0;

            if (c.chatId != 0) {
                loadChatHistoryFromSD(c);
                g_chats.push_back(c);
            }
        }
    }
    f.close();
    doc.clear();
}

// ============================================================================
// CHAT LIST MANAGEMENT (PRESERVES CURRENTLY SELECTED CHAT ON INCOMING UPDATES)
// ============================================================================
int64_t getSelectedChatId() {
    if (g_chats.empty() || g_selectedChatIndex < 0 || g_selectedChatIndex >= static_cast<int>(g_chats.size())) {
        return 0;
    }
    return g_chats[g_selectedChatIndex].chatId;
}

void restoreSelectedChatById(int64_t targetChatId) {
    if (targetChatId == 0 || g_chats.empty()) return;
    for (size_t i = 0; i < g_chats.size(); ++i) {
        if (g_chats[i].chatId == targetChatId) {
            g_selectedChatIndex = static_cast<int>(i);
            return;
        }
    }
}

/**
 * @brief Finds an existing contact or inserts a new contact without hijacking the user's active chat!
 */
int findOrCreateChat(int64_t chatId, const String& displayName, const String& username) {
    for (size_t i = 0; i < g_chats.size(); ++i) {
        if (g_chats[i].chatId == chatId) {
            if (!displayName.isEmpty()) g_chats[i].name = displayName;
            if (!username.isEmpty())    g_chats[i].username = username;
            return static_cast<int>(i);
        }
    }

    int64_t previousSelectedId = getSelectedChatId();

    ContactChat newChat;
    newChat.chatId       = chatId;
    newChat.name         = displayName.isEmpty() ? String(chatId) : displayName;
    newChat.username     = username;
    newChat.hasUnread    = false;
    newChat.lastActivity = millis() / 1000;

    if (g_chats.size() >= MAX_CONTACTS) {
        g_chats.pop_back();
    }
    g_chats.insert(g_chats.begin(), newChat);

    // Keep user's current chat selection anchored so a new user /start-ing the bot
    // NEVER switches the active conversation away from who you are currently texting!
    if (previousSelectedId != 0) {
        restoreSelectedChatById(previousSelectedId);
    }

    return 0;
}

void promoteChatToTop(int index, bool switchSelectionToTop) {
    if (index <= 0 || index >= static_cast<int>(g_chats.size())) {
        if (switchSelectionToTop) g_selectedChatIndex = 0;
        return;
    }

    int64_t previousSelectedId = getSelectedChatId();

    ContactChat moved = g_chats[index];
    g_chats.erase(g_chats.begin() + index);
    g_chats.insert(g_chats.begin(), moved);

    if (switchSelectionToTop) {
        g_selectedChatIndex = 0;
    } else if (previousSelectedId != 0) {
        restoreSelectedChatById(previousSelectedId);
    }
}

void appendMessageToChat(int chatIdx, bool isOutgoing, const String& text, uint32_t ts,
                         MediaType mediaType = MediaType::NONE,
                         const String& mediaFileId = "",
                         uint16_t mediaDurationSec = 0,
                         const String& localAudioPath = "") {
    if (chatIdx < 0 || chatIdx >= static_cast<int>(g_chats.size())) return;

    ContactChat& chat = g_chats[chatIdx];
    ChatMessage msg;
    msg.isOutgoing       = isOutgoing;
    msg.text             = sanitizeText(text);
    msg.timestamp        = ts;
    msg.mediaType        = mediaType;
    msg.mediaFileId      = mediaFileId;
    msg.localAudioPath   = localAudioPath;
    msg.mediaDurationSec = mediaDurationSec;

    if (chat.messages.size() >= MAX_MESSAGES_IN_RAM) {
        chat.messages.pop_front();
    }
    chat.messages.push_back(msg);
    chat.lastActivity = ts;

    appendMessageToSDHistory(chat.chatId, msg);

    if (!isOutgoing) {
        bool activelyViewingThisChat = (g_currentView == AppView::CONVERSATION_THREAD &&
                                        g_selectedChatIndex == chatIdx &&
                                        g_screenState != ScreenPowerState::OFF);
        if (!activelyViewingThisChat) {
            chat.hasUnread = true;
        }
    }
}

// ============================================================================
// CORE 0 BACKGROUND POLLING TASK (PURE NETWORK ONLY - NEVER TOUCHES SPI/SD!)
// ============================================================================
String buildBotApiUrl(const char* method) {
    String url = "https://api.telegram.org/bot";
    url += g_config.botToken;
    url += "/";
    url += method;
    return url;
}

void backgroundPollTaskCore0(void* param) {
    (void)param;
    WiFiClientSecure pollClient;
    pollClient.setInsecure();
    pollClient.setTimeout(3);

    unsigned long lastPollMs = 0;

    while (true) {
        unsigned long now = millis();
        unsigned long interval = getCurrentPollIntervalMs();

        if (!g_pauseBackgroundPoll && (now - lastPollMs >= interval)) {
            lastPollMs = now;

            if (WiFi.status() != WL_CONNECTED) {
                WiFi.reconnect();
                g_netStatusBadge = "RECONN";
            } else {
                HTTPClient https;
                https.setReuse(true);
                https.setTimeout(2200);

                String url = buildBotApiUrl("getUpdates");
                url += "?timeout=0&limit=5&allowed_updates=%5B%22message%22%5D";
                int64_t offsetId = g_lastUpdateId;
                if (offsetId > 0) {
                    url += "&offset=" + String(offsetId + 1);
                }

                if (https.begin(pollClient, url)) {
                    int httpCode = https.GET();
                    if (httpCode == HTTP_CODE_OK) {
                        DynamicJsonDocument doc(JSON_POLL_DOC_SIZE);
                        DeserializationError err = deserializeJson(doc, https.getStream());

                        if (!err && (doc["ok"] | false)) {
                            g_netStatusBadge = (g_screenState == ScreenPowerState::OFF) ? "SLEEP" : "LIVE";
                            JsonArray results = doc["result"].as<JsonArray>();

                            for (JsonObject update : results) {
                                int64_t updateId = update["update_id"].as<int64_t>();
                                if (updateId > g_lastUpdateId) {
                                    g_lastUpdateId = updateId;
                                }

                                JsonObject message = update["message"];
                                if (message.isNull()) continue;

                                JsonObject chat = message["chat"];
                                if (chat.isNull()) continue;

                                IncomingUpdateEvent ev;
                                ev.updateId = updateId;
                                ev.chatId   = chat["id"].as<int64_t>();

                                String firstName = sanitizeText(String(chat["first_name"] | ""));
                                String lastName  = sanitizeText(String(chat["last_name"]  | ""));
                                ev.username      = sanitizeText(String(chat["username"]   | ""));
                                String title     = sanitizeText(String(chat["title"]      | ""));

                                ev.displayName = firstName;
                                if (!lastName.isEmpty()) {
                                    if (!ev.displayName.isEmpty()) ev.displayName += " ";
                                    ev.displayName += lastName;
                                }
                                if (ev.displayName.isEmpty()) ev.displayName = title;
                                if (ev.displayName.isEmpty()) ev.displayName = ev.username;
                                if (ev.displayName.isEmpty()) ev.displayName = String(ev.chatId);

                                ev.text             = String(message["text"] | "");
                                ev.mediaType        = MediaType::NONE;
                                ev.mediaFileId      = "";
                                ev.mediaDurationSec = 0;

                                if (!message["photo"].isNull()) {
                                    JsonArray photos = message["photo"].as<JsonArray>();
                                    String bestFileId = "";
                                    for (JsonObject p : photos) {
                                        int w = p["width"] | 0;
                                        int h = p["height"] | 0;
                                        int fsize = p["file_size"] | 0;
                                        String fid = String(p["file_id"] | "");
                                        if (!fid.isEmpty()) {
                                            if (bestFileId.isEmpty() || (w <= 320 && h <= 320 && fsize <= 32768)) {
                                                bestFileId = fid;
                                            }
                                        }
                                    }
                                    ev.mediaFileId = bestFileId;
                                    ev.mediaType   = MediaType::PHOTO;
                                    String cap     = String(message["caption"] | "");
                                    ev.text        = "[Photo - Enter to View]";
                                    if (!cap.isEmpty()) ev.text += " " + cap;
                                } else if (!message["voice"].isNull()) {
                                    JsonObject voiceObj = message["voice"];
                                    ev.mediaFileId      = String(voiceObj["file_id"] | "");
                                    ev.mediaDurationSec = voiceObj["duration"] | 1;
                                    ev.mediaType        = MediaType::VOICE;
                                    ev.text             = "[Voice " + String(ev.mediaDurationSec) + "s]";
                                } else if (!message["audio"].isNull()) {
                                    JsonObject audioObj = message["audio"];
                                    ev.mediaFileId      = String(audioObj["file_id"] | "");
                                    ev.mediaDurationSec = audioObj["duration"] | 1;
                                    ev.mediaType        = MediaType::AUDIO;
                                    String fName        = sanitizeText(String(audioObj["file_name"] | ""));
                                    if (!fName.isEmpty()) {
                                        ev.text = "[WAV: " + fName + " - Enter]";
                                    } else {
                                        ev.text = "[Audio " + String(ev.mediaDurationSec) + "s - Enter]";
                                    }
                                } else if (!message["document"].isNull()) {
                                    JsonObject docObj = message["document"];
                                    String docFileId  = String(docObj["file_id"] | "");
                                    String docName    = sanitizeText(String(docObj["file_name"] | ""));
                                    String docMime    = String(docObj["mime_type"] | "");
                                    String lowerName  = docName;
                                    lowerName.toLowerCase();
                                    String lowerMime  = docMime;
                                    lowerMime.toLowerCase();
                                    String cap        = sanitizeText(String(message["caption"] | ""));

                                    if (lowerName.endsWith(".wav") || lowerMime.indexOf("wav") >= 0 || lowerMime.startsWith("audio/")) {
                                        ev.mediaFileId      = docFileId;
                                        ev.mediaDurationSec = 1;
                                        ev.mediaType        = MediaType::AUDIO;
                                        ev.text             = docName.isEmpty() ? "[WAV Audio - Enter]" : ("[WAV: " + docName + " - Enter]");
                                        if (!cap.isEmpty()) ev.text += " " + cap;
                                    } else if (lowerName.endsWith(".jpg") || lowerName.endsWith(".jpeg") || lowerMime.indexOf("jpeg") >= 0 || lowerMime.indexOf("jpg") >= 0) {
                                        ev.mediaFileId      = docFileId;
                                        ev.mediaType        = MediaType::PHOTO;
                                        ev.text             = "[Photo - Enter to View]";
                                        if (!cap.isEmpty()) ev.text += " " + cap;
                                    } else {
                                        ev.text = docName.isEmpty() ? "[Document]" : ("[File: " + docName + "]");
                                        if (!cap.isEmpty()) ev.text += " " + cap;
                                    }
                                } else if (!message["video_note"].isNull()) {
                                    JsonObject vnObj    = message["video_note"];
                                    ev.mediaFileId      = String(vnObj["file_id"] | "");
                                    ev.mediaDurationSec = vnObj["duration"] | 1;
                                    ev.mediaType        = MediaType::VIDEO_AUDIO_ONLY;
                                    ev.text             = "[Video Msg " + String(ev.mediaDurationSec) + "s]";
                                } else if (!message["video"].isNull()) {
                                    JsonObject vidObj   = message["video"];
                                    ev.mediaFileId      = String(vidObj["file_id"] | "");
                                    ev.mediaDurationSec = vidObj["duration"] | 1;
                                    ev.mediaType        = MediaType::VIDEO_AUDIO_ONLY;
                                    String cap          = String(message["caption"] | "");
                                    ev.text             = "[Video Msg " + String(ev.mediaDurationSec) + "s]";
                                    if (!cap.isEmpty()) ev.text += " " + cap;
                                } else if (ev.text.isEmpty()) {
                                    if (!message["sticker"].isNull()) ev.text = "[Sticker]";
                                    else                              ev.text = "[Media]";
                                }

                                ev.timestamp = message["date"] | static_cast<uint32_t>(millis() / 1000);

                                xSemaphoreTake(g_queueMutex, portMAX_DELAY);
                                g_incomingQueue.push_back(ev);
                                xSemaphoreGive(g_queueMutex);
                            }
                        }
                        doc.clear();
                    }
                    https.end();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

/**
 * @brief Drains any newly arrived messages from `g_incomingQueue` on Core 1,
 *        writes them to MicroSD, plays the notification chime, and updates the UI
 *        without switching away from the user's currently active chat!
 */
void processIncomingQueueCore1() {
    std::vector<IncomingUpdateEvent> batch;
    xSemaphoreTake(g_queueMutex, portMAX_DELAY);
    if (!g_incomingQueue.empty()) {
        batch.swap(g_incomingQueue);
    }
    xSemaphoreGive(g_queueMutex);

    if (batch.empty()) return;

    int64_t activeChatBefore = getSelectedChatId();

    for (const auto& ev : batch) {
        int idx = findOrCreateChat(ev.chatId, ev.displayName, ev.username);
        bool wasAtBottom = true;
        if (idx >= 0 && idx < static_cast<int>(g_chats.size()) && !g_chats[idx].messages.empty()) {
            wasAtBottom = (g_selectedMsgIndex < 0 ||
                           g_selectedMsgIndex >= static_cast<int>(g_chats[idx].messages.size()) - 1);
        }

        appendMessageToChat(idx, false, ev.text, ev.timestamp,
                            ev.mediaType, ev.mediaFileId, ev.mediaDurationSec, "");
        // Promote incoming chat to top of the list, but NEVER switch the user's active selection!
        promoteChatToTop(idx, false);

        // If the incoming message belongs to the chat we are actively viewing right now
        if (g_currentView == AppView::CONVERSATION_THREAD && ev.chatId == activeChatBefore) {
            if (wasAtBottom && g_selectedChatIndex >= 0 && g_selectedChatIndex < static_cast<int>(g_chats.size())) {
                g_selectedMsgIndex   = static_cast<int>(g_chats[g_selectedChatIndex].messages.size()) - 1;
                g_threadScrollOffset = 0;
            }
        }
    }

    saveContactsIndexToSD();
    playNotificationChime();
    g_uiDirty = true;
}

// ============================================================================
// CORE 1 OUTGOING SEND & MEDIA DOWNLOAD (CALLED ONLY ON EXPLICIT USER ACTION)
// ============================================================================
bool sendTelegramMessage(int64_t chatId, const String& text) {
    if (WiFi.status() != WL_CONNECTED || text.isEmpty()) return false;

    g_pauseBackgroundPoll = true;
    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(5);

    HTTPClient https;
    https.setTimeout(5000);

    bool success = false;
    String url = buildBotApiUrl("sendMessage");
    if (https.begin(client, url)) {
        https.addHeader("Content-Type", "application/json");

        String requestBody;
        {
            DynamicJsonDocument reqDoc(JSON_SEND_DOC_SIZE);
            reqDoc["chat_id"] = chatId;
            reqDoc["text"]    = text;
            serializeJson(reqDoc, requestBody);
            reqDoc.clear();
        }

        int httpCode = https.POST(requestBody);
        if (httpCode == HTTP_CODE_OK) {
            DynamicJsonDocument respDoc(JSON_SEND_DOC_SIZE);
            if (deserializeJson(respDoc, https.getStream()) == DeserializationError::Ok &&
                (respDoc["ok"] | false)) {
                success = true;
            }
            respDoc.clear();
        }
        https.end();
    }

    g_pauseBackgroundPoll = false;
    return success;
}

/**
 * @brief Uploads a 16-bit PCM WAV file from MicroSD to Telegram via `/sendAudio`.
 */
bool sendTelegramWavAudio(int64_t chatId, const char* sdFilePath, uint16_t durationSec, String& outFileId) {
    if (WiFi.status() != WL_CONNECTED || !SD.exists(sdFilePath)) return false;

    File audioFile = SD.open(sdFilePath, FILE_READ);
    if (!audioFile) return false;
    size_t fileSize = audioFile.size();
    if (fileSize <= sizeof(WavHeader)) {
        audioFile.close();
        return false;
    }

    g_pauseBackgroundPoll = true;
    WiFiClientSecure uploadClient;
    uploadClient.setInsecure();
    uploadClient.setTimeout(10);

    if (!uploadClient.connect("api.telegram.org", 443)) {
        audioFile.close();
        g_pauseBackgroundPoll = false;
        return false;
    }

    const String boundary = "----TeleputerAudioBoundary7MA4YWxkTrZu0gW";
    String headPart = "--" + boundary + "\r\n";
    headPart += "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n";
    headPart += String(chatId) + "\r\n";

    headPart += "--" + boundary + "\r\n";
    headPart += "Content-Disposition: form-data; name=\"duration\"\r\n\r\n";
    headPart += String(durationSec) + "\r\n";

    headPart += "--" + boundary + "\r\n";
    headPart += "Content-Disposition: form-data; name=\"title\"\r\n\r\n";
    headPart += "Voice Message\r\n";

    headPart += "--" + boundary + "\r\n";
    headPart += "Content-Disposition: form-data; name=\"performer\"\r\n\r\n";
    headPart += "Teleputer ADV\r\n";

    headPart += "--" + boundary + "\r\n";
    headPart += "Content-Disposition: form-data; name=\"audio\"; filename=\"voice.wav\"\r\n";
    headPart += "Content-Type: audio/wav\r\n\r\n";

    String tailPart = "\r\n--" + boundary + "--\r\n";
    size_t totalContentLen = headPart.length() + fileSize + tailPart.length();

    String reqHeader = "POST /bot" + g_config.botToken + "/sendAudio HTTP/1.1\r\n";
    reqHeader += "Host: api.telegram.org\r\n";
    reqHeader += "User-Agent: Teleputer-ADV/3.1\r\n";
    reqHeader += "Connection: close\r\n";
    reqHeader += "Content-Type: multipart/form-data; boundary=" + boundary + "\r\n";
    reqHeader += "Content-Length: " + String(totalContentLen) + "\r\n\r\n";

    uploadClient.print(reqHeader);
    uploadClient.print(headPart);

    uint8_t chunkBuf[1024];
    while (audioFile.available()) {
        size_t n = audioFile.read(chunkBuf, sizeof(chunkBuf));
        if (n > 0) {
            uploadClient.write(chunkBuf, n);
        }
    }
    audioFile.close();

    uploadClient.print(tailPart);

    unsigned long waitStart = millis();
    while (!uploadClient.available() && (millis() - waitStart < 8000UL)) {
        delay(10);
    }

    while (uploadClient.connected() || uploadClient.available()) {
        String line = uploadClient.readStringUntil('\n');
        if (line == "\r" || line.isEmpty()) break;
    }

    bool ok = false;
    {
        DynamicJsonDocument respDoc(JSON_SEND_DOC_SIZE);
        DeserializationError err = deserializeJson(respDoc, uploadClient);
        if (!err && (respDoc["ok"] | false)) {
            ok = true;
            JsonObject res = respDoc["result"];
            if (!res["audio"].isNull()) {
                outFileId = String(res["audio"]["file_id"] | "");
            } else if (!res["voice"].isNull()) {
                outFileId = String(res["voice"]["file_id"] | "");
            }
        }
        respDoc.clear();
    }
    uploadClient.stop();
    g_pauseBackgroundPoll = false;
    return ok;
}

bool downloadTelegramMediaFile(const String& fileId, const char* destPath) {
    if (WiFi.status() != WL_CONNECTED || fileId.isEmpty()) return false;

    ensureStorageDirectories();
    g_pauseBackgroundPoll = true;

    WiFiClientSecure dlClient;
    dlClient.setInsecure();
    dlClient.setTimeout(6);

    HTTPClient https;
    https.setReuse(true);
    https.setTimeout(6000);

    String getFileUrl = buildBotApiUrl("getFile") + "?file_id=" + fileId;
    if (!https.begin(dlClient, getFileUrl)) {
        g_pauseBackgroundPoll = false;
        return false;
    }

    int code = https.GET();
    String remoteFilePath = "";
    if (code == HTTP_CODE_OK) {
        DynamicJsonDocument doc(JSON_SEND_DOC_SIZE);
        if (deserializeJson(doc, https.getStream()) == DeserializationError::Ok && (doc["ok"] | false)) {
            remoteFilePath = String(doc["result"]["file_path"] | "");
        }
        doc.clear();
    }
    https.end();

    if (remoteFilePath.isEmpty()) {
        g_pauseBackgroundPoll = false;
        return false;
    }

    String fileUrl = "https://api.telegram.org/file/bot" + g_config.botToken + "/" + remoteFilePath;
    String dlUrl   = g_config.transcodeUrl.isEmpty()
                     ? fileUrl
                     : (g_config.transcodeUrl + "?url=" + fileUrl);

    if (!https.begin(dlClient, dlUrl)) {
        g_pauseBackgroundPoll = false;
        return false;
    }

    int dlCode = https.GET();
    if (dlCode != HTTP_CODE_OK) {
        https.end();
        g_pauseBackgroundPoll = false;
        return false;
    }

    if (SD.exists(destPath)) SD.remove(destPath);
    File out = SD.open(destPath, FILE_WRITE);
    if (!out) {
        https.end();
        g_pauseBackgroundPoll = false;
        return false;
    }

    WiFiClient* stream = https.getStreamPtr();
    int remaining = https.getSize();
    uint8_t buf[1024];
    unsigned long lastReadMs = millis();

    while (https.connected() && (remaining > 0 || remaining == -1)) {
        size_t avail = stream->available();
        if (avail > 0) {
            size_t toRead = (avail > sizeof(buf)) ? sizeof(buf) : avail;
            int readBytes = stream->readBytes(buf, toRead);
            if (readBytes > 0) {
                out.write(buf, readBytes);
                if (remaining > 0) remaining -= readBytes;
                lastReadMs = millis();
            }
        } else {
            if (millis() - lastReadMs > 4000UL) break;
            delay(5);
        }
    }

    out.close();
    https.end();
    g_pauseBackgroundPoll = false;
    return true;
}

// ============================================================================
// SEQUENTIAL WAV VOICE RECORDING, .WAV-ONLY PLAYBACK & JPEG PHOTO VIEWER
// ============================================================================
void renderUI(); // Forward declaration

static void accumulateRecordingStats(const int16_t* samples, size_t count, uint32_t sampleStartIndex) {
    const uint32_t skipSamples = (VOICE_SAMPLE_RATE * 80) / 1000; // Skip first 80ms codec power-on pop
    for (size_t i = 0; i < count; ++i) {
        if (sampleStartIndex + i >= skipSamples) {
            int32_t v = samples[i];
            g_recSampleSum += v;
            ++g_recSampleCount;
            if (v > g_recMaxSample) g_recMaxSample = v;
            if (v < g_recMinSample) g_recMinSample = v;
        }
    }
}

bool startVoiceRecording() {
    if (g_isRecordingVoice || g_isSendingMessage || g_isPlayingAudio || g_isViewingPhoto) return false;
    if (g_chats.empty()) return false;

    ensureStorageDirectories();
    int64_t targetChatId = g_chats[g_selectedChatIndex].chatId;
    g_activeRecordPath = generateUniqueAudioFilePath(targetChatId, true, ".wav");

    if (SD.exists(TEMP_REC_RAW_PATH)) {
        SD.remove(TEMP_REC_RAW_PATH);
    }
    g_recordFile = SD.open(TEMP_REC_RAW_PATH, FILE_WRITE);
    if (!g_recordFile) {
        playErrorChime();
        g_netStatusBadge = "SD ERR";
        g_uiDirty = true;
        return false;
    }

    g_pauseBackgroundPoll = true;
    ensureMicActive();

    memset(g_audioDmaBuf, 0, sizeof(g_audioDmaBuf));
    g_isRecordingVoice  = true;
    g_recordStartMs     = millis();
    g_recordedDataBytes = 0;
    g_recWriteStep      = 0;
    g_recQueueStep      = 2;
    g_recSampleSum      = 0;
    g_recSampleCount    = 0;
    g_recMaxSample      = -32768;
    g_recMinSample      =  32767;

    // Prime 2 DMA slots in Mic_Class queue while 3rd slot stands by for lock-free SD writing
    M5Cardputer.Mic.record(g_audioDmaBuf[0], AUDIO_DMA_SAMPLES, VOICE_SAMPLE_RATE);
    M5Cardputer.Mic.record(g_audioDmaBuf[1], AUDIO_DMA_SAMPLES, VOICE_SAMPLE_RATE);

    // Wait briefly for Mic_Class::_rec_task to run M5Unified's ES8311 enable callback, then boost PGA/ADC gain
    delay(15);
    boostCardputerAdvMicGain();

    g_netStatusBadge = "REC";
    g_uiDirty        = true;
    return true;
}

void pumpVoiceRecording() {
    if (!g_isRecordingVoice) return;

    while (M5Cardputer.Mic.isRecording() < 2) {
        if (g_recWriteStep <= 2) {
            boostCardputerAdvMicGain();
        }
        int nextQueueSlot = static_cast<int>(g_recQueueStep % 3);
        int completedSlot = static_cast<int>(g_recWriteStep % 3);

        // Immediately queue the free 3rd slot so DMA recording remains 100% continuous
        M5Cardputer.Mic.record(g_audioDmaBuf[nextQueueSlot], AUDIO_DMA_SAMPLES, VOICE_SAMPLE_RATE);
        ++g_recQueueStep;

        uint32_t sampleStartIdx = g_recordedDataBytes / sizeof(int16_t);
        accumulateRecordingStats(g_audioDmaBuf[completedSlot], AUDIO_DMA_SAMPLES, sampleStartIdx);

        size_t bytesToWrite = AUDIO_DMA_SAMPLES * sizeof(int16_t);
        g_recordFile.write(reinterpret_cast<const uint8_t*>(g_audioDmaBuf[completedSlot]), bytesToWrite);
        g_recordedDataBytes += bytesToWrite;
        ++g_recWriteStep;
    }

    static unsigned long lastRecUiUpdate = 0;
    if (millis() - lastRecUiUpdate >= 250) {
        lastRecUiUpdate = millis();
        g_uiDirty = true;
    }
}

/**
 * @brief Builds a finalized RIFF WAVE (.wav) file sequentially from `TEMP_REC_RAW_PATH`
 *        (writes the complete 44-byte `WavHeader` at byte 0 with true `dataSize`,
 *        removes DC offset, mutes the 80ms codec turn-on pop, and normalizes speech volume).
 */
bool buildFinalWavFromRawTemp(const char* finalWavPath, uint32_t pcmBytes) {
    if (pcmBytes < AUDIO_DMA_SAMPLES * sizeof(int16_t)) return false;

    File rawIn = SD.open(TEMP_REC_RAW_PATH, FILE_READ);
    if (!rawIn) return false;

    if (SD.exists(finalWavPath)) {
        SD.remove(finalWavPath);
    }
    File wavOut = SD.open(finalWavPath, FILE_WRITE);
    if (!wavOut) {
        rawIn.close();
        return false;
    }

    // 1. Write the finalized 44-byte RIFF WAV header sequentially at byte 0
    WavHeader finalHdr;
    finalHdr.dataSize    = pcmBytes;
    finalHdr.overallSize = 36 + pcmBytes;
    wavOut.write(reinterpret_cast<const uint8_t*>(&finalHdr), sizeof(WavHeader));

    // 2. Compute DC offset and auto-gain factor from stats gathered during recording
    int32_t dcOffset = (g_recSampleCount > 0)
                       ? static_cast<int32_t>(g_recSampleSum / static_cast<int64_t>(g_recSampleCount))
                       : 0;
    int32_t peakPos  = g_recMaxSample - dcOffset;
    int32_t peakNeg  = dcOffset - g_recMinSample;
    int32_t maxPeak  = (peakPos > peakNeg) ? peakPos : peakNeg;
    if (maxPeak < 1) maxPeak = 1;

    int32_t gainQ8 = (26000 * 256) / maxPeak;
    if (gainQ8 < 256)  gainQ8 = 256;   // Minimum 1.0x
    if (gainQ8 > 6144) gainQ8 = 6144;  // Maximum 24.0x boost

    const uint32_t skipSamples = (VOICE_SAMPLE_RATE * 80) / 1000;
    uint32_t sampleIdx = 0;
    int16_t* chunk = g_audioDmaBuf[0];

    while (rawIn.available()) {
        size_t nBytes = rawIn.read(reinterpret_cast<uint8_t*>(chunk), AUDIO_DMA_SAMPLES * sizeof(int16_t));
        size_t nSamps = nBytes / sizeof(int16_t);
        if (nSamps == 0) break;

        for (size_t i = 0; i < nSamps; ++i) {
            uint32_t pos = sampleIdx + i;
            if (pos < skipSamples) {
                chunk[i] = 0;
            } else {
                int32_t ac = ((static_cast<int32_t>(chunk[i]) - dcOffset) * gainQ8) >> 8;
                if (pos < skipSamples + 320) {
                    ac = (ac * static_cast<int32_t>(pos - skipSamples)) / 320;
                }
                if (ac > 32767)  ac = 32767;
                if (ac < -32768) ac = -32768;
                chunk[i] = static_cast<int16_t>(ac);
            }
        }

        wavOut.write(reinterpret_cast<const uint8_t*>(chunk), nSamps * sizeof(int16_t));
        sampleIdx += nSamps;
    }

    wavOut.close();
    rawIn.close();
    SD.remove(TEMP_REC_RAW_PATH);
    return true;
}

void stopAndSendVoiceRecording(bool cancelOnly = false) {
    if (!g_isRecordingVoice) return;

    while (M5Cardputer.Mic.isRecording()) {
        delay(1);
    }

    while (g_recWriteStep < g_recQueueStep) {
        int slot = static_cast<int>(g_recWriteStep % 3);
        uint32_t sampleStartIdx = g_recordedDataBytes / sizeof(int16_t);
        accumulateRecordingStats(g_audioDmaBuf[slot], AUDIO_DMA_SAMPLES, sampleStartIdx);

        size_t bytesToWrite = AUDIO_DMA_SAMPLES * sizeof(int16_t);
        g_recordFile.write(reinterpret_cast<const uint8_t*>(g_audioDmaBuf[slot]), bytesToWrite);
        g_recordedDataBytes += bytesToWrite;
        ++g_recWriteStep;
    }

    g_recordFile.close();
    g_isRecordingVoice = false;
    ensureSpeakerActive();

    uint32_t elapsedSec = (millis() - g_recordStartMs + 500) / 1000;
    if (elapsedSec < 1) elapsedSec = 1;

    if (cancelOnly || g_recordedDataBytes < 2048 || g_chats.empty()) {
        if (SD.exists(TEMP_REC_RAW_PATH)) {
            SD.remove(TEMP_REC_RAW_PATH);
        }
        g_activeRecordPath    = "";
        g_pauseBackgroundPoll = false;
        g_netStatusBadge      = "REC CANC";
        g_uiDirty             = true;
        return;
    }

    g_isSendingMessage = true;
    g_netStatusBadge   = "SAVING WAV";
    g_uiDirty          = true;
    renderUI();

    if (!buildFinalWavFromRawTemp(g_activeRecordPath.c_str(), g_recordedDataBytes)) {
        playErrorChime();
        g_activeRecordPath    = "";
        g_isSendingMessage    = false;
        g_pauseBackgroundPoll = false;
        g_netStatusBadge      = "WAV ERR";
        g_uiDirty             = true;
        return;
    }

    int64_t targetChatId = g_chats[g_selectedChatIndex].chatId;
    g_netStatusBadge     = "UPLOADING";
    g_uiDirty            = true;
    renderUI();

    String uploadedFileId = "";
    bool sentOk = sendTelegramWavAudio(targetChatId, g_activeRecordPath.c_str(), elapsedSec, uploadedFileId);
    if (sentOk) {
        playSendChime();
        String label = "[Voice Sent " + String(elapsedSec) + "s - Enter]";
        appendMessageToChat(g_selectedChatIndex, true, label, millis() / 1000,
                            MediaType::VOICE, uploadedFileId, elapsedSec, g_activeRecordPath);
        promoteChatToTop(g_selectedChatIndex, true);
        g_selectedMsgIndex   = static_cast<int>(g_chats[g_selectedChatIndex].messages.size()) - 1;
        g_threadScrollOffset = 0;
        g_netStatusBadge     = "LIVE";
        saveContactsIndexToSD();
    } else {
        playErrorChime();
        g_netStatusBadge = "VOICE ERR";
    }

    g_activeRecordPath    = "";
    g_isSendingMessage    = false;
    g_pauseBackgroundPoll = false;
    g_uiDirty             = true;
}

/**
 * @brief Plays ONLY uncompressed RIFF WAVE (.wav) files from MicroSD.
 *        Blocks any non-WAV (.oga, .mp4, etc.) files cleanly without crashing!
 */
void playWavFileFromSD(const char* sdPath) {
    File f = SD.open(sdPath, FILE_READ);
    if (!f || f.size() < 44) {
        if (f) f.close();
        playErrorChime();
        return;
    }

    uint8_t magic[12] = {0};
    f.read(magic, 12);
    bool isRiffWave = (memcmp(magic, "RIFF", 4) == 0 && memcmp(magic + 8, "WAVE", 4) == 0);
    if (!isRiffWave) {
        f.close();
        g_netStatusBadge  = "WAV ONLY";
        g_audioPlayBanner = "Blocked: Only .wav playable";
        g_uiDirty         = true;
        renderUI();
        playErrorChime();
        delay(1100);
        return;
    }

    uint16_t numChannels   = 1;
    uint32_t sampleRate    = VOICE_SAMPLE_RATE;
    uint16_t bitsPerSample = 16;
    uint32_t dataOffset    = 44;
    uint32_t dataSize      = f.size() - 44;

    while (f.available() >= 8) {
        char chunkId[4];
        uint32_t chunkLen = 0;
        f.readBytes(chunkId, 4);
        f.readBytes(reinterpret_cast<char*>(&chunkLen), 4);

        if (memcmp(chunkId, "fmt ", 4) == 0 && chunkLen >= 16) {
            uint16_t audioFmt   = 1;
            uint32_t byteRate   = 0;
            uint16_t blockAlign = 0;
            f.readBytes(reinterpret_cast<char*>(&audioFmt), 2);
            f.readBytes(reinterpret_cast<char*>(&numChannels), 2);
            f.readBytes(reinterpret_cast<char*>(&sampleRate), 4);
            f.readBytes(reinterpret_cast<char*>(&byteRate), 4);
            f.readBytes(reinterpret_cast<char*>(&blockAlign), 2);
            f.readBytes(reinterpret_cast<char*>(&bitsPerSample), 2);
            if (chunkLen > 16) {
                f.seek(f.position() + (chunkLen - 16));
            }
        } else if (memcmp(chunkId, "data", 4) == 0) {
            dataOffset = f.position();
            dataSize   = chunkLen;
            break;
        } else {
            f.seek(f.position() + chunkLen);
        }
    }

    if (dataSize == 0 && f.size() > dataOffset) {
        dataSize = f.size() - dataOffset;
    }

    f.seek(dataOffset);
    if (sampleRate < 4000 || sampleRate > 48000) {
        sampleRate = VOICE_SAMPLE_RATE;
    }

    g_pauseBackgroundPoll = true;
    ensureSpeakerActive();
    M5Cardputer.Speaker.setVolume(255);

    int bufIdx = 0;
    uint32_t bytesLeft = dataSize;

    while (bytesLeft > 0) {
        size_t maxBytes  = sizeof(g_audioDmaBuf[0]);
        size_t toRead    = (bytesLeft < maxBytes) ? bytesLeft : maxBytes;
        size_t bytesRead = f.read(reinterpret_cast<uint8_t*>(g_audioDmaBuf[bufIdx]), toRead);
        if (bytesRead == 0) break;
        bytesLeft -= bytesRead;

        size_t samplesRead = bytesRead / sizeof(int16_t);
        if (numChannels == 2) samplesRead /= 2;
        if (samplesRead == 0) break;

        while (M5Cardputer.Speaker.isPlaying(0) > 1) {
            delay(1);
        }

        M5Cardputer.Speaker.playRaw(
            g_audioDmaBuf[bufIdx],
            samplesRead,
            sampleRate,
            (numChannels == 2),
            1,
            0
        );
        bufIdx = (bufIdx + 1) % 3;
    }

    while (M5Cardputer.Speaker.isPlaying(0)) {
        delay(2);
    }

    f.close();
    g_pauseBackgroundPoll = false;
    g_netStatusBadge = "LIVE";
}

// ============================================================================
// FULLSCREEN JPEG PHOTO VIEWER (USES BUILT-IN ROM TJPGD IN M5GFX)
// ============================================================================
static bool parseJpegDimensions(const uint8_t* jpgBuf, size_t len, uint16_t& outW, uint16_t& outH) {
    if (len < 12 || jpgBuf[0] != 0xFF || jpgBuf[1] != 0xD8) return false;
    size_t i = 2;
    while (i + 9 < len) {
        if (jpgBuf[i] != 0xFF) {
            ++i;
            continue;
        }
        while (i < len && jpgBuf[i] == 0xFF) ++i;
        if (i >= len) break;
        uint8_t marker = jpgBuf[i++];
        if (marker == 0xD8 || marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7)) {
            continue;
        }
        if (marker == 0xD9 || marker == 0xDA) {
            break;
        }
        if (i + 2 > len) break;
        uint16_t segLen = static_cast<uint16_t>((jpgBuf[i] << 8) | jpgBuf[i + 1]);
        if (segLen < 2 || i + segLen > len) break;

        if (marker == 0xC0 || marker == 0xC1 || marker == 0xC2) {
            if (i + 7 <= len) {
                outH = static_cast<uint16_t>((jpgBuf[i + 3] << 8) | jpgBuf[i + 4]);
                outW = static_cast<uint16_t>((jpgBuf[i + 5] << 8) | jpgBuf[i + 6]);
                return (outW > 0 && outH > 0);
            }
            break;
        }
        i += segLen;
    }
    return false;
}

void renderFullscreenPhotoViewer() {
    canvas.fillSprite(0x0000); // Pure black backdrop for photos

    File f = SD.open(g_activePhotoPath.c_str(), FILE_READ);
    if (!f || f.size() == 0 || f.size() > 65536) {
        if (f) f.close();
        canvas.setFont(&fonts::Font0);
        canvas.setTextColor(COL_STATUS_ERR);
        canvas.drawString("Photo unavailable or too large", 24, 60);
        canvas.pushSprite(0, 0);
        return;
    }

    size_t fileSize = f.size();
    uint8_t* jpgBuf = static_cast<uint8_t*>(malloc(fileSize));
    if (!jpgBuf) {
        f.close();
        canvas.setFont(&fonts::Font0);
        canvas.setTextColor(COL_STATUS_ERR);
        canvas.drawString("Not enough RAM for photo", 40, 60);
        canvas.pushSprite(0, 0);
        return;
    }

    f.read(jpgBuf, fileSize);
    f.close();

    uint16_t imgW = 0, imgH = 0;
    jpeg_div_t div = JPEG_DIV_NONE;
    int scale = 1;

    if (parseJpegDimensions(jpgBuf, fileSize, imgW, imgH)) {
        if (imgW > SCREEN_WIDTH * 4 || imgH > SCREEN_HEIGHT * 4) {
            div   = JPEG_DIV_8;
            scale = 8;
        } else if (imgW > SCREEN_WIDTH * 2 || imgH > SCREEN_HEIGHT * 2) {
            div   = JPEG_DIV_4;
            scale = 4;
        } else if (imgW > SCREEN_WIDTH || imgH > SCREEN_HEIGHT) {
            div   = JPEG_DIV_2;
            scale = 2;
        }
    }

    int drawX = 0;
    int drawY = 0;
    if (imgW > 0 && imgH > 0) {
        int scaledW = imgW / scale;
        int scaledH = imgH / scale;
        if (scaledW < SCREEN_WIDTH)  drawX = (SCREEN_WIDTH  - scaledW) / 2;
        if (scaledH < SCREEN_HEIGHT) drawY = (SCREEN_HEIGHT - scaledH) / 2;
    }

    canvas.drawJpg(jpgBuf, fileSize, drawX, drawY, SCREEN_WIDTH, SCREEN_HEIGHT, 0, 0, div);
    free(jpgBuf);

    // Subtle bottom hint bar
    canvas.fillRect(0, SCREEN_HEIGHT - 13, SCREEN_WIDTH, 13, COL_BG_HEADER);
    canvas.drawFastHLine(0, SCREEN_HEIGHT - 13, SCREEN_WIDTH, COL_BORDER);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(1);
    canvas.setTextDatum(top_left);
    canvas.setTextColor(COL_ACCENT_CYAN);
    canvas.drawString(ellipsizeText(canvas, g_activePhotoTitle, 145), 4, SCREEN_HEIGHT - 10);
    canvas.setTextColor(COL_TEXT_SECONDARY);
    canvas.drawString("[Any Key: Close]", SCREEN_WIDTH - 96, SCREEN_HEIGHT - 10);

    canvas.pushSprite(0, 0);
}

/**
 * @brief Triggered when pressing ENTER in View B with an empty input bar:
 *        - Plays `.wav` audio messages (and blocks non-.wav received audio).
 *        - Downloads and opens `MediaType::PHOTO` messages fullscreen!
 */
void activateSelectedMessage() {
    if (g_chats.empty() || g_isPlayingAudio || g_isRecordingVoice || g_isSendingMessage) return;

    ContactChat& chat = g_chats[g_selectedChatIndex];
    if (chat.messages.empty()) return;

    if (g_selectedMsgIndex < 0 || g_selectedMsgIndex >= static_cast<int>(chat.messages.size())) {
        g_selectedMsgIndex = static_cast<int>(chat.messages.size()) - 1;
    }

    ChatMessage& targetMsg = chat.messages[g_selectedMsgIndex];
    if (targetMsg.mediaType == MediaType::NONE) {
        return;
    }

    // Case 1: Photo Message -> Download JPEG thumbnail (if not cached on SD) and open Fullscreen Viewer
    if (targetMsg.mediaType == MediaType::PHOTO) {
        if (targetMsg.localAudioPath.isEmpty() || !SD.exists(targetMsg.localAudioPath.c_str())) {
            if (targetMsg.mediaFileId.isEmpty()) {
                playErrorChime();
                return;
            }
            targetMsg.localAudioPath = generateUniquePhotoFilePath(chat.chatId);
            g_isPlayingAudio  = true;
            g_audioPlayBanner = "Downloading Photo...";
            g_uiDirty         = true;
            renderUI();

            bool dlOk = downloadTelegramMediaFile(targetMsg.mediaFileId, targetMsg.localAudioPath.c_str());
            g_isPlayingAudio  = false;
            g_audioPlayBanner = "";

            if (!dlOk) {
                playErrorChime();
                g_netStatusBadge = "DL FAIL";
                g_uiDirty = true;
                return;
            }
            rewriteFullChatHistoryToSD(chat);
        }

        g_activePhotoPath  = targetMsg.localAudioPath;
        g_activePhotoTitle = chat.name + ": Photo";
        g_isViewingPhoto   = true;
        g_uiDirty          = true;
        renderUI();
        return;
    }

    // Case 2: Voice / Audio / Video Message -> Block anything received other than .wav!
    if (!targetMsg.isOutgoing && targetMsg.mediaType != MediaType::AUDIO) {
        // Incoming Telegram voice (.oga) and video notes (.mp4) are blocked on Cardputer
        g_isPlayingAudio  = true;
        g_netStatusBadge  = "WAV ONLY";
        g_audioPlayBanner = "Blocked: Only .wav playable";
        g_uiDirty         = true;
        renderUI();
        playErrorChime();
        delay(1100);
        g_isPlayingAudio  = false;
        g_audioPlayBanner = "";
        g_uiDirty         = true;
        return;
    }

    g_isPlayingAudio  = true;
    g_audioPlayBanner = "Playing .wav Audio...";
    g_uiDirty         = true;
    renderUI();

    if (!targetMsg.localAudioPath.isEmpty() && SD.exists(targetMsg.localAudioPath.c_str())) {
        playWavFileFromSD(targetMsg.localAudioPath.c_str());
    } else if (!targetMsg.mediaFileId.isEmpty()) {
        if (targetMsg.localAudioPath.isEmpty()) {
            targetMsg.localAudioPath = generateUniqueAudioFilePath(chat.chatId, targetMsg.isOutgoing, ".wav");
        }
        g_audioPlayBanner = "Downloading Audio...";
        g_uiDirty         = true;
        renderUI();

        if (downloadTelegramMediaFile(targetMsg.mediaFileId, targetMsg.localAudioPath.c_str())) {
            rewriteFullChatHistoryToSD(chat);
            g_audioPlayBanner = "Playing .wav Audio...";
            g_uiDirty         = true;
            renderUI();
            playWavFileFromSD(targetMsg.localAudioPath.c_str());
        } else {
            playErrorChime();
            g_netStatusBadge = "DL FAIL";
        }
    } else {
        playErrorChime();
        g_netStatusBadge = "NO FILE";
    }

    g_isPlayingAudio  = false;
    g_audioPlayBanner = "";
    g_uiDirty         = true;
}

// ============================================================================
// UI RENDERING ENGINE (240x135 DARK THEME)
// ============================================================================
static void buildWrappedChatLines(const ContactChat& chat, std::vector<WrappedMessageLine>& outLines) {
    outLines.clear();
    const int maxBubbleTextWidth = SCREEN_WIDTH - 16;

    for (size_t mIdx = 0; mIdx < chat.messages.size(); ++mIdx) {
        const ChatMessage& msg = chat.messages[mIdx];
        std::vector<String> msgLines;
        wrapTextToLines(canvas, msg.text, maxBubbleTextWidth, msgLines);
        for (size_t i = 0; i < msgLines.size(); ++i) {
            WrappedMessageLine wl;
            wl.msgIndex         = static_cast<int>(mIdx);
            wl.isOutgoing       = msg.isOutgoing;
            wl.isFirstLineOfMsg = (i == 0);
            wl.isLastLineOfMsg  = (i + 1 == msgLines.size());
            wl.lineText         = msgLines[i];
            outLines.push_back(wl);
        }
    }
}

static void scrollViewportToSelectedMessage(const ContactChat& chat) {
    if (chat.messages.empty()) {
        g_selectedMsgIndex   = -1;
        g_threadScrollOffset = 0;
        return;
    }

    if (g_selectedMsgIndex < 0) {
        g_selectedMsgIndex = 0;
    }
    if (g_selectedMsgIndex >= static_cast<int>(chat.messages.size())) {
        g_selectedMsgIndex = static_cast<int>(chat.messages.size()) - 1;
    }

    std::vector<WrappedMessageLine> allLines;
    buildWrappedChatLines(chat, allLines);

    const int lineHeight       = 12;
    const int visibleLineCount = VIEW_B_BODY_H / lineHeight; // 8 lines visible
    const int totalLines       = static_cast<int>(allLines.size());
    const int maxScroll        = (totalLines > visibleLineCount) ? (totalLines - visibleLineCount) : 0;

    int msgFirstLine = -1;
    int msgLastLine  = -1;
    for (int i = 0; i < totalLines; ++i) {
        if (allLines[i].msgIndex == g_selectedMsgIndex) {
            if (msgFirstLine == -1) msgFirstLine = i;
            msgLastLine = i;
        }
    }

    if (msgFirstLine == -1) return;

    int startLine = totalLines - visibleLineCount - g_threadScrollOffset;
    if (startLine < 0) startLine = 0;
    int endLine = startLine + visibleLineCount - 1;

    if (msgFirstLine < startLine) {
        startLine = msgFirstLine;
        g_threadScrollOffset = totalLines - visibleLineCount - startLine;
    } else if (msgLastLine > endLine) {
        startLine = msgLastLine - visibleLineCount + 1;
        if (startLine < 0) startLine = 0;
        g_threadScrollOffset = totalLines - visibleLineCount - startLine;
    }

    if (g_threadScrollOffset > maxScroll) g_threadScrollOffset = maxScroll;
    if (g_threadScrollOffset < 0)         g_threadScrollOffset = 0;
}

void renderViewAChatList() {
    canvas.fillSprite(COL_BG_DARK);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(1);
    canvas.setTextDatum(top_left);

    // 1. Top Header Bar (18px high)
    canvas.fillRect(0, 0, SCREEN_WIDTH, HEADER_HEIGHT, COL_BG_HEADER);
    canvas.drawFastHLine(0, HEADER_HEIGHT - 1, SCREEN_WIDTH, COL_BORDER);

    canvas.setTextColor(COL_ACCENT_CYAN);
    canvas.drawString("TELEPUTER", 6, 5);

    canvas.setTextColor(COL_TEXT_SECONDARY);
    String countStr = "[" + String(g_chats.size()) + "]";
    canvas.drawString(countStr, 66, 5);

    bool wifiOk = (WiFi.status() == WL_CONNECTED);
    uint16_t dotCol = wifiOk ? COL_STATUS_OK : COL_STATUS_ERR;
    canvas.fillCircle(SCREEN_WIDTH - 54, 8, 3, dotCol);
    canvas.setTextColor(COL_TEXT_SECONDARY);
    canvas.drawString(g_netStatusBadge, SCREEN_WIDTH - 47, 5);

    // 2. Left 40% Panel (96px wide): Scrollable Chat List (up to 5 visible rows)
    canvas.fillRect(0, VIEW_A_BODY_Y, VIEW_A_LEFT_WIDTH, VIEW_A_BODY_HEIGHT, COL_BG_PANEL_LEFT);

    if (g_chats.empty()) {
        canvas.setTextColor(COL_TEXT_SECONDARY);
        canvas.drawString("No chats yet", 10, VIEW_A_BODY_Y + 40);
        canvas.drawString("Msg your bot!", 9, VIEW_A_BODY_Y + 54);
    } else {
        if (g_selectedChatIndex < 0) g_selectedChatIndex = 0;
        if (g_selectedChatIndex >= static_cast<int>(g_chats.size())) {
            g_selectedChatIndex = static_cast<int>(g_chats.size()) - 1;
        }
        if (g_selectedChatIndex < g_listScrollOffset) {
            g_listScrollOffset = g_selectedChatIndex;
        }
        if (g_selectedChatIndex >= g_listScrollOffset + VIEW_A_MAX_VISIBLE) {
            g_listScrollOffset = g_selectedChatIndex - VIEW_A_MAX_VISIBLE + 1;
        }

        for (int vis = 0; vis < VIEW_A_MAX_VISIBLE; ++vis) {
            int chatIdx = g_listScrollOffset + vis;
            if (chatIdx >= static_cast<int>(g_chats.size())) break;

            const ContactChat& chat = g_chats[chatIdx];
            bool isSelected = (chatIdx == g_selectedChatIndex);
            int rowY = VIEW_A_BODY_Y + (vis * VIEW_A_ROW_HEIGHT);

            uint16_t rowBg = isSelected
                             ? COL_BG_ROW_SEL
                             : ((vis % 2 == 0) ? COL_BG_PANEL_LEFT : COL_BG_ROW_ALT);
            canvas.fillRect(0, rowY, VIEW_A_LEFT_WIDTH - 1, VIEW_A_ROW_HEIGHT, rowBg);

            if (isSelected) {
                canvas.fillRect(0, rowY, 3, VIEW_A_ROW_HEIGHT, COL_ACCENT_CYAN);
                canvas.drawRect(0, rowY, VIEW_A_LEFT_WIDTH - 1, VIEW_A_ROW_HEIGHT, COL_ACCENT_CYAN);
            } else {
                canvas.drawFastHLine(0, rowY + VIEW_A_ROW_HEIGHT - 1, VIEW_A_LEFT_WIDTH - 1, COL_BORDER);
            }

            int nameMaxW = VIEW_A_LEFT_WIDTH - 12;
            if (chat.hasUnread) {
                canvas.fillCircle(VIEW_A_LEFT_WIDTH - 9, rowY + (VIEW_A_ROW_HEIGHT / 2), 3, COL_UNREAD_DOT);
                nameMaxW -= 10;
            }

            canvas.setTextColor(isSelected ? COL_TEXT_PRIMARY : COL_TEXT_SECONDARY);
            String shortName = ellipsizeText(canvas, chat.name, nameMaxW);
            canvas.drawString(shortName, 6, rowY + 8);
        }
    }

    canvas.drawFastVLine(VIEW_A_LEFT_WIDTH, VIEW_A_BODY_Y, VIEW_A_BODY_HEIGHT, COL_BORDER);

    // 3. Right 60% Panel (144px wide): Preview Pane
    int rightX = VIEW_A_LEFT_WIDTH + 1;
    int rightW = VIEW_A_RIGHT_WIDTH - 1;
    canvas.fillRect(rightX, VIEW_A_BODY_Y, rightW, VIEW_A_BODY_HEIGHT, COL_BG_PANEL_RIGHT);

    if (g_chats.empty()) {
        canvas.setTextColor(COL_ACCENT_CYAN);
        canvas.drawString("Waiting for Users", rightX + 8, VIEW_A_BODY_Y + 12);
        canvas.drawFastHLine(rightX + 8, VIEW_A_BODY_Y + 24, rightW - 16, COL_BORDER);

        canvas.setTextColor(COL_TEXT_SECONDARY);
        canvas.drawString("1. Open Telegram", rightX + 8, VIEW_A_BODY_Y + 34);
        canvas.drawString("2. Send text/photo", rightX + 8, VIEW_A_BODY_Y + 46);
        canvas.drawString("   to your Bot", rightX + 8, VIEW_A_BODY_Y + 58);
        canvas.drawString("3. All chats & media", rightX + 8, VIEW_A_BODY_Y + 70);
        canvas.drawString("   saved on SD!", rightX + 8, VIEW_A_BODY_Y + 82);
    } else {
        const ContactChat& sel = g_chats[g_selectedChatIndex];

        canvas.setTextColor(COL_ACCENT_CYAN);
        String headerName = ellipsizeText(canvas, sel.name, rightW - 12);
        canvas.drawString(headerName, rightX + 6, VIEW_A_BODY_Y + 6);

        canvas.setTextColor(COL_TEXT_SECONDARY);
        String handleStr = sel.username.isEmpty()
                           ? ("ID:" + String(sel.chatId))
                           : ("@" + sel.username);
        canvas.drawString(ellipsizeText(canvas, handleStr, rightW - 12), rightX + 6, VIEW_A_BODY_Y + 17);

        canvas.drawFastHLine(rightX + 6, VIEW_A_BODY_Y + 28, rightW - 12, COL_BORDER);

        canvas.setTextColor(COL_TEXT_SECONDARY);
        String msgCountHdr = "LAST MSG (" + String(sel.messages.size()) + " saved):";
        canvas.drawString(msgCountHdr, rightX + 6, VIEW_A_BODY_Y + 33);

        if (sel.messages.empty()) {
            canvas.setTextColor(COL_TEXT_SECONDARY);
            canvas.drawString("(No messages)", rightX + 6, VIEW_A_BODY_Y + 48);
        } else {
            const ChatMessage& lastMsg = sel.messages.back();
            String prefix = lastMsg.isOutgoing ? "You: " : "";
            String fullSnippet = prefix + lastMsg.text;

            std::vector<String> snippetLines;
            wrapTextToLines(canvas, fullSnippet, rightW - 12, snippetLines);

            canvas.setTextColor(lastMsg.isOutgoing ? COL_TEXT_OUTGOING : COL_TEXT_INCOMING);

            int maxPreviewLines = 4;
            for (int i = 0; i < static_cast<int>(snippetLines.size()) && i < maxPreviewLines; ++i) {
                String line = snippetLines[i];
                if (i == maxPreviewLines - 1 && static_cast<int>(snippetLines.size()) > maxPreviewLines) {
                    line = ellipsizeText(canvas, line + "..", rightW - 12);
                }
                canvas.drawString(line, rightX + 6, VIEW_A_BODY_Y + 46 + (i * 11));
            }
        }

        canvas.drawFastHLine(rightX + 6, SCREEN_HEIGHT - 16, rightW - 12, COL_BORDER);
        canvas.setTextColor(COL_TEXT_SECONDARY);
        canvas.drawString("[TAB/ENT] Open  [;/.] Nav", rightX + 5, SCREEN_HEIGHT - 11);
    }

    canvas.pushSprite(0, 0);
}

/**
 * @brief Renders View B: Conversation Chat Thread
 *        - Uniform dark background (no zebra/alternating backgrounds).
 *        - Highlighted message selection (`g_selectedMsgIndex`) moved via Up/Down arrows (`Fn+;` / `Fn+.`).
 *        - Messages separated cleanly by a thin dark horizontal line (`COL_MSG_SEPARATOR`).
 *        - Received messages are grey (left-aligned); sent messages are blue (right-aligned).
 */
void renderViewBConversation() {
    canvas.fillSprite(COL_BG_DARK);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(1);
    canvas.setTextDatum(top_left);

    // 1. Header Bar (18px high)
    canvas.fillRect(0, 0, SCREEN_WIDTH, VIEW_B_HEADER_H, COL_BG_HEADER);
    canvas.drawFastHLine(0, VIEW_B_HEADER_H - 1, SCREEN_WIDTH, COL_BORDER);

    if (g_chats.empty() || g_selectedChatIndex < 0 || g_selectedChatIndex >= static_cast<int>(g_chats.size())) {
        canvas.setTextColor(COL_TEXT_PRIMARY);
        canvas.drawString("< [TAB] No Active Contact", 6, 5);
    } else {
        const ContactChat& chat = g_chats[g_selectedChatIndex];
        canvas.setTextColor(COL_ACCENT_CYAN);
        canvas.drawString("<", 5, 5);
        canvas.setTextColor(COL_TEXT_PRIMARY);
        String title = ellipsizeText(canvas, chat.name, 118);
        canvas.drawString(title, 15, 5);

        canvas.setTextColor(COL_TEXT_SECONDARY);
        canvas.drawString("Fn+R:Mic Ent:Open", SCREEN_WIDTH - 105, 5);
    }

    // 2. Message History Body (Y = 18 to 119, Height = 102px)
    if (!g_chats.empty() && g_selectedChatIndex >= 0 && g_selectedChatIndex < static_cast<int>(g_chats.size())) {
        const ContactChat& chat = g_chats[g_selectedChatIndex];
        if (!chat.messages.empty()) {
            if (g_selectedMsgIndex < 0 || g_selectedMsgIndex >= static_cast<int>(chat.messages.size())) {
                g_selectedMsgIndex = static_cast<int>(chat.messages.size()) - 1;
            }
        } else {
            g_selectedMsgIndex = -1;
        }

        std::vector<WrappedMessageLine> allLines;
        buildWrappedChatLines(chat, allLines);

        const int lineHeight = 12;
        const int visibleLineCount = VIEW_B_BODY_H / lineHeight; // 8 lines visible at once

        int totalLines = static_cast<int>(allLines.size());
        int maxScroll = (totalLines > visibleLineCount) ? (totalLines - visibleLineCount) : 0;
        if (g_threadScrollOffset > maxScroll) g_threadScrollOffset = maxScroll;
        if (g_threadScrollOffset < 0)         g_threadScrollOffset = 0;

        int startLine = totalLines - visibleLineCount - g_threadScrollOffset;
        if (startLine < 0) startLine = 0;

        if (totalLines == 0) {
            canvas.setTextColor(COL_TEXT_SECONDARY);
            canvas.drawString("Type text or press Fn+R for Voice!", 18, VIEW_B_BODY_Y + 44);
        } else {
            for (int row = 0; row < visibleLineCount; ++row) {
                int lineIdx = startLine + row;
                if (lineIdx >= totalLines) break;

                const WrappedMessageLine& item = allLines[lineIdx];
                int rowY = VIEW_B_BODY_Y + 2 + (row * lineHeight);
                bool isMsgSelected = (item.msgIndex == g_selectedMsgIndex);

                if (isMsgSelected) {
                    canvas.fillRect(2, rowY, SCREEN_WIDTH - 4, lineHeight - 1, COL_BG_MSG_SEL);
                    if (item.isOutgoing) {
                        canvas.fillRect(SCREEN_WIDTH - 4, rowY, 2, lineHeight - 1, COL_ACCENT_CYAN);
                    } else {
                        canvas.fillRect(2, rowY, 2, lineHeight - 1, COL_ACCENT_CYAN);
                    }
                }

                if (item.isOutgoing) {
                    // Sent messages: right-aligned, blue text
                    canvas.setTextColor(COL_TEXT_OUTGOING);
                    int textW = canvas.textWidth(item.lineText);
                    int drawX = SCREEN_WIDTH - 7 - textW;
                    if (drawX < 6) drawX = 6;
                    canvas.drawString(item.lineText, drawX, rowY + 1);
                } else {
                    // Received messages: left-aligned, grey text
                    canvas.setTextColor(COL_TEXT_INCOMING);
                    canvas.drawString(item.lineText, 7, rowY + 1);
                }

                // Thin dark separator line at the bottom of each message
                if (item.isLastLineOfMsg) {
                    canvas.drawFastHLine(4, rowY + lineHeight - 1, SCREEN_WIDTH - 8, COL_MSG_SEPARATOR);
                }
            }
        }
    }

    // 3. Dedicated 15px High Bottom Input Bar (Y = 120 to 135)
    uint16_t inputBg = COL_BG_INPUT;
    if (g_isRecordingVoice)      inputBg = COL_BG_INPUT_REC;
    else if (g_isSendingMessage) inputBg = COL_BG_INPUT_BUSY;
    else if (g_isPlayingAudio)   inputBg = COL_BG_ROW_SEL;

    canvas.fillRect(0, VIEW_B_INPUT_Y, SCREEN_WIDTH, VIEW_B_INPUT_H, inputBg);
    canvas.drawRect(0, VIEW_B_INPUT_Y, SCREEN_WIDTH, VIEW_B_INPUT_H, COL_BORDER);

    if (g_isRecordingVoice) {
        uint32_t recSec = (millis() - g_recordStartMs) / 1000;
        canvas.fillCircle(8, VIEW_B_INPUT_Y + 7, 3, COL_UNREAD_DOT);
        canvas.setTextColor(COL_TEXT_PRIMARY);
        String recStr = "REC " + String(recSec) + "s [ENT:Send / DEL:Cancel]";
        canvas.drawString(recStr, 15, VIEW_B_INPUT_Y + 4);
    } else if (g_isPlayingAudio) {
        canvas.setTextColor(COL_TEXT_PRIMARY);
        canvas.drawString(g_audioPlayBanner, 6, VIEW_B_INPUT_Y + 4);
    } else if (g_isSendingMessage) {
        canvas.setTextColor(COL_ACCENT_CYAN);
        canvas.drawString("Sending to Telegram...", 6, VIEW_B_INPUT_Y + 4);
    } else {
        canvas.setTextColor(COL_ACCENT_CYAN);
        canvas.drawString(">", 4, VIEW_B_INPUT_Y + 4);

        String displayInput = g_inputBuffer + "_";
        const int maxInputPx = SCREEN_WIDTH - 18;
        while (canvas.textWidth(displayInput) > maxInputPx && displayInput.length() > 1) {
            displayInput.remove(0, 1);
        }

        canvas.setTextColor(COL_TEXT_PRIMARY);
        canvas.drawString(displayInput, 13, VIEW_B_INPUT_Y + 4);
    }

    canvas.pushSprite(0, 0);
}

void renderUI() {
    if (!g_uiDirty || g_screenState == ScreenPowerState::OFF) return;
    g_uiDirty = false;
    if (g_isViewingPhoto) {
        renderFullscreenPhotoViewer();
        return;
    }
    if (g_currentView == AppView::CHAT_LIST_HUB) {
        renderViewAChatList();
    } else {
        renderViewBConversation();
    }
}

// ============================================================================
// KEYBOARD INPUT HANDLER (DEBOUNCED, NON-BLOCKING)
// ============================================================================
void handlePhysicalKeyboard() {
    if (!M5Cardputer.Keyboard.isChange()) return;
    if (!M5Cardputer.Keyboard.isPressed()) return;

    // Hardware debounce guard to filter matrix switch bounce
    unsigned long now = millis();
    if (now - g_lastKeyPressMs < 30UL) return;
    g_lastKeyPressMs = now;

    bool wasScreenOff = (g_screenState == ScreenPowerState::OFF);
    registerUserActivity();
    if (wasScreenOff) {
        return;
    }

    Keyboard_Class::KeysState status = M5Cardputer.Keyboard.keysState();

    // If viewing a fullscreen photo, pressing any key closes the photo viewer
    if (g_isViewingPhoto) {
        g_isViewingPhoto = false;
        g_uiDirty        = true;
        return;
    }

    // If currently recording a voice message, handle Del (cancel), Enter (send), or Fn+R (send)
    // with a 600ms guard so releasing Fn+R after starting recording never stops it early!
    if (g_isRecordingVoice) {
        if (status.del) {
            stopAndSendVoiceRecording(true);
            return;
        }
        if (now - g_recordStartMs < 600UL) {
            return;
        }
        if (status.enter) {
            stopAndSendVoiceRecording(false);
            return;
        }
        if (status.fn) {
            for (char c : status.word) {
                if (c == 'r' || c == 'R') {
                    stopAndSendVoiceRecording(false);
                    return;
                }
            }
        }
        return;
    }

    // 1. Toggle between View A and View B using the 'Tab' key
    if (status.tab) {
        if (g_currentView == AppView::CHAT_LIST_HUB) {
            if (!g_chats.empty()) {
                g_currentView = AppView::CONVERSATION_THREAD;
                ContactChat& activeChat = g_chats[g_selectedChatIndex];
                if (activeChat.hasUnread) {
                    activeChat.hasUnread = false;
                    saveContactsIndexToSD();
                }
                g_selectedMsgIndex   = static_cast<int>(activeChat.messages.size()) - 1;
                g_threadScrollOffset = 0;
            }
        } else {
            g_currentView = AppView::CHAT_LIST_HUB;
        }
        g_uiDirty = true;
        return;
    }

    // 2. View A: Chat Selection List Navigation
    if (g_currentView == AppView::CHAT_LIST_HUB) {
        for (char c : status.word) {
            if (c == ';' || c == 'w' || c == 'W') {
                if (g_selectedChatIndex > 0) {
                    --g_selectedChatIndex;
                    g_uiDirty = true;
                }
            } else if (c == '.' || c == 's' || c == 'S') {
                if (g_selectedChatIndex + 1 < static_cast<int>(g_chats.size())) {
                    ++g_selectedChatIndex;
                    g_uiDirty = true;
                }
            }
        }

        if (status.enter && !g_chats.empty()) {
            g_currentView = AppView::CONVERSATION_THREAD;
            ContactChat& activeChat = g_chats[g_selectedChatIndex];
            if (activeChat.hasUnread) {
                activeChat.hasUnread = false;
                saveContactsIndexToSD();
            }
            g_selectedMsgIndex   = static_cast<int>(activeChat.messages.size()) - 1;
            g_threadScrollOffset = 0;
            g_uiDirty = true;
        }
        return;
    }

    // 3. View B: Conversation Thread Input, Arrow Message Selection & Voice Recording
    if (g_currentView == AppView::CONVERSATION_THREAD) {
        if (g_isSendingMessage || g_isPlayingAudio) return;

        if (status.fn) {
            for (char c : status.word) {
                if (c == ';') {
                    // Up Arrow (Fn + ;): Select previous message and auto-scroll viewport
                    if (!g_chats.empty()) {
                        ContactChat& chat = g_chats[g_selectedChatIndex];
                        if (!chat.messages.empty()) {
                            if (g_selectedMsgIndex < 0) {
                                g_selectedMsgIndex = static_cast<int>(chat.messages.size()) - 1;
                            } else if (g_selectedMsgIndex > 0) {
                                --g_selectedMsgIndex;
                            }
                            scrollViewportToSelectedMessage(chat);
                            g_uiDirty = true;
                        }
                    }
                } else if (c == '.') {
                    // Down Arrow (Fn + .): Select next message and auto-scroll viewport
                    if (!g_chats.empty()) {
                        ContactChat& chat = g_chats[g_selectedChatIndex];
                        if (!chat.messages.empty()) {
                            if (g_selectedMsgIndex + 1 < static_cast<int>(chat.messages.size())) {
                                ++g_selectedMsgIndex;
                            } else {
                                g_selectedMsgIndex = static_cast<int>(chat.messages.size()) - 1;
                            }
                            scrollViewportToSelectedMessage(chat);
                            g_uiDirty = true;
                        }
                    }
                } else if (c == 'r' || c == 'R') {
                    startVoiceRecording();
                }
            }
            return;
        }

        // Backspace deletes last character from input buffer
        if (status.del) {
            if (!g_inputBuffer.isEmpty()) {
                g_inputBuffer.remove(g_inputBuffer.length() - 1);
                g_uiDirty = true;
            }
        }

        // Append printable characters to input buffer in real time
        for (char c : status.word) {
            if (static_cast<uint8_t>(c) >= 32 && static_cast<uint8_t>(c) <= 126) {
                if (g_inputBuffer.length() < 240) {
                    g_inputBuffer += c;
                    g_uiDirty = true;
                }
            }
        }

        // Pressing 'Enter':
        // - If input buffer has text: freeze input, send text message, clear input buffer.
        // - If input buffer is EMPTY: play selected .wav audio or view selected Photo!
        if (status.enter && !g_chats.empty()) {
            if (g_inputBuffer.isEmpty()) {
                activateSelectedMessage();
            } else {
                String payload = g_inputBuffer;
                int64_t targetChatId = g_chats[g_selectedChatIndex].chatId;

                g_isSendingMessage = true;
                g_uiDirty = true;
                renderUI();

                bool sentOk = sendTelegramMessage(targetChatId, payload);
                if (sentOk) {
                    playSendChime();
                    appendMessageToChat(g_selectedChatIndex, true, payload, millis() / 1000);
                    promoteChatToTop(g_selectedChatIndex, true);
                    g_selectedMsgIndex   = static_cast<int>(g_chats[g_selectedChatIndex].messages.size()) - 1;
                    g_threadScrollOffset = 0;
                    g_inputBuffer        = "";
                    saveContactsIndexToSD();
                } else {
                    playErrorChime();
                    g_netStatusBadge = "SEND FAIL";
                    g_inputBuffer    = "";
                }

                g_isSendingMessage = false;
                g_uiDirty = true;
            }
        }
    }
}

// ============================================================================
// BOOT SPLASH & WIFI INITIALIZATION
// ============================================================================
void renderBootStatus(const String& stepText, const String& subText) {
    canvas.fillSprite(COL_BG_DARK);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(1);
    canvas.setTextDatum(top_left);

    canvas.fillRect(0, 0, SCREEN_WIDTH, 22, COL_BG_HEADER);
    canvas.drawFastHLine(0, 21, SCREEN_WIDTH, COL_ACCENT_CYAN);
    canvas.setTextColor(COL_ACCENT_CYAN);
    canvas.drawString("TELEPUTER ADV // BOT CLIENT", 8, 7);

    canvas.drawRect(8, 32, SCREEN_WIDTH - 16, SCREEN_HEIGHT - 42, COL_BORDER);
    canvas.fillRect(9, 33, SCREEN_WIDTH - 18, SCREEN_HEIGHT - 44, COL_BG_PANEL_LEFT);

    canvas.setTextColor(COL_TEXT_PRIMARY);
    canvas.drawString(stepText, 18, 52);

    canvas.setTextColor(COL_TEXT_SECONDARY);
    canvas.drawString(subText, 18, 72);

    canvas.pushSprite(0, 0);
}

void connectToWiFi() {
    renderBootStatus("Connecting to Wi-Fi...", "SSID: " + g_config.wifiSsid);

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); // Prevent RF wake-spike glitches on keyboard matrix pins
    WiFi.begin(g_config.wifiSsid.c_str(), g_config.wifiPass.c_str());

    unsigned long startAttempt = millis();
    int dots = 0;
    while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt) < 15000UL) {
        delay(250);
        String progress = "SSID: " + g_config.wifiSsid + " ";
        for (int i = 0; i < (dots % 4); ++i) progress += ".";
        renderBootStatus("Connecting to Wi-Fi...", progress);
        ++dots;
    }

    if (WiFi.status() == WL_CONNECTED) {
        g_netStatusBadge = "LIVE";
        renderBootStatus("Wi-Fi Connected!", "IP: " + WiFi.localIP().toString());
        delay(350);
    } else {
        g_netStatusBadge = "OFFLINE";
        playErrorChime();
        renderBootStatus("Wi-Fi Timeout (Retrying in BG)", "Check SSID/Password in SD config");
        delay(1200);
    }
}

// ============================================================================
// ARDUINO SETUP & NON-BLOCKING MAIN LOOP (CORE 1)
// ============================================================================
void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);

    configureCardputerMicrophone();

    M5Cardputer.Display.setRotation(1);
    M5Cardputer.Display.setBrightness(BRIGHTNESS_AWAKE);
    M5Cardputer.Speaker.setVolume(255);

    g_queueMutex = xSemaphoreCreateMutex();

    canvas.setColorDepth(16);
    canvas.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT);

    renderBootStatus("Initializing MicroSD...", "Mounting FAT32 on GPIO 12");

    if (!initSDAndLoadConfig(g_config)) {
        while (true) {
            M5Cardputer.update();
            delay(100);
        }
    }

    renderBootStatus("Loading Saved History...", "Reading /tg_chats & /tg_audio");
    loadContactsAndHistoriesFromSD();

    connectToWiFi();

    g_lastUserActivityMs = millis();
    g_uiDirty            = true;
    renderUI();

    // Launch lightweight background polling task on Core 0 (never touches SPI/SD!)
    xTaskCreatePinnedToCore(
        backgroundPollTaskCore0,
        "TeleputerPollTask",
        8192,
        nullptr,
        1,
        &g_pollTaskHandle,
        0 // Core 0
    );
}

void loop() {
    // 1. Poll physical keyboard matrix on Core 1
    M5Cardputer.update();
    handlePhysicalKeyboard();

    // 2. Drain any newly arrived messages from Core 0 background poller
    if (!g_isRecordingVoice && !g_isPlayingAudio) {
        processIncomingQueueCore1();
    }

    // 3. If recording voice, pump 512-sample ping-pong DMA buffers to SD card
    if (g_isRecordingVoice) {
        pumpVoiceRecording();
        if (millis() - g_recordStartMs >= (MAX_RECORD_SECONDS * 1000UL)) {
            stopAndSendVoiceRecording(false);
        }
    }

    // 4. Manage 15s inactivity dim + 5s screen off power states
    updateScreenPowerState();

    // 5. Render UI immediately whenever state changes and screen is awake/dimmed
    if (g_uiDirty) {
        renderUI();
    }

    delay(5);
}
