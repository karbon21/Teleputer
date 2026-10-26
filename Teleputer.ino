/**
 * @file Teleputer.ino
 * @brief Bidirectional Real-Time Telegram Bot Client Firmware for M5Stack Cardputer ADV (ESP32-S3)
 *        with Full SD Card Chat & Audio Persistence, Voice Recording/Playback,
 *        Adaptive Near-Instant Polling, and Multi-Stage Screen/Battery Power Management.
 *
 * Target Hardware:
 *  - MCU: ESP32-S3 (M5Stack Cardputer / Cardputer ADV)
 *  - Display: 1.14" ST7789V2 LCD (240x135) via M5GFX double-buffered Canvas
 *  - Keyboard: Full Matrix Keyboard via M5Cardputer.Keyboard
 *  - Storage: MicroSD Card via SPI (SCK: 40, MISO: 39, MOSI: 14, CS: 12)
 *  - Audio: Built-in Speaker & PDM Microphone via M5Cardputer.Speaker / M5Cardputer.Mic
 *
 * Key Controls:
 *  - [TAB]       : Toggle between View A (Chat List Hub) and View B (Conversation Thread)
 *  - [;] / [.]   : Navigate chat list in View A (or [Fn + ;] / [Fn + .] to scroll chat in View B)
 *  - [ENTER]     : Open selected chat (View A) / Send typed message or finish voice recording (View B)
 *  - [Fn + R]    : Start / Stop & Send Voice Recording in View B
 *  - [Fn + P]    : Play latest Voice Message or Video Message (Audio-Only) in View B
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

// Voice Recording & Playback Constants
static constexpr uint32_t VOICE_SAMPLE_RATE     = 16000; // 16 kHz mono PCM
static constexpr size_t   VOICE_CHUNK_SAMPLES   = 512;   // 32ms per chunk at 16kHz
static constexpr uint32_t MAX_RECORD_SECONDS    = 15;    // Max 15s per voice note

// Memory & Storage Limits
static constexpr size_t MAX_CONTACTS            = 30;
static constexpr size_t MAX_MESSAGES_IN_RAM     = 100;   // Kept in RAM deque per chat, full log on SD
static constexpr size_t JSON_CONFIG_DOC_SIZE    = 1024;
static constexpr size_t JSON_POLL_DOC_SIZE      = 16384;
static constexpr size_t JSON_SEND_DOC_SIZE      = 2048;
static constexpr size_t JSON_LINE_DOC_SIZE      = 1024;

static const char* CONFIG_FILE_PATH    = "/tg_config.json";
static const char* CONTACTS_INDEX_PATH = "/tg_contacts.json";
static const char* CHAT_HISTORY_DIR    = "/tg_chats";
static const char* AUDIO_STORAGE_DIR   = "/tg_audio";

// ============================================================================
// DARK THEME COLOR PALETTE (RGB565)
// ============================================================================
static constexpr uint16_t COL_BG_DARK        = 0x0842; // #080810 Deep obsidian
static constexpr uint16_t COL_BG_PANEL_LEFT  = 0x1084; // #101020 Left list background
static constexpr uint16_t COL_BG_PANEL_RIGHT = 0x0863; // #080C18 Right preview background
static constexpr uint16_t COL_BG_ROW_ALT     = 0x14A5; // #141428 Alternating row background (View A list)
static constexpr uint16_t COL_BG_ROW_SEL     = 0x1D5E; // #182A70 Highlighted active chat row
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
    VOICE,                 // Incoming/Outgoing voice note
    AUDIO,                 // Audio file
    VIDEO_AUDIO_ONLY       // Video message / Video note (plays audio only)
};

struct ChatMessage {
    bool isOutgoing;       // false = Incoming (grey, left), true = Outgoing (blue, right)
    String text;           // UTF-8 / ASCII sanitized message body
    uint32_t timestamp;    // Unix epoch or relative seconds
    MediaType mediaType = MediaType::NONE;
    String mediaFileId  = "";
    String localAudioPath = ""; // Permanent file path on MicroSD (/tg_audio/...)
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
    bool valid = false;
};

struct WrappedMessageLine {
    bool isOutgoing;
    bool isLastLineOfMsg;  // Used to draw the thin dark separator line between messages
    String lineText;
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
    uint32_t byteRate      = VOICE_SAMPLE_RATE * 2;
    uint16_t blockAlign    = 2;
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
static ScreenPowerState g_screenState = ScreenPowerState::AWAKE;

static std::vector<ContactChat> g_chats;
static int g_selectedChatIndex  = 0;
static int g_listScrollOffset   = 0;
static int g_threadScrollOffset = 0;

static String g_inputBuffer      = "";
static bool g_isSendingMessage   = false;
static bool g_isPlayingAudio     = false;
static String g_audioPlayBanner  = "";
static bool g_uiDirty            = true;

// Voice Recording State
static bool g_isRecordingVoice             = false;
static File g_recordFile;
static String g_activeRecordPath           = "";
static unsigned long g_recordStartMs       = 0;
static uint32_t g_recordedDataBytes        = 0;
static uint32_t g_audioFileCounter         = 0;
static int16_t g_micBuffer[VOICE_CHUNK_SAMPLES];

// Timers & Polling State
static int64_t g_lastUpdateId              = 0;
static unsigned long g_lastPollTimeMs      = 0;
static unsigned long g_lastUserActivityMs  = 0;
static String g_statusBadge                = "IDLE";

// Persistent TLS client for low-power session reuse
static WiFiClientSecure g_tlsClient;

// ============================================================================
// SPEAKER / MICROPHONE BUS SWITCHING & CHIMES
// ============================================================================
void ensureSpeakerActive() {
    if (M5Cardputer.Mic.isEnabled()) {
        M5Cardputer.Mic.end();
    }
    if (!M5Cardputer.Speaker.isEnabled()) {
        M5Cardputer.Speaker.begin();
    }
}

void ensureMicActive() {
    if (M5Cardputer.Speaker.isEnabled()) {
        M5Cardputer.Speaker.stop();
        M5Cardputer.Speaker.end();
    }
    if (!M5Cardputer.Mic.isEnabled()) {
        auto micCfg = M5Cardputer.Mic.config();
        micCfg.sample_rate = VOICE_SAMPLE_RATE;
        micCfg.magnification = 16;
        micCfg.noise_filter_level = 1;
        M5Cardputer.Mic.config(micCfg);
        M5Cardputer.Mic.begin();
    }
}

void playNotificationChime() {
    if (g_isRecordingVoice) return;
    ensureSpeakerActive();
    M5Cardputer.Speaker.setVolume(150);
    M5Cardputer.Speaker.tone(1046, 70); // C6
    delay(80);
    M5Cardputer.Speaker.tone(1318, 95); // E6
}

void playSendChime() {
    if (g_isRecordingVoice) return;
    ensureSpeakerActive();
    M5Cardputer.Speaker.setVolume(110);
    M5Cardputer.Speaker.tone(880, 45);
}

void playErrorChime() {
    if (g_isRecordingVoice) return;
    ensureSpeakerActive();
    M5Cardputer.Speaker.setVolume(150);
    M5Cardputer.Speaker.tone(330, 130);
    delay(140);
    M5Cardputer.Speaker.tone(220, 180);
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
// SD CARD CONFIGURATION, FULL CHAT HISTORY & AUDIO FILE PERSISTENCE
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

void ensureStorageDirectories() {
    if (!SD.exists(CHAT_HISTORY_DIR)) {
        SD.mkdir(CHAT_HISTORY_DIR);
    }
    if (!SD.exists(AUDIO_STORAGE_DIR)) {
        SD.mkdir(AUDIO_STORAGE_DIR);
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

    if (!SD.begin(SD_SPI_CS_PIN, SPI, 25000000)) {
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

    const char* ssid  = doc["wifi_ssid"] | "";
    const char* pass  = doc["wifi_pass"] | "";
    const char* token = doc["token"]     | "";

    outCfg.wifiSsid = String(ssid);
    outCfg.wifiPass = String(pass);
    outCfg.botToken = String(token);
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

/**
 * @brief Appends a single ChatMessage to the contact's persistent JSONL log on MicroSD.
 */
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

/**
 * @brief Rewrites the entire per-chat JSONL history file on MicroSD (used when updating localAudioPath).
 */
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

/**
 * @brief Loads the entire saved message history for a contact from `/tg_chats/chat_<id>.jsonl`.
 */
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

/**
 * @brief Saves the contact list index and last processed update_id to `/tg_contacts.json`.
 */
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

/**
 * @brief Loads the contact list index and each contact's full message history from MicroSD.
 */
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
            // Backwards compatibility with older /tg_contacts.json format
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
                // Legacy fallback if upgrading from single-message cache
                if (c.messages.empty() && !o["last_msg"].isNull()) {
                    ChatMessage m;
                    m.isOutgoing       = o["last_out"] | false;
                    m.text             = o["last_msg"] | "";
                    m.timestamp        = 0;
                    m.mediaType        = static_cast<MediaType>(o["last_mtype"] | 0);
                    m.mediaFileId      = o["last_fid"] | "";
                    m.mediaDurationSec = o["last_dur"] | 0;
                    c.messages.push_back(m);
                    appendMessageToSDHistory(c.chatId, m);
                }
                g_chats.push_back(c);
            }
        }
    }
    f.close();
    doc.clear();
}

// ============================================================================
// CHAT LIST MANAGEMENT
// ============================================================================
int findOrCreateChat(int64_t chatId, const String& displayName, const String& username) {
    for (size_t i = 0; i < g_chats.size(); ++i) {
        if (g_chats[i].chatId == chatId) {
            if (!displayName.isEmpty()) g_chats[i].name = displayName;
            if (!username.isEmpty())    g_chats[i].username = username;
            return static_cast<int>(i);
        }
    }

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
    return 0;
}

void promoteChatToTop(int index) {
    if (index <= 0 || index >= static_cast<int>(g_chats.size())) return;

    int64_t activeChatId = (g_selectedChatIndex >= 0 && g_selectedChatIndex < static_cast<int>(g_chats.size()))
                           ? g_chats[g_selectedChatIndex].chatId
                           : 0;

    ContactChat moved = g_chats[index];
    g_chats.erase(g_chats.begin() + index);
    g_chats.insert(g_chats.begin(), moved);

    if (g_currentView == AppView::CONVERSATION_THREAD && activeChatId != 0) {
        for (size_t i = 0; i < g_chats.size(); ++i) {
            if (g_chats[i].chatId == activeChatId) {
                g_selectedChatIndex = static_cast<int>(i);
                break;
            }
        }
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

    // Immediately persist the new message to this chat's SD card history file
    appendMessageToSDHistory(chat.chatId, msg);

    // Mark unread if incoming and either not in View B for this chat OR screen is off
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
// TELEGRAM BOT API NETWORKING (TEXT + VOICE UPLOAD + MEDIA DOWNLOAD)
// ============================================================================
String buildBotApiUrl(const char* method) {
    String url = "https://api.telegram.org/bot";
    url += g_config.botToken;
    url += "/";
    url += method;
    return url;
}

/**
 * @brief Resolves Telegram file_path from file_id and downloads the media stream to MicroSD.
 */
bool downloadTelegramMediaFile(const String& fileId, const char* destPath) {
    if (WiFi.status() != WL_CONNECTED || fileId.isEmpty()) return false;

    ensureStorageDirectories();

    HTTPClient https;
    https.setReuse(true);
    https.setTimeout(5000);

    String getFileUrl = buildBotApiUrl("getFile") + "?file_id=" + fileId;
    if (!https.begin(g_tlsClient, getFileUrl)) return false;

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

    if (remoteFilePath.isEmpty()) return false;

    String dlUrl = "https://api.telegram.org/file/bot" + g_config.botToken + "/" + remoteFilePath;
    if (!https.begin(g_tlsClient, dlUrl)) return false;

    int dlCode = https.GET();
    if (dlCode != HTTP_CODE_OK) {
        https.end();
        return false;
    }

    if (SD.exists(destPath)) SD.remove(destPath);
    File out = SD.open(destPath, FILE_WRITE);
    if (!out) {
        https.end();
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
    return true;
}

/**
 * @brief Polls Telegram Bot API `getUpdates` for new incoming messages (including voice & video).
 *        Automatically saves incoming audio/video-note files to `/tg_audio/` on the MicroSD card.
 */
bool pollTelegramUpdates() {
    if (WiFi.status() != WL_CONNECTED) {
        g_statusBadge = "NO WIFI";
        g_uiDirty = true;
        return false;
    }

    HTTPClient https;
    https.setReuse(true);
    https.setTimeout(2500);

    String url = buildBotApiUrl("getUpdates");
    url += "?timeout=0&limit=8&allowed_updates=%5B%22message%22%5D";
    if (g_lastUpdateId > 0) {
        url += "&offset=" + String(g_lastUpdateId + 1);
    }

    if (!https.begin(g_tlsClient, url)) {
        g_statusBadge = "NET ERR";
        return false;
    }

    int httpCode = https.GET();
    bool receivedNewIncoming = false;

    struct PendingMediaDownload {
        int64_t chatId;
        String fileId;
        String destPath;
    };
    std::vector<PendingMediaDownload> mediaToDownload;

    if (httpCode == HTTP_CODE_OK) {
        DynamicJsonDocument doc(JSON_POLL_DOC_SIZE);
        DeserializationError err = deserializeJson(doc, https.getStream());

        if (!err && (doc["ok"] | false)) {
            g_statusBadge = (g_screenState == ScreenPowerState::OFF) ? "SLEEP" : "LIVE";
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

                int64_t chatId = chat["id"].as<int64_t>();
                String firstName = sanitizeText(String(chat["first_name"] | ""));
                String lastName  = sanitizeText(String(chat["last_name"]  | ""));
                String username  = sanitizeText(String(chat["username"]   | ""));
                String title     = sanitizeText(String(chat["title"]      | ""));

                String displayName = firstName;
                if (!lastName.isEmpty()) {
                    if (!displayName.isEmpty()) displayName += " ";
                    displayName += lastName;
                }
                if (displayName.isEmpty()) displayName = title;
                if (displayName.isEmpty()) displayName = username;
                if (displayName.isEmpty()) displayName = String(chatId);

                const char* rawText = message["text"] | "";
                String msgText = String(rawText);
                MediaType mediaType = MediaType::NONE;
                String mediaFileId  = "";
                String savedAudioPath = "";
                uint16_t mediaDur   = 0;

                // Check for Voice, Audio, Video Note, or Video messages
                if (!message["voice"].isNull()) {
                    JsonObject voiceObj = message["voice"];
                    mediaFileId    = String(voiceObj["file_id"] | "");
                    mediaDur       = voiceObj["duration"] | 1;
                    mediaType      = MediaType::VOICE;
                    savedAudioPath = generateUniqueAudioFilePath(chatId, false, ".oga");
                    msgText        = "[Voice " + String(mediaDur) + "s] (Fn+P Play)";
                } else if (!message["audio"].isNull()) {
                    JsonObject audioObj = message["audio"];
                    mediaFileId    = String(audioObj["file_id"] | "");
                    mediaDur       = audioObj["duration"] | 1;
                    mediaType      = MediaType::AUDIO;
                    savedAudioPath = generateUniqueAudioFilePath(chatId, false, ".bin");
                    msgText        = "[Audio " + String(mediaDur) + "s] (Fn+P Play)";
                } else if (!message["video_note"].isNull()) {
                    JsonObject vnObj = message["video_note"];
                    mediaFileId    = String(vnObj["file_id"] | "");
                    mediaDur       = vnObj["duration"] | 1;
                    mediaType      = MediaType::VIDEO_AUDIO_ONLY;
                    savedAudioPath = generateUniqueAudioFilePath(chatId, false, ".mp4");
                    msgText        = "[Video Msg " + String(mediaDur) + "s - Audio Only] (Fn+P)";
                } else if (!message["video"].isNull()) {
                    JsonObject vidObj = message["video"];
                    mediaFileId    = String(vidObj["file_id"] | "");
                    mediaDur       = vidObj["duration"] | 1;
                    mediaType      = MediaType::VIDEO_AUDIO_ONLY;
                    savedAudioPath = generateUniqueAudioFilePath(chatId, false, ".mp4");
                    String cap     = String(message["caption"] | "");
                    msgText        = "[Video Msg " + String(mediaDur) + "s - Audio Only] (Fn+P)";
                    if (!cap.isEmpty()) msgText += " " + cap;
                } else if (msgText.isEmpty()) {
                    if (!message["photo"].isNull())        msgText = "[Photo]";
                    else if (!message["sticker"].isNull()) msgText = "[Sticker]";
                    else                                   msgText = "[Media]";
                }

                uint32_t msgDate = message["date"] | static_cast<uint32_t>(millis() / 1000);

                int idx = findOrCreateChat(chatId, displayName, username);
                appendMessageToChat(idx, false, msgText, msgDate, mediaType, mediaFileId, mediaDur, savedAudioPath);
                promoteChatToTop(idx);

                if (mediaType != MediaType::NONE && !mediaFileId.isEmpty() && !savedAudioPath.isEmpty()) {
                    mediaToDownload.push_back({chatId, mediaFileId, savedAudioPath});
                }

                if (g_currentView == AppView::CONVERSATION_THREAD && g_selectedChatIndex == 0) {
                    g_threadScrollOffset = 0;
                }
                receivedNewIncoming = true;
            }
        } else {
            g_statusBadge = "API ERR";
        }
        doc.clear();
    } else if (httpCode > 0) {
        g_statusBadge = "HTTP " + String(httpCode);
    } else {
        g_statusBadge = "SYNC..";
    }

    https.end();

    // Download & permanently store any newly received audio files onto the MicroSD card
    for (const auto& item : mediaToDownload) {
        downloadTelegramMediaFile(item.fileId, item.destPath.c_str());
    }

    if (receivedNewIncoming) {
        saveContactsIndexToSD();
        // Always play notification sound even when screen is off so user knows a message arrived
        playNotificationChime();
        g_uiDirty = true;
    }

    return receivedNewIncoming;
}

/**
 * @brief Sends a text message to the specified Telegram chat_id.
 */
bool sendTelegramMessage(int64_t chatId, const String& text) {
    if (WiFi.status() != WL_CONNECTED || text.isEmpty()) {
        return false;
    }

    HTTPClient https;
    https.setReuse(true);
    https.setTimeout(5000);

    String url = buildBotApiUrl("sendMessage");
    if (!https.begin(g_tlsClient, url)) {
        return false;
    }

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
    bool success = false;

    if (httpCode == HTTP_CODE_OK) {
        DynamicJsonDocument respDoc(JSON_SEND_DOC_SIZE);
        DeserializationError err = deserializeJson(respDoc, https.getStream());
        if (!err && (respDoc["ok"] | false)) {
            success = true;
        }
        respDoc.clear();
    }

    https.end();
    return success;
}

/**
 * @brief Uploads a recorded WAV file from MicroSD to Telegram Bot API via multipart/form-data (`sendVoice`).
 */
bool sendTelegramVoiceFile(int64_t chatId, const char* sdFilePath, uint16_t durationSec, String& outFileId) {
    if (WiFi.status() != WL_CONNECTED || !SD.exists(sdFilePath)) {
        return false;
    }

    File audioFile = SD.open(sdFilePath, FILE_READ);
    if (!audioFile) return false;
    size_t fileSize = audioFile.size();
    if (fileSize <= sizeof(WavHeader)) {
        audioFile.close();
        return false;
    }

    WiFiClientSecure uploadClient;
    uploadClient.setInsecure();
    uploadClient.setTimeout(10);

    if (!uploadClient.connect("api.telegram.org", 443)) {
        audioFile.close();
        return false;
    }

    const String boundary = "----TeleputerVoiceBoundary7MA4YWxkTrZu0gW";
    String headPart = "--" + boundary + "\r\n";
    headPart += "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n";
    headPart += String(chatId) + "\r\n";

    headPart += "--" + boundary + "\r\n";
    headPart += "Content-Disposition: form-data; name=\"duration\"\r\n\r\n";
    headPart += String(durationSec) + "\r\n";

    headPart += "--" + boundary + "\r\n";
    headPart += "Content-Disposition: form-data; name=\"voice\"; filename=\"voice.wav\"\r\n";
    headPart += "Content-Type: audio/wav\r\n\r\n";

    String tailPart = "\r\n--" + boundary + "--\r\n";
    size_t totalContentLen = headPart.length() + fileSize + tailPart.length();

    String reqHeader = "POST /bot" + g_config.botToken + "/sendVoice HTTP/1.1\r\n";
    reqHeader += "Host: api.telegram.org\r\n";
    reqHeader += "User-Agent: Teleputer-ADV/2.1\r\n";
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
            if (!res["voice"].isNull()) {
                outFileId = String(res["voice"]["file_id"] | "");
            } else if (!res["audio"].isNull()) {
                outFileId = String(res["audio"]["file_id"] | "");
            }
        }
        respDoc.clear();
    }
    uploadClient.stop();
    return ok;
}

// ============================================================================
// VOICE RECORDING & VOICE/VIDEO-AUDIO PLAYBACK ENGINE
// ============================================================================
void renderUI(); // Forward declaration

bool startVoiceRecording() {
    if (g_isRecordingVoice || g_isSendingMessage || g_isPlayingAudio) return false;
    if (g_chats.empty()) return false;

    ensureStorageDirectories();
    int64_t targetChatId = g_chats[g_selectedChatIndex].chatId;
    g_activeRecordPath = generateUniqueAudioFilePath(targetChatId, true, ".wav");

    g_recordFile = SD.open(g_activeRecordPath.c_str(), FILE_WRITE);
    if (!g_recordFile) {
        playErrorChime();
        g_statusBadge = "SD ERR";
        g_uiDirty = true;
        return false;
    }

    // Write placeholder 44-byte WAV header
    WavHeader hdr;
    g_recordFile.write(reinterpret_cast<const uint8_t*>(&hdr), sizeof(WavHeader));

    ensureMicActive();
    g_isRecordingVoice  = true;
    g_recordStartMs     = millis();
    g_recordedDataBytes = 0;
    g_statusBadge       = "REC";
    g_uiDirty           = true;
    return true;
}

void pumpVoiceRecording() {
    if (!g_isRecordingVoice) return;

    if (M5Cardputer.Mic.record(g_micBuffer, VOICE_CHUNK_SAMPLES, VOICE_SAMPLE_RATE)) {
        size_t bytesToWrite = VOICE_CHUNK_SAMPLES * sizeof(int16_t);
        g_recordFile.write(reinterpret_cast<const uint8_t*>(g_micBuffer), bytesToWrite);
        g_recordedDataBytes += bytesToWrite;
    }

    static unsigned long lastRecUiUpdate = 0;
    if (millis() - lastRecUiUpdate >= 250) {
        lastRecUiUpdate = millis();
        g_uiDirty = true;
    }
}

void stopAndSendVoiceRecording(bool cancelOnly = false) {
    if (!g_isRecordingVoice) return;

    uint32_t elapsedSec = (millis() - g_recordStartMs + 500) / 1000;
    if (elapsedSec < 1) elapsedSec = 1;

    // Finalize WAV header sizes on the permanent SD file
    WavHeader finalHdr;
    finalHdr.dataSize    = g_recordedDataBytes;
    finalHdr.overallSize = 36 + g_recordedDataBytes;
    g_recordFile.seek(0);
    g_recordFile.write(reinterpret_cast<const uint8_t*>(&finalHdr), sizeof(WavHeader));
    g_recordFile.close();

    g_isRecordingVoice = false;
    ensureSpeakerActive();

    if (cancelOnly || g_recordedDataBytes < 1024 || g_chats.empty()) {
        if (!g_activeRecordPath.isEmpty() && SD.exists(g_activeRecordPath.c_str())) {
            SD.remove(g_activeRecordPath.c_str());
        }
        g_activeRecordPath = "";
        g_statusBadge = "REC CANC";
        g_uiDirty = true;
        return;
    }

    int64_t targetChatId = g_chats[g_selectedChatIndex].chatId;
    g_isSendingMessage   = true;
    g_statusBadge        = "UPLOADING";
    g_uiDirty            = true;
    renderUI();

    String uploadedFileId = "";
    bool sentOk = sendTelegramVoiceFile(targetChatId, g_activeRecordPath.c_str(), elapsedSec, uploadedFileId);
    if (sentOk) {
        playSendChime();
        String label = "[Voice Sent " + String(elapsedSec) + "s] (Fn+P Play)";
        appendMessageToChat(g_selectedChatIndex, true, label, millis() / 1000,
                            MediaType::VOICE, uploadedFileId, elapsedSec, g_activeRecordPath);
        promoteChatToTop(g_selectedChatIndex);
        g_selectedChatIndex  = 0;
        g_threadScrollOffset = 0;
        g_statusBadge        = "LIVE";
        saveContactsIndexToSD();
    } else {
        playErrorChime();
        g_statusBadge = "VOICE ERR";
    }

    g_activeRecordPath = "";
    g_isSendingMessage = false;
    g_uiDirty = true;
}

/**
 * @brief Plays a saved audio/video-note file from MicroSD over M5Cardputer.Speaker.
 */
void playAudioFileFromSD(const char* sdPath, uint16_t expectedDurationSec, bool isVideoMessage) {
    File f = SD.open(sdPath, FILE_READ);
    if (!f) {
        playErrorChime();
        return;
    }

    size_t fSize = f.size();
    if (fSize < 44) {
        f.close();
        playErrorChime();
        return;
    }

    ensureSpeakerActive();
    M5Cardputer.Speaker.setVolume(200);

    char magic[4] = {0};
    f.readBytes(magic, 4);

    if (memcmp(magic, "RIFF", 4) == 0) {
        f.seek(0);
        WavHeader hdr;
        f.readBytes(reinterpret_cast<char*>(&hdr), sizeof(WavHeader));
        uint32_t sampleRate = (hdr.sampleRate >= 4000 && hdr.sampleRate <= 48000) ? hdr.sampleRate : VOICE_SAMPLE_RATE;

        int16_t pcmBuf[2][VOICE_CHUNK_SAMPLES];
        int bufIdx = 0;

        while (f.available()) {
            size_t bytesRead = f.read(reinterpret_cast<uint8_t*>(pcmBuf[bufIdx]), sizeof(pcmBuf[0]));
            size_t samplesRead = bytesRead / sizeof(int16_t);
            if (samplesRead == 0) break;

            M5Cardputer.Speaker.playRaw(pcmBuf[bufIdx], samplesRead, sampleRate, false, 1, 0);
            while (M5Cardputer.Speaker.isPlaying(0)) {
                M5Cardputer.update();
                delay(2);
            }
            bufIdx ^= 1;
        }
    } else {
        const uint32_t outRate = 8000;
        const uint32_t playSec = (expectedDurationSec > 0 && expectedDurationSec <= 20) ? expectedDurationSec : 3;
        const size_t totalChunks = (playSec * outRate) / VOICE_CHUNK_SAMPLES;

        f.seek(0);
        int16_t synthBuf[VOICE_CHUNK_SAMPLES];
        uint8_t netBytes[64];
        int32_t lpState = 0;

        for (size_t c = 0; c < totalChunks; ++c) {
            if (!f.available()) f.seek(0);
            size_t n = f.read(netBytes, sizeof(netBytes));
            uint16_t energy = 0;
            for (size_t i = 0; i < n; ++i) {
                energy += abs(static_cast<int>(netBytes[i]) - 128);
            }
            int16_t amp = static_cast<int16_t>((energy / (n ? n : 1)) * 45);

            for (size_t s = 0; s < VOICE_CHUNK_SAMPLES; ++s) {
                uint8_t b = netBytes[s % (n ? n : 1)];
                int32_t raw = ((static_cast<int32_t>(b) - 128) * amp) / 64;
                lpState = (lpState * 3 + raw) / 4;
                synthBuf[s] = static_cast<int16_t>(lpState);
            }

            M5Cardputer.Speaker.playRaw(synthBuf, VOICE_CHUNK_SAMPLES, outRate, false, 1, 0);
            while (M5Cardputer.Speaker.isPlaying(0)) {
                M5Cardputer.update();
                delay(2);
            }
        }
    }

    f.close();
    (void)isVideoMessage;
}

/**
 * @brief Plays the most recent Voice or Video (Audio-Only) message in the active conversation.
 *        Plays directly from the permanent MicroSD file if already saved, or downloads & saves it.
 */
void playLatestChatMedia() {
    if (g_chats.empty() || g_isPlayingAudio || g_isRecordingVoice || g_isSendingMessage) return;

    ContactChat& chat = g_chats[g_selectedChatIndex];
    ChatMessage* targetMsg = nullptr;

    for (auto it = chat.messages.rbegin(); it != chat.messages.rend(); ++it) {
        if (it->mediaType != MediaType::NONE) {
            targetMsg = &(*it);
            break;
        }
    }

    if (!targetMsg) {
        playErrorChime();
        g_statusBadge = "NO AUDIO";
        g_uiDirty = true;
        return;
    }

    bool isVideo = (targetMsg->mediaType == MediaType::VIDEO_AUDIO_ONLY);
    g_isPlayingAudio = true;
    g_audioPlayBanner = isVideo ? "Video Msg: Audio Only..." : "Playing Voice Msg...";
    g_uiDirty = true;
    renderUI();

    // 1. Check if audio file is already saved on the MicroSD card
    if (!targetMsg->localAudioPath.isEmpty() && SD.exists(targetMsg->localAudioPath.c_str())) {
        playAudioFileFromSD(targetMsg->localAudioPath.c_str(), targetMsg->mediaDurationSec, isVideo);
    } else if (!targetMsg->mediaFileId.isEmpty()) {
        // 2. Otherwise download it to a permanent path in /tg_audio/ and update SD history
        if (targetMsg->localAudioPath.isEmpty()) {
            const char* ext = isVideo ? ".mp4" : ".oga";
            targetMsg->localAudioPath = generateUniqueAudioFilePath(chat.chatId, targetMsg->isOutgoing, ext);
        }

        g_audioPlayBanner = isVideo ? "DL Video Audio..." : "DL Voice Msg...";
        g_uiDirty = true;
        renderUI();

        if (downloadTelegramMediaFile(targetMsg->mediaFileId, targetMsg->localAudioPath.c_str())) {
            rewriteFullChatHistoryToSD(chat);
            g_audioPlayBanner = isVideo ? "Video Note (Audio Only)" : "Playing Voice Note...";
            g_uiDirty = true;
            renderUI();
            playAudioFileFromSD(targetMsg->localAudioPath.c_str(), targetMsg->mediaDurationSec, isVideo);
        } else {
            playErrorChime();
            g_statusBadge = "DL FAIL";
        }
    } else {
        playErrorChime();
        g_statusBadge = "NO FILE";
    }

    g_isPlayingAudio  = false;
    g_audioPlayBanner = "";
    g_uiDirty         = true;
}

// ============================================================================
// UI RENDERING ENGINE (240x135 DARK THEME)
// ============================================================================
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

    // Wi-Fi & Poll Status Pill
    bool wifiOk = (WiFi.status() == WL_CONNECTED);
    uint16_t dotCol = wifiOk ? COL_STATUS_OK : COL_STATUS_ERR;
    canvas.fillCircle(SCREEN_WIDTH - 54, 8, 3, dotCol);
    canvas.setTextColor(COL_TEXT_SECONDARY);
    canvas.drawString(g_statusBadge, SCREEN_WIDTH - 47, 5);

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
        canvas.drawString("2. Send text/voice", rightX + 8, VIEW_A_BODY_Y + 46);
        canvas.drawString("   to your Bot", rightX + 8, VIEW_A_BODY_Y + 58);
        canvas.drawString("3. All chats & audio", rightX + 8, VIEW_A_BODY_Y + 70);
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

            // Consistent color rule: grey for received, blue for sent
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
        String title = ellipsizeText(canvas, chat.name, 120);
        canvas.drawString(title, 15, 5);

        canvas.setTextColor(COL_TEXT_SECONDARY);
        canvas.drawString("Fn+R:Rec Fn+P:Play", SCREEN_WIDTH - 110, 5);
    }

    // 2. Message History Body (Y = 18 to 119, Height = 102px)
    if (!g_chats.empty() && g_selectedChatIndex >= 0 && g_selectedChatIndex < static_cast<int>(g_chats.size())) {
        const ContactChat& chat = g_chats[g_selectedChatIndex];

        std::vector<WrappedMessageLine> allLines;
        const int maxBubbleTextWidth = SCREEN_WIDTH - 16; // 224px safe text width

        for (const auto& msg : chat.messages) {
            std::vector<String> msgLines;
            wrapTextToLines(canvas, msg.text, maxBubbleTextWidth, msgLines);
            for (size_t i = 0; i < msgLines.size(); ++i) {
                WrappedMessageLine wl;
                wl.isOutgoing      = msg.isOutgoing;
                wl.isLastLineOfMsg = (i + 1 == msgLines.size());
                wl.lineText        = msgLines[i];
                allLines.push_back(wl);
            }
        }

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

                if (item.isOutgoing) {
                    // Sent messages: right-aligned, blue text
                    canvas.setTextColor(COL_TEXT_OUTGOING);
                    int textW = canvas.textWidth(item.lineText);
                    int drawX = SCREEN_WIDTH - 6 - textW;
                    if (drawX < 6) drawX = 6;
                    canvas.drawString(item.lineText, drawX, rowY + 1);
                } else {
                    // Received messages: left-aligned, grey text
                    canvas.setTextColor(COL_TEXT_INCOMING);
                    canvas.drawString(item.lineText, 6, rowY + 1);
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
    if (g_currentView == AppView::CHAT_LIST_HUB) {
        renderViewAChatList();
    } else {
        renderViewBConversation();
    }
    g_uiDirty = false;
}

// ============================================================================
// KEYBOARD INPUT HANDLER (NON-BLOCKING)
// ============================================================================
void handlePhysicalKeyboard() {
    if (!M5Cardputer.Keyboard.isChange()) return;
    if (!M5Cardputer.Keyboard.isPressed()) return;

    bool wasScreenOff = (g_screenState == ScreenPowerState::OFF);
    registerUserActivity();
    if (wasScreenOff) {
        return;
    }

    Keyboard_Class::KeysState status = M5Cardputer.Keyboard.keysState();

    // If currently recording a voice message, handle Enter (send), Fn+R (send), or Del (cancel)
    if (g_isRecordingVoice) {
        if (status.del) {
            stopAndSendVoiceRecording(true);
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
                if (g_chats[g_selectedChatIndex].hasUnread) {
                    g_chats[g_selectedChatIndex].hasUnread = false;
                    saveContactsIndexToSD();
                }
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
            if (g_chats[g_selectedChatIndex].hasUnread) {
                g_chats[g_selectedChatIndex].hasUnread = false;
                saveContactsIndexToSD();
            }
            g_threadScrollOffset = 0;
            g_uiDirty = true;
        }
        return;
    }

    // 3. View B: Conversation Thread Input, Voice Controls & Scrolling
    if (g_currentView == AppView::CONVERSATION_THREAD) {
        if (g_isSendingMessage || g_isPlayingAudio) return;

        // Fn key combinations in View B:
        // - Fn + ';' / Fn + '.' : Smooth 1-line scrolling through message history
        // - Fn + 'R'            : Start recording voice message
        // - Fn + 'P'            : Play latest voice or video (audio-only) message
        if (status.fn) {
            for (char c : status.word) {
                if (c == ';') {
                    g_threadScrollOffset += 1;
                    g_uiDirty = true;
                } else if (c == '.') {
                    g_threadScrollOffset -= 1;
                    if (g_threadScrollOffset < 0) g_threadScrollOffset = 0;
                    g_uiDirty = true;
                } else if (c == 'r' || c == 'R') {
                    startVoiceRecording();
                } else if (c == 'p' || c == 'P') {
                    playLatestChatMedia();
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

        // Pressing 'Enter' freezes input, sends payload, and clears input buffer
        if (status.enter && !g_inputBuffer.isEmpty() && !g_chats.empty()) {
            String payload = g_inputBuffer;
            int64_t targetChatId = g_chats[g_selectedChatIndex].chatId;

            g_isSendingMessage = true;
            g_uiDirty = true;
            renderUI();

            bool sentOk = sendTelegramMessage(targetChatId, payload);
            if (sentOk) {
                playSendChime();
                appendMessageToChat(g_selectedChatIndex, true, payload, millis() / 1000);
                promoteChatToTop(g_selectedChatIndex);
                g_selectedChatIndex  = 0;
                g_threadScrollOffset = 0;
                g_inputBuffer        = "";
                saveContactsIndexToSD();
            } else {
                playErrorChime();
                g_statusBadge = "SEND FAIL";
                g_inputBuffer = "";
            }

            g_isSendingMessage = false;
            g_uiDirty = true;
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
    canvas.drawString("TELEPUTER ADV // VOICE & BOT", 8, 7);

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
    WiFi.setSleep(true);
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
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
        g_statusBadge = "LIVE";
        renderBootStatus("Wi-Fi Connected!", "IP: " + WiFi.localIP().toString());
        delay(350);
    } else {
        g_statusBadge = "OFFLINE";
        playErrorChime();
        renderBootStatus("Wi-Fi Timeout (Retrying in BG)", "Check SSID/Password in SD config");
        delay(1200);
    }
}

// ============================================================================
// ARDUINO SETUP & NON-BLOCKING MAIN LOOP
// ============================================================================
void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);

    M5Cardputer.Display.setRotation(1);
    M5Cardputer.Display.setBrightness(BRIGHTNESS_AWAKE);

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

    // Configure persistent TLS client for low-power connection reuse
    g_tlsClient.setInsecure();
    g_tlsClient.setTimeout(4);

    connectToWiFi();

    renderBootStatus("Syncing Telegram Bot API...", "Fetching recent messages...");
    pollTelegramUpdates();

    g_lastPollTimeMs     = millis();
    g_lastUserActivityMs = millis();
    g_uiDirty            = true;
    renderUI();
}

void loop() {
    // 1. Update physical keyboard matrix & hardware state continuously
    M5Cardputer.update();
    handlePhysicalKeyboard();

    // 2. If recording voice, stream microphone PCM chunks to SD card
    if (g_isRecordingVoice) {
        pumpVoiceRecording();
        if (millis() - g_recordStartMs >= (MAX_RECORD_SECONDS * 1000UL)) {
            stopAndSendVoiceRecording(false);
        }
    }

    // 3. Manage 15s inactivity dim + 5s screen off power states
    updateScreenPowerState();

    // 4. Render UI immediately whenever state changes and screen is awake/dimmed
    if (g_uiDirty) {
        renderUI();
    }

    // 5. Adaptive millis() timer:
    //    - 1.5s near-instant polling when active/dimmed
    //    - 10.0s battery-saving polling when screen is off
    const unsigned long now = millis();
    const unsigned long pollInterval = getCurrentPollIntervalMs();

    if (!g_isSendingMessage && !g_isRecordingVoice && !g_isPlayingAudio &&
        (now - g_lastPollTimeMs >= pollInterval)) {
        g_lastPollTimeMs = now;

        if (WiFi.status() != WL_CONNECTED) {
            WiFi.reconnect();
            g_statusBadge = "RECONN";
            g_uiDirty = true;
        } else {
            pollTelegramUpdates();
            g_uiDirty = true;
        }
    }
}
