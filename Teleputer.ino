/**
 * @file Teleputer.ino
 * @brief Bidirectional Real-Time Telegram Bot Client Firmware for M5Stack Cardputer ADV (ESP32-S3)
 *
 * Target Hardware:
 *  - MCU: ESP32-S3 (M5Stack Cardputer / Cardputer ADV)
 *  - Display: 1.14" ST7789V2 LCD (240x135) via M5GFX double-buffered Canvas
 *  - Keyboard: Full TCA8418 / Matrix Keyboard via M5Cardputer.Keyboard
 *  - Storage: MicroSD Card via SPI (SCK: 40, MISO: 39, MOSI: 14, CS: 12)
 *  - Audio: Built-in Speaker via M5Cardputer.Speaker
 *
 * Dependencies (Arduino Library Manager):
 *  - M5Cardputer (includes M5Unified & M5GFX)
 *  - ArduinoJson (v6.x / v7.x compatible)
 */

#include <Arduino.h>
#include <M5Cardputer.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
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

// Polling & Memory Limits
static constexpr unsigned long POLL_INTERVAL_MS   = 5000UL; // Poll every 5 seconds
static constexpr size_t MAX_CONTACTS              = 25;
static constexpr size_t MAX_MESSAGES_PER_CHAT     = 40;
static constexpr size_t JSON_CONFIG_DOC_SIZE      = 1024;
static constexpr size_t JSON_POLL_DOC_SIZE        = 16384;
static constexpr size_t JSON_SEND_DOC_SIZE        = 2048;

static const char* CONFIG_FILE_PATH = "/tg_config.json";
static const char* CONTACTS_CACHE_PATH = "/tg_contacts.json";

// ============================================================================
// DARK THEME COLOR PALETTE (RGB565)
// ============================================================================
static constexpr uint16_t COL_BG_DARK        = 0x0842; // #080810 Deep obsidian
static constexpr uint16_t COL_BG_PANEL_LEFT  = 0x1084; // #101020 Left list background
static constexpr uint16_t COL_BG_PANEL_RIGHT = 0x0863; // #080C18 Right preview background
static constexpr uint16_t COL_BG_ROW_ALT     = 0x14A5; // #141428 Alternating row background
static constexpr uint16_t COL_BG_ROW_SEL     = 0x1D5E; // #182A70 Highlighted active chat row
static constexpr uint16_t COL_BG_HEADER      = 0x10A6; // #101430 Header bar dark navy
static constexpr uint16_t COL_BG_INPUT       = 0x18E7; // #181C38 Bottom input bar background
static constexpr uint16_t COL_BG_INPUT_BUSY  = 0x2945; // Dimmed input when freezing/sending
static constexpr uint16_t COL_BORDER         = 0x31A9; // #303448 Crisp divider/border
static constexpr uint16_t COL_TEXT_PRIMARY   = 0xFFFF; // Crisp white
static constexpr uint16_t COL_TEXT_SECONDARY = 0x9CF3; // Muted silver-gray
static constexpr uint16_t COL_TEXT_INCOMING  = 0xCE79; // Light gray for incoming messages
static constexpr uint16_t COL_BUBBLE_IN_BG   = 0x2126; // Subtle dark gray row tint for incoming
static constexpr uint16_t COL_TEXT_OUTGOING  = 0x667F; // Vibrant sky blue for outgoing text
static constexpr uint16_t COL_BUBBLE_OUT_BG  = 0x096B; // Deep blue tint for outgoing row
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

struct ChatMessage {
    bool isOutgoing;       // false = Incoming (gray, left), true = Outgoing (blue, right)
    String text;           // UTF-8 / ASCII sanitized message body
    uint32_t timestamp;    // Unix epoch or relative seconds
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

// Wrapped line entry used by View B renderer
struct WrappedMessageLine {
    bool isOutgoing;
    bool isFirstLineOfMsg;
    String lineText;
};

// ============================================================================
// GLOBAL APPLICATION STATE
// ============================================================================
static M5Canvas canvas(&M5Cardputer.Display);
static AppConfig g_config;
static AppView g_currentView = AppView::CHAT_LIST_HUB;

static std::vector<ContactChat> g_chats;
static int g_selectedChatIndex = 0;
static int g_listScrollOffset  = 0;
static int g_threadScrollOffset = 0; // Lines scrolled up from bottom (0 = auto-bottom)

static String g_inputBuffer = "";
static bool g_isSendingMessage = false;
static bool g_isPollingActive  = false;
static bool g_uiDirty          = true;

static int64_t g_lastUpdateId          = 0;
static unsigned long g_lastPollTimeMs  = 0;
static String g_statusBadge            = "IDLE";

// Persistent TLS client for lower overhead polling
static WiFiClientSecure g_tlsClient;

// ============================================================================
// AUDIO NOTIFICATION HELPERS
// ============================================================================
void playNotificationChime() {
    M5Cardputer.Speaker.setVolume(140);
    M5Cardputer.Speaker.tone(1046, 70); // C6
    delay(80);
    M5Cardputer.Speaker.tone(1318, 90); // E6
}

void playSendChime() {
    M5Cardputer.Speaker.setVolume(100);
    M5Cardputer.Speaker.tone(880, 45);
}

void playErrorChime() {
    M5Cardputer.Speaker.setVolume(150);
    M5Cardputer.Speaker.tone(330, 140);
    delay(150);
    M5Cardputer.Speaker.tone(220, 200);
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
            // Skip multi-byte UTF-8 continuation sequences gracefully or replace with '?'
            out += '?';
            while (i + 1 < raw.length() && (static_cast<uint8_t>(raw[i + 1]) & 0xC0) == 0x80) {
                ++i;
            }
        }
    }
    return out;
}

/**
 * @brief Wraps a text string into multiple lines guaranteed to fit within maxWidthPx
 *        using the current font metrics of the target M5Canvas.
 */
void wrapTextToLines(M5Canvas& gfx, const String& text, int maxWidthPx, std::vector<String>& outLines) {
    if (text.isEmpty()) {
        outLines.push_back("");
        return;
    }

    String currentLine = "";
    int idx = 0;
    const int len = text.length();

    while (idx < len) {
        // Extract next word
        int nextSpace = text.indexOf(' ', idx);
        if (nextSpace == -1) nextSpace = len;
        String word = text.substring(idx, nextSpace);

        // If a single word itself is wider than maxWidthPx, hard-break character by character
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

/**
 * @brief Truncates string with ".." if it exceeds pixel width
 */
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
// SD CARD CONFIGURATION & CONTACT PERSISTENCE
// ============================================================================
void renderFatalErrorScreen(const String& title, const String& subtitle, const String& hint1, const String& hint2) {
    canvas.fillSprite(COL_BG_DARK);

    // Header banner
    canvas.fillRect(0, 0, SCREEN_WIDTH, 22, COL_STATUS_ERR);
    canvas.setTextColor(COL_TEXT_PRIMARY, COL_STATUS_ERR);
    canvas.setTextDatum(top_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(1);
    canvas.drawString("! SD CARD SETUP FAILED", 8, 7);

    // Card frame
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

    // Expected JSON structure preview
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

    // Scoped DynamicJsonDocument to prevent heap fragmentation / leaks
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

    outCfg.valid = true;
    return true;
}

void saveContactsToSD() {
    File f = SD.open(CONTACTS_CACHE_PATH, FILE_WRITE);
    if (!f) return;

    DynamicJsonDocument doc(4096);
    JsonArray arr = doc.to<JsonArray>();
    for (const auto& c : g_chats) {
        JsonObject o = arr.createNestedObject();
        o["id"]   = c.chatId;
        o["name"] = c.name;
        o["user"] = c.username;
        if (!c.messages.empty()) {
            o["last_msg"] = c.messages.back().text;
            o["last_out"] = c.messages.back().isOutgoing;
        }
    }
    serializeJson(doc, f);
    f.close();
    doc.clear();
}

void loadCachedContactsFromSD() {
    if (!SD.exists(CONTACTS_CACHE_PATH)) return;
    File f = SD.open(CONTACTS_CACHE_PATH, FILE_READ);
    if (!f) return;

    DynamicJsonDocument doc(4096);
    if (deserializeJson(doc, f) == DeserializationError::Ok && doc.is<JsonArray>()) {
        JsonArray arr = doc.as<JsonArray>();
        for (JsonObject o : arr) {
            if (g_chats.size() >= MAX_CONTACTS) break;
            ContactChat c;
            c.chatId       = o["id"].as<int64_t>();
            c.name         = o["name"] | "Unknown";
            c.username     = o["user"] | "";
            c.hasUnread    = false;
            c.lastActivity = 0;
            String lastMsg = o["last_msg"] | "";
            if (!lastMsg.isEmpty()) {
                ChatMessage m;
                m.isOutgoing = o["last_out"] | false;
                m.text       = lastMsg;
                m.timestamp  = 0;
                c.messages.push_back(m);
            }
            if (c.chatId != 0) {
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

    // New user contacted the bot -> add to the top of the contact list
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

    // Keep user selection anchored to the same conversation if inside View B
    if (g_currentView == AppView::CONVERSATION_THREAD && activeChatId != 0) {
        for (size_t i = 0; i < g_chats.size(); ++i) {
            if (g_chats[i].chatId == activeChatId) {
                g_selectedChatIndex = static_cast<int>(i);
                break;
            }
        }
    }
}

void appendMessageToChat(int chatIdx, bool isOutgoing, const String& text, uint32_t ts) {
    if (chatIdx < 0 || chatIdx >= static_cast<int>(g_chats.size())) return;

    ContactChat& chat = g_chats[chatIdx];
    ChatMessage msg;
    msg.isOutgoing = isOutgoing;
    msg.text       = sanitizeText(text);
    msg.timestamp  = ts;

    if (chat.messages.size() >= MAX_MESSAGES_PER_CHAT) {
        chat.messages.pop_front();
    }
    chat.messages.push_back(msg);
    chat.lastActivity = ts;

    // Mark unread if incoming and not actively viewing this chat in View B
    if (!isOutgoing) {
        bool activelyViewingThisChat = (g_currentView == AppView::CONVERSATION_THREAD && g_selectedChatIndex == chatIdx);
        if (!activelyViewingThisChat) {
            chat.hasUnread = true;
        }
    }
}

// ============================================================================
// TELEGRAM BOT API NETWORKING (HTTPS + ARDUINOJSON)
// ============================================================================
String buildBotApiUrl(const char* method) {
    String url = "https://api.telegram.org/bot";
    url += g_config.botToken;
    url += "/";
    url += method;
    return url;
}

/**
 * @brief Polls Telegram Bot API `getUpdates` for new incoming messages.
 *        Uses scoped DynamicJsonDocument to prevent heap leaks.
 */
bool pollTelegramUpdates() {
    if (WiFi.status() != WL_CONNECTED) {
        g_statusBadge = "NO WIFI";
        g_uiDirty = true;
        return false;
    }

    g_isPollingActive = true;
    HTTPClient https;
    https.setReuse(true);
    https.setTimeout(3500);

    String url = buildBotApiUrl("getUpdates");
    url += "?timeout=0&limit=10&allowed_updates=%5B%22message%22%5D";
    if (g_lastUpdateId > 0) {
        url += "&offset=" + String(g_lastUpdateId + 1);
    }

    if (!https.begin(g_tlsClient, url)) {
        g_isPollingActive = false;
        g_statusBadge = "NET ERR";
        return false;
    }

    int httpCode = https.GET();
    bool receivedNewIncoming = false;

    if (httpCode == HTTP_CODE_OK) {
        DynamicJsonDocument doc(JSON_POLL_DOC_SIZE);
        DeserializationError err = deserializeJson(doc, https.getStream());

        if (!err && (doc["ok"] | false)) {
            g_statusBadge = "ONLINE";
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
                if (msgText.isEmpty()) {
                    if (!message["photo"].isNull())        msgText = "[Photo]";
                    else if (!message["sticker"].isNull()) msgText = "[Sticker]";
                    else if (!message["voice"].isNull())   msgText = "[Voice]";
                    else                                   msgText = "[Media]";
                }

                uint32_t msgDate = message["date"] | static_cast<uint32_t>(millis() / 1000);

                int idx = findOrCreateChat(chatId, displayName, username);
                appendMessageToChat(idx, false, msgText, msgDate);
                promoteChatToTop(idx);

                if (g_currentView == AppView::CONVERSATION_THREAD && g_selectedChatIndex == 0) {
                    g_threadScrollOffset = 0; // Auto-scroll to newest message
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
        g_statusBadge = "TLS WAIT";
    }

    https.end();
    g_isPollingActive = false;

    if (receivedNewIncoming) {
        saveContactsToSD();
        playNotificationChime();
        g_uiDirty = true;
    }

    return receivedNewIncoming;
}

/**
 * @brief Sends an outgoing message to the specified Telegram chat_id via Bot API `sendMessage`.
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

// ============================================================================
// UI RENDERING ENGINE (240x135 DARK THEME)
// ============================================================================

/**
 * @brief Renders View A: Chat Selection List (Main Hub)
 *        - Left 40% (96px): Scrollable vertical list of up to 5 recent chat names,
 *          highlighted selection, and red unread indicator dot.
 *        - Right 60% (144px): Preview pane showing the last message snippet & metadata.
 */
void renderViewAChatList() {
    canvas.fillSprite(COL_BG_DARK);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(1);
    canvas.setTextDatum(top_left);

    // 1. Top Header Bar (18px high)
    canvas.fillRect(0, 0, SCREEN_WIDTH, HEADER_HEIGHT, COL_BG_HEADER);
    canvas.drawFastHLine(0, HEADER_HEIGHT - 1, SCREEN_WIDTH, COL_BORDER);

    // Brand title + Chat counter
    canvas.setTextColor(COL_ACCENT_CYAN);
    canvas.drawString("TELEPUTER", 6, 5);

    canvas.setTextColor(COL_TEXT_SECONDARY);
    String countStr = "[" + String(g_chats.size()) + "]";
    canvas.drawString(countStr, 66, 5);

    // Wi-Fi & Poll Status Pill on right of header
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
        // Clamp selection and scroll window (max 5 visible rows)
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

            // Alternating row background or highlighted selection
            uint16_t rowBg = isSelected
                             ? COL_BG_ROW_SEL
                             : ((vis % 2 == 0) ? COL_BG_PANEL_LEFT : COL_BG_ROW_ALT);
            canvas.fillRect(0, rowY, VIEW_A_LEFT_WIDTH - 1, VIEW_A_ROW_HEIGHT, rowBg);

            if (isSelected) {
                // Active selection accent bar on left edge + crisp border
                canvas.fillRect(0, rowY, 3, VIEW_A_ROW_HEIGHT, COL_ACCENT_CYAN);
                canvas.drawRect(0, rowY, VIEW_A_LEFT_WIDTH - 1, VIEW_A_ROW_HEIGHT, COL_ACCENT_CYAN);
            } else {
                canvas.drawFastHLine(0, rowY + VIEW_A_ROW_HEIGHT - 1, VIEW_A_LEFT_WIDTH - 1, COL_BORDER);
            }

            // Unread message red dot indicator
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

    // Crisp vertical divider at X = 96 (40% split)
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
        canvas.drawString("2. Send /start to", rightX + 8, VIEW_A_BODY_Y + 46);
        canvas.drawString("   your Bot", rightX + 8, VIEW_A_BODY_Y + 58);
        canvas.drawString("3. User appears", rightX + 8, VIEW_A_BODY_Y + 70);
        canvas.drawString("   here live!", rightX + 8, VIEW_A_BODY_Y + 82);
    } else {
        const ContactChat& sel = g_chats[g_selectedChatIndex];

        // Contact header inside preview pane
        canvas.setTextColor(COL_ACCENT_CYAN);
        String headerName = ellipsizeText(canvas, sel.name, rightW - 12);
        canvas.drawString(headerName, rightX + 6, VIEW_A_BODY_Y + 6);

        canvas.setTextColor(COL_TEXT_SECONDARY);
        String handleStr = sel.username.isEmpty()
                           ? ("ID:" + String(sel.chatId))
                           : ("@" + sel.username);
        canvas.drawString(ellipsizeText(canvas, handleStr, rightW - 12), rightX + 6, VIEW_A_BODY_Y + 17);

        canvas.drawFastHLine(rightX + 6, VIEW_A_BODY_Y + 28, rightW - 12, COL_BORDER);

        // Last message snippet box
        canvas.setTextColor(COL_TEXT_SECONDARY);
        canvas.drawString("LAST MESSAGE:", rightX + 6, VIEW_A_BODY_Y + 33);

        if (sel.messages.empty()) {
            canvas.setTextColor(COL_TEXT_SECONDARY);
            canvas.drawString("(No messages)", rightX + 6, VIEW_A_BODY_Y + 48);
        } else {
            const ChatMessage& lastMsg = sel.messages.back();
            String prefix = lastMsg.isOutgoing ? "You: " : "";
            String fullSnippet = prefix + lastMsg.text;

            std::vector<String> snippetLines;
            wrapTextToLines(canvas, fullSnippet, rightW - 12, snippetLines);

            canvas.setTextColor(lastMsg.isOutgoing ? COL_TEXT_OUTGOING : COL_TEXT_PRIMARY);
            int maxPreviewLines = 4;
            for (int i = 0; i < static_cast<int>(snippetLines.size()) && i < maxPreviewLines; ++i) {
                String line = snippetLines[i];
                if (i == maxPreviewLines - 1 && static_cast<int>(snippetLines.size()) > maxPreviewLines) {
                    line = ellipsizeText(canvas, line + "..", rightW - 12);
                }
                canvas.drawString(line, rightX + 6, VIEW_A_BODY_Y + 46 + (i * 11));
            }
        }

        // Bottom hint bar inside preview pane
        canvas.drawFastHLine(rightX + 6, SCREEN_HEIGHT - 16, rightW - 12, COL_BORDER);
        canvas.setTextColor(COL_TEXT_SECONDARY);
        canvas.drawString("[TAB/ENT] Open  [;/.] Nav", rightX + 5, SCREEN_HEIGHT - 11);
    }

    canvas.pushSprite(0, 0);
}

/**
 * @brief Renders View B: Conversation Chat Thread
 *        - Header Bar (18px): Active contact's name with clean background bar.
 *        - Message History Body (102px): Alternating scrollable lines with word-wrapping.
 *          Left-aligned gray text for incoming, right-aligned blue text for outgoing.
 *        - Bottom Input Bar (15px): Real-time keyboard input & send status.
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
        String title = ellipsizeText(canvas, chat.name, 165);
        canvas.drawString(title, 15, 5);

        canvas.setTextColor(COL_TEXT_SECONDARY);
        canvas.drawString("[TAB] Hub", SCREEN_WIDTH - 58, 5);
    }

    // 2. Message History Body (Y = 18 to 119, Height = 102px)
    if (!g_chats.empty() && g_selectedChatIndex >= 0 && g_selectedChatIndex < static_cast<int>(g_chats.size())) {
        const ContactChat& chat = g_chats[g_selectedChatIndex];

        // Build wrapped lines across conversation history
        std::vector<WrappedMessageLine> allLines;
        const int maxBubbleTextWidth = SCREEN_WIDTH - 24; // 216px safe width inside 240px screen

        for (const auto& msg : chat.messages) {
            std::vector<String> msgLines;
            wrapTextToLines(canvas, msg.text, maxBubbleTextWidth, msgLines);
            for (size_t i = 0; i < msgLines.size(); ++i) {
                WrappedMessageLine wl;
                wl.isOutgoing       = msg.isOutgoing;
                wl.isFirstLineOfMsg = (i == 0);
                wl.lineText         = msgLines[i];
                allLines.push_back(wl);
            }
        }

        const int lineHeight = 12;
        const int visibleLineCount = VIEW_B_BODY_H / lineHeight; // 8 lines visible at once (96px)

        int totalLines = static_cast<int>(allLines.size());
        int maxScroll = (totalLines > visibleLineCount) ? (totalLines - visibleLineCount) : 0;
        if (g_threadScrollOffset > maxScroll) g_threadScrollOffset = maxScroll;
        if (g_threadScrollOffset < 0)         g_threadScrollOffset = 0;

        int startLine = totalLines - visibleLineCount - g_threadScrollOffset;
        if (startLine < 0) startLine = 0;

        if (totalLines == 0) {
            canvas.setTextColor(COL_TEXT_SECONDARY);
            canvas.drawString("No messages yet. Type below!", 36, VIEW_B_BODY_Y + 44);
        } else {
            for (int row = 0; row < visibleLineCount; ++row) {
                int lineIdx = startLine + row;
                if (lineIdx >= totalLines) break;

                const WrappedMessageLine& item = allLines[lineIdx];
                int rowY = VIEW_B_BODY_Y + 2 + (row * lineHeight);

                // Alternating subtle row background based on message direction and row parity
                uint16_t rowBg = item.isOutgoing
                                 ? ((row % 2 == 0) ? COL_BUBBLE_OUT_BG : COL_BG_DARK)
                                 : ((row % 2 == 0) ? COL_BUBBLE_IN_BG  : COL_BG_PANEL_LEFT);
                canvas.fillRect(2, rowY, SCREEN_WIDTH - 4, lineHeight - 1, rowBg);

                if (item.isOutgoing) {
                    // Right-aligned text (blue) for outgoing messages
                    canvas.setTextColor(COL_TEXT_OUTGOING);
                    int textW = canvas.textWidth(item.lineText);
                    int drawX = SCREEN_WIDTH - 6 - textW;
                    if (drawX < 6) drawX = 6;
                    canvas.drawString(item.lineText, drawX, rowY + 2);
                } else {
                    // Left-aligned text (gray) for incoming messages
                    canvas.setTextColor(COL_TEXT_INCOMING);
                    canvas.drawString(item.lineText, 6, rowY + 2);
                }
            }
        }
    }

    // 3. Dedicated 15px High Bottom Input Bar (Y = 120 to 135)
    uint16_t inputBg = g_isSendingMessage ? COL_BG_INPUT_BUSY : COL_BG_INPUT;
    canvas.fillRect(0, VIEW_B_INPUT_Y, SCREEN_WIDTH, VIEW_B_INPUT_H, inputBg);
    canvas.drawRect(0, VIEW_B_INPUT_Y, SCREEN_WIDTH, VIEW_B_INPUT_H, COL_BORDER);

    if (g_isSendingMessage) {
        canvas.setTextColor(COL_ACCENT_CYAN);
        canvas.drawString("Sending message...", 6, VIEW_B_INPUT_Y + 4);
    } else {
        canvas.setTextColor(COL_ACCENT_CYAN);
        canvas.drawString(">", 4, VIEW_B_INPUT_Y + 4);

        // Horizontal tail-scroll so long typed strings stay visible in the 15px bar
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
    if (!g_uiDirty) return;
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

    Keyboard_Class::KeysState status = M5Cardputer.Keyboard.keysState();

    // 1. Toggle between View A and View B using the 'Tab' key
    if (status.tab) {
        if (g_currentView == AppView::CHAT_LIST_HUB) {
            if (!g_chats.empty()) {
                g_currentView = AppView::CONVERSATION_THREAD;
                g_chats[g_selectedChatIndex].hasUnread = false;
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
                // Up
                if (g_selectedChatIndex > 0) {
                    --g_selectedChatIndex;
                    g_uiDirty = true;
                }
            } else if (c == '.' || c == 's' || c == 'S') {
                // Down
                if (g_selectedChatIndex + 1 < static_cast<int>(g_chats.size())) {
                    ++g_selectedChatIndex;
                    g_uiDirty = true;
                }
            }
        }

        if (status.enter && !g_chats.empty()) {
            g_currentView = AppView::CONVERSATION_THREAD;
            g_chats[g_selectedChatIndex].hasUnread = false;
            g_threadScrollOffset = 0;
            g_uiDirty = true;
        }
        return;
    }

    // 3. View B: Conversation Thread Input & Scrolling
    if (g_currentView == AppView::CONVERSATION_THREAD) {
        if (g_isSendingMessage) return; // Input frozen while sending

        // Fn + ';' or Fn + '.' scrolls message history vertically
        if (status.fn) {
            for (char c : status.word) {
                if (c == ';') {
                    g_threadScrollOffset += 2;
                    g_uiDirty = true;
                } else if (c == '.') {
                    g_threadScrollOffset -= 2;
                    if (g_threadScrollOffset < 0) g_threadScrollOffset = 0;
                    g_uiDirty = true;
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

            // Freeze input & render sending state immediately
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
                saveContactsToSD();
            } else {
                playErrorChime();
                g_statusBadge = "SEND FAIL";
                // Clear buffer as requested on Enter submission
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
    WiFi.setSleep(false); // Maximize responsiveness on ESP32-S3
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
        g_statusBadge = "ONLINE";
        renderBootStatus("Wi-Fi Connected!", "IP: " + WiFi.localIP().toString());
        delay(400);
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
    M5Cardputer.Display.setBrightness(180);

    // Initialize full-screen double-buffered sprite (240x135, 16-bit RGB565)
    canvas.setColorDepth(16);
    canvas.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT);

    renderBootStatus("Initializing MicroSD...", "Mounting FAT32 on GPIO 12");

    // Load /tg_config.json from MicroSD card; halt on error UI if missing/corrupt
    if (!initSDAndLoadConfig(g_config)) {
        while (true) {
            M5Cardputer.update();
            delay(100);
        }
    }

    // Restore cached contacts if available
    loadCachedContactsFromSD();

    // Configure TLS client for Telegram Bot API
    g_tlsClient.setInsecure();
    g_tlsClient.setTimeout(5);

    // Connect to Wi-Fi
    connectToWiFi();

    // Initial poll on startup
    renderBootStatus("Syncing Telegram Bot API...", "Fetching recent messages...");
    pollTelegramUpdates();
    g_lastPollTimeMs = millis();
    g_uiDirty = true;
    renderUI();
}

void loop() {
    // 1. Update physical keyboard matrix & hardware state continuously
    M5Cardputer.update();
    handlePhysicalKeyboard();

    // 2. Render UI immediately whenever state changes
    if (g_uiDirty) {
        renderUI();
    }

    // 3. Lightweight millis() timer to poll background updates every 5 seconds
    const unsigned long now = millis();
    if (!g_isSendingMessage && (now - g_lastPollTimeMs >= POLL_INTERVAL_MS)) {
        g_lastPollTimeMs = now;

        // Auto-reconnect Wi-Fi if disconnected
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
