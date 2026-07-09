#pragma once

#include <cstdint>
#include <string>
#include <stdexcept>

// Підключаємо специфічні залежності залежно від платформи
#if defined(USE_FT4232H)
    #include "FT4232H_I2C.hpp"
#endif


/**
 * PCA9685 — 16-канальний PWM-контролер на I2C
 *
 * Використовує Linux I2C (/dev/i2c-N) через ioctl.
 * Розрахований для Raspberry Pi (будь-яка ревізія).
 *
 * Типове підключення:
 *   SDA → GPIO 2  (pin 3)
 *   SCL → GPIO 3  (pin 5)
 *   VCC → 3.3V або 5V
 *   OE  → GND (активно увімкнено)
 */
class PCA9685 {
public:
    // ── Константи ────────────────────────────────────────────────
    static constexpr uint8_t  DEFAULT_ADDRESS = 0x40;
    static constexpr float    OSC_CLOCK_HZ    = 25'000'000.0f;
    static constexpr int      STEPS           = 4096;   // 12-бітна роздільність
    static constexpr float    MIN_FREQ_HZ     = 24.0f;
    static constexpr float    MAX_FREQ_HZ     = 1526.0f;

    // ── Конструктор / деструктор ──────────────────────────────────

    /**
     * @param bus      Номер I2C-шини (1 для більшості Pi)
     * @param address  I2C-адреса пристрою (0x40–0x7F)
     */
#if defined(USE_FT4232H)
    explicit PCA9685(FT4232H_I2C& i2c_bus, uint8_t address = DEFAULT_ADDRESS);
    ~PCA9685();
#else
    static constexpr int      DEFAULT_BUS     = 1;      // /dev/i2c-1
    explicit PCA9685(int bus = DEFAULT_BUS, uint8_t address = DEFAULT_ADDRESS);
    ~PCA9685();

    // Заборонити копіювання (RAII-ресурс)
    PCA9685(const PCA9685&)            = delete;
    PCA9685& operator=(const PCA9685&) = delete;

    // Дозволити переміщення тільки для системного I2C
    PCA9685(PCA9685&& other) noexcept;
    PCA9685& operator=(PCA9685&& other) noexcept;
#endif

    // ── Ініціалізація ─────────────────────────────────────────────

    /**
     * Відкрити шину і скинути пристрій до початкового стану.
     * Викидає std::runtime_error при невдачі.
     */
    void open();

    /** Закрити файловий дескриптор. */
    void close() noexcept;

    /** Апаратний скид (SWRST через I2C General Call). */
    void reset();

    // ── Частота PWM ───────────────────────────────────────────────

    /**
     * Встановити частоту PWM для всіх каналів.
     * @param freqHz  24 – 1526 Гц
     */
    void setFrequency(float freqHz);

    /** Повернути реально встановлену частоту (Гц). */
    float getFrequency() const noexcept { return m_freqHz; }

    // ── Управління каналами ───────────────────────────────────────

    /**
     * Встановити ON/OFF-відліки безпосередньо (0–4095).
     * @param channel  0–15
     * @param on       Відлік початку HIGH (фаза)
     * @param off      Відлік кінця HIGH
     */
    void setChannel(uint8_t channel, uint16_t on, uint16_t off);

    /**
     * Встановити скважність за допомогою значення 0–4095.
     * on-відлік = 0; off-відлік = value.
     * @param channel  0–15
     * @param value    0 (завжди LOW) … 4095 (майже завжди HIGH)
     */
    void setPWM(uint8_t channel, uint16_t value);

    /**
     * Встановити скважність у відсотках.
     * @param channel     0–15
     * @param dutyCyclePc 0.0 – 100.0 %
     */
    void setDutyCycle(uint8_t channel, float dutyCyclePc);

    /**
     * Встановити ширину імпульсу в мікросекундах.
     *
     * Зручно для точного керування сервоприводами або будь-яким
     * пристроєм, специфікація якого задана в мкс.
     *
     * Приклад (серво SG90, freq = 50 Гц):
     *   setPulseWidth(0, 500);   // ~0°
     *   setPulseWidth(0, 1500);  // ~90°
     *   setPulseWidth(0, 2500);  // ~180°
     *
     * @param channel  0–15
     * @param pulseUs  Ширина імпульсу в мікросекундах.
     *                 Обрізається до [0 … 1 000 000 / freqHz].
     */
    void setPulseWidth(uint8_t channel, float pulseUs);

    /**
     * Встановити кут сервоприводу.
     * @param channel    0–15
     * @param angleDeg   мінАнгл … максАнгл (за замовчуванням 0–180°)
     * @param minPulseUs Мінімальна ширина імпульсу, мкс (типово 500)
     * @param maxPulseUs Максимальна ширина імпульсу, мкс (типово 2500)
     * @param minAngle   Мінімальний кут (за замовчуванням 0)
     * @param maxAngle   Максимальний кут (за замовчуванням 180)
     */
    void setServoAngle(uint8_t channel, float angleDeg,
                       float minPulseUs = 500.0f,
                       float maxPulseUs = 2500.0f,
                       float minAngle   = 0.0f,
                       float maxAngle   = 180.0f);

    /** Примусово LOW (вихід вимкнено). */
    void setChannelOff(uint8_t channel);

    /** Примусово HIGH (вихід увімкнено повністю). */
    void setChannelOn(uint8_t channel);

    /**
     * Встановити всі 16 каналів одночасно через ALL_LED-регістри.
     * @param on   0–4095
     * @param off  0–4095
     */
    void setAllChannels(uint16_t on, uint16_t off);

    /** Вимкнути всі канали. */
    void allOff();

    // ── Режим сну / пробудження ───────────────────────────────────

    /** Перейти у сплячий режим (зупиняє осцилятор). */
    void sleep();

    /** Вийти зі сплячого режиму і зачекати стабілізацію генератора. */
    void wakeUp();

    // ── Утиліти ───────────────────────────────────────────────────

    /** Прочитати регістр MODE1. */
    uint8_t getMode1() const;

    /** Прочитати регістр MODE2. */
    uint8_t getMode2() const;

    /** true якщо шина відкрита і пристрій ініціалізовано. */
    bool isOpen() const noexcept;

private:
    // ── Регістри PCA9685 ──────────────────────────────────────────
    static constexpr uint8_t REG_MODE1      = 0x00;
    static constexpr uint8_t REG_MODE2      = 0x01;
    static constexpr uint8_t REG_LED0_ON_L  = 0x06;
    static constexpr uint8_t REG_ALL_ON_L   = 0xFA;
    static constexpr uint8_t REG_PRE_SCALE  = 0xFE;

    // Біти MODE1
    static constexpr uint8_t MODE1_RESTART = 0x80;
    static constexpr uint8_t MODE1_EXTCLK  = 0x40;
    static constexpr uint8_t MODE1_AI      = 0x20; // auto-increment
    static constexpr uint8_t MODE1_SLEEP   = 0x10;
    static constexpr uint8_t MODE1_ALLCALL = 0x01;

    // Біти MODE2
    static constexpr uint8_t MODE2_INVRT   = 0x10;
    static constexpr uint8_t MODE2_OCH     = 0x08;
    static constexpr uint8_t MODE2_OUTDRV  = 0x04;

    // ── Стан ─────────────────────────────────────────────────────
#if defined(USE_FT4232H)
    FT4232H_I2C& m_i2c;
#else
    int     m_fd      = -1;
    int     m_bus;
#endif
    uint8_t m_address;
    float   m_freqHz  = 50.0f;

    // ── Низькорівневий I2C ────────────────────────────────────────
    void     writeReg(uint8_t reg, uint8_t value) const;
    uint8_t  readReg(uint8_t reg) const;

    void     validateChannel(uint8_t channel) const;
    uint8_t  calcPrescale(float freqHz) const;
};