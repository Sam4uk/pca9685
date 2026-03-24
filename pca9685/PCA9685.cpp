#include "PCA9685.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include <cmath>
#include <cstring>
#include <thread>
#include <chrono>
#include <sstream>
#include <algorithm>

// ══════════════════════════════════════════════════════════════════════
//  Конструктор / деструктор
// ══════════════════════════════════════════════════════════════════════

PCA9685::PCA9685(int bus, uint8_t address)
    : m_address(address), m_bus(bus)
{}

PCA9685::~PCA9685()
{
    close();
}

PCA9685::PCA9685(PCA9685&& other) noexcept
    : m_fd(other.m_fd)
    , m_address(other.m_address)
    , m_bus(other.m_bus)
    , m_freqHz(other.m_freqHz)
{
    other.m_fd = -1;
}

PCA9685& PCA9685::operator=(PCA9685&& other) noexcept
{
    if (this != &other) {
        close();
        m_fd      = other.m_fd;
        m_address = other.m_address;
        m_bus     = other.m_bus;
        m_freqHz  = other.m_freqHz;
        other.m_fd = -1;
    }
    return *this;
}

// ══════════════════════════════════════════════════════════════════════
//  Ініціалізація
// ══════════════════════════════════════════════════════════════════════

void PCA9685::open()
{
    std::string path = "/dev/i2c-" + std::to_string(m_bus);

    m_fd = ::open(path.c_str(), O_RDWR);
    if (m_fd < 0) {
        throw std::runtime_error("PCA9685: не вдалося відкрити " + path +
                                 ": " + std::strerror(errno));
    }

    if (ioctl(m_fd, I2C_SLAVE, m_address) < 0) {
        ::close(m_fd);
        m_fd = -1;
        std::ostringstream oss;
        oss << "PCA9685: ioctl I2C_SLAVE 0x"
            << std::hex << static_cast<int>(m_address)
            << " помилка: " << std::strerror(errno);
        throw std::runtime_error(oss.str());
    }

    reset();
}

void PCA9685::close() noexcept
{
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
}

void PCA9685::reset()
{
    // Скинути MODE1: auto-increment + allcall, прибрати sleep
    writeReg(REG_MODE1, MODE1_AI | MODE1_ALLCALL);
    // Вихід: push-pull, оновлення при STOP
    writeReg(REG_MODE2, MODE2_OUTDRV);

    // Зачекати стабілізації (500 мкс за специфікацією)
    std::this_thread::sleep_for(std::chrono::microseconds(500));
}

// ══════════════════════════════════════════════════════════════════════
//  Частота PWM
// ══════════════════════════════════════════════════════════════════════

uint8_t PCA9685::calcPrescale(float freqHz) const
{
    freqHz = std::clamp(freqHz, MIN_FREQ_HZ, MAX_FREQ_HZ);
    // За формулою з даташиту: prescale = round(osc / (4096 * freq)) - 1
    float prescaleF = std::round(OSC_CLOCK_HZ / (static_cast<float>(STEPS) * freqHz)) - 1.0f;
    return static_cast<uint8_t>(std::clamp(prescaleF, 3.0f, 255.0f));
}

void PCA9685::setFrequency(float freqHz)
{
    uint8_t prescale = calcPrescale(freqHz);

    // Зміна PRE_SCALE дозволена тільки в режимі SLEEP
    uint8_t oldMode = readReg(REG_MODE1);
    uint8_t sleepMode = (oldMode & ~MODE1_RESTART) | MODE1_SLEEP;

    writeReg(REG_MODE1, sleepMode);
    writeReg(REG_PRE_SCALE, prescale);
    writeReg(REG_MODE1, oldMode);

    // Затримка ≥ 500 мкс для старту осцилятора
    std::this_thread::sleep_for(std::chrono::microseconds(500));

    // Відновити RESTART-біт, якщо він був встановлений
    writeReg(REG_MODE1, oldMode | MODE1_RESTART);

    // Зберегти реальну частоту
    m_freqHz = OSC_CLOCK_HZ / (static_cast<float>(STEPS) * (prescale + 1));
}

// ══════════════════════════════════════════════════════════════════════
//  Управління каналами
// ══════════════════════════════════════════════════════════════════════

void PCA9685::setChannel(uint8_t channel, uint16_t on, uint16_t off)
{
    validateChannel(channel);

    uint8_t base = REG_LED0_ON_L + channel * 4;

    // Записуємо 4 байти з auto-increment:
    // ON_L, ON_H, OFF_L, OFF_H
    uint8_t buf[5] = {
        base,
        static_cast<uint8_t>(on  & 0xFF),
        static_cast<uint8_t>(on  >> 8),
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>(off >> 8)
    };

    if (::write(m_fd, buf, sizeof(buf)) != sizeof(buf)) {
        throw std::runtime_error("PCA9685: помилка запису каналу " +
                                 std::to_string(channel));
    }
}

void PCA9685::setPWM(uint8_t channel, uint16_t value)
{
    value = std::min<uint16_t>(value, 4095u);
    setChannel(channel, 0, value);
}

void PCA9685::setDutyCycle(uint8_t channel, float dutyCyclePc)
{
    dutyCyclePc = std::clamp(dutyCyclePc, 0.0f, 100.0f);

    if (dutyCyclePc <= 0.0f) {
        setChannelOff(channel);
        return;
    }
    if (dutyCyclePc >= 100.0f) {
        setChannelOn(channel);
        return;
    }

    auto value = static_cast<uint16_t>(dutyCyclePc / 100.0f * (STEPS - 1));
    setChannel(channel, 0, value);
}

void PCA9685::setPulseWidth(uint8_t channel, float pulseUs)
{
    // Тривалість одного PWM-періоду в мікросекундах
    const float periodUs = 1'000'000.0f / m_freqHz;

    pulseUs = std::clamp(pulseUs, 0.0f, periodUs);

    // Кількість кроків = (pulseUs / periodUs) × 4096
    auto offStep = static_cast<uint16_t>(
        std::round(pulseUs / periodUs * static_cast<float>(STEPS - 1))
    );

    setChannel(channel, 0, offStep);
}

void PCA9685::setServoAngle(uint8_t channel, float angleDeg,
                             float minPulseUs, float maxPulseUs,
                             float minAngle,   float maxAngle)
{
    angleDeg = std::clamp(angleDeg, minAngle, maxAngle);

    // Ширина одного кроку в мікросекундах
    float usPerStep = (1'000'000.0f / m_freqHz) / static_cast<float>(STEPS);

    // Лінійно маппуємо кут → ширину імпульсу
    float pulseUs = minPulseUs +
                    (angleDeg - minAngle) / (maxAngle - minAngle) *
                    (maxPulseUs - minPulseUs);

    auto offStep = static_cast<uint16_t>(pulseUs / usPerStep);
    offStep = std::clamp<uint16_t>(offStep, 0, 4095);

    setChannel(channel, 0, offStep);
}

void PCA9685::setChannelOff(uint8_t channel)
{
    validateChannel(channel);
    // Bit 4 в OFF_H встановлює повний LOW
    uint8_t base = REG_LED0_ON_L + channel * 4;
    uint8_t buf[5] = { base, 0x00, 0x00, 0x00, 0x10 };
    if (::write(m_fd, buf, sizeof(buf)) != sizeof(buf))
        throw std::runtime_error("PCA9685: setChannelOff failed");
}

void PCA9685::setChannelOn(uint8_t channel)
{
    validateChannel(channel);
    // Bit 4 в ON_H встановлює повний HIGH
    uint8_t base = REG_LED0_ON_L + channel * 4;
    uint8_t buf[5] = { base, 0x00, 0x10, 0x00, 0x00 };
    if (::write(m_fd, buf, sizeof(buf)) != sizeof(buf))
        throw std::runtime_error("PCA9685: setChannelOn failed");
}

void PCA9685::setAllChannels(uint16_t on, uint16_t off)
{
    uint8_t buf[5] = {
        REG_ALL_ON_L,
        static_cast<uint8_t>(on  & 0xFF),
        static_cast<uint8_t>(on  >> 8),
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>(off >> 8)
    };
    if (::write(m_fd, buf, sizeof(buf)) != sizeof(buf))
        throw std::runtime_error("PCA9685: setAllChannels failed");
}

void PCA9685::allOff()
{
    setAllChannels(0, 0);
}

// ══════════════════════════════════════════════════════════════════════
//  Сон / пробудження
// ══════════════════════════════════════════════════════════════════════

void PCA9685::sleep()
{
    uint8_t mode = readReg(REG_MODE1);
    writeReg(REG_MODE1, mode | MODE1_SLEEP);
}

void PCA9685::wakeUp()
{
    uint8_t mode = readReg(REG_MODE1);
    writeReg(REG_MODE1, mode & ~MODE1_SLEEP);
    std::this_thread::sleep_for(std::chrono::microseconds(500));
}

// ══════════════════════════════════════════════════════════════════════
//  Утиліти
// ══════════════════════════════════════════════════════════════════════

uint8_t PCA9685::getMode1() const { return readReg(REG_MODE1); }
uint8_t PCA9685::getMode2() const { return readReg(REG_MODE2); }

// ══════════════════════════════════════════════════════════════════════
//  Приватні методи
// ══════════════════════════════════════════════════════════════════════

void PCA9685::writeReg(uint8_t reg, uint8_t value) const
{
    uint8_t buf[2] = { reg, value };
    if (::write(m_fd, buf, 2) != 2) {
        throw std::runtime_error("PCA9685: writeReg(0x" +
            std::to_string(reg) + ") помилка: " + std::strerror(errno));
    }
}

uint8_t PCA9685::readReg(uint8_t reg) const
{
    // Потрібен I2C repeated start, щоб не втратити адресу регістру.
    // Два повідомлення в одній атомарній транзакції:
    //   [WRITE addr+reg] → Sr → [READ 1 байт]
    uint8_t value = 0;

    struct i2c_msg msgs[2] = {
        // 1. Надіслати адресу регістру (без STOP)
        {
            .addr  = m_address,
            .flags = 0,
            .len   = 1,
            .buf   = &reg
        },
        // 2. Repeated start → зчитати 1 байт
        {
            .addr  = m_address,
            .flags = I2C_M_RD,
            .len   = 1,
            .buf   = &value
        }
    };

    struct i2c_rdwr_ioctl_data data = {
        .msgs  = msgs,
        .nmsgs = 2
    };

    if (ioctl(m_fd, I2C_RDWR, &data) < 0) {
        throw std::runtime_error(
            std::string("PCA9685: readReg(0x") +
            std::to_string(reg) + ") помилка: " + std::strerror(errno));
    }

    return value;
}

void PCA9685::validateChannel(uint8_t channel) const
{
    if (channel > 15)
        throw std::invalid_argument("PCA9685: канал " +
            std::to_string(channel) + " поза діапазоном 0–15");
}