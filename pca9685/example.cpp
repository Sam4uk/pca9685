// example.cpp — приклад використання PCA9685
// Компіляція:  g++ -std=c++17 example.cpp PCA9685.cpp -o example

#include "PCA9685.h"
#include <iostream>
#include <thread>
#include <chrono>

int main()
{
    try {
        // Відкрити шину i2c-1, адреса 0x40 (за замовчуванням)
        PCA9685 pca;
        pca.open();

        // ── Базове PWM (наприклад, підсвічування) ────────────────
        pca.setFrequency(1000.0f);   // 1 кГц
        pca.setDutyCycle(0, 75.0f);  // канал 0 → 75%
        pca.setPWM(1, 2048);         // канал 1 → 50% (2048/4096)

        // ── Сервопривід на каналі 2 ───────────────────────────────
        pca.setFrequency(50.0f);     // стандартні 50 Гц для сервоприводів

        for (float angle = 0; angle <= 180; angle += 10) {
            pca.setServoAngle(2, angle);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // ── Вимкнути всі канали ───────────────────────────────────
        pca.allOff();

        std::cout << "Реальна частота: " << pca.getFrequency() << " Гц\n";
        std::cout << "MODE1: 0x" << std::hex
                  << static_cast<int>(pca.getMode1()) << "\n";

    } catch (const std::exception& e) {
        std::cerr << "Помилка: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
