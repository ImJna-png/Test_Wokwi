/* BME280 pour STM32 HAL : implementation independante.
 * Registres / compensation : Bosch, BST-BME280-DS001, chapitres 4, 5, 8.
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
 */
#include "bme280.h"
#include <stddef.h>
#include <math.h>

#define IO_TIMEOUT_MS 100u
#define STATUS_TIMEOUT_MS 100u

static bool fail(BME280_Handle *dev, const char *message)
{
    if (dev != NULL) dev->error = message;
    return false;
}

static bool read_bytes(BME280_Handle *dev, uint8_t reg,
                       uint8_t *bytes, uint16_t count)
{
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->address_hal, reg,
                         I2C_MEMADD_SIZE_8BIT, bytes, count,
                         IO_TIMEOUT_MS) != HAL_OK)
        return fail(dev, "Lecture I2C impossible");
    return true;
}

static bool write_byte(BME280_Handle *dev, uint8_t reg, uint8_t value)
{
    if (HAL_I2C_Mem_Write(dev->hi2c, dev->address_hal, reg,
                          I2C_MEMADD_SIZE_8BIT, &value, 1,
                          IO_TIMEOUT_MS) != HAL_OK)
        return fail(dev, "Ecriture I2C impossible");
    return true;
}

static uint16_t u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t s16(const uint8_t *p)
{
    uint16_t n = u16(p);
    return (int16_t)((n & 0x8000u) ? (int32_t)n - 65536 : (int32_t)n);
}

static int16_t s12(uint16_t n)
{
    return (int16_t)((n & 0x0800u) ? (int32_t)n - 4096 : (int32_t)n);
}

static bool wait_status_clear(BME280_Handle *dev, uint8_t mask)
{
    uint32_t start = HAL_GetTick();
    do {
        uint8_t status = 0;
        if (!read_bytes(dev, 0xF3, &status, 1)) return false;
        if ((status & mask) == 0) return true;
        HAL_Delay(1);
    } while ((uint32_t)(HAL_GetTick() - start) < STATUS_TIMEOUT_MS);
    return fail(dev, "Delai de conversion/calibration depasse");
}

static bool read_calibration(BME280_Handle *dev)
{
    uint8_t a[26], h[7];
    if (!read_bytes(dev, 0x88, a, sizeof a) ||
        !read_bytes(dev, 0xE1, h, sizeof h)) return false;

    BME280_CalibData *c = &dev->calib;
    c->dig_T1 = u16(a);
    c->dig_T2 = s16(a + 2);
    c->dig_T3 = s16(a + 4);
    c->dig_P1 = u16(a + 6);
    c->dig_P2 = s16(a + 8);
    c->dig_P3 = s16(a + 10);
    c->dig_P4 = s16(a + 12);
    c->dig_P5 = s16(a + 14);
    c->dig_P6 = s16(a + 16);
    c->dig_P7 = s16(a + 18);
    c->dig_P8 = s16(a + 20);
    c->dig_P9 = s16(a + 22);
    c->dig_H1 = a[25];
    c->dig_H2 = s16(h);
    c->dig_H3 = h[2];
    /* H4 / H5 sont signes sur 12 bits, et partagent le registre 0xE5. */
    c->dig_H4 = s12((uint16_t)(((uint16_t)h[3] << 4) | (h[4] & 15u)));
    c->dig_H5 = s12((uint16_t)(((uint16_t)h[5] << 4) | (h[4] >> 4)));
    c->dig_H6 = (int8_t)((h[6] & 128u) ? (int)h[6] - 256 : (int)h[6]);

    if (c->dig_T1 == 0 || c->dig_T1 == 0xFFFFu ||
        c->dig_P1 == 0 || c->dig_P1 == 0xFFFFu)
        return fail(dev, "Calibration invalide");
    return true;
}

bool BME280_Init(BME280_Handle *dev, I2C_HandleTypeDef *hi2c)
{
    if (dev == NULL) return false;
    dev->initialized = false;
    dev->hi2c = hi2c;
    dev->address_hal = 0;
    dev->t_fine = 0;
    if (hi2c == NULL) return fail(dev, "Bus I2C absent");

    bool found = false;
    for (uint8_t address = 0x76; address <= 0x77; ++address) {
        uint8_t id = 0;
        dev->address_hal = (uint16_t)(address << 1);
        if (read_bytes(dev, 0xD0, &id, 1) && id == BME280_CHIP_ID) {
            found = true;
            break;
        }
    }
    if (!found) return fail(dev, "BME280 non detecte a 0x76 ou 0x77 (ID attendu : 0x60)");

    if (!write_byte(dev, 0xE0, 0xB6)) return false;
    HAL_Delay(10);
    if (!wait_status_clear(dev, 0x01) || !read_calibration(dev)) return false;

    /* Mode sommeil, filtre IIR x4, humidite x1, temperature x2, pression x16. */
    if (!write_byte(dev, 0xF4, 0x00) ||
        !write_byte(dev, 0xF5, 0x08) ||
        !write_byte(dev, 0xF2, 0x01) ||
        !write_byte(dev, 0xF4, 0x54)) return false;

    dev->initialized = true;
    dev->error = "OK";
    return true;
}

bool BME280_Read(BME280_Handle *dev, BME280_Data *data)
{
    if (dev == NULL) return false;
    if (!dev->initialized) return fail(dev, "Capteur non initialise");
    if (data == NULL) return fail(dev, "Destination absente");
    if (!write_byte(dev, 0xF4, 0x55)) return false;

    /* Conversion T x2 / P x16 / H x1 : environ 46.1 ms au maximum.
     * Attendre avant le polling evite de lire les donnees precedentes
     * si measuring n'est pas encore passe a 1 juste apres l'ecriture.
     */
    HAL_Delay(50);
    if (!wait_status_clear(dev, 0x08)) return false;

    uint8_t raw[8];
    if (!read_bytes(dev, 0xF7, raw, sizeof raw)) return false;
    uint32_t rp = ((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | (raw[2] >> 4);
    uint32_t rt = ((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | (raw[5] >> 4);
    uint32_t rh = ((uint32_t)raw[6] << 8) | raw[7];
    if (rp == 0x80000u || rt == 0x80000u || rh == 0x8000u)
        return fail(dev, "Mesure brute indisponible");

    const BME280_CalibData *c = &dev->calib;
    /* Calcul en double pour eviter les decalages signes indefinis en C. */
    double delta = rt / 131072.0 - c->dig_T1 / 8192.0;
    double fine = (rt / 16384.0 - c->dig_T1 / 1024.0) * c->dig_T2
                + delta * delta * c->dig_T3;
    dev->t_fine = (int32_t)fine;
    double temperature = fine / 5120.0;

    double x = fine / 2.0 - 64000.0;
    double offset = x * x * c->dig_P6 / 32768.0 + x * c->dig_P5 * 2.0;
    offset = offset / 4.0 + c->dig_P4 * 65536.0;
    double divisor = (c->dig_P3 * x * x / 524288.0 + c->dig_P2 * x) / 524288.0;
    divisor = (1.0 + divisor / 32768.0) * c->dig_P1;
    if (divisor == 0.0) return fail(dev, "Compensation pression impossible");
    double pressure = (1048576.0 - rp - offset / 4096.0) * 6250.0 / divisor;
    pressure += (c->dig_P9 * pressure * pressure / 2147483648.0
                 + pressure * c->dig_P8 / 32768.0 + c->dig_P7) / 16.0;

    double ht = fine - 76800.0;
    double humidity = (rh - (c->dig_H4 * 64.0 + c->dig_H5 * ht / 16384.0))
                    * (c->dig_H2 / 65536.0
                    * (1.0 + c->dig_H6 * ht / 67108864.0
                    * (1.0 + c->dig_H3 * ht / 67108864.0)));
    humidity *= 1.0 - c->dig_H1 * humidity / 524288.0;
    if (!isfinite(temperature) || !isfinite(pressure) || !isfinite(humidity))
        return fail(dev, "Resultat non numerique");
    if (humidity < 0.0) humidity = 0.0;
    if (humidity > 100.0) humidity = 100.0;
    if (temperature < -40.0 || temperature > 85.0 ||
        pressure < 30000.0 || pressure > 110000.0)
        return fail(dev, "Mesure hors domaine du BME280");

    BME280_Data result = {(float)temperature, (float)humidity, (float)(pressure / 100.0)};
    *data = result;
    dev->error = "OK";
    return true;
}

uint8_t BME280_Address(const BME280_Handle *dev)
{
    return dev != NULL ? (uint8_t)(dev->address_hal >> 1) : 0;
}

const char *BME280_Error(const BME280_Handle *dev)
{
    return dev != NULL && dev->error != NULL ? dev->error : "Capteur non initialise";
}
