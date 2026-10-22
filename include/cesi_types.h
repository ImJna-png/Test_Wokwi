#ifndef CESI_TYPES_H
#define CESI_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// Énumération des 4 modes de fonctionnement
typedef enum {
    MODE_STANDARD = 0,
    MODE_CONFIG,
    MODE_MAINTENANCE,
    MODE_ECO
} SystemMode;

// Énumération des états d'erreur pour le pilotage de la LED RGB
typedef enum {
    SYS_OK = 0,
    ERR_RTC,
    ERR_GPS,
    ERR_SENSOR,
    ERR_INCOHERENT_DATA,
    ERR_SD_FULL,
    ERR_SD_ACCESS
} SystemError;

// Structure regroupant l'ensemble des données mesurées par les capteurs
typedef struct {
    float temperature_c;
    float humidity_percent;
    float pressure_hpa;
    float gas_resistance_ohm;
} Env_Data;

// Structure de configuration globale (paramètres modifiables en Mode Configuration)
typedef struct {
    uint16_t log_interval_s;     // Intervalle entre 2 mesures en secondes (ex: 10 s)
    uint32_t file_max_size;      // Taille max d'un fichier log en octets (ex: 2048)
    uint16_t timeout_sec;        // Timeout d'acquisition capteur en secondes (ex: 30)

    // Paramètres Luminosité
    uint8_t  lumin_active;
    uint16_t lumin_low;
    uint16_t lumin_high;

    // Paramètres Température Air
    uint8_t lumin_temp_air_active; // 1 = actif, 0 = inactif
    int8_t  min_temp_air;
    int8_t  max_temp_air;

    // Paramètres Hygrométrie
    uint8_t  hygr_active;
    int8_t   hygr_mint;
    int8_t   hygr_maxt;

    // Paramètres Pression Atmosphérique
    uint8_t  pressure_active;
    uint16_t pressure_min;
    uint16_t pressure_max;
} SystemConfig;

/* Types necessaires aux pilotes RTC et GPS deja valides sur la carte. */
typedef struct {
    uint8_t seconds, minutes, hours, dayOfWeek, day, month;
    uint16_t year;
} RTC_DateTime;
typedef struct {
    bool valid;
    char raw_sentence[96];
    double latitude, longitude;
    uint8_t fix_quality, satellites;
} GPS_Data;

#endif // CESI_TYPES_H