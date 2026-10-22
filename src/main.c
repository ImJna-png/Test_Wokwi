#include "setup.h"
#include "board.h"
#include "sd_logger.h"
#include "grove_button.h"
#include "bme280.h"
#include "grove_rtc_ds1307.h"
#include "grove_light.h"
#include "grove_gps_air530z.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
static BME280_Handle bme;
static BME280_Data env;
SystemMode g_current_mode=MODE_STANDARD;
SystemError g_current_error=SYS_OK;
SystemConfig g_config={
    .log_interval_min=10, .file_max_size=2048, .timeout_sec=30,
    .lumin_active=1, .lumin_low=255, .lumin_high=768
};
uint32_t g_last_log_tick;
bool g_gps_skip_flag;
static bool bme_ready;
static bool sd_ready;


/** Reprend le test SD de main2.c, sans dupliquer HAL, horloge, UART ou boucle principale. */
static void initialiser_sd(void)
{
    printf("\r\n=== Debut du test SD Logger ===\r\n");
    sd_ready=SDLogger_Init();
    printf("Label CS actif : %s\r\n",SDLogger_GetActiveCsLabel());
    if (sd_ready) {
        SDLoggerCardType type=SDLogger_GetCardType();
        printf("Succes de l'initialisation de la carte SD !\r\n");
        printf("Type de carte : %s\r\n",type==SDLOGGER_CARD_SDHC ? "SDHC" : type==SDLOGGER_CARD_SDSC ? "SDSC" : "Inconnu");
    } else {
        printf("Echec initialisation SD (code : %d)\r\n",(int)SDLogger_GetLastInitStatus());
        printf("--- Diagnostic Bit-Bang du main SD original ---\r\n");
        (void)SDLogger_BitBangCMD0Test();
        /* CMD0 seul ne valide pas une initialisation complete ni une ecriture. */
    }
}

/** @brief Affiche la date lue ou un avertissement ; aucune date de remplacement n'est inventee. */
static void afficher_date(const RTC_DateTime *dt, bool valide)
{
    if (valide) {
        printf("%02u/%02u/%04u %02u:%02u:%02u | ",
               (unsigned)dt->day, (unsigned)dt->month, (unsigned)dt->year,
               (unsigned)dt->hours, (unsigned)dt->minutes, (unsigned)dt->seconds);
    } else printf("[RTC indisponible ou date invalide] | ");
}

/** @brief Affiche temperature, humidite et pression avec un arrondi a deux decimales. */
static void afficher_mesures(void)
{
    /* Meme affichage entier que le test BME280 valide sur la carte. */
    long t = (long)(env.temperature_c * 100.0f + (env.temperature_c < 0 ? -0.5f : 0.5f));
    long h = (long)(env.humidity_percent * 100.0f + 0.5f);
    long p = (long)(env.pressure_hpa * 100.0f + 0.5f);
    long magnitude = t < 0 ? -t : t;
    printf("Temp: %s%ld.%02ld C | Hum: %ld.%02ld %% | Pression: %ld.%02ld hPa\r\n",
           t < 0 ? "-" : "", magnitude / 100, magnitude % 100,
           h / 100, h % 100, p / 100, p % 100);
}

/** @brief Controle le calendrier 2000-2099 et les bornes horaires avant une commande SET. */
static bool valid_date(unsigned y, unsigned m, unsigned d,
                       unsigned h, unsigned min, unsigned s)
{
    static const unsigned days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (y < 2000 || y > 2099 || m < 1 || m > 12 || h > 23 || min > 59 || s > 59)
        return false;
    unsigned last = days[m - 1] + (m == 2 && y % 4 == 0);
    return d >= 1 && d <= last;
}
/** @brief Calcule le jour de semaine attendu par le pilote RTC : lundi=1, dimanche=7. */
static unsigned weekday(unsigned y, unsigned m, unsigned d)
{
    static const unsigned t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
    if (m < 3) --y;
    unsigned w = (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
    return w == 0 ? 7 : w; /* lundi=1, dimanche=7 */
}

/* Reception UART par interruption : conserve les caracteres pendant
 * les lectures I2C bloquantes du BME280 et de la RTC. */
#define RX_SIZE 128U
static volatile uint8_t rx_buffer[RX_SIZE];
static volatile unsigned rx_head, rx_tail;
static volatile bool rx_fault, rx_restart;
static uint8_t rx_byte;
static char line[64];
static unsigned line_size;
static bool discard_line;

/** @brief Transmet l'interruption de la console au gestionnaire UART de la HAL. */
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }

/** @brief Oriente GPS et console ; range un octet puis rearme la reception, sans printf ni I2C. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart4) { GroveGPS_RxComplete(uart); return; }
    if (uart != &huart2) return;
    unsigned next = (rx_head + 1U) % RX_SIZE;
    if (next == rx_tail) rx_fault = true;
    else { rx_buffer[rx_head] = rx_byte; rx_head = next; }
    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1) != HAL_OK)
        rx_restart = true;
}

/** @brief Signale la perte de caracteres et tente de relancer la reception du bon UART. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart4) { GroveGPS_RxError(uart); return; }
    if (uart != &huart2) return;
    rx_fault = true;
    HAL_UART_AbortReceive(&huart2);
    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1) != HAL_OK)
        rx_restart = true;
}

/** @brief Lit un entier positif et rejette caracteres parasites et depassement de capacite. */
static bool parse_uint(const char *text,uint32_t max,uint32_t *out)
{
    if (!text || !*text) return false;
    uint32_t value=0;
    for (const char *p=text; *p; ++p) {
        if (*p<'0' || *p>'9') return false;
        uint32_t digit=(uint32_t)(*p-'0');
        if (value>max/10U || (value==max/10U && digit>max%10U)) return false;
        value=value*10U+digit;
    }
    *out=value; return true;
}

/** @brief Affiche mode, configuration, boutons, stockage et toutes les erreurs encore actives. */
static void afficher_statut(void)
{
    printf("Mode=%s | erreur=%u | LOG_INTERVAL=%u min | FILE_MAX_SIZE=%lu\r\n",
           Board_ModeName(g_current_mode),(unsigned)g_current_error,
           (unsigned)g_config.log_interval_min,(unsigned long)g_config.file_max_size);
    printf("Bouton1 VERT=%u | Bouton2 ROUGE=%u (1=appuye)\r\n",
           (unsigned)GroveButton1_IsPressed(),(unsigned)GroveButton2_IsPressed());
    printf("SD : %s | CS=%s | sauvegarde dans AAMMJJ_0.LOG en STANDARD et ECO.\r\n",
           sd_ready ? "initialisee" : "echec initialisation",SDLogger_GetActiveCsLabel());
}

/** @brief Valide puis execute les commandes serie ; les reglages du groupe exigent CONFIGURATION. */
static void traiter_commande(const char *texte)
{
    Board_NotifyActivity();
    if (strcmp(texte,"HELP")==0) {
        printf("HELP | STATUS | SET AAAA-MM-JJ HH:MM:SS\r\n");
        printf("En CONFIG : LOG_INTERVAL=minutes | FILE_MAX_SIZE=octets | RESET | VERSION\r\n");
        printf("Vert 5 s depuis STANDARD : ECO. Rouge 5 s : STANDARD/MAINTENANCE, ECO->STANDARD.\r\n");
        printf("Rouge relache apres 0.8 a moins de 5 s depuis STANDARD : CONFIG.\r\n");
        return;
    }
    if (strcmp(texte,"STATUS")==0) { afficher_statut(); return; }
    bool config_command=strncmp(texte,"LOG_INTERVAL=",13)==0 ||
                        strncmp(texte,"FILE_MAX_SIZE=",14)==0 ||
                        strcmp(texte,"VERSION")==0 || strcmp(texte,"RESET")==0;
    if (config_command) {
        if (g_current_mode!=MODE_CONFIG) {
            printf("Commande reservee au mode CONFIGURATION.\r\n"); return;
        }
        uint32_t value=0;
        if (strncmp(texte,"LOG_INTERVAL=",13)==0) {
            if (!parse_uint(texte+13,1440U,&value) || value==0)
                printf("ERREUR: LOG_INTERVAL doit etre entre 1 et 1440 minutes.\r\n");
            else { g_config.log_interval_min=(uint16_t)value; printf("OK: LOG_INTERVAL configure a %u minutes\r\n",(unsigned)value); }
        } else if (strncmp(texte,"FILE_MAX_SIZE=",14)==0) {
            if (!parse_uint(texte+14,UINT32_MAX,&value) || value==0)
                printf("ERREUR: FILE_MAX_SIZE doit etre un entier positif sur 32 bits.\r\n");
            else {
                g_config.file_max_size=value;
                printf("OK: FILE_MAX_SIZE=%lu octets (pas encore applique a l'ecriture SD).\r\n",(unsigned long)value);
            }
        } else if (strcmp(texte,"VERSION")==0) {
            printf("AIVM Station Météo embarquée v1.0 - Lot #2026-A\r\n");
        } else {
            g_config.log_interval_min=10; g_config.file_max_size=2048;
            printf("OK: Parametres reinitialises par defaut.\r\n");
        }
        return;
    }
    unsigned y,m,d,h,min,s;
    char extra;
    if (sscanf(texte, "SET %4u-%2u-%2u %2u:%2u:%2u %c",
               &y,&m,&d,&h,&min,&s,&extra) != 6 || !valid_date(y,m,d,h,min,s)) {
        printf("Commande invalide. Format : SET AAAA-MM-JJ HH:MM:SS\r\n");
        return;
    }
    RTC_DateTime dt = {0};
    dt.year=y; dt.month=m; dt.day=d; dt.dayOfWeek=weekday(y,m,d);
    dt.hours=h; dt.minutes=min; dt.seconds=s;
    if (!GroveRTC_SetDateTime(&dt)) {
        printf("Echec du reglage RTC | erreur HAL=0x%08lX\r\n",
               (unsigned long)HAL_I2C_GetError(&hi2c1));
        return;
    }
    RTC_DateTime relue = {0};
    if (GroveRTC_GetDateTime(&relue)) {
        afficher_date(&relue, true);
        printf("RTC reglee.\r\n");
    } else printf("Reglage envoye mais relecture RTC impossible.\r\n");
}

/** @brief Reconstruit les lignes de commande hors interruption et abandonne une ligne incomplete. */
static void traiter_reception(void)
{
    /* Sections critiques courtes : aucun printf ni I2C en interruption. */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool faute = rx_fault;
    if (faute) { rx_tail=rx_head; rx_fault=false; }
    if (rx_restart && HAL_UART_Receive_IT(&huart2, &rx_byte, 1) == HAL_OK)
        rx_restart=false;
    __set_PRIMASK(mask);
    if (faute) {
        line_size=0; discard_line=true;
        printf("Reception serie interrompue : Entree, puis renvoyer la commande.\r\n");
    }
    while (rx_tail != rx_head) {
        uint8_t ch=rx_buffer[rx_tail];
        Board_NotifyActivity();
        rx_tail=(rx_tail+1U)%RX_SIZE;
        if (ch=='\r' || ch=='\n') {
            if (discard_line) printf("Commande abandonnee : recommencer.\r\n");
            else if (line_size) { line[line_size]='\0'; traiter_commande(line); }
            line_size=0; discard_line=false;
        } else if (ch==8 || ch==127) {
            if (line_size && !discard_line) --line_size;
        } else if (ch>=32 && !discard_line) {
            if (line_size < sizeof(line)-1) line[line_size++]=(char)ch;
            else discard_line=true;
        }
    }
}

/* Acquisition GPS bornee et non bloquante : les boutons, SET et les LED
 * restent traites pendant le timeout de 30 s du programme du groupe. */
typedef struct {
    RTC_DateTime dt;
    GPS_Data gps;
    BME280_Data env;
    uint16_t light;
    bool rtc_ok, bme_ok, light_active, gps_requested, gps_received;
    uint32_t started;
    SystemMode mode;
} Sample;
static Sample pending;
static bool waiting;

/** @brief Acquiert BME280, RTC et ADC ; chaque donnee garde son indicateur de validite. */
static void read_sensors(Sample *s,SystemMode mode)
{
    memset(s,0,sizeof *s);
    s->mode=mode;
    if (!bme_ready) bme_ready=BME280_Init(&bme,&hi2c1);
    if (bme_ready) {
        s->bme_ok=BME280_Read(&bme,&s->env);
        if (!s->bme_ok) bme_ready=false;
    }
    s->rtc_ok=GroveRTC_GetDateTime(&s->dt);
    s->light_active=g_config.lumin_active!=0;
    s->light=s->light_active ? GroveLight_ReadRaw() : 0;
    s->started=HAL_GetTick();
}
/** @brief Affiche le resultat reel de l'acquisition et distingue absence de GGA et absence de position. */
static void print_sample(const Sample *s)
{
    printf("[%s] ",Board_ModeName(s->mode));
    afficher_date(&s->dt,s->rtc_ok);
    if (s->bme_ok) { env=s->env; afficher_mesures(); }
    else printf("BME280 indisponible : %s\r\n",BME280_Error(&bme));
    if (s->light_active) {
        if (s->light==0xFFFFU) printf("Lumiere : erreur ADC\r\n");
        else printf("Lumiere : %u / 4095 (ADC brut)\r\n",(unsigned)s->light);
    }
    if (s->mode!=MODE_MAINTENANCE) {
        if (!s->gps_requested) printf("GPS : acquisition sautee volontairement (ECO, 1 sur 2).\r\n");
        else if (!s->gps_received) printf("GPS : delai depasse, aucune GGA recue pour cette acquisition.\r\n");
        else if (!s->gps.valid) {
            printf("GPS : recherche de position | satellites=%u | qualite=%u\r\n",(unsigned)s->gps.satellites,(unsigned)s->gps.fix_quality);
            printf("GPS trame brute : %s\r\n",s->gps.raw_sentence); /* diagnostic : 7e champ = qualite du fix */
        }
        else printf("GPS : latitude=%.6f | longitude=%.6f | satellites=%u | qualite=%u\r\n",s->gps.latitude,s->gps.longitude,(unsigned)s->gps.satellites,(unsigned)s->gps.fix_quality);
    }
}
/** @brief Ajoute une ligne CSV dans AAMMJJ_0.LOG ; "NA" pour une valeur absente. */
static bool save_sample(const Sample *s)
{
    char name[20], date[40], bme[40], light[8], gps[32], row[128];
    if (s->rtc_ok) {
        snprintf(name,sizeof name,"%02u%02u%02u_0.LOG",(unsigned)(s->dt.year%100U),(unsigned)s->dt.month,(unsigned)s->dt.day);
        snprintf(date,sizeof date,"%02u/%02u/%04u;%02u:%02u:%02u",(unsigned)s->dt.day,(unsigned)s->dt.month,(unsigned)s->dt.year,
                 (unsigned)s->dt.hours,(unsigned)s->dt.minutes,(unsigned)s->dt.seconds);
    } else {
        snprintf(name,sizeof name,"NODATE_0.LOG");
        snprintf(date,sizeof date,"NA;NA");
    }
    if (s->bme_ok) snprintf(bme,sizeof bme,"%.2f;%.2f;%.2f",s->env.temperature_c,s->env.humidity_percent,s->env.pressure_hpa);
    else snprintf(bme,sizeof bme,"NA;NA;NA");
    if (s->light_active && s->light!=0xFFFFU) snprintf(light,sizeof light,"%u",(unsigned)s->light);
    else snprintf(light,sizeof light,"NA");
    if (s->gps_received && s->gps.valid) snprintf(gps,sizeof gps,"%.6f;%.6f",s->gps.latitude,s->gps.longitude);
    else snprintf(gps,sizeof gps,"NA;NA");
    snprintf(row,sizeof row,"%s;%s;%s;%s;%s\r\n",date,Board_ModeName(s->mode),bme,light,gps);
    bool ok=SDLogger_WriteLine(name,row);
    if (ok) printf("[SD] Ligne ajoutee dans %s\r\n",name);
    else printf("[SD] Echec d'ecriture dans %s\r\n",name);
    return ok;
}
/** Conserve la priorite RTC/capteur/GPS et signale un echec du test SD si ces capteurs sont disponibles. */
static void update_sample_error(const Sample *s)
{
    if (!s->rtc_ok) g_current_error=ERR_RTC;
    else if (!s->bme_ok || (s->light_active && s->light==0xFFFFU)) g_current_error=ERR_SENSOR;
    else if (s->mode==MODE_STANDARD && s->gps_requested && !s->gps_received) g_current_error=ERR_GPS;
    else if (!sd_ready) g_current_error=ERR_SD_ACCESS;
    else g_current_error=SYS_OK;
}
/** Termine l'acquisition : l'affiche puis l'enregistre sur la SD (STANDARD et ECO uniquement). */
static void finish_sample(void)
{
    update_sample_error(&pending); print_sample(&pending);
    if (sd_ready && !save_sample(&pending) && g_current_error==SYS_OK) g_current_error=ERR_SD_ACCESS;
    g_last_log_tick=HAL_GetTick(); waiting=false;
}
/** @brief Lance une acquisition ; en ECO, ne demande le GPS qu'une fois sur deux. */
static void begin_sample(SystemMode mode)
{
    read_sensors(&pending,mode);
    if (mode==MODE_ECO) {
        g_gps_skip_flag=!g_gps_skip_flag;
        pending.gps_requested=g_gps_skip_flag;
    } else pending.gps_requested=true;
    waiting=true;
    if (!pending.gps_requested || !GroveGPS_IsReady()) finish_sample();
}
/** @brief Termine a la nouvelle GGA ou au timeout, sans boucle d'attente de trente secondes. */
static void service_pending(bool fresh_gga,const GPS_Data *gps)
{
    if (!waiting) return;
    if (fresh_gga) { pending.gps=*gps; pending.gps_received=true; finish_sample(); }
    else if ((uint32_t)(HAL_GetTick()-pending.started)>=(uint32_t)g_config.timeout_sec*1000U)
        finish_sample();
}

/** @brief Initialise une seule fois la carte puis ordonnance modes, commandes, GPS et enregistrements. */
int main(void)
{
    Global_Init(); Board_Init();
    printf("\r\nSTATION - MODES DU GROUPE IBRAHIM + CAPTEURS VALIDES\r\n");
    printf("Bouton1=VERT ; Bouton2=ROUGE. HELP pour les commandes.\r\n");
    initialiser_sd();
    HAL_NVIC_SetPriority(USART2_IRQn,2,0); HAL_NVIC_EnableIRQ(USART2_IRQn);
    if (HAL_UART_Receive_IT(&huart2,&rx_byte,1)!=HAL_OK) rx_restart=true;
    bool rtc_detected=GroveRTC_Init();
    bme_ready=BME280_Init(&bme,&hi2c1);
    GroveLight_Init(); GroveGPS_Init();
    printf("RTC : %s a 0x68 | BME280 : %s a 0x%02X\r\n",rtc_detected ? "detectee" : "non detectee",bme_ready ? "initialise" : "indisponible",(unsigned)BME280_Address(&bme));
    printf("Lumiere ADC : %s | GPS UART4 : %s\r\n",GroveLight_IsReady() ? "initialise" : "erreur",GroveGPS_IsReady() ? "initialise, attente de trames" : "erreur");
#ifdef TEST_INTERVAL_S
    printf("MODE TEST : STANDARD toutes les %u s ; ECO : %u s. MAINTENANCE : affichage toutes les 2 s.\r\n",(unsigned)TEST_INTERVAL_S,(unsigned)TEST_INTERVAL_S*2U);
#else
    printf("STANDARD : 10 min par defaut ; ECO : 20 min. MAINTENANCE : affichage toutes les 2 s.\r\n");
#endif
    if (!rtc_detected) g_current_error=ERR_RTC;
    else if (!bme_ready || !GroveLight_IsReady()) g_current_error=ERR_SENSOR;
    else if (!sd_ready) g_current_error=ERR_SD_ACCESS;
    g_last_log_tick=HAL_GetTick();
    uint32_t last_maintenance=HAL_GetTick();
    SystemMode previous=(SystemMode)-1;
    GPS_Data latest={0};
    while (1) {
        Board_Update(); traiter_reception();
        bool fresh_gga=GroveGPS_Process(&latest);
        uint32_t now=HAL_GetTick();
        if (previous!=g_current_mode) {
            if (waiting) { waiting=false; g_last_log_tick=now; printf("Acquisition en attente annulee lors du changement de mode.\r\n"); }
            printf("[MODE] %s\r\n",Board_ModeName(g_current_mode));
            previous=g_current_mode; last_maintenance=now-2000U;
            if (g_current_mode==MODE_CONFIG) printf("LOG_INTERVAL=minutes | FILE_MAX_SIZE=octets | RESET | VERSION\r\n");
        }
        if (g_current_mode==MODE_MAINTENANCE) {
            if (!line_size && !discard_line && (uint32_t)(now-last_maintenance)>=2000U) {
                last_maintenance=now; Sample s;
                read_sensors(&s,MODE_MAINTENANCE); update_sample_error(&s); print_sample(&s);
            }
        } else if (g_current_mode==MODE_STANDARD || g_current_mode==MODE_ECO) {
            if (waiting) {
                service_pending(fresh_gga,&latest);
            } else {
#ifdef TEST_INTERVAL_S
                /* Mode test (platformio.ini) : une mesure toutes les TEST_INTERVAL_S secondes. */
                uint32_t interval=(uint32_t)TEST_INTERVAL_S*1000U;
#else
                uint32_t interval=(uint32_t)g_config.log_interval_min*60000U;
#endif
                if (g_current_mode==MODE_ECO) interval*=2U;
                if (!line_size && !discard_line && (uint32_t)(now-g_last_log_tick)>=interval)
                    begin_sample(g_current_mode);
            }
        }
        HAL_Delay(1);
    }
}
