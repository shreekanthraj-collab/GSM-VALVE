#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* EOL Web Server                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Local EOL web interface.
 *
 * The EOL web server provides a bench/factory interface through
 * the ESP32-S3 Wi-Fi SoftAP.
 *
 * Default address:
 *
 *     http://192.168.4.1/
 *
 * The web layer does not implement EOL tests itself.
 * It obtains EOL information from the EOL manager and
 * EOL persistence manager.
 */


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#define EOL_WEB_AP_SSID_MAX_LEN       32U
#define EOL_WEB_AP_PASSWORD_MAX_LEN   64U

#define EOL_WEB_DEFAULT_AP_IP         "192.168.4.1"

#define EOL_WEB_DEFAULT_SSID_PREFIX   "GSM-VALVE-EOL"


/* -------------------------------------------------------------------------- */
/* Web server configuration                                                   */
/* -------------------------------------------------------------------------- */

typedef struct
{
    /*
     * Wi-Fi SoftAP SSID.
     *
     * If empty, the implementation may generate:
     *
     *     GSM-VALVE-EOL-XXXX
     */
    char ssid[EOL_WEB_AP_SSID_MAX_LEN];

    /*
     * SoftAP password.
     *
     * For the initial bench implementation this may be empty,
     * which creates an open local EOL network.
     *
     * Production security policy will be added separately.
     */
    char password[EOL_WEB_AP_PASSWORD_MAX_LEN];

    /*
     * Maximum number of connected stations.
     */
    uint8_t max_connections;

    /*
     * Start the HTTP server immediately after Wi-Fi AP startup.
     */
    bool start_http_server;

} eol_web_config_t;


/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                  */
/* -------------------------------------------------------------------------- */

/*
 * Initialize the EOL web subsystem.
 *
 * This does not start Wi-Fi or the HTTP server.
 */
esp_err_t eol_web_init(
    const eol_web_config_t *config);


/*
 * Start the EOL local Wi-Fi access point and HTTP server.
 */
esp_err_t eol_web_start(void);


/*
 * Stop the HTTP server and Wi-Fi SoftAP.
 */
esp_err_t eol_web_stop(void);


/*
 * Deinitialize the EOL web subsystem.
 */
esp_err_t eol_web_deinit(void);


/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

/*
 * Return true when the EOL web subsystem has been initialized.
 */
bool eol_web_is_initialized(void);


/*
 * Return true when the local EOL Wi-Fi AP is running.
 */
bool eol_web_is_ap_running(void);


/*
 * Return true when the EOL HTTP server is running.
 */
bool eol_web_is_http_server_running(void);


/* -------------------------------------------------------------------------- */
/* Manual EOL control                                                         */
/* -------------------------------------------------------------------------- */

/*
 * Request an EOL run from the local web interface.
 *
 * The web layer should request the operation through the
 * application/EOL control path rather than directly accessing
 * hardware.
 */
esp_err_t eol_web_request_eol_run(void);


/*
 * Request the current EOL status to be refreshed.
 */
esp_err_t eol_web_request_refresh(void);


/* -------------------------------------------------------------------------- */
/* Local address                                                              */
/* -------------------------------------------------------------------------- */

/*
 * Get the local EOL web server IPv4 address.
 *
 * Example:
 *
 *     192.168.4.1
 */
const char *eol_web_get_ip_address(void);


/*
 * Get the currently configured SoftAP SSID.
 */
const char *eol_web_get_ssid(void);


/* -------------------------------------------------------------------------- */
/* Diagnostic                                                                */
/* -------------------------------------------------------------------------- */

/*
 * Print the EOL web server state to the ESP-IDF log.
 */
void eol_web_log_status(void);


#ifdef __cplusplus
}
#endif