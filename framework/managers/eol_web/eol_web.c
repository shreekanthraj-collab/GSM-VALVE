#include "eol_web.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"


/* -------------------------------------------------------------------------- */
/* Logging                                                                    */
/* -------------------------------------------------------------------------- */

static const char *TAG = "EOL_WEB";


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

typedef struct
{
    bool initialized;
    bool ap_running;
    bool http_server_running;

    eol_web_config_t config;

    httpd_handle_t http_server;

    esp_netif_t *ap_netif;

    bool eol_run_requested;
    bool refresh_requested;

    char ip_address[16];

} eol_web_context_t;


static eol_web_context_t s_ctx =
{
    .initialized = false,
    .ap_running = false,
    .http_server_running = false,

    .config = {{0}},

    .http_server = NULL,

    .ap_netif = NULL,

    .eol_run_requested = false,
    .refresh_requested = false,

    .ip_address = EOL_WEB_DEFAULT_AP_IP
};


/* -------------------------------------------------------------------------- */
/* HTML page                                                                  */
/* -------------------------------------------------------------------------- */

static const char *EOL_HTML_PAGE =
"<!DOCTYPE html>"
"<html>"
"<head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>GSM-VALVE EOL</title>"
"<style>"
"body{font-family:Arial,sans-serif;margin:0;background:#f2f2f2;color:#222}"
".header{background:#202020;color:white;padding:18px;text-align:center}"
".container{max-width:900px;margin:auto;padding:15px}"
".card{background:white;border-radius:10px;padding:18px;margin-bottom:15px;"
"box-shadow:0 2px 6px rgba(0,0,0,.12)}"
".status{font-size:22px;font-weight:bold;padding:10px;border-radius:6px}"
".pass{background:#d9f7df;color:#146b28}"
".fail{background:#ffdede;color:#9b111e}"
".warn{background:#fff1c7;color:#795900}"
".unknown{background:#e6e6e6;color:#555}"
"table{width:100%;border-collapse:collapse}"
"th,td{padding:10px;border-bottom:1px solid #ddd;text-align:left}"
"button{padding:12px 20px;border:0;border-radius:6px;"
"font-size:16px;cursor:pointer;margin-right:8px}"
".run{background:#1976d2;color:white}"
".refresh{background:#555;color:white}"
".mono{font-family:monospace}"
"</style>"
"</head>"
"<body>"
"<div class='header'>"
"<h1>GSM-VALVE</h1>"
"<div>EOL FACTORY DIAGNOSTICS</div>"
"</div>"

"<div class='container'>"

"<div class='card'>"
"<h2>Factory Verification</h2>"
"<div id='factoryStatus' class='status unknown'>Loading...</div>"
"</div>"

"<div class='card'>"
"<h2>EOL Tests</h2>"
"<table>"
"<tr><th>Test</th><th>Result</th><th>Value</th></tr>"
"<tbody id='tests'>"
"<tr><td colspan='3'>Loading...</td></tr>"
"</tbody>"
"</table>"
"</div>"

"<div class='card'>"
"<h2>Summary</h2>"
"<table>"
"<tr><td>Passed</td><td id='passed'>-</td></tr>"
"<tr><td>Failed</td><td id='failed'>-</td></tr>"
"<tr><td>Skipped</td><td id='skipped'>-</td></tr>"
"<tr><td>Not Run</td><td id='notrun'>-</td></tr>"
"</table>"
"</div>"

"<div class='card'>"
"<h2>Identity</h2>"
"<table>"
"<tr><td>Hardware Fingerprint</td>"
"<td id='hw' class='mono'>-</td></tr>"
"<tr><td>Firmware Fingerprint</td>"
"<td id='fw' class='mono'>-</td></tr>"
"<tr><td>Firmware</td><td id='version'>-</td></tr>"
"<tr><td>EOL Compatibility</td><td id='compat'>-</td></tr>"
"</table>"
"</div>"

"<div class='card'>"
"<button class='run' onclick='runEol()'>RUN EOL</button>"
"<button class='refresh' onclick='loadStatus()'>REFRESH</button>"
"<p id='message'></p>"
"</div>"

"</div>"

"<script>"

"function resultClass(r){"
" if(r==='PASS')return 'pass';"
" if(r==='FAIL')return 'fail';"
" if(r==='SKIP')return 'warn';"
" return 'unknown';"
"}"

"function loadStatus(){"
" fetch('/api/status')"
" .then(r=>r.json())"
" .then(d=>{"

"  let fs=document.getElementById('factoryStatus');"
"  fs.textContent=d.factory_status;"
"  fs.className='status '+resultClass(d.factory_class);"

"  document.getElementById('passed').textContent=d.tests_passed;"
"  document.getElementById('failed').textContent=d.tests_failed;"
"  document.getElementById('skipped').textContent=d.tests_skipped;"
"  document.getElementById('notrun').textContent=d.tests_not_run;"

"  document.getElementById('hw').textContent=d.hardware_fingerprint;"
"  document.getElementById('fw').textContent=d.firmware_fingerprint;"
"  document.getElementById('version').textContent=d.firmware_version;"
"  document.getElementById('compat').textContent=d.compatibility_version;"

"  let html='';"

"  d.tests.forEach(function(t){"
"   html += '<tr>';"
"   html += '<td>'+t.name+'</td>';"
"   html += '<td class=\"'+resultClass(t.result)+'\">'+t.result+'</td>';"
"   html += '<td>'+t.value+'</td>';"
"   html += '</tr>';"
"  });"

"  document.getElementById('tests').innerHTML=html;"
" })"
" .catch(e=>{"
"  document.getElementById('message').textContent="
"  'Status request failed: '+e;"
" });"
"}"

"function runEol(){"
" document.getElementById('message').textContent='EOL run requested...';"
" fetch('/api/run',{method:'POST'})"
" .then(r=>r.text())"
" .then(t=>{"
"  document.getElementById('message').textContent=t;"
"  setTimeout(loadStatus,1000);"
" })"
" .catch(e=>{"
"  document.getElementById('message').textContent="
"  'EOL request failed: '+e;"
" });"
"}"

"loadStatus();"
"setInterval(loadStatus,3000);"

"</script>"
"</body>"
"</html>";


/* -------------------------------------------------------------------------- */
/* HTTP handlers                                                              */
/* -------------------------------------------------------------------------- */

static esp_err_t eol_web_root_handler(
    httpd_req_t *req)
{
    httpd_resp_set_type(
        req,
        "text/html");

    return httpd_resp_send(
        req,
        EOL_HTML_PAGE,
        HTTPD_RESP_USE_STRLEN);
}


/* -------------------------------------------------------------------------- */

static esp_err_t eol_web_status_handler(
    httpd_req_t *req)
{
    /*
     * Status JSON will be connected to the EOL manager/persistence
     * data in the next integration step.
     *
     * For now return the web-layer state so the HTTP path itself
     * can be validated safely.
     */

    const char *json =
        "{"
        "\"factory_status\":\"WEB SERVER READY\","
        "\"factory_class\":\"UNKNOWN\","
        "\"tests_passed\":0,"
        "\"tests_failed\":0,"
        "\"tests_skipped\":0,"
        "\"tests_not_run\":0,"
        "\"hardware_fingerprint\":\"-\","
        "\"firmware_fingerprint\":\"-\","
        "\"firmware_version\":\"-\","
        "\"compatibility_version\":0,"
        "\"tests\":[]"
        "}";

    httpd_resp_set_type(
        req,
        "application/json");

    return httpd_resp_send(
        req,
        json,
        HTTPD_RESP_USE_STRLEN);
}


/* -------------------------------------------------------------------------- */

static esp_err_t eol_web_run_handler(
    httpd_req_t *req)
{
    s_ctx.eol_run_requested =
        true;

    ESP_LOGI(
        TAG,
        "EOL run requested from local web interface");

    httpd_resp_set_type(
        req,
        "text/plain");

    return httpd_resp_send(
        req,
        "EOL run requested",
        HTTPD_RESP_USE_STRLEN);
}


/* -------------------------------------------------------------------------- */

static esp_err_t eol_web_refresh_handler(
    httpd_req_t *req)
{
    s_ctx.refresh_requested =
        true;

    httpd_resp_set_type(
        req,
        "text/plain");

    return httpd_resp_send(
        req,
        "Refresh requested",
        HTTPD_RESP_USE_STRLEN);
}


/* -------------------------------------------------------------------------- */
/* HTTP server                                                                */
/* -------------------------------------------------------------------------- */

static esp_err_t start_http_server(void)
{
    if (s_ctx.http_server_running) {
        return ESP_OK;
    }

    httpd_config_t config =
        HTTPD_DEFAULT_CONFIG();

    config.server_port = 80;
    config.max_uri_handlers = 8;

    esp_err_t err =
        httpd_start(
            &s_ctx.http_server,
            &config);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "HTTP server start failed: %s",
            esp_err_to_name(err));

        return err;
    }


    const httpd_uri_t root_uri =
    {
        .uri = "/",
        .method = HTTP_GET,
        .handler = eol_web_root_handler,
        .user_ctx = NULL
    };


    const httpd_uri_t status_uri =
    {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = eol_web_status_handler,
        .user_ctx = NULL
    };


    const httpd_uri_t run_uri =
    {
        .uri = "/api/run",
        .method = HTTP_POST,
        .handler = eol_web_run_handler,
        .user_ctx = NULL
    };


    const httpd_uri_t refresh_uri =
    {
        .uri = "/api/refresh",
        .method = HTTP_GET,
        .handler = eol_web_refresh_handler,
        .user_ctx = NULL
    };


    err =
        httpd_register_uri_handler(
            s_ctx.http_server,
            &root_uri);

    if (err != ESP_OK) {
        httpd_stop(s_ctx.http_server);
        s_ctx.http_server = NULL;
        return err;
    }


    err =
        httpd_register_uri_handler(
            s_ctx.http_server,
            &status_uri);

    if (err != ESP_OK) {
        httpd_stop(s_ctx.http_server);
        s_ctx.http_server = NULL;
        return err;
    }


    err =
        httpd_register_uri_handler(
            s_ctx.http_server,
            &run_uri);

    if (err != ESP_OK) {
        httpd_stop(s_ctx.http_server);
        s_ctx.http_server = NULL;
        return err;
    }


    err =
        httpd_register_uri_handler(
            s_ctx.http_server,
            &refresh_uri);

    if (err != ESP_OK) {
        httpd_stop(s_ctx.http_server);
        s_ctx.http_server = NULL;
        return err;
    }


    s_ctx.http_server_running =
        true;

    ESP_LOGI(
        TAG,
        "EOL HTTP server started: http://%s/",
        s_ctx.ip_address);

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Wi-Fi AP                                                                   */
/* -------------------------------------------------------------------------- */

static esp_err_t start_wifi_ap(void)
{
    if (s_ctx.ap_running) {
        return ESP_OK;
    }


    if (s_ctx.ap_netif == NULL) {

        s_ctx.ap_netif =
            esp_netif_create_default_wifi_ap();

        if (s_ctx.ap_netif == NULL) {

            ESP_LOGE(
                TAG,
                "Failed to create Wi-Fi AP netif");

            return ESP_FAIL;
        }
    }


    wifi_init_config_t wifi_init =
        WIFI_INIT_CONFIG_DEFAULT();

    esp_err_t err =
        esp_wifi_init(
            &wifi_init);

    if (err != ESP_OK &&
        err != ESP_ERR_INVALID_STATE) {

        ESP_LOGE(
            TAG,
            "Wi-Fi init failed: %s",
            esp_err_to_name(err));

        return err;
    }


    wifi_config_t ap_config =
    {
        .ap =
        {
            .ssid_len = 0,
            .channel = 1,
            .authmode = WIFI_AUTH_OPEN,
            .max_connection = 1,
            .pmf_cfg =
            {
                .required = false
            }
        }
    };


    strncpy(
        (char *)ap_config.ap.ssid,
        s_ctx.config.ssid,
        sizeof(ap_config.ap.ssid) - 1U);


    ap_config.ap.ssid_len =
        strlen(
            s_ctx.config.ssid);


    /*
     * Initial bench implementation:
     *
     * Open local AP.
     *
     * Password security can be added later as part of
     * production EOL security policy.
     */
    if (s_ctx.config.password[0] != '\0') {

        ap_config.ap.authmode =
            WIFI_AUTH_WPA2_PSK;

        strncpy(
            (char *)ap_config.ap.password,
            s_ctx.config.password,
            sizeof(ap_config.ap.password) - 1U);
    }


    err =
        esp_wifi_set_mode(
            WIFI_MODE_AP);

    if (err != ESP_OK) {
        return err;
    }


    err =
        esp_wifi_set_config(
            WIFI_IF_AP,
            &ap_config);

    if (err != ESP_OK) {
        return err;
    }


    err =
        esp_wifi_start();

    if (err != ESP_OK &&
        err != ESP_ERR_INVALID_STATE) {

        ESP_LOGE(
            TAG,
            "Wi-Fi AP start failed: %s",
            esp_err_to_name(err));

        return err;
    }


    s_ctx.ap_running =
        true;


    ESP_LOGI(
        TAG,
        "EOL Wi-Fi AP started");

    ESP_LOGI(
        TAG,
        "SSID: %s",
        s_ctx.config.ssid);

    ESP_LOGI(
        TAG,
        "EOL web address: http://%s/",
        s_ctx.ip_address);


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t eol_web_init(
    const eol_web_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    if (s_ctx.initialized) {
        return ESP_OK;
    }


    memset(
        &s_ctx,
        0,
        sizeof(s_ctx));


    memcpy(
        &s_ctx.config,
        config,
        sizeof(s_ctx.config));


    strncpy(
        s_ctx.ip_address,
        EOL_WEB_DEFAULT_AP_IP,
        sizeof(s_ctx.ip_address) - 1U);


    s_ctx.initialized =
        true;


    ESP_LOGI(
        TAG,
        "EOL web subsystem initialized");


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_web_start(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }


    esp_err_t err =
        start_wifi_ap();

    if (err != ESP_OK) {
        return err;
    }


    if (s_ctx.config.start_http_server) {

        err =
            start_http_server();

        if (err != ESP_OK) {
            return err;
        }
    }


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_web_stop(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }


    if (s_ctx.http_server_running) {

        httpd_stop(
            s_ctx.http_server);

        s_ctx.http_server =
            NULL;

        s_ctx.http_server_running =
            false;
    }


    if (s_ctx.ap_running) {

        esp_wifi_stop();

        s_ctx.ap_running =
            false;
    }


    ESP_LOGI(
        TAG,
        "EOL web subsystem stopped");

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_web_deinit(void)
{
    if (!s_ctx.initialized) {
        return ESP_OK;
    }


    (void)eol_web_stop();


    if (s_ctx.ap_netif != NULL) {

        esp_netif_destroy(
            s_ctx.ap_netif);

        s_ctx.ap_netif =
            NULL;
    }


    memset(
        &s_ctx,
        0,
        sizeof(s_ctx));


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

bool eol_web_is_initialized(void)
{
    return s_ctx.initialized;
}


/* -------------------------------------------------------------------------- */

bool eol_web_is_ap_running(void)
{
    return s_ctx.ap_running;
}


/* -------------------------------------------------------------------------- */

bool eol_web_is_http_server_running(void)
{
    return s_ctx.http_server_running;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_web_request_eol_run(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_ctx.eol_run_requested =
        true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_web_request_refresh(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_ctx.refresh_requested =
        true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

const char *eol_web_get_ip_address(void)
{
    return s_ctx.ip_address;
}


/* -------------------------------------------------------------------------- */

const char *eol_web_get_ssid(void)
{
    return s_ctx.config.ssid;
}


/* -------------------------------------------------------------------------- */

void eol_web_log_status(void)
{
    ESP_LOGI(
        TAG,
        "EOL Web: initialized=%s AP=%s HTTP=%s SSID=%s IP=%s",
        s_ctx.initialized ? "yes" : "no",
        s_ctx.ap_running ? "yes" : "no",
        s_ctx.http_server_running ? "yes" : "no",
        s_ctx.config.ssid,
        s_ctx.ip_address);
}
