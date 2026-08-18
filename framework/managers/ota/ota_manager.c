#include "ota_manager.h"

#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"


/* -------------------------------------------------------------------------- */
/* Private definitions                                                        */
/* -------------------------------------------------------------------------- */

static const char *TAG = "OTA_MANAGER";

/*
 * GSM-VALVE partition table:
 *
 *   factory = 1 MB
 *   ota_0   = 1 MB
 *   ota_1   = 1 MB
 *
 * Therefore the maximum firmware image size accepted by this
 * manager is limited to one OTA application slot.
 */
#define OTA_MANAGER_MAX_PARTITION_SIZE    (1024U * 1024U)

#define OTA_MANAGER_DEFAULT_MAX_IMAGE_SIZE \
    OTA_MANAGER_MAX_PARTITION_SIZE


/* -------------------------------------------------------------------------- */
/* Private state                                                              */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;

static ota_manager_config_t s_config = {
    .max_image_size = OTA_MANAGER_DEFAULT_MAX_IMAGE_SIZE,
    .require_https = true
};

static ota_manager_status_t s_status = {
    .state = OTA_MANAGER_STATE_IDLE,
    .error = OTA_MANAGER_ERROR_NONE,
    .progress_percent = 0U,
    .bytes_downloaded = 0U,
    .image_size = 0U,
    .current_version = {0},
    .target_version = {0},
    .active = false,
    .reboot_required = false,
    .running_image_valid = false
};


/* -------------------------------------------------------------------------- */
/* Safety interlock                                                           */
/* -------------------------------------------------------------------------- */

static bool s_motor_running = false;

static bool s_safety_fault = false;

static bool s_battery_safe = false;


/* -------------------------------------------------------------------------- */
/* Private helpers                                                            */
/* -------------------------------------------------------------------------- */

static void ota_set_error(
    ota_manager_error_t error)
{
    s_status.error = error;

    s_status.state =
        OTA_MANAGER_STATE_ERROR;

    s_status.active = false;

    s_status.reboot_required = false;
}


static bool ota_url_is_https(
    const char *url)
{
    if (url == NULL) {
        return false;
    }

    return strncmp(
        url,
        "https://",
        8U) == 0;
}


static bool ota_safety_allows_start(void)
{
    /*
     * OTA must never start while the actuator motor is running.
     */
    if (s_motor_running) {
        return false;
    }

    /*
     * OTA must not start while the safety manager reports
     * a fault.
     */
    if (s_safety_fault) {
        return false;
    }

    /*
     * Battery must be explicitly reported safe.
     */
    if (!s_battery_safe) {
        return false;
    }

    return true;
}


static void ota_clear_transfer_status(void)
{
    s_status.progress_percent = 0U;

    s_status.bytes_downloaded = 0U;

    s_status.image_size = 0U;
}


static void ota_load_running_version(void)
{
    const esp_app_desc_t *app_desc =
        esp_app_get_description();

    if (app_desc == NULL) {

        s_status.current_version[0] =
            '\0';

        return;
    }

    snprintf(
        s_status.current_version,
        sizeof(s_status.current_version),
        "%s",
        app_desc->version);
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t ota_manager_init(
    const ota_manager_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->max_image_size == 0U) {

        ESP_LOGE(
            TAG,
            "Invalid OTA maximum image size");

        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Current GSM-VALVE ota_0 and ota_1 partitions are 1 MB.
     *
     * Do not allow the manager configuration to advertise
     * an image size larger than an OTA partition.
     */
    if (config->max_image_size >
        OTA_MANAGER_MAX_PARTITION_SIZE) {

        ESP_LOGE(
            TAG,
            "OTA image limit exceeds OTA partition size");

        return ESP_ERR_INVALID_ARG;
    }

    s_config = *config;

    /*
     * Reset runtime status.
     */
    s_status =
        (ota_manager_status_t){0};

    s_status.state =
        OTA_MANAGER_STATE_IDLE;

    s_status.error =
        OTA_MANAGER_ERROR_NONE;

    ota_load_running_version();

    /*
     * Determine whether the running image is pending
     * boot validation.
     */
    const esp_partition_t *running_partition =
        esp_ota_get_running_partition();

    if (running_partition == NULL) {

        ESP_LOGE(
            TAG,
            "Unable to determine running partition");

        return ESP_FAIL;
    }

    esp_ota_img_states_t image_state;

    esp_err_t err =
        esp_ota_get_state_partition(
            running_partition,
            &image_state);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Unable to read running image state: %s",
            esp_err_to_name(err));

        return err;
    }

    if (image_state ==
        ESP_OTA_IMG_PENDING_VERIFY) {

        /*
         * New OTA image has booted but has not yet been
         * accepted by the application.
         */
        s_status.running_image_valid =
            false;

        ESP_LOGW(
            TAG,
            "Running image is pending verification");

    } else {

        s_status.running_image_valid =
            true;
    }

    /*
     * OTA starts disabled until the application explicitly
     * supplies a safe battery state.
     */
    s_motor_running = false;

    s_safety_fault = false;

    s_battery_safe = false;

    s_initialized = true;

    ESP_LOGI(
        TAG,
        "OTA manager initialized");

    ESP_LOGI(
        TAG,
        "Running firmware version: %s",
        s_status.current_version);

    ESP_LOGI(
        TAG,
        "OTA maximum image size: %u bytes",
        (unsigned)s_config.max_image_size);

    ESP_LOGI(
        TAG,
        "HTTPS required: %s",
        s_config.require_https ? "yes" : "no");

    ESP_LOGI(
        TAG,
        "Running image valid: %s",
        s_status.running_image_valid ? "yes" : "no");

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* OTA execution                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t ota_manager_start(
    const char *url,
    const char *target_version)
{
    if (!s_initialized) {

        ota_set_error(
            OTA_MANAGER_ERROR_NOT_INITIALIZED);

        return ESP_ERR_INVALID_STATE;
    }

    if (url == NULL ||
        target_version == NULL ||
        target_version[0] == '\0') {

        ota_set_error(
            OTA_MANAGER_ERROR_INVALID_ARGUMENT);

        return ESP_ERR_INVALID_ARG;
    }

    if (s_status.active) {

        ota_set_error(
            OTA_MANAGER_ERROR_ALREADY_RUNNING);

        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Check the actuator/safety interlock before doing anything
     * related to the OTA transfer.
     */
    if (s_motor_running) {

        ota_set_error(
            OTA_MANAGER_ERROR_MOTOR_RUNNING);

        ESP_LOGW(
            TAG,
            "OTA rejected: motor is running");

        return ESP_ERR_INVALID_STATE;
    }

    if (s_safety_fault) {

        ota_set_error(
            OTA_MANAGER_ERROR_SAFETY_FAULT);

        ESP_LOGW(
            TAG,
            "OTA rejected: safety fault active");

        return ESP_ERR_INVALID_STATE;
    }

    if (!s_battery_safe) {

        ota_set_error(
            OTA_MANAGER_ERROR_BATTERY_UNSAFE);

        ESP_LOGW(
            TAG,
            "OTA rejected: battery is not marked safe");

        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Production OTA must use HTTPS.
     */
    if (s_config.require_https &&
        !ota_url_is_https(url)) {

        ota_set_error(
            OTA_MANAGER_ERROR_INVALID_URL);

        ESP_LOGE(
            TAG,
            "OTA rejected: URL is not HTTPS");

        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Save target version.
     */
    snprintf(
        s_status.target_version,
        sizeof(s_status.target_version),
        "%s",
        target_version);

    /*
     * Prepare transfer state.
     */
    ota_clear_transfer_status();

    s_status.error =
        OTA_MANAGER_ERROR_NONE;

    s_status.state =
        OTA_MANAGER_STATE_PREPARING;

    s_status.active =
        true;

    s_status.reboot_required =
        false;

    ESP_LOGI(
        TAG,
        "OTA preparation started");

    ESP_LOGI(
        TAG,
        "Target version: %s",
        s_status.target_version);

    ESP_LOGI(
        TAG,
        "OTA URL: %s",
        url);

    /*
     * ----------------------------------------------------------------------
     * Stage 1 boundary
     * ----------------------------------------------------------------------
     *
     * The actual network transfer is deliberately not implemented here.
     *
     * Stage 2 will provide the LTE/HTTPS transport and will use the
     * ESP-IDF OTA partition APIs to:
     *
     *   1. obtain the next OTA partition
     *   2. begin OTA
     *   3. write firmware data
     *   4. validate the image
     *   5. end OTA
     *   6. set the boot partition
     *
     * We therefore fail explicitly rather than pretending an OTA
     * download occurred.
     */
    s_status.state =
        OTA_MANAGER_STATE_ERROR;

    s_status.error =
        OTA_MANAGER_ERROR_DOWNLOAD_FAILED;

    s_status.active =
        false;

    ESP_LOGW(
        TAG,
        "OTA transport is not implemented yet");

    return ESP_ERR_NOT_SUPPORTED;
}


/* -------------------------------------------------------------------------- */
/* OTA abort                                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t ota_manager_abort(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_status.active) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Stage 1 has no active transport object to close.
     *
     * Stage 2 will add transport-specific abort handling.
     */
    s_status.active =
        false;

    s_status.state =
        OTA_MANAGER_STATE_ERROR;

    s_status.error =
        OTA_MANAGER_ERROR_INTERNAL;

    s_status.reboot_required =
        false;

    ESP_LOGW(
        TAG,
        "OTA operation aborted");

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* OTA reboot                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t ota_manager_reboot(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Reboot is permitted only after a successful OTA has
     * selected a new boot partition.
     */
    if (s_status.state !=
        OTA_MANAGER_STATE_READY_TO_REBOOT) {

        ESP_LOGW(
            TAG,
            "Reboot rejected: OTA image is not ready");

        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Final safety check immediately before reboot.
     */
    if (s_motor_running) {

        ESP_LOGW(
            TAG,
            "Reboot rejected: motor is running");

        return ESP_ERR_INVALID_STATE;
    }

    if (s_safety_fault) {

        ESP_LOGW(
            TAG,
            "Reboot rejected: safety fault active");

        return ESP_ERR_INVALID_STATE;
    }

    if (!s_battery_safe) {

        ESP_LOGW(
            TAG,
            "Reboot rejected: battery is unsafe");

        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(
        TAG,
        "Rebooting into OTA image");

    esp_restart();

    /*
     * esp_restart() does not normally return.
     */
    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t ota_manager_get_status(
    ota_manager_status_t *status)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *status =
        s_status;

    return ESP_OK;
}


ota_manager_state_t ota_manager_get_state(void)
{
    return s_status.state;
}


ota_manager_error_t ota_manager_get_error(void)
{
    return s_status.error;
}


bool ota_manager_is_initialized(void)
{
    return s_initialized;
}


bool ota_manager_is_active(void)
{
    return s_status.active;
}


/* -------------------------------------------------------------------------- */
/* Running image validation                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t ota_manager_mark_running_image_valid(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const esp_partition_t *running_partition =
        esp_ota_get_running_partition();

    if (running_partition == NULL) {

        ESP_LOGE(
            TAG,
            "Running partition unavailable");

        return ESP_FAIL;
    }

    esp_ota_img_states_t image_state;

    esp_err_t err =
        esp_ota_get_state_partition(
            running_partition,
            &image_state);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Unable to read running image state: %s",
            esp_err_to_name(err));

        return err;
    }

    /*
     * If the image isn't pending verification, there is
     * nothing to cancel.
     */
    if (image_state !=
        ESP_OTA_IMG_PENDING_VERIFY) {

        s_status.running_image_valid =
            true;

        return ESP_OK;
    }

    /*
     * Tell the bootloader that the new image has passed
     * application startup validation.
     *
     * This prevents automatic rollback on the next reboot.
     */
    err =
        esp_ota_mark_app_valid_cancel_rollback();

    if (err != ESP_OK) {

        s_status.error =
            OTA_MANAGER_ERROR_ROLLBACK_FAILED;

        ESP_LOGE(
            TAG,
            "Failed to mark running image valid: %s",
            esp_err_to_name(err));

        return err;
    }

    s_status.running_image_valid =
        true;

    ESP_LOGI(
        TAG,
        "Running image marked valid");

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Pending verification                                                       */
/* -------------------------------------------------------------------------- */

bool ota_manager_is_image_pending_verify(void)
{
    if (!s_initialized) {
        return false;
    }

    const esp_partition_t *running_partition =
        esp_ota_get_running_partition();

    if (running_partition == NULL) {
        return false;
    }

    esp_ota_img_states_t image_state;

    esp_err_t err =
        esp_ota_get_state_partition(
            running_partition,
            &image_state);

    if (err != ESP_OK) {
        return false;
    }

    return image_state ==
           ESP_OTA_IMG_PENDING_VERIFY;
}


/* -------------------------------------------------------------------------- */
/* Safety interlock                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t ota_manager_set_safety_state(
    bool motor_running,
    bool safety_fault,
    bool battery_safe)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Do not permit safety inputs to change while an OTA
     * transfer is active.
     */
    if (s_status.active) {

        ESP_LOGW(
            TAG,
            "Cannot change OTA safety state while OTA is active");

        return ESP_ERR_INVALID_STATE;
    }

    s_motor_running =
        motor_running;

    s_safety_fault =
        safety_fault;

    s_battery_safe =
        battery_safe;

    ESP_LOGI(
        TAG,
        "OTA safety state: motor=%s fault=%s battery=%s",
        motor_running ? "RUNNING" : "STOPPED",
        safety_fault ? "FAULT" : "OK",
        battery_safe ? "SAFE" : "UNSAFE");

    return ESP_OK;
}