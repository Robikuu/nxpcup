#include "pixy.h"
#include "peripherals.h"
#include "fsl_lpi2c_edma.h"
#include <stdbool.h>

#define PIXY_TIMEOUT_CYCLES 5000000UL

static void pixy_edma_cb(LPI2C_Type *base,
                         lpi2c_master_edma_handle_t *handle,
                         status_t status,
                         void *userData)
{
    (void)base;
    (void)handle;
    pixy_t *cam = (pixy_t *)userData;
    if (cam != NULL) {
        cam->transferStatus = status;
        cam->transferDone   = true;
    }
}

void pixy_init(pixy_t *cam,
               LPI2C_Type *inst,
               uint8_t addr,
               edma_handle_t *rxHandle,
               edma_handle_t *txHandle)
{
    cam->instance       = inst;
    cam->address        = addr;
    cam->transferDone   = false;
    cam->transferStatus = kStatus_Success;
    LPI2C_MasterCreateEDMAHandle(
        cam->instance,
        &cam->edmaHandle,
        rxHandle,
        txHandle,
        pixy_edma_cb,
        (void *)cam);
}

static status_t pixy_send(pixy_t *cam, const uint8_t *cmd, size_t len)
{
    cam->transferDone   = false;
    cam->transferStatus = kStatus_LPI2C_Busy;
    lpi2c_master_transfer_t xfer = {
        .slaveAddress   = cam->address,
        .direction      = kLPI2C_Write,
        .subaddressSize = 0,
        .data           = (uint8_t *)cmd,
        .dataSize       = len,
        .flags          = kLPI2C_TransferDefaultFlag,
    };
    status_t s = LPI2C_MasterTransferEDMA(cam->instance, &cam->edmaHandle, &xfer);
    if (s != kStatus_Success) return s;

    uint32_t timeout = PIXY_TIMEOUT_CYCLES;
    while (!cam->transferDone && --timeout > 0) {}

    if (timeout == 0) {
        LPI2C_MasterTransferAbortEDMA(cam->instance, &cam->edmaHandle);
        return kStatus_Timeout;
    }
    return cam->transferStatus;
}

static status_t pixy_recv(pixy_t *cam, uint8_t *buf, size_t len)
{
    cam->transferDone   = false;
    cam->transferStatus = kStatus_LPI2C_Busy;
    lpi2c_master_transfer_t xfer = {
        .slaveAddress   = cam->address,
        .direction      = kLPI2C_Read,
        .subaddressSize = 0,
        .data           = buf,
        .dataSize       = len,
        .flags          = kLPI2C_TransferDefaultFlag,
    };
    status_t s = LPI2C_MasterTransferEDMA(cam->instance, &cam->edmaHandle, &xfer);
    if (s != kStatus_Success) return s;

    uint32_t timeout = PIXY_TIMEOUT_CYCLES;
    while (!cam->transferDone && --timeout > 0) {}

    if (timeout == 0) {
        LPI2C_MasterTransferAbortEDMA(cam->instance, &cam->edmaHandle);
        return kStatus_Timeout;
    }
    return cam->transferStatus;
}

status_t pixy_set_led(pixy_t *cam, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t cmd[7] = {0xAE, 0xC1, 20, 3, r, g, b};
    status_t s = pixy_send(cam, cmd, sizeof(cmd));
    if (s != kStatus_Success) return s;
    uint8_t resp[10];
    return pixy_recv(cam, resp, sizeof(resp));
}

status_t pixy_set_lamp(pixy_t *cam, uint8_t upper, uint8_t lower)
{
    uint8_t cmd[6] = {0xAE, 0xC1, 22, 2, upper, lower};
    status_t s = pixy_send(cam, cmd, sizeof(cmd));
    if (s != kStatus_Success) return s;
    uint8_t resp[10];
    return pixy_recv(cam, resp, sizeof(resp));
}

status_t pixy_get_vectors(pixy_t *cam,
                          uint16_t *vectors,
                          size_t max_vectors,
                          size_t *num_vectors)
{
    if (num_vectors != NULL) {
        *num_vectors = 0;
    }
    if (cam == NULL || vectors == NULL || num_vectors == NULL) {
        return kStatus_InvalidArgument;
    }

    uint8_t cmd[6] = {0xAE, 0xC1, 48, 2, 1, 1};
    status_t s = pixy_send(cam, cmd, sizeof(cmd));
    if (s != kStatus_Success) return s;

    uint8_t buf[68];
    s = pixy_recv(cam, buf, sizeof(buf));
    if (s != kStatus_Success) return s;

    /* Locate Pixy2 response sync (0xAF, 0xC1) and type 49 (CCC/Line main features response) */
    uint16_t hdr_off = 0xFFFFU;
    for (uint16_t h = 0; h <= 2U; h++) {
        if (buf[h] == 0xAFU && buf[h + 1U] == 0xC1U && buf[h + 2U] == 49U) {
            hdr_off = h;
            break;
        }
    }
    if (hdr_off == 0xFFFFU) {
        return kStatus_Fail;
    }

    uint16_t packetLen = hdr_off + 6U + (uint16_t)buf[hdr_off + 3U];
    if (packetLen > sizeof(buf)) {
        packetLen = sizeof(buf);
    }

    size_t count = 0;
    uint16_t idx = hdr_off + 6U;

    while (idx + 1U < packetLen && count < max_vectors) {
        uint8_t featType = buf[idx];
        uint8_t featLen  = buf[idx + 1U];

        if (featLen == 0U || (size_t)(idx + 2U + featLen) > packetLen) {
            break;
        }

        if (featType == 1U) { /* Vector feature type */
            uint8_t *data = &buf[idx + 2U];
            uint8_t nvec  = featLen / 6U;

            for (uint8_t i = 0; i < nvec && count < max_vectors; i++) {
                size_t off = (size_t)i * 6U;
                if (data[off + 0] <= 78U && data[off + 1] <= 51U &&
                    data[off + 2] <= 78U && data[off + 3] <= 51U) {
                    vectors[4 * count + 0] = data[off + 0]; /* x0 */
                    vectors[4 * count + 1] = data[off + 1]; /* y0 */
                    vectors[4 * count + 2] = data[off + 2]; /* x1 */
                    vectors[4 * count + 3] = data[off + 3]; /* y1 */
                    count++;
                }
            }
        }
        idx += 2U + featLen;
    }

    *num_vectors = count;
    return kStatus_Success;
}
