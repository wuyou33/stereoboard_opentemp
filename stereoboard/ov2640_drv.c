


#include <stdint.h>

#include "ov2640_reg.h"

struct ov2640_win_size {
        char                            *name;
        enum ov2640_width               width;
        enum ov2640_height              height;
        const struct regval_list        *regs;
};


struct ov2640_priv {
        struct ov2640_camera_info       *info;
        const struct ov2640_win_size    *win;
        int                             model;
};


#define OV2640_SIZE(n, w, h, r) \
        {.name = n, .width = w , .height = h, .regs = r }

static const struct ov2640_win_size ov2640_supported_win_sizes[] = {
        OV2640_SIZE("QCIF", W_QCIF, H_QCIF, ov2640_qcif_regs),
        OV2640_SIZE("QVGA", W_QVGA, H_QVGA, ov2640_qvga_regs),
        OV2640_SIZE("CIF", W_CIF, H_CIF, ov2640_cif_regs),
        OV2640_SIZE("VGA", W_VGA, H_VGA, ov2640_vga_regs),
        OV2640_SIZE("SVGA", W_SVGA, H_SVGA, ov2640_svga_regs),
        OV2640_SIZE("XGA", W_XGA, H_XGA, ov2640_xga_regs),
        OV2640_SIZE("SXGA", W_SXGA, H_SXGA, ov2640_sxga_regs),
        OV2640_SIZE("UXGA", W_UXGA, H_UXGA, ov2640_uxga_regs),
};


/*

static int ov2640_mask_set(struct i2c_client *client,
                           u8  reg, u8  mask, u8  set)
{
        s32 val = i2c_smbus_read_byte_data(client, reg);
        if (val < 0)
                return val;

        val &= ~mask;
        val |= set & mask;

        dev_vdbg(&client->dev, "masks: 0x%02x, 0x%02x", reg, val);

        return i2c_smbus_write_byte_data(client, reg, val);
}

*/
/*

static unsigned long ov2640_query_bus_param(struct soc_camera_device *icd)
{
        struct soc_camera_link *icl = to_soc_camera_link(icd);
        unsigned long flags = SOCAM_PCLK_SAMPLE_RISING | SOCAM_MASTER |
                SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_HSYNC_ACTIVE_HIGH |
                SOCAM_DATA_ACTIVE_HIGH;

        if (icl->query_bus_param)
                flags |= icl->query_bus_param(icl) & SOCAM_DATAWIDTH_MASK;
        else
                flags |= SOCAM_DATAWIDTH_10;

        return soc_camera_apply_sensor_flags(icl, flags);
}

static int ov2640_set_params(struct i2c_client *client, u32 *width, u32 *height,
                             enum v4l2_mbus_pixelcode code)
{
        struct ov2640_priv       *priv = to_ov2640(client);
        const struct regval_list *selected_cfmt_regs;
        int ret;

        // select win
        priv->win = ov2640_select_win(width, height);

        // select format
        priv->cfmt_code = 0;
        switch (code) {
        case V4L2_MBUS_FMT_RGB565_2X8_LE:
                dev_dbg(&client->dev, "%s: Selected cfmt RGB565", __func__);
                selected_cfmt_regs = ov2640_rgb565_regs;
                break;
        default:
        case V4L2_MBUS_FMT_UYVY8_2X8:
                dev_dbg(&client->dev, "%s: Selected cfmt YUV422", __func__);
                selected_cfmt_regs = ov2640_yuv422_regs;
        }

        // reset hardware
        ov2640_reset(client);

        // initialize the sensor with default data
        dev_dbg(&client->dev, "%s: Init default", __func__);
        ret = ov2640_write_array(client, ov2640_init_regs);
        if (ret < 0)
                goto err;

        // select preamble
        dev_dbg(&client->dev, "%s: Set size to %s", __func__, priv->win->name);
        ret = ov2640_write_array(client, ov2640_size_change_preamble_regs);
        if (ret < 0)
                goto err;

        // set size win
        ret = ov2640_write_array(client, priv->win->regs);
        if (ret < 0)
                goto err;

        // cfmt preamble
        dev_dbg(&client->dev, "%s: Set cfmt", __func__);
        ret = ov2640_write_array(client, ov2640_format_change_preamble_regs);
        if (ret < 0)
                goto err;

        // set cfmt
        ret = ov2640_write_array(client, selected_cfmt_regs);
        if (ret < 0)
                goto err;

        priv->cfmt_code = code;
        *width = priv->win->width;
        *height = priv->win->height;

        return 0;

err:
        dev_err(&client->dev, "%s: Error %d", __func__, ret);
        ov2640_reset(client);
        priv->win = NULL;

        return ret;
}

*/
