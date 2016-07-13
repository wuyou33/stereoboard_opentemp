#include "blob.h"

void image_labeling(struct image_t *input, struct image_t *output, struct image_filter_t *filters, uint8_t filters_cnt,
                    struct image_label_t *labels, uint16_t *labels_count)
{
  uint8_t *input_buf = (uint8_t *)input->buf;
  uint16_t *output_buf = (uint16_t *)output->buf;

  // Initialize labels
  uint16_t labels_size = *labels_count;
  uint16_t labels_cnt = 0;
  uint16_t y, x;
  uint16_t i;

  // Do steps of 2 for YUV image
  for (y = 0; y < input->h; y++) {
    for (x = 0; x < input->w / 2; x++) {
      uint8_t p_y = (input_buf[y * input->w * 2 + x * 4 + 1] + input_buf[y * input->w * 2 + x * 4 + 3]) / 2;
      uint8_t p_u = input_buf[y * input->w * 2 + x * 4];
      uint8_t p_v = input_buf[y * input->w * 2 + x * 4 + 2];

      // Go trough the filters
      uint8_t f = 0;
      for (; f < filters_cnt; f++) {
        if (p_y > filters[f].y_min && p_y < filters[f].y_max &&
            p_u > filters[f].u_min && p_u < filters[f].u_max &&
            p_v > filters[f].v_min && p_v < filters[f].v_max) {
          break;
        }
      }

      // Check if this pixel belongs to a filter else goto next
      if (f >= filters_cnt) {
        output_buf[y * output->w + x] = 0xFFFF;
        continue;
      }

      // Check pixel above (if the same filter then take same group)
      uint16_t lid = output_buf[(y - 1) * output->w + x];
      if (y > 0 && lid != 0xFFFF && labels[lid].filter == f) {
        output_buf[y * output->w + x] = lid;
        labels[lid].pixel_cnt++;
        continue;
      }

      // Check pixel top right (check for merging)
      lid = output_buf[(y - 1) * output->w + x + 1];
      if (y > 0 && x < output->w - 1 && lid != 0xFFFF && labels[lid].filter == f) {

        // Merging labels if needed
        uint16_t lid_tl = output_buf[(y - 1) * output->w + x - 1]; // Top left
        uint16_t lid_l = output_buf[y * output->w + x - 1]; // Left
        uint16_t m = labels[lid].id, n = labels[lid].id;
        if (x > 0 && lid_tl != 0xFFFF && labels[lid_tl].filter == f) {
          // Merge with top left
          m = labels[lid].id;
          n = labels[lid_tl].id;
        } else if (x > 0 && lid_l != 0xFFFF && labels[lid_l].filter == f) {
          // Merge with left
          m = labels[lid].id;
          n = labels[lid_l].id;
        }

        // Change the id of the highest id label
        if (m != n) {
          if (m > n) {
            m = n;
            n = labels[lid].id;
          }

          for (i = 0; i < labels_cnt; i++) {
            if (labels[i].id == n) {
              labels[i].id = m;
            }
          }
        }

        // Update the label
        output_buf[y * output->w + x] = lid;
        labels[lid].pixel_cnt++;
        continue;
      }

      // Take top left
      lid = output_buf[(y - 1) * output->w + x - 1];
      if (y > 0 && x > 0 && lid != 0xFFFF && labels[lid].filter == f) {
        output_buf[y * output->w + x] = lid;
        labels[lid].pixel_cnt++;
        continue;
      }

      // Take left
      lid = output_buf[y * output->w + x - 1];
      if (x > 0 && lid != 0xFFFF && labels[lid].filter == f) {
        output_buf[y * output->w + x] = lid;
        labels[lid].pixel_cnt++;
        continue;
      }

      // Check if there is enough space
      if (labels_cnt >= labels_size - 1) {
        break;
      }

      // Create new group
      lid = labels_cnt;
      output_buf[y * output->w + x] = lid;
      labels[lid].id = lid;
      labels[lid].filter = f;
      labels[lid].pixel_cnt = 1;
      labels[lid].x_min = x;
      labels[lid].y_min = y;
      labels_cnt++;
    }
  }

  // Merge connected labels
  for (i = 0; i < labels_cnt; i++) {
    if (labels[i].id != i) {
      uint16_t new_id = labels[i].id;
      labels[new_id].pixel_cnt += labels[i].pixel_cnt;

      if (labels[i].x_min < labels[new_id].x_min) { labels[new_id].x_min = labels[i].x_min; }
      if (labels[i].y_min < labels[new_id].y_min) { labels[new_id].y_min = labels[i].y_min; }
    }
  }

  *labels_count = labels_cnt;
}
