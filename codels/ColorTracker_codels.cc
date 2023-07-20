
#include "acColorTracker.h"
#include "fg3utils/trace_f.h"
#include "fg3utils/macros.h"
#include <cstdio>

#include "ColorTracker_c_types.h"

/* --- Function set_color ----------------------------------------------- */

/** Codel SetColor of function set_color.
 *
 * Returns genom_ok.
 * Throws ColorTracker_e_OPENCV_ERROR.
 */
genom_event
SetColor(const or_ColorTrack_ColorInfo *color_info,
         or_ColorTrack_ColorInfo *color, const genom_context self)
{
  *color = *color_info;
  return genom_ok;
}

/* --- Function set_map_size -------------------------------------------- */

/** Codel SetMapSize of function set_map_size.
 *
 * Returns genom_ok.
 * Throws ColorTracker_e_BAD_OG_PORT.
 */
genom_event
SetMapSize(uint8_t map_width, uint8_t map_height,
           ColorTracker_BlobMap *blob_map, const genom_context self)
{
  if (map_width <= 0 || map_height <= 0)
  {
    ColorTracker_e_BAD_OG_PORT_detail *msg;
    CODEL_LOG_ERROR("Invalid map size: %f x %f", map_width, map_height);
    snprintf(msg->message, sizeof(msg->message), "%s", "Acquired invalid map size");
    return ColorTracker_e_BAD_OG_PORT(msg, self);
  }

  blob_map->grid_map.width = map_width;
  blob_map->grid_map.height = map_height;
  // blob_map->grid_map.data = (uint8_t *)malloc(map_width * map_height);

  return genom_ok;
}
