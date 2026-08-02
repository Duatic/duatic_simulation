Occupancy maps of the worlds in `worlds/`, one pair per world under the same basename.

A map is derived from its world's geometry: move a wall in the `.sdf` and the map is
silently wrong, so the two belong together and change together. Maps recorded at a real
site are not kept here — those are instance data and live in `/srv/duatic/maps`.
