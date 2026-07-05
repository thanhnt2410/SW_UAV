"""Domain model for a UAV survey area (the polygon to be covered).

Replaces passing raw ``vertices`` lists/tuples directly between the many
free functions in ``calculation_helpers.py`` and ``map_helpers.py``
(``split_area``, ``split_polygon_into_areas``, ``generate_grid``, ...).

This class only holds data plus light validation and derived properties.
The actual geometry algorithms (area calculation, polygon splitting, grid
generation, path planning) should live in the ``planning`` package and
take a ``SurveyArea`` as input/output — keeping this class from becoming a
second "god object" full of algorithm code.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

LatLon = Tuple[float, float]


@dataclass
class SurveyArea:
    """A polygon area to be surveyed, defined by its vertices."""

    vertices: List[LatLon]
    name: str = "area"

    grid_size_m: Optional[float] = None
    """Spacing between scan lines, if already decided (e.g. via
    ``calculate_grid_size_from_hfov_and_vfov``). Left as None otherwise —
    it's the planning service's job to fill this in, not this class's."""

    sub_areas: List["SurveyArea"] = field(default_factory=list)
    """Populated after splitting this area for multi-UAV coverage (was the
    return value of ``split_area`` / ``split_polygon_into_areas``)."""

    def __post_init__(self) -> None:
        if len(self.vertices) < 3:
            raise ValueError(
                f"SurveyArea '{self.name}' needs at least 3 vertices, got {len(self.vertices)}"
            )

    @property
    def is_split(self) -> bool:
        """Whether this area has already been divided into sub-areas."""
        return len(self.sub_areas) > 0

    @property
    def vertex_count(self) -> int:
        return len(self.vertices)

    def leaf_areas(self) -> List["SurveyArea"]:
        """Return the areas that should actually be flown: this area
        itself if it hasn't been split, or the flattened list of its
        sub-areas (recursively) if it has.
        """
        if not self.is_split:
            return [self]
        leaves: List["SurveyArea"] = []
        for sub in self.sub_areas:
            leaves.extend(sub.leaf_areas())
        return leaves