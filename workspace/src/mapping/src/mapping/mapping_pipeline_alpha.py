def main() -> None:
    raise NotImplementedError()


def subtract_unreachable_area() -> None:
    """
    Don't worry about hull subtraction.
    Subtract two polygonal geometries to find the 
    navigable area.
    """
    raise NotImplementedError()


def label_hull() -> None:
    """
    This might not be required? As the convex hull can be open
    given that the interior hull will take care of featured walls.
    Draw a line from edge to center. See if it intersects 
    interior hull. Check distance of intersection and determine
    if the hull is traversable. Arbitrary epsilon threshold.
    """
    raise NotImplementedError()


def pipeline_alpha(points) -> None:
    """
    Mapping: Interior Hull -> Navigable Map (Occupancy Grid)
    """
    raise NotImplementedError()


def generate_interior_alpha() -> None:
    raise NotImplementedError()


def derive_navigable_area() -> None:
    """
    Mapping: Navigable Map
    ~
    Convert the derived shape into an occupancy grid.
    Publish to /spotters/mapping/map topic.
    """
    raise NotImplementedError


if __name__ == "__main__":
    main()
