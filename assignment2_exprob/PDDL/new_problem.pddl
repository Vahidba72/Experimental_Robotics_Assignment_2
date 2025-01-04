(define (problem task)
(:domain my_domain)
(:objects
    rosbot - robot
    wp0 wp1 wp2 wp3 wp4 - waypoint
    m1 m2 m3 m4 - marker
)
(:init
    (robot_at rosbot wp4)

    (marker_at m1 wp0)
    (marker_at m2 wp1)
    (marker_at m3 wp2)
    (marker_at m4 wp3)


    (not_visited wp0)
    (not_visited wp1)
    (not_visited wp2)
    (not_visited wp3)
    (not_visited wp4)


    (not_detected m1)
    (not_detected m2)
    (not_detected m3)
    (not_detected m4)


)
(:goal (and
    (visited wp0)
    (visited wp1)
    (visited wp2)
    (visited wp3)
    (detected m1)
    (detected m2)
    (detected m3)
    (detected m4)
    (all_markers_detected)
))
)
