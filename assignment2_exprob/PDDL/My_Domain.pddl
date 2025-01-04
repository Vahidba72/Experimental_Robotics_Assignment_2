

(define (domain My_Domain)

;remove requirements that are not needed
(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types 
    robot
    waypoint
    marker
)

; un-comment following line if constants are needed
;(:constants )

(:predicates 
    (robot_at ?r - robot ?w - waypoint)
    (marker_at ?m - marker ?w - waypoint)
    (visited ?w - waypoint)
    (not_visited ?w - waypoint)
    (detected ?m - marker)
    (not_detected ?m - marker)
    (all_markers_detected)
    
)


;(:functions ;todo: define numeric functions here
;)

;define actions here



(:action go_to
    :parameters (?r - robot ?from ?to - waypoint)
    :precondition (and 
                    (robot_at ?r ?from)
                    (not_visited ?to)
                    
    )
    :effect (and 
                
                (not (not_visited ?to))
                (visited ?to)
                (robot_at ?r ?to)
                (not (robot_at ?r ?from))
    )
)

(:action detect
    :parameters (?r - robot ?w - waypoint ?m - marker)
    :precondition (and 
                    (robot_at ?r ?w)
                    (marker_at ?m ?w)
                    (not_detected ?m)
    )
    :effect (and 
                
                (not (not_detected ?m))
                (detected ?m)
    )
)

(:action check_markers ; Checks to see if all markers are detected
    :parameters ()
    :precondition (and 
                    (forall (?m - marker) (detected ?m)) ; All markers must be detected
    )
    :effect (and 
                (all_markers_detected) ; Set the predicate indicating all markers are detected
    )
)


)
