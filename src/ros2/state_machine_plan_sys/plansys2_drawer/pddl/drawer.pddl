(define (domain module)
    (:requirements :strips :typing :adl :fluents :durative-actions)
    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        drawer 
        robot
        led_color
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
    (:predicates
        (drawer_is_closed ?r - robot ?d - drawer)
        (drawer_is_locked ?r - robot ?d - drawer)
        (drawer_available ?r - robot ?d - drawer)

        (led_color_is ?r - robot ?d - drawer ?l - led_color)
        (color_transition ?oc ?nc - led_color)


        (robot_available ?r - robot)

    );; end Predicates ;;;;;;;;;;;;;;;;;;;;
    ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    (:durative-action drawer_unlock
        :parameters (?r - robot ?d - drawer ?lc1 ?lc2 - led_color)
        :duration ( = ?duration 1)
        :condition (and
            (at start(drawer_available ?r ?d))
            (at start(robot_available ?r))
            (at start(led_color_is ?r ?d ?lc1))
            (at start(drawer_is_locked ?r ?d))
            (at start(drawer_is_closed ?r ?d))
            (at start(color_transition ?lc1 ?lc2))
        )
        :effect (and
            (at start(not(robot_available ?r)))
            (at start(not(drawer_available ?r ?d)))
            (at end(drawer_available ?r ?d))
            (at end(robot_available ?r))
            (at start(not(led_color_is ?r ?d ?lc1)))
            (at end(led_color_is ?r ?d ?lc2))
            (at end(not(drawer_is_locked ?r ?d)))
            (at end(drawer_is_closed ?r ?d))
        )
    )

    (:durative-action drawer_open
        :parameters (?r - robot ?d - drawer ?lc1 ?lc2 - led_color)
        :duration ( = ?duration 1)
        :condition (and
            (at start(drawer_available ?r ?d))
            (at start(robot_available ?r))
            (at start(led_color_is ?r ?d ?lc1))
            (at start(not(drawer_is_locked ?r ?d)))
            (at start(drawer_is_closed ?r ?d))
            (at start(color_transition ?lc1 ?lc2))
        )
        :effect (and
            (at start(not(robot_available ?r)))
            (at start(not(drawer_available ?r ?d)))
            (at end(robot_available ?r))
            (at end(drawer_available ?r ?d))
            (at start(not(led_color_is ?r ?d ?lc1)))
            (at end(led_color_is ?r ?d ?lc2))
            (at end(not(drawer_is_locked ?r ?d)))
            (at end(not(drawer_is_closed ?r ?d)))
        )
    )

    (:durative-action drawer_close
        :parameters (?r - robot ?d - drawer ?lc1 ?lc2 - led_color)
        :duration ( = ?duration 1)
        :condition (and
            (at start(drawer_available ?r ?d))
            (at start(robot_available ?r))
            (at start(led_color_is ?r ?d ?lc1))
            (at start(not(drawer_is_locked ?r ?d)))
            (at start(not(drawer_is_closed ?r ?d)))
            (at start(color_transition ?lc1 ?lc2))
        )
        :effect (and
            (at start(not(robot_available ?r)))
            (at start(not(drawer_available ?r ?d)))
            (at end(robot_available ?r))
            (at end(drawer_available ?r ?d))
            (at start(not(led_color_is ?r ?d ?lc1)))
            (at end(led_color_is ?r ?d ?lc2))
            (at end(drawer_is_closed ?r ?d))
            (at end(drawer_is_locked ?r ?d))
        )
    )

    (:durative-action drawer_lock
        :parameters (?r - robot ?d - drawer ?lc1 ?lc2 - led_color)
        :duration ( = ?duration 1)
        :condition (and
            (at start(drawer_available ?r ?d))
            (at start(robot_available ?r))
            (at start(led_color_is ?r ?d ?lc1))
            (at end(drawer_is_closed ?r ?d))
            (at end(drawer_is_locked ?r ?d))
            (at start(color_transition ?lc1 ?lc2))
        )
        :effect (and
            (at start(not(robot_available ?r)))
            (at start(not(drawer_available ?r ?d)))
            (at end(robot_available ?r))
            (at end(drawer_available ?r ?d))
            (at start(not(led_color_is ?r ?d ?lc1)))
            (at end(led_color_is ?r ?d ?lc2))
            (at end(drawer_is_closed ?r ?d))
            (at end(drawer_is_locked ?r ?d))
        )
    )

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;