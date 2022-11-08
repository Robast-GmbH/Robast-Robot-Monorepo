(define (domain factory)
    (:requirements :strips :typing :adl :fluents :durative-actions)
    ;konzeptionell frage: ist das hier eine sm aufm robo oder zerntaL?
    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        drawer robot led_color lock
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    ; (:constants
    ;     robot
    ; )

    ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
    (:predicates
        (drawer_is_part_of ?r - robot ?d - drawer)
        (drawer_is_open ?r - robot ?d - drawer)
        (drawer_is_locked ?r - robot ?d - drawer ?lo - lock)
        (drawer_available ?r - robot ?d - drawer)

        (color_transition ?r - robot ?d - drawer ?original_color ?new_color - led_color)

        (led_color_is_green ?r - robot ?d - drawer ?lc - led_color)
        (led_color_is_blue ?r - robot ?d - drawer ?lc - led_color)
        (led_color_is_white ?r - robot ?d - drawer ?lc - led_color)
        (led_color_is_animation ?r - robot ?d - drawer ?lc - led_color)

        (robot_available ?r - robot)

    );; end Predicates ;;;;;;;;;;;;;;;;;;;;
    ;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
    (:functions
        (led_color_is ?r - robot ?d - drawer ?lc - led_color)

    );; end Functions ;;;;;;;;;;;;;;;;;;;;
    ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ; (:durative-action change_color
    ;     :parameters (?r - robot ?d - drawer ?lc1 ?lc2 - led_color)
    ;     :duration ( = ?duration 1)
    ;     :condition (and        
    ;         (at start(robot_available ?r))
    ;         )
    ;     :effect (and
    ;         ; (at start(not(led_color_is ?r ?d ?lc1)))
    ;         ; (at end(led_color_is ?r ?d ?lc2))
    ;         (at start(not(robot_available ?r)))
    ;         (at end(robot_available ?r))
    ;     )
    ; )

    (:durative-action drawer_unlock
        :parameters (?r - robot ?d - drawer ?lo - lock ?lc - led_color)
        :duration ( = ?duration 1)
        :condition (and
            (at start(drawer_available ?r ?d))
            (at start(robot_available ?r))
            (at start(led_color_is_blue ?r ?d ?lc))
            (at start(drawer_is_locked ?r ?d ?lo))
            (at start(not(drawer_is_open ?r ?d)))
        )
        :effect (and
            (at start(not(robot_available ?r)))
            (at start(not(drawer_available ?r ?d)))
            (at end(drawer_available ?r ?d))
            (at end(robot_available ?r))
            (at end(led_color_is_green ?r ?d ?lc))
            (at end(not(drawer_is_locked ?r ?d ?lo)))
            (at end(not(drawer_is_open ?r ?d)))
        )
    )

    (:durative-action drawer_open
        :parameters (?r - robot ?d - drawer ?lo - lock ?lc - led_color)
        :duration ( = ?duration 1)
        :condition (and
            (at start(drawer_available ?r ?d))
            (at start(robot_available ?r))
            (at start(led_color_is_green ?r ?d ?lc))
            (at start(not(drawer_is_locked ?r ?d ?lo)))
            (at start(not(drawer_is_open ?r ?d)))
        )
        :effect (and
            (at start(not(robot_available ?r)))
            (at start(not(drawer_available ?r ?d)))
            (at end(robot_available ?r))
            (at end(drawer_available ?r ?d))
            (at end(led_color_is_white ?r ?d ?lc))
            (at end(not(drawer_is_locked ?r ?d ?lo)))
            (at end(drawer_is_open ?r ?d))
        )
    )

    (:durative-action drawer_close
        :parameters (?r - robot ?d - drawer ?lo - lock ?lc - led_color)
        :duration ( = ?duration 1)
        :condition (and
            (at start(drawer_available ?r ?d))
            (at start(robot_available ?r))
            (at start(led_color_is_white ?r ?d ?lc))
            (at start(not(drawer_is_locked ?r ?d ?lo)))
            (at start(drawer_is_open ?r ?d))
        )
        :effect (and
            (at start(not(robot_available ?r)))
            (at start(not(drawer_available ?r ?d)))
            (at end(robot_available ?r))
            (at end(drawer_available ?r ?d))
            (at end(led_color_is_animation ?r ?d ?lc))
            (at end(not(drawer_is_open ?r ?d)))
            (at end(drawer_is_locked ?r ?d ?lo))
        )
    )

    (:durative-action drawer_lock
        :parameters (?r - robot ?d - drawer ?lo - lock ?lc - led_color)
        :duration ( = ?duration 1)
        :condition (and
            (at start(drawer_available ?r ?d))
            (at start(robot_available ?r))
            (at start(led_color_is_animation ?r ?d ?lc))
            (at end(not(drawer_is_open ?r ?d)))
            (at end(drawer_is_locked ?r ?d ?lo))
        )
        :effect (and
            (at start(not(robot_available ?r)))
            (at start(not(drawer_available ?r ?d)))
            (at end(robot_available ?r))
            (at end(drawer_available ?r ?d))
            (at end(led_color_is_blue ?r ?d ?lc))
            (at end(not(drawer_is_open ?r ?d)))
            (at end(drawer_is_locked ?r ?d ?lo))
        )
    )

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;