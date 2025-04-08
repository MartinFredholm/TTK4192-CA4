(define (domain robplan)
(:requirements :typing :durative-actions :strips :fluents)
  (:types  
     turtlebot robot camera camera_eo camera_ir robo_arm charger - vehicle
     vehicle photo valve pump pipe sound gas_ind obj battery - subject
     city_location city - location
     waypoint battery_station - city_location
     route
     )

  (:predicates
    (at ?physical_obj1 - subject ?location1 - location)
    (available ?vehicle1 - vehicle)
    (available ?camera1 - camera)
    (available ?charger1 - vehicle) 
    (connects ?route1 - route ?location1 - location ?location2 - location)
    (in_city ?location1 - location ?city1 - city)
    (route_available ?route1 - route)
    (no_photo ?subject1 - subject)
    (photo ?subject1 - subject)
    (no_seals_check ?subject1 - subject)
    (seals_check ?subject1 - subject)
    (battery_dead ?subject1 - subject)
    (battery_full ?subject1 - subject)
    

   )

(:functions 
           (distance ?O - location ?L - location)
           (route-length ?O - route)
	    (speed ?V - vehicle)
            )

  (:durative-action move_robot
       :parameters ( ?V - robot ?O - location ?L - location ?R - route ?B - battery)
       :duration (= ?duration (/ (route-length ?R) (speed ?V)))
       :condition (and 
			          (at start (at ?V ?O))
            		(at start (connects ?R ?O ?L))
                (at start (battery_full ?B))
       )
       :effect (and 
		  (at start (not (at ?V ?O)))
                  (at end (at ?V ?L))
        )
    )


 (:durative-action check_seals_valve_picture_EO
       :parameters ( ?V - robot ?L - location ?G - camera_eo ?B - valve)
       :duration (= ?duration 10)
       :condition (and 
            (over all (at ?V ?L))
            (at start (at ?B ?L))
            (at start (available ?G))
            (at start (no_seals_check ?B))
       )
       :effect (and 
	    (at start (not (no_seals_check ?B)))
            (at end (seals_check ?B))
        )
    )
  (:durative-action take_pump_picture_IR
       :parameters ( ?V - robot ?L - location ?G - camera_ir ?B - pump)
       :duration (= ?duration 10)
       :condition (and 
            (over all (at ?V ?L))
            (at start (at ?B ?L))
            (at start (available ?G))
            (at start (no_photo ?B))
       )
       :effect (and 
	    (at start (not (no_photo ?B)))
            (at end (photo ?B))
        )
    )
  (:durative-action charge_battery
       :parameters ( ?V - robot ?L - location ?C - charger ?B - battery)
       :duration (= ?duration 10)
       :condition (and 
            (over all (at ?V ?L))
            (over all (at ?C ?L))
            (at start (available ?C))
            (at start (battery_dead ?B))
       )
       :effect (and 
	    (at start (not (battery_dead ?B)))
            (at end (battery_full ?B))
        )
    )
)