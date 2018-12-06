;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Developed by: Fardad Haghpanah
;;; Date: Sep. 1, 2016
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


breed [people person]
breed [trappeds trapped]
breed [starts start]
breed [dummies dummy]
directed-link-breed [arrows arrow]
turtles-own [
  follow-wall?
  target
  dest
  br-dist          ;; max distance from int-target to target
  int-target       ;; intermediate target: patch on the other side of the barrier (target patch to pass the barrier)
  direction        ;; 1: follow right wall; -1: follow left wall (when reaching a barrier)
  wspeed           ;; vision radius/speed
  H                ;; patch at which agent reaches an obstacle
  r                ;; vision radius
  L                ;; patch along the boundary of the obstacle with minimum distance to target
  Q                ;; distance from current patch to target when agent is walking around an obstacle
  R2               ;; integrated length of the boundary starting at H
  R3               ;; integrated length of the boundary starting at L
  search-mode?     ;; boolean, to indicate whether agent has found the L-point or not yet
  m-line           ;; list containing patches on the straight line from start point to target
  step             ;; minimum(vision radius, minimal distance between obstacles) [ref. to the source paper]
  d-min            ;; minimum distance to target when following obstacle boundary
                   ;; minimum distance to target observed along the visible portion of the blocking obstacle
  extension2?      ;; boolean; indicates if follow direction has been changed once according to the second proposed extension to the algorithm
  cn               ;; counter to allow agent pass through H point when direction is changed by extension 2 clause
  extension3?      ;; boolean; indicate whether a virtual obstacle is already met when following an obstacle boundary
  vir-obs          ;; list containing patches of the virtual obstacles created according to the third proposed extension
  ltg-list         ;; list containing LTG points
  transition1?     ;; boolean; indicate if agent is transiting from motion-toward-target to follow-boundary algorithm
  transition2?     ;; boolean; indicate if agent is transiting from follow-boundary to motion-toward-target algorithm
  node             ;; selected waypoint when an obstacle is found
  node1            ;; temp
  node2            ;; temp
  edge             ;; edges of the identified onstacle
  last-node        ;; last selected waypoint (stored to avoid going back to the last waypoint)
  tolerance        ;; number of time steps the agent waits behind another in a congestion before changing its position
  tol-count        ;; time step counter for tolerance
  step_forward?    ;; true: agent can step forward; false: otherwise (used for agent interaction)
  pre-loc
  pre-dir
  last-dummy
  ]

globals
[
  n
  wpc1
  wpc2
  lst
  lyr
  opt
  target-set
  remve-br-done
  edges
  vertices
  target-location
]


;;; Make different types of obstacles
to setup-random
  clear-all
  ask patches [ if random-float 1.0 < 0.04 [ set pcolor brown ] ]
  ask patches with [pcolor = brown] [ ask patches in-radius random-float 3 [ set pcolor brown ] ]
  repeat 4
  [
  ask patches with [count neighbors4 with [pcolor = brown] >= 3] [ set pcolor brown ]
  ]
end

to L-obstacle
  clear-all
  ask patches with [pycor = -10 and pxcor <= 25 and pxcor >= -25] [set pcolor brown]
  ask patches with [pxcor = -25 and pycor >= -10 and pycor <= 15] [set pcolor brown]
end


to U-obstacle
  clear-all
  ask patches with [pycor = -15 and pxcor <= 10 and pxcor >= -10] [set pcolor brown]
  ask patches with [pxcor = -10 and pycor >= -15 and pycor <= 5] [set pcolor brown]
  ask patches with [pxcor = 10 and pycor >= -15 and pycor <= 5] [set pcolor brown]
end

to close-obstacle
  clear-all
  ask patches with [pycor = -15 and pxcor <= 10 and pxcor >= -10] [set pcolor brown]
  ask patches with [pycor = 15 and pxcor <= 10 and pxcor >= -10] [set pcolor brown]
  ask patches with [pxcor = -10 and pycor >= -15 and pycor <= 15] [set pcolor brown]
  ask patches with [pxcor = 10 and pycor >= -15 and pycor <= 15] [set pcolor brown]
end

to square
  clear-all
  ask patches with [pycor >= -10 and pycor <= 10 and pxcor <= 20 and pxcor >= -20] [set pcolor brown]
end

to triangle
  ask patches with [pycor >= -10 and pycor <= 10 and pxcor >= -20 and pxcor <= 20] [set pcolor brown]
  ask patches with [pcolor = brown] [if pycor < 0.5 * pxcor [set pcolor black]]
end

to triangle-random
  clear-all
  ask patches with [pycor >= -15 and pycor <= 15 and pxcor >= -25 and pxcor <= 25] [set pcolor brown]
  ask patches with [pcolor = brown] [if pycor < (7 / 12) * pxcor [set pcolor black]]
  ask patches with [pcolor = brown] [if (pycor = (7 / 12) * pxcor) and random 2 = 1 [set pcolor black]]
  ask patches with [pcolor = brown] [if (pycor <= (7 / 12) * pxcor + 1) and random 3 = 1 [set pcolor black]]
  ask patches with [pcolor = black and count neighbors with [pcolor = black] = 1] [set pcolor brown]
  ask patches with [pcolor = black and count neighbors4 with [pcolor = brown] = 4] [set pcolor brown]
  ask patches with [pcolor = brown and count neighbors4 with [pcolor = black] = 4] [set pcolor black]
end

to corridor1
  clear-all
  ask patches with [pycor >= -10 and pycor <= 10 and pxcor >= -35 and pxcor <= 35] [set pcolor brown]
  ask patches with [pxcor = 0] [set pcolor black]
  ask patches with [pxcor = max-pxcor or pxcor = min-pxcor or pycor = max-pycor or pycor = min-pycor] [set pcolor black]
end

to corridor3
  clear-all
  ask patches with [pycor >= -10 and pycor <= 10 and pxcor >= -35 and pxcor <= 35] [set pcolor brown]
  ask patches with [pxcor = 0 or pxcor = 1 or pxcor = -1] [set pcolor black]
  ask patches with [pxcor = max-pxcor or pxcor = min-pxcor or pycor = max-pycor or pycor = min-pycor] [set pcolor black]
end

to T-corridor
  clear-all
  ask patches with [pycor >= -15 and pycor <= -1 and pxcor >= -15 and pxcor <= -1] [set pcolor brown]
  ask patches with [pycor >= -15 and pycor <= -1 and pxcor >= 1 and pxcor <= 15] [set pcolor brown]
  ask patches with [pycor >= 1 and pycor <= 15 and pxcor >= -7 and pxcor <= 7] [set pcolor brown]

end

to indent1
  clear-all
  ask patches with [pycor >= -10 and pycor <= 10 and pxcor >= -10 and pxcor <= 10] [set pcolor brown]
  ask patches with [(pycor >= 9 or pycor <= -9) and (pxcor >= -6 and pxcor <= 6)] [set pcolor black]
  ask patches with [(pxcor >= 9 or pxcor <= -9) and (pycor >= -6 and pycor <= 6)] [set pcolor black]
end

to indent2
  clear-all
  ask patches with [pycor >= -10 and pycor <= 10 and pxcor >= -10 and pxcor <= 10] [set pcolor brown]
  ask patches with [(pycor >= 8 or pycor <= -8) and (pxcor <= -8 or pxcor >= 8)] [set pcolor black]
  ask patches with [(pxcor >= 8 or pxcor <= -8) and (pycor <= -8 or pycor >= 8)] [set pcolor black]
end
;;;;;;

to go
  if ticks = 0 [
    reset-timer
    ask patches [set plabel ""]
    output-print "NEW RUN:"
  ]
  tick
  ask people [
    if algorithm = "Bug1" [walk-bug1]
    if algorithm = "Bug2" [
      if not follow-wall? [create-mline]
      walk-bug2
    ]
    if algorithm = "DistBug" [walk-distbug]
    if algorithm = "TangentBug" or algorithm = "TangentBug(2)" [walk-tangent]
    if algorithm = "K-Bug" [walk-kbug]
    if algorithm = "M-DB" [walk-mdb]
    if algorithm = "Vertex" [walk-vertex]
    ;pen-down
  ]
  ;if ticks mod 10 < 0.5 [draw-link]
  if count people = 0 [show timer stop]
end

to draw-link
  if link? [
    ;let d heading
    let c [color] of self
    ask patch-here [
      sprout-dummies 1 [
        set hidden? true
;        set color grey
;        set heading d
      ]
    ]
    let a item 1 last-dummy
    set last-dummy replace-item 0 last-dummy a
    set last-dummy replace-item 1 last-dummy one-of dummies-here
    set a item 1 last-dummy
    if a != (item 0 last-dummy) [
      ask (item 0 last-dummy) [
        create-arrow-to a [
          set shape "link1"
          set thickness 0.1
          set color c
        ]
      ]
    ]
  ]
end

to walk-tangent
  ifelse transition1? [
    ifelse patch-here != H [ifelse human-interaction? [interaction-walk] [fd wspeed]] [
      set transition1? false
      set follow-wall? true
      while [wall? 0 wspeed != nobody] [lt direction * 44.5]
      ifelse (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0] [
        ifelse round (heading / 90) = 1 [set heading 90] [
          ifelse round (heading / 90) = 2 [set heading 180] [set heading 270]
        ]
      ]
    ]
  ]
  [
    ifelse transition2? [
      ifelse distance target <= d-min [
        face target
        set transition2? false
        set H nobody
      ]
      [ifelse human-interaction? [interaction-walk] [fd wspeed]]
    ]
    [
      ifelse follow-wall? [follow-boundary] [motion-toward-target]
    ]
  ]
end

to walk-vertex
  if patch-here = dest [die]
  if member? patch-here [neighbors] of dest [die]
  ifelse target-visible? target [
    face target
    set int-target target
  ]
  [
    let options nobody
    if patch-here = int-target [
      set options (vertices in-radius r) with [self != [patch-here] of myself and [target-visible? myself] of myself and self != [int-target] of myself and not member? self [ltg-list] of myself]
      let a patches in-radius r with [pcolor = black]
      let b patches in-radius (r - 2) with [pcolor = black]
      set a a with [not member? self b and [target-visible? myself] of myself]
      set options (patch-set options a)
      set int-target min-one-of options [distance2v self ([target] of myself) + distance2v self [patch-here] of myself]
      set ltg-list lput int-target ltg-list
      if int-target != nobody [
        face int-target
      ]
    ]
  ]

  ifelse int-target != nobody [interaction-walk] [
    set ltg-list []
    set int-target patch-here
  ]

end

to interaction-walk
  let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
  ifelse count people_ahead < 1 or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) <= 90 [fd wspeed]
  [
    set tol-count tol-count + 1
    if tol-count > tolerance [
      set tol-count 0
      let alter one-of neighbors with [pcolor = black and count people-here < 1]
      if alter != nobody [
        move-to alter
        if follow-wall? [
          set follow-wall? false
          set h nobody
        ]
        face target
        set int-target patch-here
      ]
    ]
  ]
end

to walk-bug1
  if patch-here = target [draw-link die]
  if follow-wall? [
    set wpc1 wall-p? patch-here (heading + 90 * direction) 1
    set wpc2 wall-p? patch-here (heading + 135 * direction) 1
    if wpc1 = nobody and wpc2 != nobody [draw-link rt direction * 90]
  ]
  if wall? 0 wspeed != nobody [
    ifelse follow-wall? [
;;;;;;;;;;;; changing direction when agent gets stuck at the end of a corridor
      ifelse wall? 0 1 = patch-here and wall? (90 * direction) 1 != nobody [
        set direction -1 * direction
        set heading heading + 180
      ]
;;;;;;;;;;;;
      [
        while [wall? 0 wspeed != nobody] [draw-link lt direction * 90]
      ]
    ]
    [
      draw-link
      set search-mode? true
      set follow-wall? true
      set H patch-here
      set Q distance target
      set L patch-here
      while [wall? 0 wspeed != nobody] [lt direction * 44.5]
      ifelse (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0] [
        ifelse round (heading / 90) = 1 [set heading 90] [
          ifelse round (heading / 90) = 2 [set heading 180] [set heading 270]
        ]
      ]
      while [wall? 0 wspeed != nobody] [lt direction * 90]
    ]
  ]

  ifelse distance target <= 1 [move-to target draw-link die] [
    let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
    ifelse (human-interaction? = false) or (count people_ahead < 1) or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) <= 90 [
      fd wspeed
      ifelse search-mode? [
        set R2 R2 + wspeed
        set R3 R3 + wspeed
        if distance target < Q [
          set Q distance target
          set L patch-here
          set R3 0
        ]
        if patch-here = H and patch-right-and-ahead 180 wspeed != H and L != nobody [
          draw-link
          set search-mode? false
          if (R2 / 2) > R3 [
            set heading heading + 180
            set direction -1 * direction
          ]
        ]
      ]
      [
        if patch-here = L [
          draw-link
          set follow-wall? false
          set search-mode? false
          face target
          set R2 0
          set R3 0
          set Q 0
          set H nobody
          set L nobody
          if wall? 0 wspeed != nobody [
            ;ask patch 10 0 [set plabel "Agent is trapped!" set plabel-color yellow]
            output-type "Agent is trapped on: " output-print patch-here
            set breed trappeds
            draw-link
            ;set shape "face sad"
            ;set size 2
          ]
        ]
      ]
    ]
    [
      set tol-count tol-count + 1
      if tol-count > tolerance [
        set tol-count 0
        let alter one-of neighbors with [pcolor = black and count people-here < 1]
        if alter != nobody [
          move-to alter
          if follow-wall? [
            set follow-wall? false
            set search-mode? false
            set R2 0
            set R3 0
            set Q 0
            set H nobody
            set L nobody
          ]
          face target
        ]
      ]
    ]
  ]
end

to walk-bug2
  if patch-here = target [draw-link die]
  if follow-wall? [
    set wpc1 wall-p? patch-here (heading + 90 * direction) 1
    set wpc2 wall-p? patch-here (heading + 135 * direction) 1
    if wpc1 = nobody and wpc2 != nobody [draw-link rt direction * 90]
  ]
  if wall? 0 wspeed != nobody [
    ifelse follow-wall? [
;;;;;;;;;;;; changing direction when agent gets stuck at the end of a corridor
      ifelse wall? 0 1 = patch-here and wall? (90 * direction) 1 != nobody [
        set direction -1 * direction
        set heading heading + 180
      ]
;;;;;;;;;;;;
      [
        while [wall? 0 wspeed != nobody] [draw-link lt direction * 90]
      ]
    ]
    [
      draw-link
      set follow-wall? true
      set H patch-here
      while [wall? 0 wspeed != nobody] [lt direction * 44.5]
      ifelse (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0] [
        ifelse round (heading / 90) = 1 [set heading 90] [
          ifelse round (heading / 90) = 2 [set heading 180] [set heading 270]
        ]
      ]
      while [wall? 0 wspeed != nobody] [lt direction * 90]
    ]
  ]

  ifelse distance target <= 1 [move-to target draw-link die] [
    let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
    ifelse (human-interaction? = false) or (count people_ahead < 1) or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) <= 90 [
      fd wspeed
      if follow-wall? [
        if member? patch-here m-line and patch-here != H and distance target < distance2v H target [
          set follow-wall? false
          draw-link
          face target
          set H nobody
        ]
        if patch-here = H and patch-right-and-ahead 180 wspeed != H [
          ;ask patch 10 0 [set plabel "Agent is trapped!" set plabel-color yellow]
          output-type "Agent is trapped on: " output-print patch-here
          set breed trappeds
          draw-link
          ;set shape "face sad"
          ;set size 2
        ]
      ]
    ]
    [
      set tol-count tol-count + 1
      if tol-count > tolerance [
        set tol-count 0
        let alter one-of neighbors with [pcolor = black and count people-here < 1]
        if alter != nobody [
          move-to alter
          if follow-wall? [
            set follow-wall? false
            set H nobody
          ]
          face target
        ]
      ]
    ]
  ]

end

to walk-distbug
  if patch-here = target [draw-link die]
  if follow-wall? [
;;;;;;;;;;;; extension 2
if Extension-2 = true [
    if not extension2? [
      let a 0
      set a atan ([pxcor] of target - xcor) ([pycor] of target - ycor) - heading
      ifelse abs(a) > 180 [set a 360 - abs(a)] [set a abs(a)]
      if a > 135 [
        draw-link
        set direction -1 * direction
        set heading heading + 180
        set extension2? true
      ]
    ]
]
;;;;;;;;;;;;
    set wpc1 wall-p? patch-here (heading + 90 * direction) 1
    set wpc2 wall-p? patch-here (heading + 135 * direction) 1
    if wpc1 = nobody and wpc2 != nobody [draw-link rt direction * 90]
  ]
  if wall? 0 wspeed != nobody [
    ifelse follow-wall? [
;;;;;;;;;;;; changing direction when agent gets stuck at the end of a corridor
      ifelse wall? 0 1 = patch-here and wall? (90 * direction) 1 != nobody [
        set direction -1 * direction
        set heading heading + 180
      ]
;;;;;;;;;;;;
      [
        while [wall? 0 wspeed != nobody] [draw-link lt direction * 90]
      ]
    ]
    [
      draw-link
      set follow-wall? true
      set H patch-here
      create-mline
      set d-min distance target
      if Extension-3 = true [build-virtual-obstacles distance target]
;;;;;;;;;;;; extension 1
ifelse Extension-1 = true [
      ifelse heading mod 90 = 0 [lt 90 * direction] [
        let horz false
        let vert false
        if wall-p? patch-here 0 1 != nobody or wall-p? patch-here 180 1 != nobody [set horz true]
        if wall-p? patch-here 90 1 != nobody or wall-p? patch-here 270 1 != nobody [set vert true]
        if horz and not vert [
          ifelse heading > 0 and heading < 180 [
            ifelse heading < 90 [set direction -1] [set direction 1]
            set heading 90
            ]
          [
            ifelse heading < 270 [set direction -1] [set direction 1]
            set heading 270
            ]
        ]
        if not horz and vert [
          ifelse heading > 90 and heading < 270 [
            ifelse heading < 180 [set direction -1] [set direction 1]
            set heading 180
            ]
          [
            ifelse heading < 90 [set direction 1] [set direction -1]
            set heading 0
            ]
        ]

        if (horz and vert) or (not horz and not vert) [
          while [wall? 0 wspeed != nobody] [lt direction * 44.5]
          ifelse (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0] [
            ifelse round (heading / 90) = 1 [set heading 90] [
              ifelse round (heading / 90) = 2 [set heading 180] [set heading 270]
            ]
          ]
          while [wall? 0 wspeed != nobody] [lt direction * 90]
        ]
      ]
      while [wall? 0 wspeed != nobody] [lt direction * 90]
]
;;;;;;;;;;;;
[
  while [wall? 0 wspeed != nobody] [lt direction * 44.5]
  ifelse (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0] [
    ifelse round (heading / 90) = 1 [set heading 90] [
      ifelse round (heading / 90) = 2 [set heading 180] [set heading 270]
    ]
  ]
  while [wall? 0 wspeed != nobody] [lt direction * 90]
]
    ]
  ]

  ifelse distance target <= 1 [move-to target draw-link die] [
    let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
    ifelse (human-interaction? = false) or (count people_ahead < 1) or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) <= 90 [
      fd wspeed
      if follow-wall? [
;;;;;;;;;;;; extension 3
if Extension-3 = true [
  if member? patch-here vir-obs [
    ifelse extension3? [
      build-virtual-obstacles (distance target + r / 2)
      set extension3? false
    ]
    [
      draw-link
      set direction -1 * direction
      set heading heading + 180
      set extension3? true
      set wpc1 wall-p? patch-here (heading + 90 * direction) 1
      set wpc2 wall-p? patch-here (heading + 135 * direction) 1
      if wpc1 = nobody and wpc2 != nobody [ifelse human-interaction? [interaction-walk] [fd wspeed]]   ;;; to prevent agents from being stuck between corners of two obstacles (an identified problem)
    ]
  ]
]
;;;;;;;;;;;;
        if distance target < d-min [set d-min distance target]
        if target-visible? target or distance target - R < d-min - step or (member? patch-here m-line and patch-here != H and distance target < distance2v H target) [
          draw-link
          set follow-wall? false
          face target
          set d-min 0
          set H nobody
          set m-line []
          set extension2? false
          set cn 0
          set extension3? false
          set vir-obs []
        ]
        if patch-here = H and patch-right-and-ahead 180 wspeed != H [
          ifelse cn = 1 or (extension3? = false and Extension-3 = true and Extension-2 = false) or (Extension-3 = false and Extension-2 = false) [
            ;ask patch 10 0 [set plabel "Agent is trapped!" set plabel-color yellow]
            output-type "Agent is trapped on: " output-print patch-here
            set breed trappeds
            draw-link
            ;set shape "face sad"
            ;set size 2
          ]
          [set cn 1]
        ]
      ]
    ]
    [
      set tol-count tol-count + 1
      if tol-count > tolerance [
        set tol-count 0
        let alter one-of neighbors with [pcolor = black and count people-here < 1]
        if alter != nobody [
          move-to alter
          if follow-wall? [
            set follow-wall? false
            set d-min 0
            set H nobody
            set m-line []
            set extension2? false
            set cn 0
            set extension3? false
            set vir-obs []
          ]
          face target
        ]
      ]
    ]
  ]
end

to walk-kbug
  set step_forward? false
  if patch-here = target [draw-link die]
  if find-wall (towards target) r != nobody and node = nobody [set node patch-here]
  ifelse patch-here != node [set step_forward? true] [
    set node nobody
    ifelse target-visible? target or find-wall (towards target) r = nobody [draw-link face target set step_forward? true] [
      find-node
      set last-node patch-here
      draw-link
      face node
      set step_forward? true
    ]
  ]

  if step_forward? [
    ifelse human-interaction? [
      let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
      ifelse count people_ahead < 1 or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) <= 90 [fd wspeed] [
        set tol-count tol-count + 1
        if tol-count > tolerance [
          set tol-count 0
          let alter one-of neighbors with [pcolor = black and count people-here < 1]
          if alter != nobody [
            move-to alter
            set node nobody
            face target
          ]
        ]
      ]
    ]
    [fd wspeed]
  ]
end

to walk-mdb
  if patch-here = target [draw-link die]
  if follow-wall? [
    let c1 wall-p? patch-here (heading + 90 * direction) 1
    let c2 wall-p? patch-here (heading + 135 * direction) 1
    if c1 = nobody and c2 != nobody [
      ifelse distance2v c2 target > br-dist [rt 90 * direction] [
        face target
        set follow-wall? false
      ]
    ]
  draw-link
  ]

  if wall? 0 wspeed != nobody [
    ifelse follow-wall? [
;;;;;;;;;;;; changing direction when agent gets stuck at the end of a corridor
      ifelse wall? 0 1 = patch-here and wall? (90 * direction) 1 != nobody [
        set direction -1 * direction
        set heading heading + 180
      ]
;;;;;;;;;;;;
      [
        while [wall? 0 wspeed != nobody] [lt direction * 90]
    draw-link
      ]
    ]
    [
    draw-link
      set follow-wall? true
      set H patch-here
      set d-min distance target
      set int-target wall? 0 wspeed
      set n 1 + wspeed
      while [[pcolor] of int-target = brown] [
        set int-target patch-at-heading-and-distance heading n
        set n n + 1
      ]
      set br-dist distance2v int-target target
;;;;;;;;;;;; extension 1
      ifelse heading mod 90 = 0 [lt 90 * direction] [
        let horz false
        let vert false
        if wall-p? patch-here 0 1 != nobody or wall-p? patch-here 180 1 != nobody [set horz true]
        if wall-p? patch-here 90 1 != nobody or wall-p? patch-here 270 1 != nobody [set vert true]
        if horz and not vert [
          ifelse heading > 0 and heading < 180 [
            ifelse heading < 90 [set direction -1] [set direction 1]
            set heading 90
          ]
          [
            ifelse heading < 270 [set direction -1] [set direction 1]
            set heading 270
          ]
        ]
        if not horz and vert [
          ifelse heading > 90 and heading < 270 [
            ifelse heading < 180 [set direction -1] [set direction 1]
            set heading 180
          ]
          [
            ifelse heading < 90 [set direction 1] [set direction -1]
            set heading 0
          ]
        ]

        if (horz and vert) or (not horz and not vert) [
          while [wall? 0 wspeed != nobody] [lt direction * 44.9]
          ifelse (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0] [
            ifelse round (heading / 90) = 1 [set heading 90] [
              ifelse round (heading / 90) = 2 [set heading 180] [set heading 270]
            ]
          ]
        ]
      ]
      while [wall? 0 wspeed != nobody] [lt direction * 90]
;;;;;;;;;;;;
    ]
  ]

  ifelse distance target <= 1 [move-to target draw-link die] [
    let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
    ifelse (human-interaction? = false) or (count people_ahead < 1) or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) < 90 [
      fd wspeed
      if follow-wall? [
        if distance target < d-min [set d-min distance target]
        if target-visible? target or distance target - r < d-min - step or patch-here = int-target [
          draw-link
          set follow-wall? false
          face target
          set d-min 0
          set H nobody
        ]
        if patch-here = H and patch-right-and-ahead 180 wspeed != H [
          ifelse cn = 1 or Extension-2 = false [
            ;ask patch 10 0 [set plabel "Agent is trapped!" set plabel-color yellow]
            ;output-type "Agent is trapped on: " output-print patch-here
            set breed trappeds
            draw-link
            ;set shape "face sad"
            ;set size 2
          ]
          [set cn 1]
        ]
      ]
    ]
    [
      set tol-count tol-count + 1
      if tol-count > tolerance [
        ;set tol-count 0
        let alter one-of neighbors with [pcolor = black and count people-here < 1]
        if alter != nobody [
          set tol-count 0
          move-to alter
          if follow-wall? [
            set follow-wall? false
            set d-min 0
            set H nobody
          ]
          face target
        ]
      ]
    ]
  ]
end

to create-mline
  set m-line []
  let k 0
  while [patch-right-and-ahead 0 k != target] [
    set m-line lput (patch-right-and-ahead 0 k) m-line
    set k k + 1
  ]
end

to build-virtual-obstacles [d]
  set vir-obs []
  let b 0
  while [b < 360] [
    if [patch-at-heading-and-distance b (d + 1)] of target != nobody and [[pcolor] of patch-at-heading-and-distance b (d + 1)] of target != brown [set vir-obs (lput ([patch-at-heading-and-distance b (d + 1)] of target) vir-obs)]
    set b b + 180 / pi / d
  ]
end

to motion-toward-target
  if patch-here = target [die]
  ifelse target-visible? target or find-wall (towards target) r = nobody [
    face target
    ifelse human-interaction? [interaction-walk] [fd wspeed]
  ]
  [
    ifelse algorithm = "TangentBug" [LTG] [LTG2]
    set node (min-one-of (patches with [member? self [ltg-list] of myself]) [distance myself + distance [target] of myself])
    face node
    ifelse distance2v node target > distance target [
      set H node
      set d-min distance2v (find-wall (towards target) r) target
      ifelse ([pxcor] of node < [pxcor] of (find-wall (towards target) r)) or ([pycor] of node > [pycor] of (find-wall (towards target) r)) [set direction 1] [set direction -1]
      set transition1? true
      ]
    [
      ifelse wall? 0 wspeed != nobody [move-to min-one-of neighbors with [pcolor = black] [distance [target] of myself]] [ifelse human-interaction? [interaction-walk] [fd wspeed]]
    ]
  ]
end

to follow-boundary
  if patch-here = target [die]
  ifelse algorithm = "TangentBug" [LTG] [LTG2]

  set wpc1 wall-p? patch-here (heading + 90 * direction) 1
  set wpc2 wall-p? patch-here (heading + 135 * direction) 1
  if wpc1 = nobody and wpc2 != nobody [rt direction * 90]

  ifelse wall? 0 wspeed != nobody [
;;;;;;;;;;;; changing direction when agent gets stuck at the end of a corridor
    ifelse wall? 0 1 = patch-here and wall? (90 * direction) 1 != nobody [
      set direction -1 * direction
      set heading heading + 180
    ]
;;;;;;;;;;;;
    [
      lt 90 * direction
    ]
  ]
  [
    ifelse distance target <= 1 [move-to target die] [
      let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
      ifelse (human-interaction? = false) or (count people_ahead < 1) or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) <= 90 [
        fd wspeed
        if distance target < d-min [set d-min distance target]

        if patch-here = H and patch-right-and-ahead 180 wspeed != H [
          ;ask patch 10 0 [set plabel "Agent is trapped!" set plabel-color yellow]
          output-type "Agent is trapped on: " output-print patch-here
          set breed trappeds
          ;set shape "face sad"
          ;set size 2
        ]

        set node (min-one-of (patches with [member? self [ltg-list] of myself]) [distance [target] of myself])
        if distance2v node target < d-min [
          set follow-wall? false
          set transition2? true
          face node
        ]
      ]
      [
        if human-interaction? [
          set tol-count tol-count + 1
          if tol-count > tolerance [
            set tol-count 0
            let alter one-of neighbors with [pcolor = black and count people-here < 1]
            if alter != nobody [
              move-to alter
              if follow-wall? [
                set follow-wall? false
              ]
              face target
            ]
          ]
        ]
      ]
    ]
  ]

  if target-visible? target or find-wall (towards target) r = nobody [
    set follow-wall? false
    set H nobody
    face target
  ]

end

to find-node
  set H find-wall (towards target) r
  let HH patch-at-heading-and-distance (towards target) (distance H - 1)
  set lst (patch-set HH ([neighbors] of HH) with [pcolor = black and self != [patch-here] of myself])
  set lst lst with [self != [patch-here] of myself and [target-visible? myself] of myself and any? neighbors with [pcolor = brown]]
  set lyr lst

  while [lyr != nobody] [
    let ph patch-here
    let a nobody
    ask lyr [
      set a (patch-set a neighbors with [self != ph and pcolor = black and not member? self lst and any? neighbors with [pcolor = brown]])
    ]
    if a != nobody [set a a with [[target-visible? myself] of myself]]
    set lst (patch-set lyr lst)
    set lyr a
  ]
  set edge lst


;;;; another approach to find the available edges, considers the entire obstacle,
;;;; could lead to errors for U shape obstacles with the target inside,
;;;; also fails to identify rather narrow passages
;  set H find-wall (towards target) r
;  set lst (patch-set H ([neighbors] of H) with [pcolor = brown])
;  set lyr lst
;  while [lyr != nobody] [
;    let a nobody
;    ask lyr [
;      set a (patch-set a neighbors with [pcolor = brown and not member? self lst])
;    ]
;    set lst (patch-set lyr lst)
;    set lyr a
;  ]
;
;  set edge (patch-set ([neighbors] of lst)) with [self != [patch-here] of myself and pcolor = black and [target-visible? myself] of myself]
;;;;;

  let head (towards target)
  let rs (edge with [(atan (pxcor - [pxcor] of myself) (pycor - [pycor] of myself) - head) mod 360 < 180])
  set rs sort rs
  set rs sort-by [ [?1 ?2] -> distance ?1 < distance ?2 ] rs

  let a 0
  set node1 nobody
  foreach rs [ ?1 ->
    let b (atan ([pxcor] of ?1 - pxcor) ([pycor] of ?1 - pycor) - head) mod 360
    ifelse b >= a [set node1 ?1 set a b] [
      if abs(b - a) < 10 and distance ?1 > distance node1 [set node1 ?1 set a b]      ;;;;;;;;;; for long inclined obstacles this line would improve the algorithm (**)
    ]
  ]

  let ls (edge with [(atan (pxcor - [pxcor] of myself) (pycor - [pycor] of myself) - head) mod 360 > 180])
  set ls sort ls
  set ls sort-by [ [?1 ?2] -> distance ?1 < distance ?2 ] ls

  set a 360
  set node2 nobody
  foreach ls [ ?1 ->
    let b (atan ([pxcor] of ?1 - pxcor) ([pycor] of ?1 - pycor) - head) mod 360
    ifelse b <= a [set node2 ?1 set a b] [
      if abs(b - a) < 10 and distance ?1 > distance node2 [set node2 ?1 set a b]      ;;;;;;;;;; (**)
    ]
  ]

  if node1 = nobody [set node node2 stop]
  if node2 = nobody [set node node1 stop]

  ifelse distance2v patch-here node1 < distance2v patch-here node2 [set node node1] [
    ifelse distance2v patch-here node1 = distance2v patch-here node2 [
      set node min-one-of (patch-set node1 node2) [distance2v self [target] of myself]
    ]
    [
      set node node2
    ]
  ]

  if node = last-node [
    ifelse node = node1 [set node node2] [set node node1]
  ]

end


;;;; boolean, report if the given point is visible to the agent
to-report target-visible? [pch]
  let dir towards pch
  let m 0
  let output false
  while [m <= r and [pcolor] of (patch-at-heading-and-distance dir m) != brown] [
    ifelse patch-at-heading-and-distance dir m = pch [
      set output true
      set m r + 1     ;; to exit the loop (stop is not allowed in to-report)
    ]
    [set m m + wspeed]
  ]
  report output
end

;;; create Local Tangent Graph
to LTG
  set ltg-list []
  let dir towards target
  if find-wall dir r = nobody [set ltg-list lput (patch-at-heading-and-distance dir r) ltg-list]
  let vis (patches in-radius r) with [self != [patch-here] of myself]
  set vis sort vis
  foreach vis [ ?1 ->
    if target-visible? ?1 = false [set vis remove ?1 vis]
  ]
  let edge1 []
  foreach vis [ ?1 ->
    if any? ([neighbors] of ?1) with [pcolor = brown] [set edge1 lput ?1 edge1]
  ]
  foreach edge1 [ ?1 ->
    if count ([neighbors4] of ?1) with [member? self edge1] = 1 [set ltg-list lput ?1 ltg-list]
  ]
end

to LTG2
  set ltg-list []
  let dir towards target
  if find-wall dir r = nobody [set ltg-list lput (patch-at-heading-and-distance dir r) ltg-list]
  let ed []
  set ed lput (edges with [self != [patch-here] of myself and [target-visible? myself] of myself]) ed  ; and distance2v self ([patch-here] of myself) > 1
  set ed sort (item 0 ed)
  foreach ed [ ?1 ->
    if count ([neighbors4] of ?1) with [member? self ed] = 1 or count ([neighbors] of ?1) with [member? self ed] = 1 [set ltg-list lput ?1 ltg-list]
  ]
end


;;;; report the farthest patch visible in a specific direction on the boundry of an obstacle
;;;; if no obctacle, report nobody
to-report find-wall [dir dist]
  let m 1
  while [m <= dist] [
    ifelse (patch-at-heading-and-distance dir m = nobody) or [pcolor] of (patch-at-heading-and-distance dir m) != brown [
      set m m + wspeed
    ]
    [
      report (patch-at-heading-and-distance dir m)
      set m dist + 10
    ]
    if abs(m - dist) < 2 [report nobody]
  ]
end

to-report wall? [angle dt]
  ;; note that angle may be positive or negative. If angle is positive, the turtle looks right. If angle is negative, the turtle looks left.
  let wall nobody
  ifelse patch-right-and-ahead angle dt != nobody [
    if brown = [pcolor] of patch-right-and-ahead angle dt [set wall patch-right-and-ahead angle dt]
  ]
  [
    set wall patch-here
  ]
  report wall
end

to-report wall-p? [ph angle dt]
;; note that angle may be positive or negative. If angle is positive, the turtle looks right. If angle is negative, the turtle looks left.
  let wall nobody
  ask ph [
    ifelse patch-at-heading-and-distance angle dt != nobody [
      if brown = [pcolor] of patch-at-heading-and-distance angle dt [set wall patch-at-heading-and-distance angle dt]
    ]
    [
      set wall nobody
    ]
  ]
  report wall
end

to setup-target-random
  ct
  ask patches with [pcolor = yellow] [set pcolor black]
  set target-set one-of patches with [pcolor != brown]
  ask target-set [ask neighbors4 [set pcolor yellow]]
end


to setup-target2
  ct
  ask patches with [pcolor = yellow] [set pcolor black]
  set target-set (patch target-xcor target-ycor)
  ask target-set [ask neighbors4 [set pcolor yellow]]
end

to setup-target
  ct
  ask patches with [pcolor = yellow] [set pcolor black]
  if target-location = "up" [set target-set (patch 0 25)]
  if target-location = "down" [set target-set (patch 0 -25)]
  if target-location = "right" [set target-set (patch 25 0)]
  if target-location = "left" [set target-set (patch -25 0)]
  if target-location = "middle" [set target-set (patch 0 -8)]
  if target-location = "trapped" [set target-set (patch 0 -0)]
  if target-location = "random" [setup-target-random]
  ask target-set [ask neighbors4 [set pcolor yellow]]

  ;;;;;;;;; setup vertices
  set edges patches with [pcolor = black and any? neighbors with [pcolor = brown]]
  set vertices edges with [count neighbors with [pcolor = brown] = 1 and count neighbors4 with [pcolor = brown] = 0]
end

to make-turtles
  ask people [die]
  ask trappeds [die]
  ct
  ask patches [set plabel ""]

  create-people population [
    let b nobody
    if target-location = "up" or target-location = "middle" [
      let a patches with [pcolor = brown]
      set b min-one-of a [pycor]
      move-to one-of patches with [pycor < [pycor] of b]
    ]
    if target-location = "down" [
      let a patches with [pcolor = brown]
      set b max-one-of a [pycor]
      move-to one-of patches with [pycor > [pycor] of b]
    ]
    if target-location = "right" [
      let a patches with [pcolor = brown]
      set b min-one-of a [pxcor]
      move-to one-of patches with [pxcor < [pxcor] of b]
    ]
    if target-location = "left" [
      let a patches with [pcolor = brown]
      set b max-one-of a [pxcor]
      move-to one-of patches with [pxcor > [pxcor] of b]
    ]
    if target-location = "trapped" [
      let a patches with [pcolor = brown]
      set b max-one-of a [pycor + pxcor]
      let d distance2v target-set b
      move-to one-of patches with [distance2v self target-set > d]
    ]
    if target-location = "random" [
      move-to one-of patches with [pcolor = black]
    ]

    ask patch-here [
      sprout-starts 1 [
        set shape "circle"
        set color black
        set size 0.5
      ]
    ]

    set last-dummy []
    set last-dummy lput self last-dummy
    set last-dummy lput (one-of starts-here) last-dummy
    set pre-loc patch-here
    set color grey
    set follow-wall? false
    ifelse walk-method = 1 [set wspeed (random-float 0.5 + 0.5)] [set wspeed 1]
    set shape "circle"
    set target target-set
    face target
    if count neighbors4 with [pcolor = brown] = 4 [ die ]  ;; trapped!
    set tol-count 0
    set tolerance 0
    set size 1
    set pen-size 0.01
    set H nobody
    ifelse random 2 = 0 [set direction 1] [set direction -1]
    set pre-dir direction
    set r 70
    if algorithm = "Bug1" [
      set L nobody
      set search-mode? false
    ]
    if algorithm = "Bug2" [create-mline]
    if algorithm = "DistBug" [
      set step r   ;; set value to that of R (unless it will be problematic!)
      set d-min 0
      set m-line []
      set extension2? false
      set extension3? false
      set vir-obs []
    ]
    if algorithm = "TangentBug" or algorithm = "TangentBug(2)" [
      set ltg-list []
      set transition1? false
      set transition2? false
    ]
    if algorithm = "K-Bug" [
      set step_forward? false
      set edge nobody
      set node nobody
      ifelse not target-visible? target [set H find-wall (towards target) r] [set H nobody]
      set last-node nobody
      ifelse H != nobody [find-node face node] [face target]
    ]
    if algorithm = "New Algorithm" [
      set step r   ;; set value to that of R (unless it will be problematic!)
      set d-min 0
      set m-line []
    ]
    if algorithm = "COPE0" [
      set step r   ;; set value to that of R (unless it will be problematic!)
      set d-min 0
    ]
    if algorithm = "Vertex" [
      set ltg-list []
      set int-target patch-here
      set dest target
      set r 50
    ]
  ]

  reset-ticks
end


to reset-people
  ask people [
    pen-up
    move-to pre-loc
    ;pen-down
    set last-dummy []
    set last-dummy lput self last-dummy
    set last-dummy lput (one-of starts-here) last-dummy
    set color red
    face target
    set follow-wall? false
    set H nobody
    set direction pre-dir

    if algorithm = "Bug1" [
      set L nobody
      set search-mode? false
    ]
    if algorithm = "Bug2" [create-mline]
    if algorithm = "DistBug" [
      set step r   ;; set value to that of R (unless it will be problematic!)
      set d-min 0
      set m-line []
      set extension2? false
      set extension3? false
      set vir-obs []
    ]
    if algorithm = "TangentBug" or algorithm = "TangentBug(2)" [
      set ltg-list []
      set transition1? false
      set transition2? false
    ]
    if algorithm = "K-Bug" [
      set step_forward? false
      set edge nobody
      set node nobody
      ifelse not target-visible? target [set H find-wall (towards target) r] [set H nobody]
      set last-node nobody
      ifelse H != nobody [find-node face node] [face target]
    ]
  ]
end

to reset-trapped
  ask trappeds [
    set breed people
    pen-up
    move-to pre-loc
    ;pen-down
    set last-dummy []
    set last-dummy lput self last-dummy
    set last-dummy lput (one-of starts-here) last-dummy
    set color 43
    face target
    set follow-wall? false
    set H nobody
    set direction pre-dir

    if algorithm = "Bug1" [
      set L nobody
      set search-mode? false
    ]
    if algorithm = "Bug2" [create-mline]
    if algorithm = "DistBug" [
      set step r   ;; set value to that of R (unless it will be problematic!)
      set d-min 0
      set m-line []
      set extension2? false
      set extension3? false
      set vir-obs []
    ]
    if algorithm = "TangentBug" or algorithm = "TangentBug(2)" [
      set ltg-list []
      set transition1? false
      set transition2? false
    ]
    if algorithm = "K-Bug" [
      set step_forward? false
      set edge nobody
      set node nobody
      ifelse not target-visible? target [set H find-wall (towards target) r] [set H nobody]
      set last-node nobody
      ifelse H != nobody [find-node face node] [face target]
    ]
  ]
end


to-report distance2v [agent1 agent2]
  let x1 0 let x2 0 let y1 0 let y2 0
  ifelse is-turtle? agent1 [set x1 [xcor] of agent1 set y1 [ycor] of agent1][set x1 [pxcor] of agent1 set y1 [pycor] of agent1]
  ifelse is-turtle? agent2 [set x2 [xcor] of agent2 set y2 [ycor] of agent2][set x2 [pxcor] of agent2 set y2 [pycor] of agent2]
  report sqrt ((x1 - x2) ^ 2 + (y1 - y2) ^ 2)
end

to-report distance2v-min [agent1 agent2]
  let x1 0 let x2 0 let y1 0 let y2 0 let xm 0 let ym 0
  ifelse is-turtle? agent1 [set x1 [xcor] of agent1 set y1 [ycor] of agent1][set x1 [pxcor] of agent1 set y1 [pycor] of agent1]
  ifelse is-turtle? agent2 [set x2 [xcor] of agent2 set y2 [ycor] of agent2][set x2 [pxcor] of agent2 set y2 [pycor] of agent2]
  set xm abs (x1 - x2) set ym abs (y1 - y2)
  ifelse xm < ym
  [
    report xm
    ]
  [
      report ym
    ]

end

to-report distance2v-max [agent1 agent2]
  let x1 0 let x2 0 let y1 0 let y2 0 let xm 0 let ym 0
  ifelse is-turtle? agent1 [set x1 [xcor] of agent1 set y1 [ycor] of agent1][set x1 [pxcor] of agent1 set y1 [pycor] of agent1]
  ifelse is-turtle? agent2 [set x2 [xcor] of agent2 set y2 [ycor] of agent2][set x2 [pxcor] of agent2 set y2 [pycor] of agent2]
  set xm abs (x1 - x2) set ym abs (y1 - y2)
  ifelse xm < ym
  [
    report ym
    ]
  [
      report xm
    ]

end
@#$#@#$#@
GRAPHICS-WINDOW
719
19
1224
525
-1
-1
7.0
1
10
1
1
1
0
0
0
1
-35
35
-35
35
1
1
1
ticks
30.0

BUTTON
549
461
693
544
GO!
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
32
60
139
93
Random Space
setup-random\nset target-location \"random\"
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
404
346
543
379
Make People
make-turtles
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

INPUTBOX
405
128
480
188
walk-method
0.0
1
0
Number

MONITOR
720
739
836
784
count people left
count people
17
1
11

BUTTON
175
289
315
322
Manual Setup
setup-target2
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
31
106
139
139
L shape
L-obstacle\nset target-location \"up\"
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

TEXTBOX
499
130
720
186
0: speed/vision radius = 1\n1: random speed/vision radius [0.5 1]\n* Set walk-method before making people.
11
0.0
1

INPUTBOX
403
58
479
118
population
1000.0
1
0
Number

TEXTBOX
332
482
482
608
Instructions:\n1) Setup Obstacles\n2) Setup Target\n3) Setup Population\n4) Adjust Settings\n5) GO!
14
0.0
1

MONITOR
719
677
837
722
count trapped people
count trappeds
17
1
11

INPUTBOX
176
220
245
280
target-xcor
-1.0
1
0
Number

INPUTBOX
249
220
315
280
target-ycor
-22.0
1
0
Number

CHOOSER
406
208
544
253
algorithm
algorithm
"Bug1" "Bug2" "VisBug" "DistBug" "TangentBug" "TangentBug(2)" "K-Bug" "M-DB" "Vertex"
7

SWITCH
558
228
679
261
Extension-1
Extension-1
0
1
-1000

SWITCH
559
267
680
300
Extension-2
Extension-2
1
1
-1000

SWITCH
559
306
680
339
Extension-3
Extension-3
1
1
-1000

TEXTBOX
559
205
674
223
DistBug's Extensions:
11
0.0
1

BUTTON
31
151
139
184
U shape
U-obstacle\nset target-location \"middle\"
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
199
62
301
95
Setup Target
setup-target
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SWITCH
39
530
181
563
human-interaction?
human-interaction?
0
1
-1000

BUTTON
30
238
139
271
Triangle random
triangle-random\nset target-location \"up\"
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
29
283
138
316
T corridor
T-corridor\nset target-location \"up\"
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
29
326
138
359
Box
close-obstacle\nset target-location \"trapped\"
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SWITCH
37
481
181
514
link?
link?
1
1
-1000

BUTTON
30
370
141
403
Square
square\nset target-location \"up\"
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

TEXTBOX
35
26
142
44
Setup Obstacles:
14
0.0
1

TEXTBOX
211
24
305
42
Setup Target:
14
0.0
1

TEXTBOX
178
108
328
136
________________________
11
0.0
1

BUTTON
211
144
288
177
Random
setup-target-random
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

TEXTBOX
178
196
328
214
________________________
11
0.0
1

TEXTBOX
402
24
552
42
Setup Population:
14
0.0
1

TEXTBOX
38
443
188
461
Settings:
14
0.0
1

TEXTBOX
198
490
348
508
Bread Crumbs
11
0.0
1

TEXTBOX
199
543
349
561
To Avoid Collision
11
0.0
1

PLOT
871
589
1228
787
% people survived
ticks
NIL
0.0
10.0
0.0
100.0
true
false
"clear-plot" ""
PENS
"default" 1.0 0 -16777216 true "" "plot ((population - count people) / population * 100)"

BUTTON
31
194
139
227
U shape 2
U-obstacle\nset target-location \"down\"
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

@#$#@#$#@
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270
@#$#@#$#@
NetLogo 6.0.4
@#$#@#$#@
random-seed 2
setup
repeat 50 [ go ]
ask turtles [ pen-down ]
repeat 150 [ go ]
@#$#@#$#@
@#$#@#$#@
<experiments>
  <experiment name="DistBug" repetitions="1000" runMetricsEveryStep="false">
    <setup>close-obstacle
setup-target
make-turtles</setup>
    <go>go</go>
    <timeLimit steps="1000"/>
    <metric>count trappeds</metric>
    <metric>count people</metric>
    <metric>timer</metric>
    <enumeratedValueSet variable="walk-method">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="population">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="human-interaction?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="algorithm">
      <value value="&quot;DistBug&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Extension-1">
      <value value="true"/>
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Extension-2">
      <value value="true"/>
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Extension-3">
      <value value="true"/>
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="Bug1&amp;2KBug" repetitions="100" runMetricsEveryStep="false">
    <setup>L-obstacle
setup-target
make-turtles</setup>
    <go>go</go>
    <timeLimit steps="1000"/>
    <metric>count trappeds</metric>
    <metric>count people</metric>
    <metric>timer</metric>
    <enumeratedValueSet variable="walk-method">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="population">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="human-interaction?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="target-location">
      <value value="&quot;up&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="algorithm">
      <value value="&quot;Bug1&quot;"/>
      <value value="&quot;Bug2&quot;"/>
      <value value="&quot;K-Bug&quot;"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="Tangent" repetitions="100" runMetricsEveryStep="false">
    <setup>close-obstacle
setup-target
make-turtles</setup>
    <go>go</go>
    <timeLimit steps="1000"/>
    <metric>count trappeds</metric>
    <metric>count people</metric>
    <metric>timer</metric>
    <enumeratedValueSet variable="walk-method">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="population">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="human-interaction?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="target-location">
      <value value="&quot;trapped&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="algorithm">
      <value value="&quot;TangentBug(2)&quot;"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="NEW" repetitions="1000" runMetricsEveryStep="false">
    <setup>close-obstacle
setup-target
make-turtles</setup>
    <go>go</go>
    <timeLimit steps="1000"/>
    <metric>count trappeds</metric>
    <metric>count people</metric>
    <metric>timer</metric>
    <enumeratedValueSet variable="walk-method">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="population">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="human-interaction?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="target-location">
      <value value="&quot;trapped&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="algorithm">
      <value value="&quot;DistBug&quot;"/>
      <value value="&quot;New Algorithm&quot;"/>
      <value value="&quot;New Algorithm2&quot;"/>
      <value value="&quot;New Algorithm3&quot;"/>
    </enumeratedValueSet>
  </experiment>
</experiments>
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180

link1
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 120 225
Line -7500403 true 150 150 180 225
Polygon -7500403 true true 75 285 225 285 150 90

link2
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0

link3
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 120 225
Line -7500403 true 150 150 180 225
Polygon -7500403 true true 45 285 255 285 150 0
@#$#@#$#@
0
@#$#@#$#@
